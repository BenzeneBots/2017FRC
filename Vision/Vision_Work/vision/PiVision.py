#	Raspberry Pi Vision - 2017 Season - Steamworks
#
#		Benzene Bots - FRC Team #4384
#
# This program loads an OpenCV vision pipeline that was previously 
# generated using Grip.  The Grip python code is loaded using an
# import statement.  This loads a callable class object that processes 
# one frame of video.  The results are then returned here as a set 
# of contours. 
#

import numpy as np
import argparse
import time
import cv2

from picamera.array import PiRGBArray
from picamera import PiCamera

from networktables import NetworkTables

import logging
logging.basicConfig( level=logging.DEBUG )

# Code generated from Grip's Python export feature.
from sim_four import GripPipeline

# Simply square two numbers.
#==============================================================================
def sq( val ):
	return val * val

NetworkTables.initialize( server='10.43.84.81' )
#NetworkTables.initialize( server='roborio-4384-frc.local' )

# This adds 'Vision' to the Network Table as a sub folder under 'root'.
sd = NetworkTables.getTable( 'Vision' )
print "sd: ", sd

grip = GripPipeline()

# Setup Raspberry Pi Camera
cam = PiCamera()
cam.resolution = (640, 480)
cam.framerate = 32
rawCapture = PiRGBArray( cam, size=(640,480) )

cam.iso = 100
cam.brightness = 35

# These are the default Pi Camera settings.
print "Sharpness: ", cam.sharpness
print "Brightness: ", cam.brightness
print "Saturation: ", cam.saturation
print "ISO: ", cam.iso	# A value of zero means auto. Range = 100 to 800
print "Analog Gain: ", cam.analog_gain
print "AWB Mode: ", cam.awb_mode
print "Contrast: ", cam.contrast
print "Digital Range Compression: ", cam.drc_strength
print "Vid Stabilization: ", cam.video_stabilization
print "Exposure Comp: ", cam.exposure_compensation
print "Exposure Mode: ", cam.exposure_mode
print "Shutter Speed: ", cam.exposure_speed
print "Image DeNoise: ", cam.image_denoise
print "Vid DeNoise: ", cam.video_denoise
print "Meter Mode: ", cam.meter_mode
print "Image Effect: ", cam.image_effect
print "Color Effects: ", cam.color_effects
print "Zoom: ", cam.zoom
#cam.rotation = 0
#cam.hflip = False
#cam.vflip = False
#cam.crop = (0.0, 0.0, 1.0, 1.0)
print

time.sleep( 0.1 )	# Camera Warmup Time
i=0			# Init heartbeat counter.
guiDebug = True		# Set True for debugging GUI window.
cap = -1		# Init for capture image name generation.

# Loop until 'q' keypress.
#------------------------------------------------------------------------------
for imgArray in cam.capture_continuous( rawCapture, format="bgr", use_video_port=True ):

	frame = imgArray.array

	key = cv2.waitKey(1)
	if key & 0xFF == ord("q"):
		# Break the loop (quit program) on a 'q' keystroke.
		break
	elif key == ord("c"):
		# Save frame as a captured JPG image on a 'c' keystroke.
		# Do this BEFORE using OpneCV to draw on the frame.
		cap = cap + 1
		print "Capture Saved: ", cap
		cv2.imwrite( "capture" + str(cap) + ".jpg", frame )

	try:
		# Sometimes Grip.Process blows up with a divide by zero!
		grip.process( frame )			# Process a frame using Grip.
		contours = grip.filter_contours_output	# Get the contours found.
		nContour = len( contours )
	except:
		# On exception, setting num contours to zero prevents processing below.
		nCountour = 0
		print "Grip Error"

	sd.putNumber( 'nContour', nContour )	# Send number of contours found.

	if nContour >= 2:
		# Sort the contours and find the two largest.
		sortedCon = sorted(contours, key = cv2.contourArea, reverse = True)
		#cnt = sorted(contours, key = cv2.contourArea, reverse = True)[0]
		#cnt2 = sorted(contours, key = cv2.contourArea, reverse = True)[1]
		cnt = sortedCon[0]
		cnt2 = sortedCon[1]

		# Combine the contours and compute a rotated bounding box.
		c = cv2.minAreaRect( np.concatenate((cnt, cnt2), axis=0) )
		((cx, cy), (cw_, ch_), ca) = c
		ch = min( cw_, ch_)
		cw = max( cw_, ch_)
		#rc = np.int32( cv2.boxPoints( c ) )
		#cv2.drawContours(frame, [rc], -1, (255, 0, 0), 2)

		# Compute the (rotated) bounding box around the two largest.
		t1 = cv2.minAreaRect(cnt)	# Returns (xLoc,yLoc, width,height, angle)
		rect = np.int32(cv2.boxPoints(t1))
		t2 = cv2.minAreaRect(cnt2)
		rect2 = np.int32(cv2.boxPoints(t2))

		# Unpack minAreaRect values for each contour.
		( (x1,y1), (h1, w1), ang1 ) = t1
		( (x2,y2), (h2, w2), ang2 ) = t2
		#print wid1, ht1, ang1, wid2, ht2, ang2

		wid1 = max( h1, w1 )	# Make sure width is the bigger number!
		ht1 = min ( h1, w1 )
		wid2 = max( h2, w2 )
		ht2 = min ( h2, w2 )

		# Ratio between widths should be about the same.
		widthRatio = wid1 / (wid2 * 1.1)	
		#print "Width Ratio: %0.2f" % widthRatio

		# Ratio between heights should be 1H / (2H * 2).
		heightRatio = ht1 / (ht2 * 2.0)	
		#print "Height Ratio: %0.2f" % heightRatio

		# Group Height = 1H /((2B - 1T) *.4) 
		# Top height should be 40% of total height (4in / 10 in.)
		grpHeight = ht1 / (ch * 0.6)
		#print "Group Height: %0.2f %0.2f %0.2f" % (grpHeight, ht1, ch)

		# LEdge = ((1L - 2L) / 1W) + 1  The distance between the left edge 
		# of contour 1 and the left edge of contour 2 should be small relative 
		# to the width of the 1st contour (then we add 1 to make the ratio 
		# centered on 1
		lEdge = ( x1 - x2 ) / wid1 + 1.0
		#print "Left Edge: %0.2f" % lEdge 
 
		# Compute and overall confidence value where 1.0 is perfect.
		confidence = (sq(widthRatio) + sq(heightRatio) + sq(grpHeight) + sq(lEdge)) / 4.0
		#print "Confidence: %0.2f - %0.2f %0.2f %0.2f %0.2f" % \
		#	(confidence, widthRatio, heightRatio, grpHeight, lEdge)

		# For debugging, draw the two bounding boxes on the frame for display later.
		if guiDebug:
			cv2.drawContours(frame, [rect], -1, (0, 0, 255), 2)
			cv2.drawContours(frame, [rect2], -1, (0, 0, 255), 2)
			# On a keyboard press, also save the marked up frame.
			if key == ord('c'):
				cv2.imwrite( "capDebug" + str(cap) + ".jpg", frame )
			if key == ord('p'):
				print "%0.2f %0.2f %0.2f %0.2f" % (x1, y1, x2, y2)
				print "%0.2f %0.2f %0.2f %0.2f" % (wid1, ht1, wid2, ht2)
				print "%0.2f %0.2f %0.2f %0.2f" % (cx, cy, cw, ch)
		print "%0.2f %0.2f" % (ch, (320.0 / (cx)) )

		sd.putNumber( 'Dist', rect[0][0] )
		sd.putNumber( 'Angle', rect[2][0] )
	else:
		sd.putNumber( 'Dist', 0 )
		sd.putNumber( 'Angle', 0 )
		#sd.putNumberArray( 'Rect0', [0,0] );

	if guiDebug:
		cv2.imshow( "Target", frame )

	# Must dump the frame before capturing another.
	rawCapture.truncate( 0 )

	# Keep a rolling counter going so RoboRio "knows" the Pi
	# is up and running.  Kinda like a heartbeat.
	sd.putNumber( 'PiCounter', i )
	i = i+1		
	if i > 100000: i=0

	try:
		t = sd.getNumber( 'robotTime' )
		print( 'robotTime:', t )
	except KeyError:
		continue

	#time.sleep( 0.2 )

# cleanup the camera and close any open windows
cam.close()
cv2.destroyAllWindows()

