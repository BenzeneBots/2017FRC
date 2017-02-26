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
from sim_three import GripPipeline

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

a = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0 ]
r = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0 ]


# Give two bounding rectangles, return a measure how close they are to each other.
def testCons( a, b ):
	( (x1, y1), (w1, h1), a1 ) = a
	( (x2, y2), (w2, h2), a2 ) = b
	band = x1 * 0.05
	low = x1 - band
	hi = x1 + band
	if ( (x2 >= low) & (x2 <= hi) ):
		return 1.0
	return 0.0

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

	i = 0

	# Process all (max 100) contours into rectangles.
	for c in contours:
		a[i] = cv2.minAreaRect( c )
		r[i] = np.int32( cv2.boxPoints( a[i] ) )
		cv2.drawContours( frame, [r[i]], -1, (0, 0, 255), 2)
		i = i + 1
		if i >= 10:
			break

	for k in range( 0, i-1):
		for j in range( k+1, i):
			t = testCons( a[k], a[j] )
			if t == 1.0:
				print k, j

	if nContour >= 2:
		# Sort the contours and find the two largest.
		sortedCon = sorted(contours, key = cv2.contourArea, reverse = True)
		#cnt = sorted(contours, key = cv2.contourArea, reverse = True)[0]
		#cnt2 = sorted(contours, key = cv2.contourArea, reverse = True)[1]
		cnt = sortedCon[0]
		cnt2 = sortedCon[1]

		# Combine the contours and compute a rotated bounding box.
		((cx, cy), (cw, ch), ca) = cv2.minAreaRect( np.concatenate((cnt, cnt2), axis=0) )

		# Compute the (rotated) bounding box around the two largest.
		t1 = cv2.minAreaRect(cnt)	# Returns (xLoc,yLoc, width,height, angle)
		rect = np.int32(cv2.boxPoints(t1))
		t2 = cv2.minAreaRect(cnt2)
		rect2 = np.int32(cv2.boxPoints(t2))

		# Unpack minAreaRect values for each contour.
		( (x1,y1), (wid1, ht1), ang1 ) = t1
		( (x2,y2), (wid2, ht2), ang2 ) = t2
		#print wid1, ht1, ang1, wid2, ht2, ang2

		# Ratio between widths should be about the same.
		widthRatio = wid1 / wid2	
		#print "Width Ratio: %0.2f" % widthRatio

		# Ratio between heights should be 1H / (2H * 2).
		heightRatio = ht1 / (ht2 * 2.0)	
		#print "Height Ratio: %0.2f" % heightRatio

		# Group Height = 1H /((2B - 1T) *.4) 
		# Top height should be 40% of total height (4in / 10 in.)
		grpHeight = ht1 / (ch * 0.4)
		#print "Group Height: %0.2f %0.2f %0.2f" % (grpHeight, ht1, ch)

		# LEdge = ((1L - 2L) / 1W) + 1  The distance between the left edge 
		# of contour 1 and the left edge of contour 2 should be small relative 
		# to the width of the 1st contour (then we add 1 to make the ratio 
		# centered on 1
		lEdge = ( rect[0][0] - rect2[0][0] ) / wid1 + 1.0
		#print "Left Edge: %0.2f" % lEdge 
 
		# Compute and overall confidence value where 1.0 is perfect.
		confidence = (sq(widthRatio) + sq(heightRatio) + sq(grpHeight) + sq(lEdge)) / 4.0
		#print "Confidence: %0.2f - %0.2f %0.2f %0.2f %0.2f" % \
		#	(confidence, widthRatio, heightRatio, grpHeight, lEdge)

		"""
		# For debugging, draw the two bounding boxes on the frame for display later.
		if guiDebug:
			cv2.drawContours(frame, [rect], -1, (0, 0, 255), 2)
			cv2.drawContours(frame, [rect2], -1, (0, 0, 255), 2)
			# On a keyboard press, also save the marked up frame.
			if key == ord('c'):
				cv2.imwrite( "capDebug" + str(cap) + ".jpg", frame )
		"""		

		sd.putNumber( 'Dist', rect[0][0] )
		sd.putNumber( 'Elev', rect[1][0] )
		sd.putNumber( 'Angle', rect[2][0] )
		#sd.putNumberArray( 'Rect0', rect[0] );
		#print rect
		#print
		# print "Num Contours: ", len( contours )
		# print
	else:
		sd.putNumber( 'Dist', 0 )
		sd.putNumber( 'Elev', 0 )
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

