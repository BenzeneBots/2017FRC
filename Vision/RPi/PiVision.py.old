# 
# Benzene Bots - Python Vision for Raspberry Pi
# 
# The is the master Python code that calls the vision pipeline code that was 
# generate by Grip.  Basic vision processing and tuning is done in the Grip
# application.  Advanced final vision processing is done within the code 
# below.  Also, the vision results are sent out using the NetworkTables library.
#
# Last Updated: JKemp Feb 2017
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
from gripgreen import GripPipeline

NetworkTables.initialize( server='10.43.84.2' )
sd = NetworkTables.getTable( "SmartDashboard" )

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

time.sleep( 0.1 ) # Camera Warmup Time

# Loop until 'q' keypress.
for imgArray in cam.capture_continuous( rawCapture, format="bgr", use_video_port=True ):

	frame = imgArray.array

	grip.process( frame )			# Process a frame using Grip.
	contours = grip.filter_contours_output	# Get the contours found.

	if len( contours ) > 0:
		# Sort the contours and find the largest one.
		cnt = sorted(contours, key = cv2.contourArea, reverse = True)[0]
		# Compute the (rotated) bounding box around the largest.
		rect = np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))
		cv2.drawContours(frame, [rect], -1, (0, 0, 255), 2)
		# print rect
		# print "Num Contours: ", len( contours )
		# print
	cv2.imshow( "Target", frame )

	# Must dump the frame before capturing another.
	rawCapture.truncate( 0 )

	key = cv2.waitKey(1)
	if key & 0xFF == ord("q"):
		# Break the loop (quit program) on a 'q' keystroke.
		break
	elif key == ord("p"):
		# Save frame as a captured JPG image on a 'p' key.
		cv2.imwrite( "capture.jpg", frame )


	try:
		print( 'robotTime:', sd.getNumber( 'robotTime' ))
	except KeyError:
		continue

# cleanup the camera and close any open windows
cam.close()
cv2.destroyAllWindows()

