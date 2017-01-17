import numpy as np
import argparse
import time
import cv2

from picamera.array import PiRGBArray
from picamera import PiCamera

# Setup Raspberry Pi Camera
cam = PiCamera()
cam.resolution = (640, 480)
cam.framerate = 32
rawCapture = PiRGBArray( cam, size=(640,480) )

time.sleep( 0.1 ) # Camera Warmup Time

# Define the upper and lower boundaries for a color
# to be considered.
colorLower = np.array([30, 30, 100], dtype = "uint8")
colorUpper = np.array([60, 60, 160], dtype = "uint8")

# Loop until 'q' keypress.
for imgArray in cam.capture_continuous( rawCapture, format="bgr", use_video_port=True ):
	frame = imgArray.array

	# determine which pixels fall within the color boundaries
	# and then blur the binary image
	color = cv2.inRange(frame, colorLower, colorUpper)
	color = cv2.GaussianBlur(color, (3, 3), 0)

	# find contours in the image
	(_, cnts, _) = cv2.findContours(color.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

	# check to see if any contours were found
	if len(cnts) > 0:
		# sort the contours and find the largest one -- we
		# will assume this contour correspondes to the area
		# of my phone
		cnt = sorted(cnts, key = cv2.contourArea, reverse = True)[0]

		# compute the (rotated) bounding box around then
		# contour and then draw it		
		rect = np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))
		cv2.drawContours(frame, [rect], -1, (0, 255, 0), 2)

	# show the frame and the binary image
	cv2.imshow( "Tracking", frame )
	cv2.imshow( "Binary", color )

	# Must dump the frame before capturing another.
	rawCapture.truncate( 0 )

	# if the 'q' key is pressed, stop the loop
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break

# cleanup the camera and close any open windows
cam.close()
cv2.destroyAllWindows()

