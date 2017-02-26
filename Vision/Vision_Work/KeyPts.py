import cv2
import numpy as np

img = cv2.imread('capture0.jpg')
gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )

#sift = cv2.SIFT()
sift = cv2.xfeatures2d.SIFT_create()

#kp = sift.detect( gray, None )
kp, des = sift.detectAndCompute( gray, None )


img = cv2.drawKeypoints( gray, kp, img, (0,255,0) )

cv2.imshow( 'jpg',img)

while 1: 
	key = cv2.waitKey( 500 )
	if key & 0xFF == ord("q"):
		# Break the loop (quit program) on a 'q' keystroke.
		break

cv2.destroyAllWindows()

