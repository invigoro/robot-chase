# import the necessary packages
'''from picamera.array import PiRGBArray
from picamera import PiCamera'''
import time
import cv2 as cv
import numpy as np
'''
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    pic = cv.Canny(image, 100, 170)
    # show the frame
    cv2.imshow("Frame", pic)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
            break'''


cap = cv.VideoCapture("trail_source.avi")

kernel = np.ones((5,5), np.uint8)

while True:
    #get the current frame
    status, img = cap.read()

    #make grayscale and avg blur
    grayimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    grayblur = cv.blur(grayimg, (5,5))

    #convert back to BGR and take difference
    grayblur = cv.cvtColor(grayblur, cv.COLOR_GRAY2BGR)
    #sub = cv.absdiff(grayblur, img)
    sub = cv.subtract(img, grayblur)

    _, thresh = cv.threshold(sub, 20, 255, cv.THRESH_BINARY)
    bgr = [0, 0, 230]
    tol = 40

    minBGR = np.array([bgr[0] - tol, bgr[1] - tol, bgr[2] - tol])
    maxBGR = np.array([bgr[0] + tol, bgr[1] + tol, bgr[2] + tol])

    mask = cv.inRange(thresh, minBGR, maxBGR)
    
    img_erosion = cv.erode(mask, kernel, iterations=1)    #erode white
    img_dilation = cv.dilate(img_erosion, kernel, iterations=2) #dilate black

    moments = cv.moments(img_dilation, True)
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    print(str(cx) + " " + str(cy))
    cv.circle(img, (cx, cy), 8, (0,0,255), -1)

    cv.imshow("Dilated", img_dilation)
    #output = cv.bitwise_and(thresh, thresh, mask = mask)

    cv.imshow("Mask", mask)

    #show results
    cv.imshow("Grayblur", grayblur)
    cv.imshow("Subtracted", sub)
    cv.imshow("Image", img)
    cv.imshow("Thresh", thresh)

    #floodfill
    '''width, height, channel = img.shape
    mask = np.zeros((width + 2, height + 2, 1), np.uint8)
    tol = 25
    val = (tol, tol, tol)
    cv.floodFill(sub, mask, (0,0), (0,0,0), val, val, 8)
    cv.imshow("What", sub)'''

    #exit on escape key
    k = cv.waitKey(1)
    if k == 27:
        break

cv.destroyAllWindows()
