# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import maestro

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

degreeTot = 0
direction = 1

class Control():
    def __init__(self):
        self.tango = maestro.Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000

    def forward(self):
        self.motors = 5000
        self.tango.setTarget(MOTORS, self.motors)
        print(str(self.motors) + " MOVE")
        time.sleep(.5)
        self.motors = 6000
        self.tango.setTarget(MOTORS, self.motors)
    def left(self):
        self.turn += 1000
        self.tango.setTarget(TURN, self.turn)
        print("LEFT")
        time.sleep(.25)
        self.turn -= 1000
        self.tango.setTarget(TURN, self.turn)
    def right(self):
        self.turn -= 1000
        self.tango.setTarget(TURN, self.turn)
        print("RIGHT")
        time.sleep(.25)
        self.turn += 1000
        self.tango.setTarget(TURN, self.turn)


controller = Control()

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

#for erosion/dilation
kernel = np.ones((5,5), np.uint8)

# allow the camera to warmup
time.sleep(2)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #get the current frame
    img = frame.array
    height, width, channel = img.shape
    #img = img[int(height * 0.5):height, int(width/4):int(width*0.75)]
    #height, width, channel = img.shape
    center = width / 2

    #make grayscale and avg blur
    grayimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    grayblur = cv.blur(grayimg, (5,5))

    #convert back to BGR and take difference
    #grayblur = cv.cvtColor(grayblur, cv.COLOR_GRAY2BGR)
    #sub = cv.absdiff(grayblur, img)
    #sub = cv.subtract(img, grayblur)

    #get rid of noise
    _, thresh = cv.threshold(grayblur, 180, 255, cv.THRESH_BINARY)

    #hsv = cv.cvtColor(thresh, cv.COLOR_BGR2HSV)

    '''#red color
    bgr = [0, 0, 255]
    tol = 40

    minBGR = np.array([bgr[0] - tol, bgr[1] - tol, bgr[2] - tol])
    maxBGR = np.array([bgr[0] + tol, bgr[1] + tol, bgr[2] + tol])

    #erase everything but red
    mask = cv.inRange(thresh, minBGR, maxBGR)'''

    img_erosion = cv.erode(thresh, kernel, iterations=4)    #erode white
    img_dilation = cv.dilate(img_erosion, kernel, iterations=5) #dilate black

    #find COG and draw it
    moments = cv.moments(img_dilation, True)


    try:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        degreeTot = 0
        direction = 1
        if abs(cx - center) < 30:
            #move forward
            controller.forward()
        elif cx > center:
            #rotate right
            controller.right()
        else:
            controller.left()
        time.sleep(0.1)
    except:
        cx = 0
        cy = 0
        print("No detected line")
        if degreeTot < -90:
            print("Line Completed")
            break
        elif degreeTot > 90:
            print("switching to left")
            direction = -1
            degreeTot = 90
        elif direction == 1:
            print("moving right")
            controller.right()
            degreeTot += direction * 10
        else:
            print("moving left")
            controller.left()
            degreeTot += direction * 10
        time.sleep(0.2)


    print(str(cx) + " " + str(cy))
    cv.circle(img, (cx, cy), 8, (0,0,255), -1)





    #show results
    cv.imshow("Image", img)
    cv.imshow("Dilated", img_dilation)
    #cv.imshow("Mask", mask)
    #cv.imshow("Threshold", thresh)
    #cv.imshow("Blur", grayblur)
    #cv.imshow("Sub", sub)

    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q") or key == 27:
            break

cv.destroyAllWindows()
controller.motors = 6000
controller.tango.setTarget(MOTORS, controller.motors)
controller.turn = 6000
controller.tango.setTarget(TURN, controller.turn)