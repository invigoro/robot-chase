import cv2 as cv
import numpy as np

cap = cv.VideoCapture("trail_source.avi")


while True:
    #get the current frame
    status, img = cap.read()

    #make grayscale and avg blur
    grayimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    grayblur = cv.blur(grayimg, (3,3))

    #convert back to BGR and take difference
    grayblur = cv.cvtColor(grayblur, cv.COLOR_GRAY2BGR)
    #sub = cv.absdiff(grayblur, img)
    sub = cv.subtract(img, grayblur)

    #reduce colors
    z = sub.reshape((-1,3))
    z = np.float32(z)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 16
    ret, label, center = cv.kmeans(z, K, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((img.shape))

    #show results
    cv.imshow("Grayblur", grayblur)
    cv.imshow("Subtracted", sub)
    cv.imshow("Image", img)
    cv.imshow("Reduced Colors", res2)

    #floodfill
    '''width, height, channel = img.shape
    mask = np.zeros((width + 2, height + 2, 1), np.uint8)
    cv.floodFill(sub, mask, (0,0), (0,0,0))
    cv.imshow("What", sub)'''

    #exit on escape key
    k = cv.waitKey(1)
    if k == 27:
        break

cv.destroyAllWindows()