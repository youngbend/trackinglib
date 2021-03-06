import cv2
import Tracker
import numpy as np
import time

im = cv2.imread("/home/ybenjamin/edge_case.jpg")
im = cv2.resize(im, (640,480))
tracker = Tracker.Tracker(2,2,6,30,15,15)

begin = time.time()
gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
tracker.scan(gray, 1)
print("Time to run = {} seconds".format(time.time() - begin))

centers = tracker.get_target_centers()
print("Centers at:")
for center in centers:
    print("({},{})".format(center.row, center.col))
    im = cv2.circle(im, (center.col, center.row), 5, (0,0,255), -1)

cv2.imshow("Tracker output", im)
cv2.waitKey(0)
cv2.destroyAllWindows()
