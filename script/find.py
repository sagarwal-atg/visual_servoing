import cv2
import numpy
from matplotlib import pyplot as plt
import numpy as np
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callback(data):
    bridge = CvBridge()
    cv_image = None
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    # print(cv_image.shape)
    # buoy = cv2.imread('buoy_sim.png')
    # templ_gray = cv2.cvtColor(buoy, cv2.COLOR_BGR2GRAY)
    # frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # tem_match(cv_image, frame_gray, templ_gray)
    hsv(cv_image)

def hsv(img):
    hsv = img
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([200,200, 0])
    upper_red = np.array([255,255, 50])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(img, img, mask= mask)
    cv2.imshow("orig",img)
    cv2.imshow("hsv",hsv)
    cv2.imshow("mask",mask)
    cv2.imshow("res",res)
    cv2.waitKey(100)

def tem_match(orig, src, templ):
        img = src
        img2 = img.copy()
        template = templ
        w, h = template.shape[::-1]
        tl = bl = None
        point = None
        methods = ['cv2.TM_CCOEFF_NORMED']
        for meth in methods:
            img = img2.copy()
            resize_i = img2.copy()
            method = eval(meth)
            orig_res = None
            for i in range(1):
                resize_i = cv2.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv2.INTER_AREA)
                # print(resize_i.shape)
                # Apply template Matching
                res = cv2.matchTemplate(resize_i, template, method)
                if i == 0:
                    orig_res = res
                # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                # tl = max_loc
                # br = (tl[0] + w, tl[1] + h)
                # cv2.rectangle(orig,tl, br, 255, 2)
                threshold = 0.50
                loc = np.where( res >= threshold)
                if len(zip(*loc[::-1])) > 0:
                    print("True")
                else:
                    print("False")
                # for pt in zip(*loc[::-1]):
                #     cv2.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,0,255), 1)
                #     point = pt
                #     print("True")
        #
        # try:
        #    tl = (point[0]*int(2**(0.5*i)),point[1]*int(2**(0.5*i)))
        #    br = ((point[0] + w), (point[1] + h))
        # except Exception() as e:
        #    print(e)
        #
        # center = ((br[0]-tl[0])/2 , (br[1]-tl[1])/2)
        # cv2.circle(orig,(tl[0] + center[0], tl[1] + center[1]), 1, (0,0,255), 2)
        cv2.imshow('Matching Result', orig_res)
        cv2.imshow('Detected Point', orig)
        cv2.waitKey(100)

if __name__ == '__main__':
    rospy.init_node("asdasd")
    sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image, callback)
    rospy.spin()
