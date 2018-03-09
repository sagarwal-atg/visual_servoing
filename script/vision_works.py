#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import CMT
import numpy as np
import util

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
import geometry_msgs.msg as geometry_msgs
from cv_bridge import CvBridge, CvBridgeError

buoy_hit = False

class Extract_center:
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.CMT = CMT.CMT()
        self.bridge = CvBridge()

        self.tracker = cv2.TrackerTLD_create()

        self.count = 0

        self.first = True
        self.first_ttc = True

        self.rect = None
        self.init_img = None

        self.center = None
        self.prev_center = None

        self.hit_buoy = False
        self.track_window = None

        self.h = None
        self.w = None

        self.dt = 1.0/19.0
        self.bbox_h = None
        self.bbox_h_init = None
        self.bbox_h_prev = None

        self.linear_speed_x = 0.55
        # self.k_yaw = 0.0005
        # self.k_alt = 0.0005
        self.k_yaw = 0.0008
        self.k_alt = 0.0008

        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image,self.callback)
        self.jerk_sub = rospy.Subscriber("/jerk",Float32,self.jerk_callback)

        self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", numpy_msg(geometry_msgs.Twist), queue_size=1)

    def tem_match(self, orig, src, templ):
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
                print(resize_i.shape)
                # Apply template Matching
                res = cv2.matchTemplate(resize_i, template, method)
                if i == 0:
                    orig_res = res
                # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                # tl = max_loc
                # br = (tl[0] + w, tl[1] + h)
                # cv2.rectangle(orig,tl, br, 255, 2)
                threshold = 0.70
                loc = np.where( res >= threshold)
                for pt in zip(*loc[::-1]):
                    cv2.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,0,255), 1)
                    point = pt

        try:
           tl = (point[0]*int(2**(0.5*i)),point[1]*int(2**(0.5*i)))
           br = ((point[0] + w), (point[1] + h))
        except Exception() as e:
           print(e)

        self.center = center = ((br[0]-tl[0])/2 , (br[1]-tl[1])/2)
        cv2.circle(orig,(tl[0] + center[0], tl[1] + center[1]), 1, (0,0,255), 2)
        cv2.imshow('Matching Result', orig_res)
        cv2.imshow('Detected Point', orig)
        cv2.waitKey(100)
        return tl, br

    def preprocess(self, data):
        num_down = 2       # number of downsampling steps
        num_bilateral = 7  # number of bilateral filtering steps
        img_rgb = data
        # downsample image using Gaussian pyramid
        img_color = img_rgb
        for _ in xrange(num_down):
            img_color = cv2.pyrDown(img_color)
        # repeatedly apply small bilateral filter instead of
        # applying one large filter
        for _ in xrange(num_bilateral):
            img_color = cv2.bilateralFilter(img_color, d=9,
                                            sigmaColor=9,
                                            sigmaSpace=7)
        # upsample image to original size
        for _ in xrange(num_down):
            img_color = cv2.pyrUp(img_color)
        return img_color

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

          # if self.first or self.count%int(10+self.count/2) == 0:
        if self.first:
            self.init_img = cv_image
            self.first = False
            self.h = cv_image.shape[0]
            self.w = cv_image.shape[1]
            print("h: " + str(self.h) + " "+ "w: " + str(self.w) )
            tl, br = self.extract_init_img(self.init_img)
            bbox = (tl[0], tl[1], br[0]-tl[0], br[1]-tl[1])
            self.tracker.init(self.init_img, bbox)
            # self.CMT.initialise(self.init_img, tl, br)
            # self.track_camshift_init(tl, br, self.init_img)
            self.count += 1
        elif self.count:
            self.count += 1
            print("Tracking " + str(self.count))
            self.opencv_tracker(cv_image)
            # self.track(cv_image)
            if not self.hit_buoy:
                self.target_follower("forward")
        else:
            self.count += 1

        cv2.imshow("Final", cv_image)
        cv2.waitKey(10)

    def target_follower(self, direc):
        msg = geometry_msgs.Twist()
        if direc == 'forward':
            d_alt = self.k_alt*(self.h/2 - self.center[1])
            d_yaw = self.k_yaw*(self.w/2 - self.center[0])

            msg.linear.x = self.linear_speed_x
            msg.linear.z = d_alt

            msg.angular.z = d_yaw
        elif direc == 'backward':
            msg.linear.x = -self.linear_speed_x
            msg.linear.z = 0

            msg.angular.z = 0

        self.des_vel_pub.publish(msg)


    def extract_init_img(self, cv_frame):
        buoy = cv2.imread('buoy_sim.png')
        templ_gray = cv2.cvtColor(buoy, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
        self.h , self.w = frame_gray.shape
        tl, br = self.tem_match(cv_frame, frame_gray, templ_gray)
        # self.contour_detection(self.init_img, frame_gray)
        return tl, br

    def track_camshift_init(self, tl, br, frame):
        c, h, r , w = tl[0], br[0] - tl[0], tl[1], br[1] - tl[1]

        self.track_window = (c,r,w,h)
        self.roi = frame[r:r+h, c:c+w]
        self.hsv_roi =  cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
        self.roi_hist = cv2.calcHist([self.hsv_roi],[0],self.mask,[180],[0,180])
        cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)

        self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5, 1 )

    def track_camshift(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[0], self.roi_hist,[0,180],1)
        ret, self.track_window = cv2.CamShift(dst, self.track_window, self.term_crit)
        # print(ret[1])
        self.center = center = ( int(ret[0][0]), int(ret[0][1]))
        print(center)
        # Draw it on image
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        img2 = cv2.polylines(frame,[pts],True, 255,2)
        cv2.imshow('Cam shift',img2)
        cv2.circle(img2,(center[0], center[1]), 1, 255, 2)
        cv2.circle(img2,(self.w/2, self.h/2), 1, (0,255,0), 2)

    def opencv_tracker(self,img):
        frame = img.copy()
        ok, bbox = self.tracker.update(frame)
        # Draw bounding box
        if ok:
            # Tracking success
            tl = (int(bbox[0]), int(bbox[1]))
            br = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, tl, br, (255,0,0), 2, 1)
            self.bbox_h = bbox[2]/2
            self.time_to_contact()
        else :
            print("Tracking failure")
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        cv2.imshow('tracker',frame)
        self.center = center = ( tl[0] + (br[0]-tl[0])/2 , tl[1] + (br[1]-tl[1])/2)

    def track(self, cv_image):
        # cv_image = self.preprocess(cv_image)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.CMT.process_frame(cv_image)
        try:
            br = self.CMT.br
            tl = self.CMT.tl
            self.prev_center = self.center
            self.center = center = ( tl[0] + (br[0]-tl[0])/2 , tl[1] + (br[1]-tl[1])/2)
            print("center " + str(self.center))
            tl_x = int(self.CMT.tl[0])
            tl_y = int(self.CMT.tl[1])

            br_x = int(self.CMT.br[0])
            br_y = int(self.CMT.br[1])
            cv2.rectangle(cv_image, (tl_x, tl_y),(br_x, br_y), (255, 0, 0), 4)
            cv2.circle(cv_image,(center[0], center[1]), 1, (255,0,0), 2)
            cv2.circle(cv_image,(self.w/2, self.h/2), 1, (0,255,0), 2)
        except Exception() as e:
            print(e)
            self.center = self.prev_center

    def time_to_contact(self):
        if self.first_ttc:
            self.bbox_h_init = self.bbox_h
            self.first_ttc = False
        else:
            dh = abs(self.bbox_h_init - self.bbox_h)
            if dh != 0:
                dh_dt = dh/self.dt
                print("TIME TO CONTACT " + str(self.bbox_h_init/dh_dt) + " secs")

    def jerk_callback(self, msg):
        jerk = msg.data
        if jerk > 1.5:
            self.hit_buoy = True
            buoy_hit = True
            self.target_follower("backward")

def main(args):
  ec = Extract_center()
  rospy.init_node('target_follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
