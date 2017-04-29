#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

"""
# (rows, cols, channels) = cv_image.shape
# rospy.logdebug("rows: %s, cols: %s, channels: %s", rows, cols, channels)
"""
def get_roi_points(image, spacing):
    """hardcore fetching region of interests, in terms of subjective"""
    points_to_track = []
    for x in range(0, image.shape[0], spacing):
        for y in range(0, image.shape[1], spacing):
            new_point = [y, x]
            points_to_track.append(new_point)
    points_to_track = np.array(points_to_track, dtype=np.float32) # note: float32 required for opencv optic flow calculations
    points_to_track = points_to_track.reshape(points_to_track.shape[0], 1, points_to_track.shape[1]) # for some reason this needs to be shape (npoints, 1, 2)
    return points_to_track

def get_corners(image):
    corners = cv2.goodFeaturesToTrack(image, 25, 0.01, 10)
    #return corners.astype(int)
    return corners

def render(gray_image, points, flow):
    '''
    gray_image: opencv gray image, e.g. shape = (width, height)
    points: points at which optic flow is tracked, e.g. shape = (npoints, 1, 2)
    flow: optic flow field, should be same shape as points
    '''
    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    color_red = [0,0,255] # bgr colorspace
    linewidth = 1
    for i, point in enumerate(points):
        x = point[0,0]
        y = point[0,1]
        vx = flow[i][0,0]
        vy = flow[i][0,1]
        cv2.line(color_img, (x,y), (x+vx, y+vy), color_red, linewidth) # draw a red line from the point with vector = [vx, vy]        
    
    cv2.imshow('flow',color_img)
    cv2.waitKey(1)

class OF:
    def __init__(self):
        rospy.loginfo("image processing node is launched")
        """initialize the flow algorithm"""
        self.image_topic = "/ardrone/image_raw"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        self.feature_params = dict(maxCorners=50, qualityLevel=0.1, minDistance=5, blockSize=9)
        self.lk_params = dict(winSize=(7, 7), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.rand_color = np.random.randint(0, 255, (100, 3))

        self.prev_frame = None
        self.prev_features = None
        self.prev_good_pts = None
        self.mask = None

    def image_callback(self, data):
        """callback - entry interface to initialize the algorithm setting"""
        # rospy.loginfo("eating images")
        try:
            curr_frame = self.bridge.imgmsg_to_cv2(data, "mono8")
            if len(curr_frame.shape) > 2:
                if curr_frame.shape[2] > 1: # color image, convert it to gray
                    curr_frame = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY) # shape should now be (rows, columns)
                elif curr_frame.shape[2] == 1:  # mono image, with wrong formatting
                    curr_frame = curr_frame[:, :, 0] # shape should now be (rows, columns)

            curr_frame = cv2.GaussianBlur(curr_frame, (5, 5), 0.35)

            #cv2.imshow('blur', curr_frame)
            #cv2.waitKey(3)

            #return

            if self.prev_frame is None:
                #get the first frame
                #self.prev_features = get_roi_points(curr_frame, 50)
                self.prev_features = get_corners(curr_frame)
                self.mask = np.zeros_like(curr_frame)
            else:
                # mask image for drawing purpose
                # calculate the optical flow
                # print len(self.prev_features)
                curr_features, status, error = cv2.calcOpticalFlowPyrLK(self.prev_frame, curr_frame, self.prev_features, None, **self.lk_params)

                curr_good_pts = curr_features[status == 1]
                self.prev_good_pts = self.prev_features[status == 1]

                for i,(new,old) in enumerate(zip(np.copy(curr_good_pts), np.copy(self.prev_good_pts))):
                    a, b = new.ravel()
                    c, d = new.ravel()
                    self.mask = cv2.line(np.copy(self.mask), (a,b),(c,d), self.rand_color[i].tolist(), 2)
                    curr_frame = cv2.circle(curr_frame, (a, b), 2, self.rand_color[i].tolist(), 2)

                curr_frame = cv2.add(curr_frame, self.mask)

                # orginal image
                cv2.imshow(self.image_topic, curr_frame)
                cv2.waitKey(3)
                # capture some good points
                #self.prev_good_pts = self.prev_features[status == 1]
                #curr_good_pts = curr_features[status == 1]
                # calculate flow field
                #flow = curr_features - self.prev_features
                # draw the flow field
                # render(curr_frame, self.prev_features, flow)
                #for i in self.prev_features:
                #    x, y = i.ravel()
                #    cv2.circle(curr_frame, (x, y), 8, -1)

            self.prev_frame = curr_frame # get the first frame

        except CvBridgeError as err:
            print err
