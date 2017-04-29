#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

def getImageBrightness(frame):
    hsv_frame = np.array(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
    return np.mean(hsv_frame[:,:,2].flatten()) / 256

class OFC(object):
    """ a wrapper with using Farneback optical flow for dense implementation """
    def __init__(self):
        rospy.loginfo("image processing node is launched")
        # topics are declared here
        self.image_topic = "/ardrone/image_raw"
        self.imu_topic = "/ardrone/imu"
        self.cmd_vel_topic = "/cmd_vel"

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=2)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        self.previous_frame = None
        self.grid_size = 15
        self.enhance_factor = 1

        ## dynamic reconfigure
        self.ddyn = {
            'magnitude_threshold': 2.0,
            'portion_threshold': 0.5,
            'center_portion_flow': 0.3,
            'drift_degree': 0.7,
            'front_thrust': 0.5, # todo
            'back_thrust': 0.5, # todo
            'debug': True,
            'stop': True,
            'passive': True
        }

    def update_reconfigure(self, ddyn):
        self.ddyn = ddyn

    def split_window(self, grid_size, size=(640, 480), ratio=(2/7., 3/7., 2/7.)):
        # split the window
        win1 = int((size[0] / grid_size - 1) * ratio[0]) * grid_size
        win2 = int((size[0] / grid_size) * (1 - ratio[2])) * grid_size
        return (win1, win2)

    def stop_twist(self):
        return Twist()

    def empty_twist(self):
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        return t

    def backward_twist(self):
        t = self.empty_twist()
        t.linear.x = -self.ddyn['back_trust']
        return t

    def forward_twist(self):
        t = self.empty_twist()
        t.linear.x = self.ddyn['front_trust']
        return t

    def drift_twist(self, x, v):
        t = self.empty_twist()
        t.linear.x = x
        t.angular.z = v
        return t
    
    def passive_control(self, hint, isDebug):
        if (hint != 'Move Forward'):
            self.send_control(self.backward_twist())
            print('[PASSIVE] - BACKWARD', 'Debug', isDebug)
        else:
            self.send_control(self.empty_twist())
            print('[PASSIVE] - STATIONARY', 'Debug', isDebug)


    def draw_flow(self, img, flow, swin, step=15):
        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, lines, 0, (0, 255, 0))
        #variables for the mean
        count = [0,0,0]
        #toward left, center, right
        toward_count = [[0,0,0,0],[0,0,0,0],[0,0,0,0]]
        intensities = []
        for (x1, y1), (x2, y2) in lines:
            cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
            #checks for the mean
            intensity = np.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
            intensities.append(intensity)
            threshold_intensity = self.ddyn['magnitude_threshold']

            if x1 <= swin[0] + step:
                count[0] += 1
                # todo: thresholding intensity for valid flow
                if intensity <= threshold_intensity:
                    toward_count[0][3] += 1
                else:
                    if (x2-x1) < 0.:
                        toward_count[0][1] += 1
                    else:
                        toward_count[0][0] += 1

            elif x1 > swin[0] + step and x1 < swin[1] + step:
                count[1] += 1
                if intensity <= threshold_intensity:
                    toward_count[1][3] += 1
                else:
                    if x1 < w / 2:
                        if (x2-x1) < 0.:
                            toward_count[1][1] += 1
                        else:
                            toward_count[1][0] += 1
                    else:
                        if (x2-x1) > 0.:
                            toward_count[1][1] += 1
                        else:
                            toward_count[1][2] += 1
            else:
                count[2] += 1
                if intensity <= threshold_intensity:
                    toward_count[2][3] += 1
                else:
                    if (x2-x1) > 0.:
                        toward_count[2][1] += 1
                    else:
                        toward_count[2][2] += 1

        #calculating the mean
        #if count[0] > 0:
        #if count[1] > 0:
        #if count[2] > 0:

        #print(count)
        #print(np.mean(intensities))
        #print("------")
        return (vis, np.array(toward_count)/np.array(count).astype(float)[:,None])

    def send_control(self, twist):
        for i in range(1, 10):
            self.cmd_pub.publish(twist)
            time.sleep(0.001)

    def image_callback(self, data):
        """callback - entry interface to initialize the algorithm setting"""
        # rospy.loginfo("eating images")
        try:
            current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)

        brightness_level = getImageBrightness(current_frame)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        if self.previous_frame is None:
            print 'previous_frame is none'
            self.previous_frame = current_frame
        else:
            flow = current_frame
            flow = cv2.calcOpticalFlowFarneback(self.previous_frame, current_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)

            self.previous_frame = current_frame

            # plotting flow diagram
            origin = np.copy(current_frame)
            (rows, cols) = current_frame.shape

            Lx = []
            Ly = []
            Cx = []
            Cy = []
            Rx = []
            Ry = []#

            # 165 450
            (win1, win2) = self.split_window(self.grid_size, (cols, rows))

            for x in range(0, cols, self.grid_size):
                for y in range(0, rows, self.grid_size):
                    # with better visibility by tuning the enhance_factor
                    intensity_xy = flow[y][x] * self.enhance_factor

                    # split window
                    if (x <= win1):
                        # Left
                        Lx.append(intensity_xy[0])
                        Ly.append(intensity_xy[1])
                    elif (x > win1 and x <= win2):
                        Cx.append(intensity_xy[0])
                        Cy.append(intensity_xy[1])
                    else:
                        # Right
                        Rx.append(intensity_xy[0])
                        Ry.append(intensity_xy[1])

                    #cv2.line(origin, (x, y), (int(x + intensity_xy[0]), int(y + intensity_xy[1])), (255, 255, 0))
                    #cv2.circle(origin, (x, y), 1, (0, 0, 0), -1)

            # Get the feature vectors
            # Magnitude of flow vect#ors imitates
            # np.mean(Lx), np.mean(Ly), np.mean(Cx), np.mean(Cy), np.mean(Rx), np.mean(Ry)

            # Get another information like imu data

            # Control with Twist type
            #
            # geometry_msgs::Twist
            #
            # linear.x: move backward
            # linear.x: move forward
            # linear.y: move right
            # linear.y: move left
            # linear.z: move down
            # linear.z: move up
            # angular.z: turn right
            # angular.z: turn left

            (view, stat) = self.draw_flow(origin, flow, (win1, win2))
            left_win = stat[0]
            cen_win = stat[1]
            right_win = stat[2]
            threshold_drift = self.ddyn['portion_threshold']

            t  = self.empty_twist()

            print(left_win)
            print(cen_win)
            print(right_win)


            # todo center window threshold
            if cen_win[1] > self.ddyn['center_portion_flow']:
                hint = "[Caution] Potential Obstacle Detected"
                # t = self.drift_twist(self.ddyn['front_trust'], self.ddyn['drift_degree'])
                t = self.backward_twist()

            else:
                if left_win[1] > threshold_drift and right_win[1] > threshold_drift:
                    if cen_win[1] > threshold_drift:
                        hint = "Move Forward"
                        t = self.forward_twist()
                    else:
                        hint = "Move Backward"
                        t = self.backward_twist()
                else:
                    if cen_win[1] > threshold_drift:
                        if left_win[1] > threshold_drift or right_win[1] > threshold_drift:
                            if left_win[1] >= right_win[1]:
                                hint = "Drift Right >>>"
                                t = self.drift_twist(self.ddyn['front_trust'], -self.ddyn['drift_degree'])
                            else:
                                hint = "<<<< Drift Left"
                                t = self.drift_twist(self.ddyn['front_trust'], self.ddyn['drift_degree'])
                        else:
                            hint = "Move Forward"
                            t = self.forward_twist()
                    else:
                        hint = "Move Forward"
                        t = self.forward_twist()
        
            cv2.putText(view, hint, (400, 300), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,224), 2)
            cv2.putText(view, "Brightness level " + str(brightness_level), (400, 320), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,224), 2)
            cv2.putText(view, "Center level " + str(cen_win[1]), (400, 340), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,224), 2)

            # debug mode
            if (self.ddyn['debug']):
                if (self.ddyn['passive']):
                    self.passive_control(hint, 'True')
                else:
                    print('[DEBUG-ACTIVE] ' + hint)
            else:
                if (self.ddyn['passive']):
                    self.passive_control(hint, 'False')
                else:
                    if (self.ddyn['stop']):
                        print('[STOP]')
                        self.send_control(self.empty_twist())
                    else:
                        self.send_control(t)

            cv2.imshow(self.image_topic, view)
            cv2.waitKey(3)
