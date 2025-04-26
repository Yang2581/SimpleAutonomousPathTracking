#!/usr/bin/env python3
#coding:utf-8
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
import os

class Scan_distance():
    def __init__(self):
        rospy.init_node('leishen_scan_node',anonymous = True)
        self.sub1 = rospy.Subscriber('/leishen_scan',  LaserScan, self.callback_3)
        self.sub2 = rospy.Subscriber('/gear_fb',  UInt8, self.update_gear_status)
        self.sub3 = rospy.Subscriber('/speed_fb', Float32, self.update_speed)
        self.pub = rospy.Publisher('add_acc_val', Float32,queue_size=1)
        self.circle_lst = []
        self.Rate = rospy.Rate(2)

        # status
        self.gear_status = "N"
        self.steer_angle = 0
        self.speed = 0 # m/s
    
        # front detection region params
        
        self.__region_dict = {
        "D_f":[170, 190],
        "D_m":[165, 195],
        "D_s":[150, 210],
        }

        # rear detection area params
        self.exp_rear_dist = 0.8
        self.exp_side_dist = 0
        self.exp_front_dist = 0.3
        self.vehicle_front_dist = 3 # unit:m
        self.vehicle_rear_dist = 1.6
        self.vehicle_width = 1.9
        self.d1 = self.vehicle_rear_dist + self.exp_rear_dist
        self.d2 = (self.vehicle_width/2) + self.exp_side_dist
        self.d3 = self.vehicle_front_dist + self.exp_front_dist
        self.theta1 = np.arctan(self.d2/self.d1)
        self.theta2 = np.arctan(self.d2/self.d3)

        # front detection area params
        self.exp_rear_dist_f = 0.3
        self.exp_side_dist_f = 0.2
        self.exp_front_dist_f = 3
        self.d1_f = self.vehicle_rear_dist + self.exp_rear_dist_f
        self.d2_f = (self.vehicle_width/2) + self.exp_side_dist_f
        self.d3_f = self.vehicle_front_dist + self.exp_front_dist_f
        self.theta1_f = np.arctan(self.d2_f/self.d1_f)
        self.theta2_f = np.arctan(self.d2_f/self.d3_f)

        # ACC parameters
        self.acc_min_stop_dist = 1.5
        self.acc_safe_dist = self.acc_min_stop_dist
        self.acc_ttc = 2
        self.acc_k1 = 0.05
        self.acc_k2 = -0.3

    def update_gear_status(self, gear_msg):
        if gear_msg.data == 0:
            self.gear_status = "P"
        elif gear_msg.data == 1:
            self.gear_status = "R"
        elif gear_msg.data == 2:
            self.gear_status = "N"
        else:
            self.gear_status = "D"

    def update_steer_angle(self, steer_msg):
        self.steer_angle = steer_msg.data

    def update_speed(self, speed_msg):
        self.speed = speed_msg.data
    
    # def callback_2(self, scan_msg):
    #     # test function
    #     dist_lst = []
    #     for i in range(len(scan_msg.ranges)):
    #         dist_lst.append(scan_msg.ranges[i])
    #     min_dist = min(dist_lst)
    #     idx = np.argmin(dist_lst)
    #     angle = (idx * 360) / len(scan_msg.ranges)
    #     rospy.loginfo("min_dist: {}, idx: {}, angle: {}".format(min_dist, idx, angle))

    def calc_acc_deccel(self, min_dist):
        # calc ttc
        print("===== {} =====".format(min_dist))
        if min_dist > self.exp_front_dist_f:
            deccel = 0
        elif min_dist <= self.acc_min_stop_dist:
            deccel = -1.2
        else:
            self.acc_ttc = min_dist/(self.speed + 1)
            # calc safe distance
            self.acc_safe_dist = self.speed * self.acc_ttc + self.acc_min_stop_dist
            # calc decceleration
            deccel = 1.5*(self.acc_k1 * (min_dist - self.acc_safe_dist) + (self.acc_k2 * self.speed))
        return deccel


    def front_break(self, start_deg, end_deg, stop_distances, deccel_lst, scan_msg):
        dist_lst = []
        start_deg = round(len(scan_msg.ranges)*(start_deg/360))
        end_deg = round(len(scan_msg.ranges)*(end_deg/360))
        for i in np.arange(start_deg, end_deg):
            if scan_msg.ranges[i] > (self.vehicle_front_dist*0.9):
                dist_lst.append(scan_msg.ranges[i])

        min_dist = min(dist_lst)
        print("min_dist: {}".format(min_dist))
        if (min_dist <= (stop_distances[0] + self.vehicle_front_dist)) and (min_dist > (stop_distances[1] + self.vehicle_front_dist)):
            add_val = deccel_lst[0]
        elif (min_dist <= (stop_distances[1] + self.vehicle_front_dist)) and (min_dist >(stop_distances[2] + self.vehicle_front_dist)):
            add_val = deccel_lst[1]
        elif min_dist <= (stop_distances[2] + self.vehicle_front_dist):
            add_val = deccel_lst[2]
        else:
            add_val = 0
        return add_val

    def calc_stop_deccel(self, scan_msg, gear_status):
        # check is reverse
        if gear_status != 'R':
            theta1 = self.theta1_f
            theta2 = self.theta2_f
            d1 = self.d1_f
            d2 = self.d2_f
            d3 = self.d3_f
        else:
            theta1 = self.theta1
            theta2 = self.theta2
            d1 = self.d1
            d2 = self.d2
            d3 = self.d3

        min_idx = 0
        min_dist = 100
        angle = 360
        for i in range(len(scan_msg.ranges)):
            crnt_theta = i * (2*np.pi) / len(scan_msg.ranges)
            if crnt_theta < theta1:
                l = scan_msg.ranges[i] * abs(np.cos(crnt_theta))
                if l < d1 and l > 1.6:
                    add_val = self.calc_acc_deccel(l - self.vehicle_rear_dist) + 0.1
                    min_idx = i
                    break
                else:
                    add_val = 0
            elif (crnt_theta > theta1) and (crnt_theta < (np.pi - theta2)):
                l = scan_msg.ranges[i] * abs(np.sin(crnt_theta))
                if l < d2 and l > 1.4:
                    add_val = -0.8
                    min_idx = i
                    break
                else:
                    add_val = 0
            elif (crnt_theta > (np.pi - theta2)) and (crnt_theta < np.pi):
                l = scan_msg.ranges[i] * abs(np.cos(crnt_theta))
                if l < d3 and l > 1.4:
                    add_val = self.calc_acc_deccel(l - self.vehicle_front_dist)-0.1
                    min_idx = i
                    break
                else:
                    add_val = 0
            elif (crnt_theta > np.pi) and (crnt_theta < (np.pi + theta2)):
                l = scan_msg.ranges[i] * abs(np.cos(crnt_theta))
                if l < d3 and l > 1.4:
                    add_val = self.calc_acc_deccel(l - self.vehicle_front_dist)-0.1
                    min_idx = i
                    break
                else:
                    add_val = 0
            elif (crnt_theta > (np.pi + theta2)) and (crnt_theta < (2*np.pi - theta1)):
                l = scan_msg.ranges[i] * abs(np.sin(crnt_theta))
                if l < d2 and l > 1.4:
                    add_val = -0.8
                    min_idx = i
                    break
                else:
                    add_val = 0
            else:
                l = scan_msg.ranges[i] * abs(np.cos(crnt_theta))
                if l < d1 and l > 1.6:
                    add_val = self.calc_acc_deccel(l - self.vehicle_rear_dist) + 0.1
                    min_idx = i
                    break
                else:
                    add_val = 0
            min_idx = i
            angle = (min_idx * 360) / len(scan_msg.ranges)
            min_dist = scan_msg.ranges[min_idx]
        print("speed: {}, min_dist: {}, angle: {}".format(self.speed, min_dist, angle))
        return add_val

    def callback_3(self, scan_msg):
        deccel_lst = []
        if self.gear_status == "D":
            # deccel_lst.append(self.front_break(self.__region_dict["D_f"][0], self.__region_dict["D_f"][1], [6, 5, 4], [-0.2, -0.3, -0.4], scan_msg))
            # deccel_lst.append(self.front_break(self.__region_dict["D_m"][0], self.__region_dict["D_m"][1], [4, 3, 1.5], [-0.4, -0.8, -1], scan_msg))
            # deccel_lst.append(self.front_break(self.__region_dict["D_s"][0], self.__region_dict["D_s"][1], [1.5, 1, 1], [-1.2, -1.4, -1.5], scan_msg))
            deccel_lst.append(self.calc_stop_deccel(scan_msg, self.gear_status))
            # deccel_lst.append(self.front_break(self.__region_dict["D_s"][0], self.__region_dict["D_s"][1], [1.5, 1, 1], [-1.2, -1.4, -1.5], scan_msg))
        elif self.gear_status == "R":
            deccel_lst.append(self.calc_stop_deccel(scan_msg, self.gear_status))
        else:
            deccel_lst.append(0)
        max_deccel = min(deccel_lst)
        if max_deccel <= -1.5:
            max_deccel = -1.5
        data = Float32(data=max_deccel)
        self.pub.publish(data)
        print("gear_status: {}, deccel: {}".format(self.gear_status, max_deccel))
            
    def start(self):
        rospy.spin()

if __name__ == '__main__':
    scaner = Scan_distance()
    scaner.start()