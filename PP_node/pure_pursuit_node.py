from Pure_Pursuit_Utils import *
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
import json
from path_process import process_path

class PP_node:
    def __init__(self):
        rospy.init_node("pure_pursuit_node")
        self.pub_1 = None
        self.pub_2 = None
        self.pub_3 = rospy.Publisher("gear_pub_pp", Int8, queue_size=10)
        self.pub_4 = None
        self.sub_pose = None
        self.sub_path_id = None
        self.sub_gear_status = None
        self.sub_veh_vel = None
        self.sub_avp_status = rospy.Subscriber("status_msg", Int16, self.update_avp_status, queue_size=10)
        self.rate = rospy.Rate(20)
        self.acc_cmd_msg = Float32()
        self.steer_cmd_msg = Float32()
        self.gear_cmd_msg = Int8()
        self.path_stage_msg = UInt8()

        # status
        self.pose = {"x":0, "y":0, "yaw":0}
        self.path_id = 1
        self.gear_status = 2
        self.gear_ischanged = False
        self.node = None
        self.veh_vel = 0
        self.direct = 1
        self.speed_coef = 1 # fine tune the target speed
        rospy.loginfo("--- initialized! ---")

        # reference_path
        self.ref_path_x = None
        self.ref_path_y = None
        self.ref_direct = None
        self.path_flag = 0
        self.stage_flag = 0

        # counter
        self.counter = 0

    def update_avp_status(self, msg):
        self.avp_status = msg.data
        if self.avp_status != 2:
            self.speed_coef = 1
        else:
            print("--------- avp status = 2 ---------")
            if self.pose["x"] < 80 and self.pose["y"] > -15:
                self.speed_coef = 0.4
            else:
                pass

    def update_pose(self, msg):
        self.pose["x"] = msg.x
        self.pose["y"] = msg.y
        self.pose["yaw"] = msg.z
        gear_status = 'N'
        if self.gear_status == 0:
            gear_status = 'P'
        elif self.gear_status == 1:
            gear_status = 'R'
        elif self.gear_status == 2:
            gear_status = 'N'
        else:
            gear_status = 'D'
        # rospy.loginfo("gear status: {}, x:{}, y:{}, vel:{}".format(gear_status, self.pose["x"], self.pose["y"], self.veh_vel))
        self.process()
        self.rate.sleep()
        self.counter += 1

    def update_path_id(self, msg):
        self.path_id = msg.data
        if self.path_flag != self.path_id:
            try:

                if self.gear_status != 0:
                    rospy.loginfo("--- could not change path when running! ---")
                    return

                file_path = "path/path_" + str(self.path_id) + ".json"
                with open(file_path,'r') as f:
                    data = json.load(f)
                    self.ref_path_x, self.ref_path_y, self.ref_direct = process_path(data["x"], data["y"], data["direct"])

                    # self.node = Node(self.pose["x"], self.pose["y"],
                    # self.pose["yaw"], self.veh_vel, self.direct)

                    self.stage_flag = 0
                    self.path_flag = self.path_id
                    rospy.loginfo("--- path changed : path: {}! ---".format(self.path_id))
            except:
                rospy.loginfo("--- No such path : path: {}! ---".format(self.path_id))

    def update_gear_status(self, msg):
        self.gear_status = msg.data

    def update_veh_vel(self, msg):
        self.veh_vel = msg.data

    def process(self):
        if self.stage_flag < len(self.ref_direct):
            node = Node(x=self.pose["x"], y=self.pose["y"], yaw=self.pose["yaw"], v=self.veh_vel, direct=self.ref_direct[self.stage_flag][0])
            ref_trajectory = PATH(self.ref_path_x[self.stage_flag], self.ref_path_y[self.stage_flag])
            target_ind, _ = ref_trajectory.target_index(node)
            gear_request = 0

            if self.ref_direct[self.stage_flag][0] > 0:
                target_speed = 5*self.speed_coef #kph
                C.Ld = 3
                C.dist_stop = 2
                C.dc = -1.1
                gear_request = 4
                
                steer_coef = 1 - abs(self.steer_cmd_msg.data)/1000
                if steer_coef <= 0.5:
                    steer_coef = 0.5
                elif steer_coef >= 1:
                    steer_coef = 1
                
                target_speed = steer_coef * target_speed
            else:
                target_speed = 1.2 #kph
                C.Ld = 2
                C.dist_stop = 0.20
                C.dc = -0.4
                gear_request = 2

            xt = node.x + C.dc * math.cos(node.yaw)
            yt = node.y + C.dc * math.sin(node.yaw)
            dist = math.hypot(xt - self.ref_path_x[self.stage_flag][-1], yt - self.ref_path_y[self.stage_flag][-1])

            acceleration = pid_control(target_speed, node.v, dist, self.ref_direct[self.stage_flag][0])
            delta, target_ind = pure_pursuit(node, ref_trajectory, target_ind)

            if dist < C.dist_stop:
                if gear_request == 4:
                    gear_request = 2
                else:
                    gear_request = 4

                # check gear is changed
                if (self.gear_status == 1) and (gear_request == 2):
                    self.gear_ischanged = True
                elif (self.gear_status == 3) and (gear_request == 4):
                    self.gear_ischanged = True
                else:
                    self.gear_ischanged = False

                # arbitrate the gear switch
                if self.gear_ischanged:
                    self.stage_flag += 1
                else:
                    acceleration = -0.5
        else:
            acceleration = -0.5
            delta = 0
            gear_request = 1
        
        steering_angle = (delta * C.GearRatio)*(180/3.14)
        if steering_angle > 500:
            steering_angle = 500
        elif steering_angle < -500:
            steering_angle = -500
        else:
            pass

        if self.counter >= 10:
            if self.stage_flag < len(self.ref_direct):
                rospy.loginfo("accel: {}, steer: {}, dist: {}, stage_flag: {}, vel: {}, gear_cmd: {}".format(acceleration, steering_angle, dist, self.stage_flag, self.veh_vel, gear_request))
                self.counter = 0
            else:
                rospy.loginfo("----- destination reached! stop -----")
                self.counter = 0

        self.steer_cmd_msg.data = steering_angle
        self.acc_cmd_msg.data = acceleration
        self.gear_cmd_msg.data = gear_request
        self.path_stage_msg.data = self.stage_flag

        self.pub_1.publish(self.steer_cmd_msg)
        self.pub_2.publish(self.acc_cmd_msg)
        self.pub_3.publish(self.gear_cmd_msg)
        self.pub_4.publish(self.path_stage_msg)
        # rospy.loginfo("----- pub success -----")
        

if __name__ == '__main__':
    rospy.init_node("pure_pursuit_node")
    pp = PP_node()
    pp.pub_1 = rospy.Publisher("acc_pp", Float32, queue_size=10) # steering angle command
    pp.pub_2 = rospy.Publisher("acc_cmd", Float32, queue_size=10)
    pp.pub_4 = rospy.Publisher("path_stage", UInt8, queue_size=10) # stage flag
    pp.sub_pose = rospy.Subscriber("veh_pose", Point, pp.update_pose, queue_size=10)
    pp.sub_path_id = rospy.Subscriber("path_id", UInt8, pp.update_path_id, queue_size=10)
    pp.sub_gear_status = rospy.Subscriber("gear_fb", UInt8, pp.update_gear_status, queue_size=10)
    pp.sub_veh_vel = rospy.Subscriber("speed_fb", Float32, pp.update_veh_vel, queue_size=10)

    rospy.spin()
