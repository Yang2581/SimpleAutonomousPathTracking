import rospy
from std_msgs.msg import Float32, Int16, UInt8, Bool, Int8
from geometry_msgs.msg import Point, PoseStamped
import math

class Assemble_Node:
    def __init__(self):
        # 接收节点
        self.sub_ndt_pose = rospy.Subscriber("ndt_pose", PoseStamped, self.update_ndt_pose, queue_size=10)
        self.sub_speed = rospy.Subscriber("speed_fb", Float32, self.update_speed, queue_size=10)
        self.sub_accel = rospy.Subscriber("acc_cmd", Float32, self.update_accel, queue_size=10) # acceleration command from ppnode
        self.sub_steer = rospy.Subscriber("acc_pp", Float32, self.update_steer_cmd, queue_size=10) # steering command from ppnode
        self.sub_gear_cmd = rospy.Subscriber("gear_pub_pp", Int8, self.update_gear_cmd, queue_size=10) # steering command from ppnode
        self.sub_deccel = rospy.Subscriber("add_acc_val", Float32, self.update_deccel, queue_size=10)
        self.sub_gear_status = rospy.Subscriber("gear_fb", UInt8, self.update_gear_fb, queue_size=10)
        self.sub_automode_status = rospy.Subscriber("auto_mode", Bool, self.update_auto_mode, queue_size=10)
        self.sub_avp_status = rospy.Subscriber("status_msg", Int16, self.update_avp_status, queue_size=10)
        self.sub_lot_code = rospy.Subscriber("lotcode_msg", Int16, self.update_lot_code, queue_size=10)

        # 发送节点
        self.pub_veh_pose = rospy.Publisher("veh_pose", Point, queue_size=10)
        self.pub_acc_v = rospy.Publisher("acc_v", Float32, queue_size=10) # pub accel request to chassis
        self.pub_steer_v = rospy.Publisher("acc", Float32, queue_size=10) # pub steering request to chassis
        self.pub_path_id = rospy.Publisher("path_id", UInt8, queue_size=10)
        self.pub_gear = rospy.Publisher("gear_pub", Int8, queue_size=10) # pub gear request to chassis

        # 初始化变量
        self.veh_pose = Point()
        self.acc_v = Float32()
        self.gear_cmd_msg = Int8()
        self.steer_cmd_msg = Float32()
        self.rate = rospy.Rate(50)
        self.counter = 0
        self.path_id_published = False  # 标志位
        self.path_id = None
        self.pose = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.veh_accel = 0
        self.veh_deccel = 0
        self.gear_status = 2
        self.auto_mode = 0
        self.avp_status = None  # 初始化为 None
        self.lot_code = None  # 初始化为 None
        self.gear_rqst = 0 # gear request to chassis
        self.steer_rqst = 0 # gear request to chassis

    def update_ndt_pose(self, msg):
        self.pose["yaw"] = self.calc_yaw(msg.pose.orientation)
        self.pose["x"] = msg.pose.position.x - 0.8 * math.cos(self.pose["yaw"])
        self.pose["y"] = msg.pose.position.y - 0.8 * math.sin(self.pose["yaw"])

    def update_accel(self, msg):
        self.veh_accel = msg.data

    def update_deccel(self, msg):
        self.veh_deccel = msg.data

    def update_gear_fb(self, msg):
        self.gear_status = msg.data

    def update_auto_mode(self, msg):
        self.auto_mode = msg.data

    def update_lot_code(self, msg):
        self.lot_code = msg.data
        self.update_path_id()  # 更新路径 ID
        print("update_lot_code: {}".format(self.lot_code))

    def update_avp_status(self, msg):
        self.avp_status = msg.data
        self.update_path_id()  # 更新路径 ID
        print("update_avp_status: {}".format(self.avp_status))

    def update_steer_cmd(self, msg):
        self.steer_rqst = msg.data

    def update_gear_cmd(self, msg):
        self.gear_rqst = msg.data

    def update_path_id(self):
        # path_id_mapping = {
        #     1: {825: 20, 826: 21, 827: 22, 828: 23, 829: 24, 806: 25, 807: 26, 808: 27, 809: 28, 810:29},
        #     2: {825: 10, 826: 11, 827: 12, 828: 13, 829: 14, 806: 15, 807: 16, 808: 17, 809: 18, 810:19}
        # }
        # 20241031
        # path_id_mapping = {
        #     1: {825: 20, 828: 23, 807: 26, 810:29},
        #     2: {825: 10, 828: 13, 807: 16, 810:19}
        # }
        #20241117cddz
        path_id_mapping = {
            1: {16: 26,17: 27, 18: 28, 19: 29},
            2: {16: 16,17: 17, 18: 18, 19: 19}
        }
        # 检查当前 avp_status 和 lot_code 是否有效
        if self.avp_status is not None and self.lot_code is not None:
            if self.lot_code in path_id_mapping.get(self.avp_status, {}) and not self.path_id_published:
                self.path_id = path_id_mapping[self.avp_status][self.lot_code]
                path_id_msg = UInt8(data=self.path_id)
                self.pub_path_id.publish(path_id_msg)
                print("*********pathid: {}".format(self.path_id))
                self.path_id_published = True  # 设置标志位为已发布
            else:
                self.path_id_published = False  # 无效情况重置标志位

    def update_speed(self, msg):
        self.veh_speed = msg.data
        self.process()
        self.rate.sleep()
        self.counter += 1
        if self.counter % 5 == 0:
            self.display_info()

    def process(self):
        self.veh_pose.x = self.pose["x"]
        self.veh_pose.y = self.pose["y"]
        self.veh_pose.z = self.pose["yaw"]

        acc_v = 0
        steer_v = 0
        gear_v = 0
        if self.avp_status != 3:
            if self.avp_status != 0:
                acc_v = self.veh_accel + self.veh_deccel
                steer_v = self.steer_rqst
                gear_v = self.gear_rqst
            else:
                acc_v = -0.5
                gear_v = 1
        else:
            acc_v = -1
            steer_v = 0
            gear_v = 1
        
        self.acc_v.data = acc_v
        self.steer_cmd_msg.data = steer_v
        self.gear_cmd_msg.data = gear_v

        self.pub_veh_pose.publish(self.veh_pose)
        self.pub_acc_v.publish(self.acc_v)
        self.pub_steer_v.publish(self.steer_cmd_msg)
        self.pub_gear.publish(self.gear_cmd_msg)

    def display_info(self):
        gear_status = 'N'
        if self.gear_status == 0:
            gear_status = 'P'
        elif self.gear_status == 1:
            gear_status = 'R'
        elif self.gear_status == 2:
            gear_status = 'N'
        else:
            gear_status = 'D'

        print("----------" * 3)
        print("veh_pose: [x: {}, y: {}, yaw: {}]".format(self.pose["x"], self.pose["y"], self.pose["yaw"]))
        print("veh acc_cmd: {}, deccel_cmd: {}, pub acc_v: {}, steer_cmd: {}".format(self.veh_accel, self.veh_deccel, self.acc_v, self.steer_rqst))
        print("gear: {}, automode: {}, path_id: {}".format(gear_status, self.auto_mode, self.path_id))
        print("----------" * 3)

    def calc_yaw(self, msg):
        return math.atan2(2 * (msg.w * msg.z + msg.y * msg.x), (1 - 2 * (msg.y ** 2 + msg.z ** 2)))

if __name__ == "__main__":
    rospy.init_node("assemble_node")
    assemble_node = Assemble_Node()
    rospy.spin()

