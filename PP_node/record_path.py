import rospy
import json
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
import signal
import time
import multiprocessing as mp
import sys

# def signal_handler(signal, frame, filename, data):
#     print('Caught Ctrl+C / SIGINT signal')
#     filename = filename + ".json"
#     with open(filename, 'a+') as f:
#         json.dump(data, f)
#     sys.exit(0)


class Recoder:
    def __init__(self):
        self.path_dict = {"x":[], "y":[], "direct":[]}
        self.sub_1 = None
        self.sub_2 = None
        self.gear_status = 3
        self.rate = rospy.Rate(5)
        

    def update_gear_status(self, msg):
        # gear status feedback 
        # 0x0 P
        # 0x1 R
        # 0x2 N
        # 0x3 D
        self.gear_status = msg.data

    def record_path(self, msg):
        self.path_dict["x"].append(msg.x)
        self.path_dict["y"].append(msg.y)
        if self.gear_status != 1:
            self.path_dict["direct"].append(1)
            rospy.loginfo("x:{}, y:{}, dirc:{}".format(msg.x, msg.y, 1))
        else:
            self.path_dict["direct"].append(-1)
            rospy.loginfo("x:{}, y:{}, dirc:{}".format(msg.x, msg.y, -1))
        self.rate.sleep()

if __name__ == "__main__":
    try:
        # signal.signal(signal.SIGINT, signal_handler)
        rospy.init_node("record_path")
        path_id = str(sys.argv[1])
        recoder = Recoder()
        sub_1 = rospy.Subscriber("gear_fb", UInt8, recoder.update_gear_status, queue_size=10)
        sub_2 = rospy.Subscriber("veh_pose", Point, recoder.record_path, queue_size=10)
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        filename = "path/path_" + path_id + ".json"
        json_data = json.dumps(recoder.path_dict, indent=4, sort_keys=False)
        with open(filename, 'w+') as f:
            f.write(json_data)
            
        print("file saved!")
    
