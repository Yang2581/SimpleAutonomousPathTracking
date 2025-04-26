import rospy
from std_msgs.msg import Bool, UInt8, Int16
import tkinter as tk

class AutoDriveGUI:
    def __init__(self, master):
        self.master = master
        master.title("Auto Drive Control")

        # 初始化 ROS 节点和发布器
        rospy.init_node('auto_drive_gui', anonymous=True)

        # 订阅节点
        self.auto_mode_sub = rospy.Subscriber("auto_mode", Bool, self.update_auto_mode, queue_size=10)
        self.aeb_mode_sub = rospy.Subscriber("aeb_mode", Bool, self.update_aeb_mode, queue_size=10)
        self.path_id_sub = rospy.Subscriber("path_id", UInt8, self.update_path_id, queue_size=10)
        self.lot_code_sub = rospy.Subscriber("lotcode_msg", Int16, self.update_lot_code, queue_size=10)
        self.avp_status_sub = rospy.Subscriber("status_msg", Int16, self.update_avp_status, queue_size=10)

        # 发布节点
        self.auto_mod_pub = rospy.Publisher('auto_mode', Bool, queue_size=10)
        self.aeb_pub = rospy.Publisher('aeb_mode', Bool, queue_size=10)
        self.path_pub = rospy.Publisher('path_id', UInt8, queue_size=10)
        self.lot_code_pub = rospy.Publisher('lotcode_msg', Int16, queue_size=10)
        self.avp_status_pub = rospy.Publisher('status_msg', Int16, queue_size=10)


        # 节点状态
        self.auto_mode = False
        self.aeb_mode = False
        self.path_id = None
        self.lot_code = None
        self.avp_status = None

        # 节点消息
        self.auto_mode_msg = Bool()
        self.aeb_mode_msg = Bool()
        self.path_id_msg = UInt8()
        self.lot_code_msg = Int16()
        self.avp_status_msg = Int16()

        # 自动驾驶开关
        self.auto_mod_var = tk.BooleanVar(value=False)
        self.auto_mod_button = tk.Checkbutton(
            master,
            text="Auto Mode",
            variable=self.auto_mod_var,
            command=self.toggle_auto_mod,
            bg='lightgreen',
            fg='black',
            font=('Arial', 12)
        )
        self.auto_mod_button.pack(pady=5)

        # AEB避障开关
        self.aeb_var = tk.BooleanVar(value=False)
        self.aeb_button = tk.Checkbutton(
            master,
            text="AEB Mode",
            variable=self.aeb_var,
            command=self.toggle_aeb,
            bg='lightcoral',
            fg='black',
            font=('Arial', 12)
        )
        self.aeb_button.pack(pady=5)
        #############################################################################
        # 路径输入文本框
        self.label_path_id = tk.Label(master, text="Path ID:", font=('Arial', 12))
        self.label_path_id.pack(pady=5)

        self.entry_path_id = tk.Entry(master, font=('Arial', 12))
        self.entry_path_id.pack(pady=5)

        # 路径发送按钮
        self.button_send_path_id = tk.Button(master, text="Send Path ID", command=self.send_path_id, bg='lightblue', font=('Arial', 12))
        self.button_send_path_id.pack(pady=10)

        # 泊车位输入文本框
        self.label_lot_code = tk.Label(master, text="lotCode:", font=('Arial', 12))
        self.label_lot_code.pack(pady=5)

        self.entry_lot_code = tk.Entry(master, font=('Arial', 12))
        self.entry_lot_code.pack(pady=5)

        # 泊车位发送按钮
        self.button_send_lot_code = tk.Button(master, text="Send lotCode", command=self.send_lot_code, bg='lightblue', font=('Arial', 12))
        self.button_send_lot_code.pack(pady=10)

        # 泊车状态输入文本框
        self.label_avp_status = tk.Label(master, text="AVP Status:", font=('Arial', 12))
        self.label_avp_status.pack(pady=5)

        self.entry_avp_status = tk.Entry(master, font=('Arial', 12))
        self.entry_avp_status.pack(pady=5)

        # 泊车状态发送按钮
        self.button_send_avp_status = tk.Button(master, text="Send AVP Status", command=self.send_avp_status, bg='lightblue', font=('Arial', 12))
        self.button_send_avp_status.pack(pady=10)
        #############################################################################

        # 初始发布状态
        self.auto_mod_pub.publish(self.auto_mod_var.get())
        self.aeb_pub.publish(self.aeb_var.get())

    # 状态更新函数
    def update_auto_mode(self, msg):
        self.auto_mode = msg.data
    
    def update_aeb_mode(self, msg):
        self.aeb_mode = msg.data
    
    def update_path_id(self, msg):
        self.path_id = msg.data
        self.label_path_id.configure(text="Path_ID: {}".format(self.path_id))
    
    def update_lot_code(self, msg):
        self.lot_code = msg.data
        self.label_lot_code.configure(text="LotCode: {}".format(self.lot_code))

    def update_avp_status(self, msg):
        self.avp_status = msg.data
        self.label_avp_status.configure(text="AVP Status: {}".format(self.avp_status))
        

    # 按键处理函数
    def toggle_auto_mod(self):
        self.auto_mod_pub.publish(self.auto_mod_var.get())  # 发布当前状态

    def toggle_aeb(self):
        self.aeb_pub.publish(self.aeb_var.get())  # 发布当前状态

    def send_path_id(self):
        try:
            path_id = int(self.entry_path_id.get())
            if path_id >= 255 or path_id < 0:
                pass
            else:
                self.path_id_msg.data = path_id
                self.path_pub.publish(self.path_id_msg)  # 发送路径 ID
        except ValueError:
            print("Invalid Path ID. Please enter a valid number.")
    
    def send_lot_code(self):
        try:
            lot_code = int(self.entry_lot_code.get())
            if lot_code >= 30000 or lot_code < 0:
                pass
            else:
                self.lot_code_msg.data = lot_code
                self.lot_code_pub.publish(self.lot_code_msg)  # lotcode
        except ValueError:
            print("Invalid lot code. Please enter a valid number.")

    def send_avp_status(self):
        try:
            avp_status = int(self.entry_avp_status.get())
            if avp_status >= 4 or avp_status < 0:
                pass
            else:
                self.avp_status_msg.data = avp_status
                self.avp_status_pub.publish(self.avp_status_msg)  # lotcode
        except ValueError:
            print("Invalid status id. Please enter a valid number.")

if __name__ == "__main__":
    root = tk.Tk()
    gui = AutoDriveGUI(root)
    root.mainloop()
