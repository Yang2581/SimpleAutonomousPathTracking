#include <can_msgs/Frame.h>
#include <usbcan/can_msg.h>
#include <ros/ros.h>
#include <usbcan_msgs/Wheels.h>
#include <usbcan_msgs/WheelsEncoder.h>
#include <usbcan_msgs/gps.h>
#include <usbcan_msgs/steering_angle.h>
#include <limits>
#include <usbcan/UsbCan.h>
#include <thread>
#include <chrono>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <time.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry"
#include <tf/tf.h>
#include "usbcan/msg_node.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_listener.h"

using namespace std::chrono_literals;
// std::ofstream  oFile;
float acc = 0,acc_v = 0,speed = 0,rs_angle = 0,on_off = 0,angle_out = 0,angle_out_p = 0,e = 0,pose_x = 0, pose_y = 0, pose_z = 0,pose_yaw = 0;
int a = 0,a1 = 0,b = 0 ,gear = 0;	
uint sys_st = 0 ;//3
bool Auto_mode = false;

class VelocityToPulse {
private:
    double _pulse_counter{0};
    std::chrono::steady_clock::time_point _prev_time;
    float _wheel_radius;
    float _wheel_pulses;
    float _pulses_per_meter;
public:
    VelocityToPulse(float wheel_radius, unsigned int wheel_pulses) : _wheel_radius(wheel_radius),
                                                                     _wheel_pulses(static_cast<float>(wheel_pulses)),
                                                                     _prev_time(std::chrono::steady_clock::now()) {
        _pulses_per_meter = _wheel_pulses / (2 * boost::math::constants::pi<float>() * _wheel_radius);
    }

    uint64_t operator()(float velocity) {
        auto t = std::chrono::steady_clock::now();
        std::chrono::duration<float> d = t - _prev_time;
        _prev_time = t;
        _pulse_counter += _pulses_per_meter * velocity * d.count();
        if (_pulse_counter > 1e14) {
            _pulse_counter = 0;
        }
        return static_cast<uint64_t>(std::round(_pulse_counter));
    }
    void reset() {
        _pulse_counter = 0;
        _prev_time = std::chrono::steady_clock::now();
    }
};

double t=0,t1=0,t2=0;
static void process_loop(UsbCan &usbCan, ros::Publisher &pub1, ros::Publisher &pub2, ros::Publisher &pub3,ros::Publisher &pub4,ros::Publisher &pub5,ros::Publisher &pub6) {
    
    ROS_INFO("%d",ros::ok());
    int seq = 0;
    VelocityToPulse rl_v2p(0.72f, 4096);
    VelocityToPulse rr_v2p(0.72f, 4096);
    usbcan_msgs::gps gpsmsg;
    ros::Rate rate(100);

    while (ros::ok()) {
        /* CAN0通道数据解析 */
        std::optional<VCI_CAN_OBJ> can_obj_opt;
        while (!(can_obj_opt = usbCan.receive(1)) && ros::ok()) {//0:CAN1  1:CAN2
            std::this_thread::yield();
        }
        if (!ros::ok())
            break;
        auto can_obj = *can_obj_opt;

        if (can_obj.ID == 0x76d){
            can_msg::vehicle_frame_a1 vehicle_frame_a1(can_obj.Data);
            float car_x=vehicle_frame_a1.Target_Vehicle_PoseX();
            float car_y=vehicle_frame_a1.Target_Vehicle_PoseY();
            float car_W=vehicle_frame_a1.Target_Vehicle_Width();
            float car_H=vehicle_frame_a1.Target_Vehicle_Height();
        }

        //获取方向盘转角
        if (can_obj.ID == 0x96){
            can_msg::HONGQI_steerAngle steer(can_obj.Data);
            float HQsteer = steer.steerangle();
            }
            if (can_obj.ID == 0x10A){
            can_msg::HONGQI_CDS_1 cds(can_obj.Data);
            sys_st = cds.CDS_SystemControlSt();
            ROS_INFO("sys_st:%d",sys_st);
        }

        static usbcan_msgs::WheelsEncoder WS;
	    WS.header.stamp = ros::Time::now();

        if (can_obj.ID == 0xb4){
            can_msg::HQ_control5 Speed(can_obj.Data);
            float FL = Speed.WheelSpeed_FL_ABS_1();
            float FR = Speed.WheelSpeed_FR_ABS_1();
            ROS_INFO("FL:%f",FL);
            ROS_INFO("FR:%f",FR);
        
            WS.left_encoder =0 ;
            WS.right_encoder=0 ;
            WS.left_speed = FL/3.6;
            WS.right_speed= FR/3.6;
            float speed_fb = Speed.VehicleSpeed_ABS_1();
            std_msgs::Float32 _speed;
            _speed.data = speed_fb;
            pub2.publish(_speed);
        } 
        
        static std_msgs::UInt8 _gear;

        if (can_obj.ID == 0x248){
            can_msg::HCU_LeverInfo Gear(can_obj.Data);
            uint8_t gear_fb = Gear.leverinfo();
            _gear.data = gear_fb;
            pub1.publish(_gear);
        } 

        WS.shift=_gear.data;
	
        pub5.publish(WS); 

        std_msgs::Float64MultiArray  vehicle_pose_msg;
        vehicle_pose_msg.data.push_back(pose_x);
        vehicle_pose_msg.data.push_back(pose_y); 
        vehicle_pose_msg.data.push_back(pose_z); 
        vehicle_pose_msg.data.push_back(pose_yaw);
        pub6.publish(vehicle_pose_msg);
        rate.sleep();
    }
    
}

int CRC8(std::vector<unsigned char>::iterator &data, unsigned char size)
{
    auto crc = 0x00;
    for(int i = size-1; i >= 0; i--)
    {
        // printf("DATA:%X\n",data[i]);
        crc ^= data[i];
        for(int j = 0; j < 8; j++)
        {
            if((crc & 0x80) != 0)
            {
                crc = crc << 1;
                if(crc >= 256)
                crc = (crc - 256) ^ 0x1D;
            }
            else
            {
                crc = crc << 1;
                if(crc >= 256)
                crc = crc - 256;
            }
                
        }
    }
    return crc; 
} 
           

static void fake_can_msg_wheels(UsbCan &usbCan) {
    ros::Rate rate(100);//10ms发送一次
    int start_time =  ros::Time::now().toSec();
    while (ros::ok()) {
        //检测例子0x108
        can_msg::HQ_control HQ_steer;
        can_msg::HQ_control3 HQ_SVB2_1;

        HQ_steer.SteeringAngleRequest_SVB1(0);
        HQ_steer.SteeringAngleSignRequest_SVB1(0);
        HQ_steer.SteeringSpeedSignRequest_SVB1(0);
        HQ_steer.SteeringSpeedRequest_SVB1(0);
        HQ_steer.SteeringRequestSt_SVB1(0);
        
        HQ_SVB2_1.SteeringAngleSignRequest_SVB2_1(0);

        if(sys_st == 1&&Auto_mode )//0
        {
        ROS_INFO("CDS_ready");  
            if(acc_v>=0)
            {
                HQ_steer.AccelerationRequestSt_SVB1(1);
                HQ_steer.AccelerationRequest_SVB1(acc_v);
                HQ_steer.DecelerationRequest_SVB1(0);
            }  
            else
            {
                HQ_steer.DecelerationRequestSt_SVB1(1);
                HQ_steer.AccelerationRequest_SVB1(0);
                HQ_steer.DecelerationRequest_SVB1(acc_v);        //-acc-
            }

            HQ_steer.AccelerationRequestSt_SVB1(1);
            HQ_steer.SteeringAngleRequest_SVB1(abs(angle_out));//0

            if(angle_out>=0)
                HQ_steer.SteeringAngleSignRequest_SVB1(1);
            else
                HQ_steer.SteeringAngleSignRequest_SVB1(2);

            HQ_steer.SteeringSpeedSignRequest_SVB1(0);
            HQ_steer.SteeringSpeedRequest_SVB1(350);
            HQ_steer.GearRequest_SVB1(gear);//0: No request, 1:P, 2:R, 3:N,4:D
            HQ_steer.SteeringRequestSt_SVB1(1);
            HQ_SVB2_1.SteeringAngleSignRequest_SVB2_1(1);
        }
        HQ_steer.livecounter_SVB1(a);
        HQ_SVB2_1.livecounter_SVB2_1(a);

    /*------------------------CAN_SEND------------------------*/
        VCI_CAN_OBJ obj{},obj1{};
        obj.ID = 0x108;
        obj.DataLen = HQ_steer.data().size();
        obj.SendType = 0;
        obj.RemoteFlag = 0;
        obj.ExternFlag = 0;
        std::copy(HQ_steer.data().begin(), HQ_steer.data().end(), obj.Data);
        uint check_sum = 0;
        check_sum = obj.Data[1]^obj.Data[2]^obj.Data[3]^obj.Data[4]^obj.Data[5]^obj.Data[6]^obj.Data[7];
        obj.Data[0] = check_sum;

        obj1.ID = 0x7e;
        obj1.DataLen = HQ_SVB2_1.data().size();
        obj1.SendType = 0;
        obj1.RemoteFlag = 0;
        obj1.ExternFlag = 0;
        std::copy(HQ_SVB2_1.data().begin(), HQ_SVB2_1.data().end(), obj1.Data);
        check_sum = 0;
        check_sum = obj1.Data[1]^obj1.Data[2]^obj1.Data[3]^obj1.Data[4]^obj1.Data[5]^obj1.Data[6]^obj1.Data[7];
        obj1.Data[0] = check_sum;
        usbCan.send(obj, 1);      
        usbCan.send(obj1, 1);

        if(b>10){
            b = 0;
            can_msg::HQ_control2 HQ_light;
            can_msg::HQ_control4 HQ_light1;
            HQ_light.livecounter_SVB2(a1);
            obj.ID = 0x1a0;
            obj.DataLen = HQ_light.data().size();
            obj.SendType = 0;
            obj.RemoteFlag = 0;
            obj.ExternFlag = 0;
            std::copy(HQ_light.data().begin(), HQ_light.data().end(), obj.Data);
            check_sum = obj.Data[1]^obj.Data[2]^obj.Data[3]^obj.Data[4]^obj.Data[5]^obj.Data[6]^obj.Data[7];
            obj.Data[0] = check_sum;
            usbCan.send(obj, 1);
            obj.ID = 0x7f;
            usbCan.send(obj, 1);
            a1=a1+1;
            if (a1>15)
            a1=0;
        }
        b = b + 1;
//---------------------------------------------------

        a=a+1;
        //泊车位置上传
        can_msg::Vehicle_Pos HQ_pose;
        HQ_pose.Pos_X(pose_x);
        HQ_pose.Pos_Y(pose_y);
        HQ_pose.Pos_Z(pose_z);
        HQ_pose.theta(pose_yaw);
        
        obj.ID = 0x2c6;
        obj.DataLen = HQ_pose.data().size();
        obj.SendType = 0;
        obj.RemoteFlag = 0;
        obj.ExternFlag = 0;
        std::copy(HQ_pose.data().begin(), HQ_pose.data().end(), obj.Data);
        usbCan.send(obj, 1);
        if (a>15)
        a=0;
        rate.sleep();
    }

}

void domsg_acc(const std_msgs::Float32::ConstPtr& acc_P)
{
    angle_out = acc_P -> data;
}

void domsg_acc_v(const std_msgs::Float32::ConstPtr& acc_V)
{
    ros::Rate r_v(100);
    acc_v = acc_V -> data;
    r_v.sleep();
}

void domsg_gear(const std_msgs::Int8::ConstPtr& Gear)
{
    gear = Gear -> data;
}

void domsg_on(const std_msgs::Float32::ConstPtr& On_off)
{
    on_off = On_off -> data;
}

void domsg_auto(const std_msgs::Bool::ConstPtr& auto_mode)
{
    Auto_mode = auto_mode -> data;
}

void domsg_pose(const nav_msgs::Odometry msg)
{
    pose_x = msg.pose.pose.position.x;
    pose_y = msg.pose.pose.position.y;
    pose_z = msg.pose.pose.position.z;
    tf::Quaternion q; 
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw); // rpy得是double
    pose_yaw = yaw;
}


void receive_msg_and_update(UsbCan &usbCan, IMU_DATA &imu_data)
{
    ROS_INFO("%d", ros::ok());

    ros::Rate rate(100);

    while(ros::ok()){
        std::optional<VCI_CAN_OBJ> can_obj_opt;
        while(!(can_obj_opt = usbCan.receive(0)) && ros::ok()){ // 读取can1通道数据 can1:0 can2:1
            std::this_thread::yield();
        }
        if(!ros::ok()) break;

        auto can_obj = *can_obj_opt;
        if(can_obj.ID == 0x322){
            can_msg::Accel_IMU_Raw Accel_IMU_Raw(can_obj.Data);
            imu_data.accel_raw_x = Accel_IMU_Raw.AccelRawX();
            imu_data.accel_raw_y = Accel_IMU_Raw.AccelRawY();
            imu_data.accel_raw_z = Accel_IMU_Raw.AccelRawZ();
        }

        if(can_obj.ID == 0x321){
            can_msg::Ang_Rate_Raw_IMU Ang_Rate_Raw_IMU(can_obj.Data);
            imu_data.ang_rate_raw_x = Ang_Rate_Raw_IMU.AccelRateRawX();
            imu_data.ang_rate_raw_y = Ang_Rate_Raw_IMU.AccelRateRawY();
            imu_data.ang_rate_raw_z = Ang_Rate_Raw_IMU.AccelRateRawZ();
        }

        if(can_obj.ID == 0x32A){
            can_msg::HeadingPitchRoll HeadingPitchRoll(can_obj.Data);
            imu_data.heading = HeadingPitchRoll.AngleHeading();
            imu_data.pitch = HeadingPitchRoll.AnglePitch();
            imu_data.roll = HeadingPitchRoll.AngleRoll();
        }
        rate.sleep();
    }
}

void send_imu_data(IMU_DATA &imu_data, ros::Publisher &imu_pub)
{
    ROS_INFO("%d", ros::ok());

    ros::Rate rate(100);
    geometry_msgs::Quaternion orientation;
    sensor_msgs::Imu imu_raw;

    while(ros::ok()){
        auto roll = (imu_data.roll/180)*3.14;
        auto pitch = (imu_data.pitch/180)*3.14;
        auto heading = (imu_data.heading/180)*3.14;
        orientation=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading);

        auto accel_x = imu_data.accel_raw_x*9.8;
        auto accel_y = imu_data.accel_raw_y*9.8;
        auto accel_z = imu_data.accel_raw_z*9.8;
        
        auto ang_rate_x = (imu_data.ang_rate_raw_x/180)*3.14;
        auto ang_rate_y = (imu_data.ang_rate_raw_y/180)*3.14;
        auto ang_rate_z = (imu_data.ang_rate_raw_z/180)*3.14;

        imu_raw.header.frame_id = "imu_link";
        imu_raw.header.stamp = ros::Time::now();

        imu_raw.orientation.x = 0;
        imu_raw.orientation.y = 0;
        imu_raw.orientation.z = 0;
        imu_raw.orientation.w = 0;
        // 
        imu_raw.linear_acceleration.x = accel_x;
        imu_raw.linear_acceleration.y = accel_y;
        imu_raw.linear_acceleration.z = accel_z;
        imu_raw.angular_velocity.x = ang_rate_x;
        imu_raw.angular_velocity.y = ang_rate_y;
        imu_raw.angular_velocity.z = ang_rate_z;

        imu_pub.publish(imu_raw);
        rate.sleep();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "usbcan");
    UsbCan usbCan;
    ros::NodeHandle nodeHandle,nh,nh1;
    //ros::Publisher publisher = nodeHandle.advertise<usbcan_msgs::Wheels>("wheels", 10);
    ros::Publisher publisher = nodeHandle.advertise<usbcan_msgs::WheelsEncoder>("wheels", 10);
    ros::Publisher publisher_1 = nh.advertise<std_msgs::UInt8>("gear_fb", 1);
    ros::Publisher publisher_2 = nh.advertise<std_msgs::Float32>("speed_fb", 1);
    ros::Publisher publisher_3 = nh.advertise<std_msgs::Float32MultiArray>("pedstrain", 1);
    ros::Publisher publisher_4 = nh.advertise<std_msgs::Float32>("/angle", 1);
    ros::Publisher publisher_6 = nh.advertise<std_msgs::Float64MultiArray>("/vehecle_pose", 100);

    ros::Subscriber subscirber_1 = nodeHandle.subscribe("acc",1,domsg_acc);
    ros::Subscriber subscirber_2= nh.subscribe("acc_v",1,domsg_acc_v);
    ros::Subscriber subscirber_4= nh.subscribe("gear_pub",1,domsg_gear);
    ros::Subscriber subscirber_3= nh1.subscribe("on_off",1,domsg_on);
    ros::Subscriber subscirber_5= nh1.subscribe("auto_mode",1,domsg_auto);
    ros::Subscriber subscirber_6= nh1.subscribe("odom",1,domsg_pose);

    ROS_INFO("------------start------------");
    std::thread loop_thread(process_loop, std::ref(usbCan), std::ref(publisher_1), std::ref(publisher_2),std::ref(publisher_3),std::ref(publisher_4), std::ref(publisher),std::ref(publisher_6));
    std::thread debug(fake_can_msg_wheels, std::ref(usbCan));

    /* 处理 IMU 数据*/
    IMU_DATA imu_data;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_gci610", 10);
    std::thread t1(receive_msg_and_update, std::ref(usbCan), std::ref(imu_data));
    std::thread t2(send_imu_data, std::ref(imu_data), std::ref(imu_pub));
    /* 处理 IMU 数据*/
    ros::spin();
    t2.join();

    return 0;
}
