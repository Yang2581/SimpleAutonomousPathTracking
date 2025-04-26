#ifndef MSG_NODE_H
#define MSG_NODE_H

#include <usbcan/can_msg.h>
#include <limits>
#include <usbcan/UsbCan.h>
#include <thread>
#include <chrono>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept> 


struct IMU_DATA {
    public:
        // acceleration
        float accel_raw_x = 0;
        float accel_raw_y = 0;
        float accel_raw_z = 0;

        // angle rate
        float ang_rate_raw_x = 0;
        float ang_rate_raw_y = 0;
        float ang_rate_raw_z = 0;

        // heading pitch roll
        float heading = 0;
        float pitch = 0;
        float roll = 0;

        void update_data(UsbCan &usbCan);
};

#endif //MSG_NODE_H