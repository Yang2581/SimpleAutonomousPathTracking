#pragma once
#include <usbcan/can.h>
namespace can_msg {
//红旗车加速度1
    class HONGQI_Acceleration1  : public can::CanData {
    public:
       HONGQI_Acceleration1() : can::CanData(8) {}
        explicit HONGQI_Acceleration1(unsigned char *data) : can::CanData(data, 8) {}
    float acceleration() const {
            return get<uint16_t>(20, 12, LITTLE_ENDIAN) * static_cast<float>(0.01) + static_cast<float>(-20);
    }   
    };  
    /* None */
    /* id: 0x10a */
    class HONGQI_CDS_1  : public can::CanData {
    public:
        HONGQI_CDS_1() : can::CanData(8) {}
        explicit HONGQI_CDS_1(unsigned char *data) : can::CanData(data, 8) {}
    uint8_t CDS_SystemFailuerSt() const {
            return get<uint8_t>(24, 3, LITTLE_ENDIAN) * static_cast<float>(1) + static_cast<float>(0);
    }   
    uint8_t CDS_SystemControlSt() const {
            return get<uint8_t>(21, 2, LITTLE_ENDIAN) * static_cast<uint8_t>(1) + static_cast<uint8_t>(0);
    }   
    };       

    /* None */
    /* id: 0x1f5 */
    class RealState  : public can::CanData {
    public:
        RealState() : can::CanData(8) {}
        explicit RealState(unsigned char *data) : can::CanData(data, 8) {}
    float RealAngle() const {
            return (get<uint16_t>(40, 16, BIG_ENDIAN) + static_cast<int>(-7800)) * static_cast<float>(0.1) ;
    }   
    }; 

    /* None */
    /* id: 0x76d */
    class vehicle_frame_a1  : public can::CanData {
    public:
        vehicle_frame_a1() : can::CanData(8) {}
        explicit vehicle_frame_a1(unsigned char *data) : can::CanData(data, 8) {}
    float FCW() const {
            return (get<uint16_t>(0, 1, LITTLE_ENDIAN));
    }   
    float on_route() const {
            return (get<uint16_t>(1, 1, LITTLE_ENDIAN));
    } 
    float Vehicle_ID() const {
            return (get<uint16_t>(2, 6, LITTLE_ENDIAN));
    }  
    float Target_Vehicle_PoseX() const {
            return (get<uint16_t>(8, 12, LITTLE_ENDIAN)*static_cast<float>(0.0625));
    } 
    float Target_Vehicle_Width() const {
            return (get<uint16_t>(24, 8, LITTLE_ENDIAN)*static_cast<float>(0.05));
    }  
    float Target_Vehicle_PoseY() const {
            return (get<uint16_t>(32, 10, LITTLE_ENDIAN)*static_cast<float>(0.0625));
    } 
    float Target_Vehicle_Height() const {
            return (get<uint16_t>(42, 6, LITTLE_ENDIAN)*static_cast<float>(0.1));
    }
    float Target_Vehicle_Type() const {
            return (get<uint16_t>(48, 3, LITTLE_ENDIAN));
    }
    };

    /* None *
    /* id: 0x76e */
    class vehicle_frame_a2  : public can::CanData {
    public:
        vehicle_frame_a2() : can::CanData(8) {}
        explicit vehicle_frame_a2(unsigned char *data) : can::CanData(data, 8) {}
    float CAN_VIS_OBS_TTC_WITH_ACC() const {
            return (get<uint16_t>(0, 10, LITTLE_ENDIAN) *static_cast<float>(0.01))  ;
    } 
    float Target_Vehicle_Confidengce() const {
            return (get<uint16_t>(10, 6, LITTLE_ENDIAN) *static_cast<float>(0.02))  ;
    }  
    float Target_Vehicle_VelX() const {
            return (get<uint16_t>(16, 12, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    } 
    float Target_Vehicle_AccelX() const {
            return (get<uint16_t>(32, 10, LITTLE_ENDIAN) *static_cast<float>(0.03))  ;
    } 
    float Target_Vehicle_VelY() const {
            return (get<uint16_t>(48, 12, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    }
    };  

    /* None *
    /* id: 0x77a */
    class Pedestrian_frame_A  : public can::CanData {
    public:
        Pedestrian_frame_A() : can::CanData(8) {}
        explicit Pedestrian_frame_A(unsigned char *data) : can::CanData(data, 8) {}
    int Target_Pedstrian_ID() const {
            return (get<uint16_t>(0, 6, LITTLE_ENDIAN) *static_cast<float>(1))  ;
    } 
    float Target_Pedstrian_PosX() const {
            return (get<uint16_t>(8, 12, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    }  
    float Target_Pedstrian_PosY() const {
        if(get<uint16_t>(20, 10, LITTLE_ENDIAN)>512)
        {
            return ((get<uint16_t>(20, 10, LITTLE_ENDIAN)-1024) *static_cast<float>(0.0625))  ;

        }
            return (get<uint16_t>(20, 10, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    }  
    }; 
    /* None *
    /* id: 0x77b */
    class Pedestrian_frame_B  : public can::CanData {
    public:
        Pedestrian_frame_B() : can::CanData(8) {}
        explicit Pedestrian_frame_B(unsigned char *data) : can::CanData(data, 8) {}
    float Target_Pedstrian_PosX_a() const {
            return (get<uint16_t>(8, 12, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    }  
    float Target_Pedstrian_PosY_a() const {
        if(get<uint16_t>(20, 10, LITTLE_ENDIAN)>512)
        {
            return ((get<uint16_t>(20, 10, LITTLE_ENDIAN)-1024) *static_cast<float>(0.0625))  ;

        }
            return (get<uint16_t>(20, 10, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    }  
    float Target_Pedstrian_PosX_b() const {
            return (get<uint16_t>(39, 12, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    }  
    float Target_Pedstrian_PosY_b() const {
            return (get<uint16_t>(51, 10, LITTLE_ENDIAN) *static_cast<float>(0.0625))  ;
    }  
    }; 
    /* None */
    /* id: 0x108 */
    //所有用到的报文名称：第一行是输入的名称、数值类型、值；第二行是数值类型、起始位、位宽、intel= LITTLE_ENDIA、输出为车的值
    class HQ_control : public can::CanData {
    public:
        HQ_control() : can::CanData(8) {}
        explicit HQ_control(unsigned char *data) : can::CanData(data, 8) {}

    void checksum_SVB1(uint8_t value) {
            set<uint8_t>(0, 8, LITTLE_ENDIAN, value);
    }
    void livecounter_SVB1(float value) {
            set<uint8_t>(60, 4, LITTLE_ENDIAN, value);
    }
    void AccelerationRequest_SVB1(float value) {
            set<uint32_t>(48, 6, LITTLE_ENDIAN, value/0.1);
    }
     void AccelerationRequestSt_SVB1(float value) {
            set<uint32_t>(56, 1, LITTLE_ENDIAN, value);
    }
    void SteeringAngleRequest_SVB1(float value) {
            set<uint32_t>(24, 10, LITTLE_ENDIAN, value);
    }
    void SteeringSpeedRequest_SVB1(uint16_t value) {
            set<uint16_t>(39, 9, LITTLE_ENDIAN, value);
            }
    void SteeringRequestSt_SVB1(int value) {
            set<uint8_t>(22, 2, LITTLE_ENDIAN, value);  
            }
    void DecelerationRequest_SVB1(float value) {
            set<uint32_t>(8, 10, LITTLE_ENDIAN, (value-(-10))/0.01);  
            }
    void DecelerationRequestSt_SVB1(float value) {
            set<uint32_t>(18, 2, LITTLE_ENDIAN, value);  
            }
    void  SteeringAngleSignRequest_SVB1(int value) {
            set<uint8_t>(34, 2, LITTLE_ENDIAN, value);  
            }
    void  SteeringSpeedSignRequest_SVB1(float value) {
            set<uint8_t>(36, 2, LITTLE_ENDIAN, value);  
            }
    void  StandStillRequest_SVB1(float value) {
            set<uint32_t>(38, 1, LITTLE_ENDIAN, value);  
            }
    void  TakeOverRequest_SVB1(int value) {
            set<uint8_t>(55, 1, LITTLE_ENDIAN, value);  
            }
    void  GearRequest_SVB1(int value) {
            set<uint8_t>(57,3, LITTLE_ENDIAN, value);  
            }    
    void  FailureStatus_SVB1(float value) {
            set<uint8_t>(20,2, LITTLE_ENDIAN, value);  
            }    

};

/* id: 0x1A0 */

 class HQ_control2 : public can::CanData {
    public:
        HQ_control2() : can::CanData(8) {}
        explicit HQ_control2(unsigned char *data) : can::CanData(data, 8) {}

    void checksum_SVB2(uint8_t value) {
            set<uint8_t>(0, 8, LITTLE_ENDIAN, value);
    }
    void livecounter_SVB2(uint8_t value) {
            set<uint8_t>(60, 4, LITTLE_ENDIAN, value);
    }
    void LeftTurningLightRequest_SVB2(uint8_t value) {
            set<uint8_t>(10, 2, LITTLE_ENDIAN, value);
    }
    void RightTurningLightRequest_SVB2(uint8_t value) {
            set<uint8_t>(12, 2, LITTLE_ENDIAN, value);
    }
    void HazardLightRequest_SVB2(uint8_t value) {
            set<uint8_t>(14, 2, LITTLE_ENDIAN, value);
    }
    void FogRequest_SVB2(uint8_t value) {
            set<uint8_t>(16, 3, LITTLE_ENDIAN, value);
    }
    void HornRequest_SVB2(uint8_t value) {
            set<uint8_t>(19, 1, LITTLE_ENDIAN, value);
    }
    void LowbeamRequest_SVB2(uint8_t value) {
            set<uint8_t>(20, 2, LITTLE_ENDIAN, value);
    }
    void HighBeamRequest_SVB2(uint8_t value) {
            set<uint8_t>(22, 2, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_LeftTempAdjSW_SVB2(float value) {
            set<uint32_t>(26, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechRequest_SetLeftBLW_SVB2(float value) {
            set<uint32_t>(29, 3, LITTLE_ENDIAN, value);
    }
    void PositionLightRequest_SVB2(float value) {
            set<uint32_t>(32, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechRequrst_AC_SVB2(float value) {
            set<uint32_t>(35, 2, LITTLE_ENDIAN, value);
    }
    void HU_SpeechRequest_FRSREC_SVB2(float value) {
            set<uint32_t>(37, 2, LITTLE_ENDIAN, value);
    }
    void HU_SpeechRequest_BlwlevelAdjSW_SVB2(float value) {
            set<uint32_t>(40, 2, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_FLseatHeatLevel_SVB2(float value) {
            set<uint32_t>(42, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_FRseatHeatLevel_SVB2(float value) {
            set<uint32_t>(45, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_SetLeftTemp_SVB2(float value) {
            set<uint32_t>(48, 6, LITTLE_ENDIAN, (value - 15) / 0.5);
    }
    void EPBRrquest_SVB2(float value) {
            set<uint32_t>(54, 2, LITTLE_ENDIAN, value);
    }
    void RequestTurnOffLowVoltage_SVB2(float value) {
            set<uint32_t>(56, 2, LITTLE_ENDIAN, value);
    }
};


/* id: 0x7E */

 class HQ_control3 : public can::CanData {
    public:
        HQ_control3() : can::CanData(8) {}
        explicit HQ_control3(unsigned char *data) : can::CanData(data, 8) {}

    void checksum_SVB2_1(uint8_t value) {
            set<uint8_t>(0, 8, LITTLE_ENDIAN, value);
    }
    void livecounter_SVB2_1(uint8_t value) {
            set<uint8_t>(60, 4, LITTLE_ENDIAN, value);
    }
    void DecelerationRequest_SVB2_1(float value) {
            set<uint32_t>(8, 10, LITTLE_ENDIAN, (value-(-10))/0.01);
    }
    void DecelerationRequestSt_SVB2_1(float value) {
            set<uint32_t>(18, 2, LITTLE_ENDIAN, value);
    }
       void FailureStatus_SVB2_1(float value) {
            set<uint8_t>(20, 2, LITTLE_ENDIAN, value);  
       }  
       void SteeringRequestSt_SVB2_1(int value) {
            set<uint8_t>(22, 2, LITTLE_ENDIAN, value);  
       }  
       void SteeringAngleRequest_SVB2_1(float value) {
            set<uint32_t>(24, 10, LITTLE_ENDIAN, value);  
       }   
      void SteeringAngleSignRequest_SVB2_1(int value) {
            set<uint8_t>(34, 2, LITTLE_ENDIAN, value);  
       }   
       void ControlFlag_SVB2_1(uint8_t value) {
            set<uint32_t>(36, 2, LITTLE_ENDIAN, value);  
       }   
      void StandStillRequest_SVB2_1(uint8_t value) {
            set<uint32_t>(38, 1, LITTLE_ENDIAN, value);  
       } 
       void SteeringSpeedRequest_SVB2_1(uint8_t value) {
            set<uint32_t>(39, 9, LITTLE_ENDIAN, value);  
       }  
        void AccelerationRequest_SVB2_1(float value) {
            set<uint32_t>(48, 6, LITTLE_ENDIAN, value/0.1);  
       }
        void TakeOverRequest_SVB2_1(int value) {
            set<uint8_t>(55, 1, LITTLE_ENDIAN, value);  
       }    
        void AccelerationRequestSt_SVB2_1(float value) {
            set<uint32_t>(56, 1, LITTLE_ENDIAN, value);  
       }  
       void GearRequest_SVB2_1(int value) {
            set<uint8_t>(56, 1, LITTLE_ENDIAN, value);  
       }  

};

/* id: 0x7f */

 class HQ_control4 : public can::CanData {
    public:
        HQ_control4() : can::CanData(8) {}
        explicit HQ_control4(unsigned char *data) : can::CanData(data, 8) {}

    void checksum_SVB2_2(uint8_t value) {
            set<uint8_t>(0, 8, LITTLE_ENDIAN, value);
    }
    void livecounter_SVB2_2(uint8_t value) {
            set<uint8_t>(60, 4, LITTLE_ENDIAN, value);
    }
    void IC_TakeOverTip_SVB2_2(uint8_t value) {
            set<uint32_t>(8, 2, LITTLE_ENDIAN, value);
    }
    void LeftTurningLightRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(10, 2, LITTLE_ENDIAN, value);
    }
    void RightTurningLightRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(12, 2, LITTLE_ENDIAN, value);
    }
    void HazardLightRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(14, 2, LITTLE_ENDIAN, value);
    }
    void FogRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(16, 3, LITTLE_ENDIAN, value);
    }
    void HornRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(19, 1, LITTLE_ENDIAN, value);
    }
    void LowBeamRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(20, 2, LITTLE_ENDIAN, value);
    }
    void HighBeamRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(22, 2, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_LeftTempAdjSW_SVB2_2(uint8_t value) {
            set<uint32_t>(26, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_SetLeftBLW_SVB2_2(uint8_t value) {
            set<uint32_t>(29, 3, LITTLE_ENDIAN, value);
    }
    void PositionLightRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(32, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechRequest_AC_SVB2_2(uint8_t value) {
            set<uint32_t>(35, 2, LITTLE_ENDIAN, value);
    }
    void HU_SpeechRequest_FRSREC_SVB2_2(uint8_t value) {
            set<uint32_t>(37, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_BlwlevelAdjSW_SVB2_2(uint8_t value) {
            set<uint32_t>(40, 2, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_FLseatHeatLI_SVB2_2(uint8_t value) {
            set<uint32_t>(42, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_FRseatHeatLI_SVB2_2(uint8_t value) {
            set<uint32_t>(45, 3, LITTLE_ENDIAN, value);
    }
    void HU_SpeechReq_SetLeftTemp_SVB2_2(float value) {
            set<uint32_t>(48, 6, LITTLE_ENDIAN, (value- 15) / 0.5);
    }
    void EPBRequest_SVB2_2(uint8_t value) {
            set<uint32_t>(54, 2, LITTLE_ENDIAN, value);
    }
    void ReqTurnOffLowVoltage_SVB2_2(float value) {
            set<uint32_t>(56, 32, LITTLE_ENDIAN, value);
    }

 };

 /* id: 0xB4*/
 class HQ_control5 : public can::CanData {
    public:
        HQ_control5() : can::CanData(8) {}
        explicit HQ_control5(unsigned char *data) : can::CanData(data, 8) {}
    uint8_t checksum_ABS_1() const {
            return (get<uint8_t>(0, 8, LITTLE_ENDIAN) *static_cast<uint8_t>(1))  ;
    }
    uint8_t livecounter_ABS_1() const {
        return (get<uint8_t>(56, 4, LITTLE_ENDIAN) *static_cast<uint8_t>(1))  ;
    }
    float WheelSpeed_FL_ABS_1() const {
        return (get<uint16_t>(8, 15, LITTLE_ENDIAN) *static_cast<float>(0.01))  ;
    }
    float WheelSpeed_FR_ABS_1() const {
        return (get<uint16_t>(24, 15, LITTLE_ENDIAN) *static_cast<float>(0.01))  ;
    }
    float VehicleSpeed_ABS_1() const {
        return (get<uint16_t>(40, 15, LITTLE_ENDIAN) *static_cast<float>(0.01))  ;
    }
    uint8_t DrivingDirection_ABS_1() const {
        return (get<uint8_t>(60, 2, LITTLE_ENDIAN) *static_cast<uint8_t>(1))  ;
    }
};
 /* id: 0x248*/
 class HCU_LeverInfo : public can::CanData {
    public:
        HCU_LeverInfo() : can::CanData(8) {}
        explicit HCU_LeverInfo(unsigned char *data) : can::CanData(data, 8) {}
    uint8_t leverinfo() const {
            return (get<uint8_t>(40, 4, LITTLE_ENDIAN) *static_cast<uint8_t>(1))  ;
    }
};
 /* id: 0x2c6*/
 class Vehicle_Pos : public can::CanData {
    public:
        Vehicle_Pos() : can::CanData(8) {}
        explicit Vehicle_Pos(unsigned char *data) : can::CanData(data, 8) {}
    void Pos_X(float value) {
        set<uint16_t>(0, 16, LITTLE_ENDIAN, (value + 100)/0.01);
    }
    void Pos_Y(float value) {
        set<uint16_t>(16, 16, LITTLE_ENDIAN, (value + 100)/0.01);
    }
    void Pos_Z(float value) {
        set<uint16_t>(32, 16, LITTLE_ENDIAN, (value + 10)/0.01);
    }
    void theta(float value) {
        set<uint16_t>(48, 16, LITTLE_ENDIAN, (value + 10)/0.01);
    }
};


/* ID: 0x322 */
class Accel_IMU_Raw: public can::CanData{
    public:
        Accel_IMU_Raw():can::CanData(8){}
        explicit Accel_IMU_Raw(unsigned char *data):can::CanData(data, 8){}

        float AccelRawX(){
            return get<int32_t>(20, 20, BIG_ENDIAN) * static_cast<float>(0.0001) + static_cast<float>(0);
        }
        float AccelRawY(){
            return get<int32_t>(32, 20, BIG_ENDIAN) * static_cast<float>(0.0001) + static_cast<float>(0);
        }
        float AccelRawZ(){
            return get<int32_t>(60, 20, BIG_ENDIAN) * static_cast<float>(0.0001) + static_cast<float>(0);
        }
};

/* ID: 0x321 */
class Ang_Rate_Raw_IMU: public can::CanData{
    public:
        Ang_Rate_Raw_IMU():can::CanData(8){}
        explicit Ang_Rate_Raw_IMU(unsigned char *data):can::CanData(data, 8){}

        float AccelRateRawX(){
            return get<int32_t>(20, 20, BIG_ENDIAN) * static_cast<float>(0.01) + static_cast<float>(0);
        }
        float AccelRateRawY(){
            return get<int32_t>(32, 20, BIG_ENDIAN) * static_cast<float>(0.01) + static_cast<float>(0);
        }
        float AccelRateRawZ(){
            return get<int32_t>(60, 20, BIG_ENDIAN) * static_cast<float>(0.01) + static_cast<float>(0);
        }
};

/* ID: 0x32A */
class HeadingPitchRoll: public can::CanData{
    public:
        HeadingPitchRoll():can::CanData(8){}
        explicit HeadingPitchRoll(unsigned char *data):can::CanData(data, 8){}

        float AngleHeading(){
            return get<u_int16_t>(8, 16, BIG_ENDIAN) * static_cast<float>(0.01) + static_cast<float>(0);
        }
        float AnglePitch(){
            return get<int16_t>(24, 16, BIG_ENDIAN) * static_cast<float>(0.01) + static_cast<float>(0);
        }
        float AngleRoll(){
            return get<int16_t>(40, 16, BIG_ENDIAN) * static_cast<float>(0.01) + static_cast<float>(0);
        }
};

}