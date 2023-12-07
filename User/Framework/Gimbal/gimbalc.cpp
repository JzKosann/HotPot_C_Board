//
// Created by ShiF on 2023/9/14.
//
#include "gimbalc.hpp"
#include "math.h"
#include "INS_task.h"

/** Motor List **/
cMotor yaw, pitch;          //云台功能
cMotor fricR, fricL;
cMotor rammc;
/** algorithm **/
mPID matPID;

#define TIMpiece    0.002   //控制周期，时间切片
/** Car **/             //choose your car here
cCar Car(cCar::ONMI);
/**
 * ONMI 全向
 * MECANUM 麦步
 */

/**
 * 滤波初始化
 */
void Filter_Init()
{
    yaw.autoAim_filter.cLowPass.Init(10, 0.005);
    pitch.autoAim_filter.cLowPass.Init(10, 0.005);
    yaw.RCcrtl_filter.ckalman.Init(40, 200);
    pitch.RCcrtl_filter.ckalman.Init(40, 200);

}

void Algorithm_Init()
{
    yaw.MotorCtrl.c_PID.Init();
//    matPID.Init();
//    yaw.MotorCtrl.c_PID.setParam(20, 5, 0, 30192, 0, 0, 0, 0);
    pitch.MotorCtrl.c_PID.Init();

//    fricL.MotorCtrl.c_ADRC.Init(1500, 0.05, 0.5, 5, 1, 1000, 10,
//                                0.5, 1.25, 80, 1);
//    fricR.MotorCtrl.c_ADRC.Init(1500, 0.05, 0.5, 5, 1, 1000, 10,
//                                0.5, 1.25, 80, 1);
//    fricL.MotorCtrl.c_PID.Init();
//    fricR.MotorCtrl.c_PID.Init();
//
//    fricL.MotorCtrl.c_PID.setParam(2, 0.2, 0, 13684,
//                                   0, 0, 0, 0);
//    fricR.MotorCtrl.c_PID.setParam(2, 0.2, 0, 13684,
//                                   0, 0, 0, 0);
//    rammc.MotorCtrl.c_PID.Init();
//    rammc.MotorCtrl.c_PID.setParam(20, 3, 0, 10000,
//                                   0, 0, 0, 0);
}

void MotorInit()
{
//    yaw.canInit(0x209, &hcan2, cMotor::GM_6020);
    pitch.canInit(0x206, &hcan1, cMotor::GM_6020);


//
//    rammc.canInit(0x203, &hcan1, cMotor::M_2006_p36);
//    if(Car.CarType==cCar::UAV){
//        fricL.canInit(0x201, &hcan1, cMotor::M_2006);
//        fricR.canInit(0x202, &hcan1, cMotor::M_2006);
//    }
//    else{
//        fricL.canInit(0x201, &hcan1, cMotor::M_3508);
//        fricR.canInit(0x202, &hcan1, cMotor::M_3508);
//    }
}

void ChassisInit()
{
    ChassisYaw.send_flag = 1;
    ChassisVel.send_flag = 1;
    if (Car.CarType == cCar::UAV)
    {
        ChassisYaw.send_flag = 0;
        ChassisVel.send_flag = 0;
    }
}

bool CarInit()//
{
    Car.ShootMode = cCar::eRC;
    return false;
}

void Gimbal_Init()
{
    MotorInit();
    Filter_Init();
    Algorithm_Init();
    ChassisInit();
    CarInit();
}

/**********************************RC_Ctrl**********************************************/


bool portSetProtect()
{
    if (RC_GetDatas().rc.s[0] == 1) return true;
    else return false;
}


void portSetMove(float vx, float vy)
{
    ChassisVel.CANx_send_data[0] = (int16_t) vx >> 8;
    ChassisVel.CANx_send_data[1] = (int16_t) vx & 0xff;
    ChassisVel.CANx_send_data[2] = (int16_t) vy >> 8;
    ChassisVel.CANx_send_data[3] = (int16_t) vy & 0xff;
}

void setSpin(float spinVel)
{
    ChassisVel.CANx_send_data[4] = (int16_t) spinVel >> 8;
    ChassisVel.CANx_send_data[5] = (int16_t) spinVel & 0xff;
}

void sendChassisYaw(float yaw_offset)
{
    ChassisYaw.CANx_send_data[0] = (int16_t) yaw_offset >> 8;
    ChassisYaw.CANx_send_data[1] = (int16_t) yaw_offset & 0xff;
}

bool portSetFollowUp()      //随动
{
    static uint8_t clear_flag = 1;
    if (RC_GetDatas().rc.s[0] == 3)
    {
        if (clear_flag)
        {
            yaw.clearEcdRcnt();
        }
        clear_flag = 0;
        return true;
    }
    else
    {
        clear_flag = 1;
        return false;
    }
}

void portSetChassicStop()
{
    int16_t vx, vy, spinVel;
    vx = 0;
    vy = 0;
    spinVel = 0;
    ChassisVel.CANx_send_data[0] = vx >> 8;
    ChassisVel.CANx_send_data[1] = vx & 0xff;
    ChassisVel.CANx_send_data[2] = vy >> 8;
    ChassisVel.CANx_send_data[3] = vy & 0xff;
    ChassisVel.CANx_send_data[4] = spinVel >> 8;
    ChassisVel.CANx_send_data[5] = spinVel & 0xff;
    ChassisVel.sendBuff();
}


/**
 * 获取yaw速度，设置目标值
 */
float portSetYaw()
{
    static float portion = TIMpiece;
    static float tar_pos = 0;
    switch (Car.ShootMode)
    {

        case cCar::eRC:
            switch (Car.CarType)
            {

                case cCar::ONMI:
//                    matPID.yawSetparam(1300, 2000, 0, 0, 30192, 0.06,
//                                       1.5, 0, 0.01, 175, 300);
                    break;
                case cCar::MECANUM:
//                    matPID.yawSetparam(950, 1450, 0, 0, 30192, 0.06,
//                                       1.5, 0, 0.01, 175, 300);
                    break;
                case cCar::UAV:
//                    matPID.yawSetparam(950, 1450, 0, 0, 30192, 0.06,
//                                       1.5, 0, 0.01, 175, 300);

                    break;
            }
            tar_pos -= (float) RC_GetDatas().rc.ch[0] * portion * 360.0f / 660.0f;
            yaw.RCcrtl_filter.ckalman.Calc(tar_pos);
            break;
        case cCar::eAutoAim:
            switch (Car.CarType)
            {

                case cCar::ONMI:
//                    matPID.yawSetparam(1200, 1800, 0, 0, 30192, 0.06,
//                                       2, 0, 0.005, 175, 300);
                    break;
                case cCar::MECANUM:
//                    matPID.yawSetparam(1200, 1800, 0, 0, 30192, 0.06,
//                                       2, 0, 0.005, 175, 300);
                    break;
                case cCar::UAV:
//                    matPID.yawSetparam(1200, 1800, 0, 0, 30192, 0.06,
//                                       2, 0, 0.005, 175, 300);
                    break;
            }
            if (yaw.autoaimflag)
            {
//                tar_pos = IMU_Angle(imu_yaw) + yaw.autoAim_filter.cLowPass.filter(vision_pkt.offset_yaw) * 1.0f;
                yaw.autoaimflag = false;
            }
            break;
        case cCar::eKey:
            break;
    }
//    usart_printf("%.2f,%.2f\r\n", IMU_Angle(imu_yaw), tar_pos);
    return tar_pos;
}

/**
 * 获取pitch速度，设置目标值
 */
float portSetPitch()
{
    static float portion = TIMpiece;
    static float tar_pos = 100;
    switch (Car.ShootMode)
    {

        case cCar::eRC:
            switch (Car.CarType)
            {

                case cCar::ONMI:
                        pitch.MotorCtrl.c_PID.setParam(100, 20, 0, 30000,
                                                   1, 0.00, 0, 300);
                    break;
                case cCar::MECANUM:
                    pitch.MotorCtrl.c_PID.setParam(300, 5, 0, 30000,
                                                   2, 0.00, 0, 300);
                    break;
                case cCar::UAV:
                    pitch.MotorCtrl.c_PID.setParam(300, 5, 0, 30000,
                                                   2, 0.00, 0, 300);
                    break;
            }
            tar_pos += (float) RC_GetDatas().rc.ch[1] * portion;
            if (tar_pos >= 136) tar_pos = 136;
            else if (tar_pos <= 65) tar_pos = 65;
            pitch.RCcrtl_filter.ckalman.Calc(tar_pos);
            break;
        case cCar::eAutoAim:
            switch (Car.CarType)
            {

                case cCar::ONMI:
                    pitch.MotorCtrl.c_PID.setParam(1000, 2, 0, 30000,
                                                   2, 0.00, 0, 300);
                    break;
                case cCar::MECANUM:
                    pitch.MotorCtrl.c_PID.setParam(1000, 2, 0, 30000,
                                                   2, 0.00, 0, 300);
                    break;
                case cCar::UAV:
                    pitch.MotorCtrl.c_PID.setParam(1000, 2, 0, 30000,
                                                   2, 0.00, 0, 300);
                    break;
            }
            if (pitch.autoaimflag)
            {
//                tar_pos = IMU_Angle(imu_pitch) - pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch) * 1.0f;
                pitch.autoaimflag = false;
            }
            if (tar_pos >= 40) tar_pos = 40;
            else if (tar_pos <= -30) tar_pos = -30;
            break;
        case cCar::eKey:
            break;
    }
//    usart_printf("%.2f,%.2f,%.2f\r\n", pitch.getEcd().total_angle,tar_pos,pitch.getEcd().torque_current);
    return tar_pos;
}

void portSetShoot()
{
    if (RC_GetDatas().rc.ch[4] > 100)
    {
        rammc.MotorCtrl.c_PID.setSpdTar(-RC_GetDatas().rc.ch[4] * 0.2);
    }
    else if (RC_GetDatas().rc.ch[4] >= -300 && RC_GetDatas().rc.ch[4] <= -100)
        rammc.MotorCtrl.c_PID.setSpdTar(-RC_GetDatas().rc.ch[4] * 0.5);
    else
        rammc.MotorCtrl.c_PID.setSpdTar(0);

    static uint8_t is_fric_start = 0;
    uint16_t fric_count = 0;


    if (RC_GetDatas().rc.ch[4] <= -550 && is_fric_start == 0)
    {
        fricL.MotorCtrl.c_ADRC.setSpdTar(-8000);
        fricR.MotorCtrl.c_ADRC.setSpdTar(8000);

        for (fric_count = 0; fric_count >= 400; fric_count++)
        {
        }
        is_fric_start = 1;
    }
    else if (RC_GetDatas().rc.ch[4] <= -550 && is_fric_start == 1)
    {
        fricL.MotorCtrl.c_ADRC.setSpdTar(0);
        fricR.MotorCtrl.c_ADRC.setSpdTar(0);

        for (fric_count = 0; fric_count >= 400; fric_count++)
        {
        }
        is_fric_start = 0;
    }
//    rammc.MotorCtrl.c_PID.setSpdTar(-RC_GetDatas().rc.ch[4] * 0.2);
}

float portSetLeft()
{
    switch (RC_GetDatas().rc.s[1])
    {
        case 1:
            Car.ShootMode = cCar::eKey;
            break;
        case 2:
            Car.ShootMode = cCar::eAutoAim;
            break;
        case 3:
            Car.ShootMode = cCar::eRC;
            break;
    }
    return 1;
}

float portSetRight()
{

    if (RC_GetDatas().rc.s[0] == 2)
    {
        setSpin(50);
    }
    else
        setSpin(0);

    return 1;
}


/**
 * 获取所有遥控器数据
 */
void getCrtlData()
{
//    matPID.yawSetTar(portSetYaw());
//    yaw.MotorCtrl.c_PID.setSpdTar(portSetYaw());
//    float tar = Debug_Param().vel_rampTargetValue;
//    usart_printf("%.2f,%.2f,%.2f\r\n", yaw.getEcd().speed, yaw.MotorCtrl.c_PID.Pid_Out(), tar);
//    yaw.MotorCtrl.c_PID.setSpdTar(tar);

    pitch.MotorCtrl.c_PID.setPosTar(portSetPitch());
    portSetLeft();
    portSetRight();
    portSetShoot();
    portSetMove(RC_GetDatas().rc.ch[2] * 200.0f / 660.0f, RC_GetDatas().rc.ch[3] * 200.0f / 660.0f);

}
/**********************************Motor SpdLoop and send current**********************************************/
/**
 * 电机所有算法计算
 */
void Can_Calc()
{
//    fricL.MotorCtrl.c_ADRC.SpdLoop(fricL.getEcd().speed);
//    fricR.MotorCtrl.c_ADRC.SpdLoop(fricR.getEcd().speed);
//    rammc.MotorCtrl.c_PID.SpdLoop(rammc.getEcd().speed);
//    pitch.MotorCtrl.c_PID.PosLoop(IMU_Angle(imu_pitch), IMU_Speed(imu_pitch));
    pitch.MotorCtrl.c_PID.PosLoop(pitch.getEcd().total_angle, pitch.getEcd().speed);
//    matPID.yaw_calc(IMU_Angle(imu_yaw), IMU_Speed(imu_yaw));
//    yaw.MotorCtrl.c_PID.SpdLoop(yaw.getEcd().speed);
//    if (portSetFollowUp())
//    {
//        if (Car.getPCarInEcd() - yaw.getEcd().total_angle > 180)Car.setPCarInEcd(-360); //加减2π
//        if (yaw.getEcd().total_angle - Car.getPCarInEcd() > 180)Car.setPCarInEcd(360);//优弧劣弧处理
//        yaw.MotorCtrl.c_PID.setPosTar(Car.getPCarInEcd());
//        yaw.MotorCtrl.c_PID.setParam(0.6, 0, 22, 300);
//        float chassisyaw = -yaw.MotorCtrl.c_PID.PosLoop(yaw.getEcd().total_angle);
//        setSpin(chassisyaw);//
//    }

//    usart_printf("%.2f\r\n",vision_pkt.offset_yaw);

}

/**
 * 统一赋值控制量
 */
void Can_Send()
{
//    yaw.canSend(cMotor::eExternal, (int16_t)current);
//    usart_printf("%.2f,%.2f\r\n", current, yaw.getEcd().speed);
    pitch.canSend(cMotor::ePid);
//    fricR.canSend(cMotor::eAdrc);
//    fricL.canSend(cMotor::eAdrc);
//    rammc.canSend(cMotor::ePid);
//    sendChassisYaw(-(yaw.getEcd().total_angle - Car.getPCarInEcd()));

}
void CanMxg()
{
    uint8_t i;
    for (i = 0; i <= 5; i++)
    {
        CanReg[i].sendBuff();//
    }
}

void GimbalLoop()
{
    if (portSetProtect())
    {
//        yaw.protect();
        pitch.protect();
////        fricL.protect();
////        fricR.protect();
//        fricL.MotorCtrl.c_PID.setSpdTar(0);
//        fricR.MotorCtrl.c_PID.setSpdTar(0);
//        fricL.MotorCtrl.c_PID.SpdLoop(fricL.getEcd().speed);
//        fricR.MotorCtrl.c_PID.SpdLoop(fricR.getEcd().speed);
//        fricR.canSend(cMotor::ePid);
//        fricL.canSend(cMotor::ePid);
//        rammc.protect();
    }
    else
    {
        getCrtlData();
//    static float tar=0;
//        static float t = 0;
//    t+=0.01;
//    tar = 90 * sinf( t);
//    /**
//     * POS
//     */
//     matPID.PosSetPara(50.1993221790789,393.682367810861,0.105235467993934,985.553384610435,300);
//     matPID.PosLoop(yaw.getEcd().total_angle,tar);
//    /**
//     * Spd
//     */
//    matPID.setPara(31.1290754596081,7263.07220611607,30192);
//    matPID.settar(matPID.PosOut());
//    matPID.calc(yaw.getEcd().speed);

        Can_Calc();
        Can_Send();

    }
    CanMxg();

}

