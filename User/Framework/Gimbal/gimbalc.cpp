//
// Created by ShiF on 2023/9/14.
//
#include "gimbalc.hpp"
#include "math.h"
#include "INS_task.h"
#include "IMUC.hpp"
#include "matlab_PID.hpp"


/** Motor List **/
cMotor yaw, pitch;          //云台功能
cMotor fricR, fricL;
cMotor rammc;
/** algorithm **/

#define TIMpiece    0.002   //控制周期，时间切片

/** Car **/             //choose your car here
cCar Car(cCar::ONMI);
/**
 * ONMI 全向
 * MECANUM 麦步
 * UAV 无人机
 */

/**
 * 滤波初始化
 */
void Filter_Init()
{
    yaw.autoAim_filter.cLowPass.Init(20, TIMpiece);
    pitch.autoAim_filter.cLowPass.Init(10, TIMpiece);
    yaw.autoAim_filter.ckalman.Init(20, 200);
    yaw.RCcrtl_filter.ckalman.Init(40, 200);
    pitch.RCcrtl_filter.ckalman.Init(40, 200);
    IMU.yaw_filter.ckalman.Init(10, 60);
    IMU.pitch_filter.ckalman.Init(10, 60);
}

void Algorithm_Init()
{
    yaw.MotorCtrl.c_PID.Init();
    yaw_mat.Init();
    pitch.MotorCtrl.c_PID.Init();

    fricL.MotorCtrl.c_ADRC.Init(1500, 0.05, 0.5, 5, 1, 1000, 10,
                                0.5, 1.25, 80, 1);
    fricR.MotorCtrl.c_ADRC.Init(1500, 0.05, 0.5, 5, 1, 1000, 10,
                                0.5, 1.25, 80, 1);
    rammc.MotorCtrl.c_PID.Init();
    rammc.MotorCtrl.c_PID.setParam(100, 5, 0, 10000,
                                   0, 0, 0, 0);
}

void MotorInit()
{
    yaw.canInit(0x209, &hcan2, cMotor::GM_6020);
    pitch.canInit(0x206, &hcan1, cMotor::GM_6020);


    rammc.canInit(0x203, &hcan1, cMotor::M_2006_p36);
    if (Car.CarType == cCar::UAV)
    {
        fricL.canInit(0x201, &hcan1, cMotor::M_2006);
        fricR.canInit(0x202, &hcan1, cMotor::M_2006);
    }
    else
    {
        fricL.canInit(0x201, &hcan1, cMotor::M_3508);
        fricR.canInit(0x202, &hcan1, cMotor::M_3508);
    }
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
    Car.CtrlMode = cCar::eRC;
    Car.is_protect = true;
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
    if (RC_GetNewData)
    {
        if (RC_GetDatas().rc.s[0] == 1) Car.is_protect = true;
        else Car.is_protect = false;
        RC_GetNewData = false;
    }
    else
    {
        Car.is_protect = false;
    }
    return Car.is_protect;
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
    static float _tar_pos = 0;
    switch (Car.CarType)
    {

        case cCar::ONMI:
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    yaw_mat.SetPara(2000, 1400, 0, 0, 30000,
                                    1.8, 0, 0, 1, 50, 5);
                    _tar_pos -= (float) RC_GetDatas().rc.ch[0] * portion * 360.0f / 660.0f;
                    tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;
                case cCar::eAutoAim:
                    yaw_mat.SetPara(1500, 1000, 0, 0, 30000,
                                    0.5, 0, 0.005, 200, 50, 0);
                    if (yaw.autoaimflag)
                    {
                        static float yaw_autoaim;
                        tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + yaw.autoAim_filter.cLowPass.filter(vision_pkt.offset_yaw);
                        _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + vision_pkt.offset_yaw;
                        yaw.autoaimflag = false;
                    }
                    break;
                case cCar::eKey:
                    break;
            }
            break;
        case cCar::MECANUM:
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    yaw_mat.SetPara(2000, 1400, 0, 0, 30000,
                                    1.8, 0, 0, 1, 50, 5);
                    _tar_pos -= (float) RC_GetDatas().rc.ch[0] * portion * 360.0f / 660.0f;
                    tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;
                case cCar::eAutoAim:
                    yaw_mat.SetPara(1500, 1000, 0, 0, 30000,
                                    0.5, 0, 0.005, 200, 50, 0);
                    if (yaw.autoaimflag)
                    {
                        static float yaw_autoaim;
                        tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + yaw.autoAim_filter.cLowPass.filter(vision_pkt.offset_yaw);
                        _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + vision_pkt.offset_yaw;
                        yaw.autoaimflag = false;
                    }
                    break;
                case cCar::eKey:
                    break;
            }
            break;
        case cCar::UAV:
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    yaw_mat.SetPara(2000, 1400, 0, 0, 30000,
                                    1.8, 0, 0, 1, 50, 5);
                    _tar_pos -= (float) RC_GetDatas().rc.ch[0] * portion * 360.0f / 660.0f;
                    tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;
                case cCar::eAutoAim:
                    yaw_mat.SetPara(1500, 1000, 0, 0, 30000,
                                    0.5, 0, 0.005, 200, 50, 0);
                    if (yaw.autoaimflag)
                    {
                        static float yaw_autoaim;
                        tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + yaw.autoAim_filter.cLowPass.filter(vision_pkt.offset_yaw);
                        _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + vision_pkt.offset_yaw;
                        yaw.autoaimflag = false;
                    }
                    break;
                case cCar::eKey:
                    break;
            }
            break;
    }

//    usart_printf("%.2f,%.2f\r\n", IMU_Angle(0), IMU_Angle(1));
//    usart_printf("%.2f,%.2f\r\n",tar_pos,IMU.cAngle(cimu::Wit_imu,cimu::Yaw));
//    usart_printf("%.2f,%.2f,%.2f,%.2f\r\n", tar_pos, _tar_pos, IMU.cAngle(cimu::Wit_imu, cimu::Pitch), IMU.cAngle(cimu::Wit_imu, cimu::Yaw));
    return tar_pos;
}

/**
 * 获取pitch速度，设置目标值
 */
float portSetPitch()
{
    static float portion = TIMpiece;
    static float tar_pos = 0;

    switch (Car.CarType)
    {

        case cCar::ONMI:
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    pitch.MotorCtrl.c_PID.setParam(300, 1, 0, 30000, 0,
                                                   1, 0.0, 5, 300, 50);
                    tar_pos -= (float) RC_GetDatas().rc.ch[1] * portion;
                    pitch.RCcrtl_filter.ckalman.Calc(tar_pos);
                    break;
                case cCar::eAutoAim:
                    if ((-tar_pos - IMU.cAngle(cimu::Wit_imu, cimu::Pitch)) < -5)
                        pitch.MotorCtrl.c_PID.setParam(10, 0.5, 0, 30000, 0,
                                                       3, 0.0, 0, 300, 0);
                    else
                        pitch.MotorCtrl.c_PID.setParam(100, 1.2, 0, 30000, 0,
                                                       2, 0.0, 10, 300, 5);
                    if (pitch.autoaimflag)
                    {
                        tar_pos = -IMU.cAngle(cimu::Wit_imu, cimu::Pitch) + pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch);
                        pitch.autoaimflag = false;
                    }
                    break;
                case cCar::eKey:
                    break;
            }
            break;
        case cCar::MECANUM:
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    pitch.MotorCtrl.c_PID.setParam(300, 1, 0, 30000, 0,
                                                   1, 0.0, 5, 300, 50);
                    tar_pos -= (float) RC_GetDatas().rc.ch[1] * portion;
                    pitch.RCcrtl_filter.ckalman.Calc(tar_pos);
                    break;
                case cCar::eAutoAim:
                    if ((-tar_pos - IMU.cAngle(cimu::Wit_imu, cimu::Pitch)) < -5)
                        pitch.MotorCtrl.c_PID.setParam(10, 0.5, 0, 30000, 0,
                                                       3, 0.0, 0, 300, 0);
                    else
                        pitch.MotorCtrl.c_PID.setParam(100, 1.2, 0, 30000, 0,
                                                       2, 0.0, 10, 300, 5);
                    if (pitch.autoaimflag)
                    {
                        tar_pos = -IMU.cAngle(cimu::Wit_imu, cimu::Pitch) + pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch);
                        pitch.autoaimflag = false;
                    }
                    break;
                case cCar::eKey:
                    break;
            }
            break;
        case cCar::UAV:
        {
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    pitch.MotorCtrl.c_PID.setParam(300, 1, 0, 30000, 0,
                                                   1, 0.0, 5, 300, 50);
                    tar_pos -= (float) RC_GetDatas().rc.ch[1] * portion;
                    pitch.RCcrtl_filter.ckalman.Calc(tar_pos);
                    break;
                case cCar::eAutoAim:
                    if ((-tar_pos - IMU.cAngle(cimu::Wit_imu, cimu::Pitch)) < -5)
                        pitch.MotorCtrl.c_PID.setParam(10, 0.5, 0, 30000, 0,
                                                       3, 0.0, 0, 300, 0);
                    else
                        pitch.MotorCtrl.c_PID.setParam(100, 1.2, 0, 30000, 0,
                                                       2, 0.0, 10, 300, 5);
                    if (pitch.autoaimflag)
                    {
                        tar_pos = -IMU.cAngle(cimu::Wit_imu, cimu::Pitch) + pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch);
                        pitch.autoaimflag = false;
                    }
                    break;
                case cCar::eKey:
                    break;
            }
            break;
        }
    }

//    usart_printf("%.2f,%.2f,%.2f\r\n", pitch.getEcd().total_angle,tar_pos,pitch.getEcd().torque_current);
//    usart_printf("%.2f,%.2f,%.2f\r\n", -tar_pos, -vision_pkt.offset_pitch, IMU.cAngle(cimu::Wit_imu, cimu::Pitch));
    if (tar_pos >= 30) tar_pos = 30;
    else if (tar_pos <= -23) tar_pos = -23;
    return tar_pos;
}

void portSetShoot()
{

    static uint8_t is_fric_start = 0;
    switch (Car.CtrlMode)
    {
        case cCar::eRC:
        {

            if (RC_GetDatas().rc.ch[4] > 100)
            {
                rammc.MotorCtrl.c_PID.setSpdTar(-RC_GetDatas().rc.ch[4] * 0.2);
            }
            else if (RC_GetDatas().rc.ch[4] >= -300 && RC_GetDatas().rc.ch[4] <= -100)
                rammc.MotorCtrl.c_PID.setSpdTar(-RC_GetDatas().rc.ch[4] * 0.5);
            else
                rammc.MotorCtrl.c_PID.setSpdTar(0);


            uint16_t fric_count = 0;
            if (RC_GetDatas().rc.ch[4] <= -550 && is_fric_start == 0)
            {
                fricL.MotorCtrl.c_ADRC.setSpdTar(-6500);
                fricR.MotorCtrl.c_ADRC.setSpdTar(6500);

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
        }
            break;
        case cCar::eKey:
        {
            portHandle(&rc_ctrl.key.Q);
            if (rc_ctrl.key.Q.Is_Click_Once)
            {
                fricL.MotorCtrl.c_ADRC.setSpdTar(-6500);
                fricR.MotorCtrl.c_ADRC.setSpdTar(6500);
            }
            else
            {
                fricL.MotorCtrl.c_ADRC.setSpdTar(0);
                fricR.MotorCtrl.c_ADRC.setSpdTar(0);
            }

            if (RC_GetDatas().mouse.press_l.Now_State)
            {
                rammc.MotorCtrl.c_PID.setSpdTar(120);
            }
            else
            {
                rammc.MotorCtrl.c_PID.setSpdTar(0);
            }
        }
            break;
    }

}

float portSetLeft()
{
    switch (RC_GetDatas().rc.s[1])
    {
        case 1:
            Car.CtrlMode = cCar::eKey;
            break;
        case 2:
            Car.CtrlMode = cCar::eAutoAim;
            break;
        case 3:
            Car.CtrlMode = cCar::eRC;
            break;
    }
    return 1;
}

float portSetRight()
{
    switch (Car.CtrlMode)
    {

        case cCar::eRC:
            if (RC_GetDatas().rc.s[0] == 2)
            {
                setSpin(50);
            }
            else
                setSpin(0);
            break;
        case cCar::eKey:
            break;
    }


    return 1;
}


/**
 * 获取所有遥控器数据
 */
void getCrtlData()
{


    portSetLeft();
    portSetRight();
    portSetShoot();
    portSetMove(RC_GetDatas().rc.ch[2] * 200.0f / 660.0f, RC_GetDatas().rc.ch[3] * 200.0f / 660.0f);

    yaw_mat.SetTar(portSetYaw());
    pitch.MotorCtrl.c_PID.setPosTar(portSetPitch());
}
/**********************************Motor spdLoop and send current**********************************************/
/**
 * 电机所有算法计算
 */
void Can_Calc()
{
    fricL.MotorCtrl.c_ADRC.SpdLoop(fricL.getEcd().speed);
    fricR.MotorCtrl.c_ADRC.SpdLoop(fricR.getEcd().speed);
    rammc.MotorCtrl.c_PID.spdLoop(rammc.getEcd().speed);
    if (Car.CarType == cCar::UAV)
    {
        pitch.MotorCtrl.c_PID.posLoop(-IMU_Angle(1), -IMU_Speed(1));
        yaw_mat.Calc(-IMU_Speed(0), -IMU_Angle(0));
    }
    else
    {
        pitch.MotorCtrl.c_PID.posLoop(-IMU.cAngle(cimu::Wit_imu, cimu::Pitch), -IMU.cSpeed(cimu::Wit_imu, cimu::Pitch));
//        yaw_mat.Calc(IMU.cSpeed(cimu::C_imu,cimu::Yaw),IMU.yaw_filter.ckalman.Calc(IMU.cAngle(cimu::Bno085_imu,cimu::Yaw)));
        yaw_mat.Calc(IMU.cSpeed(cimu::Wit_imu, cimu::Yaw), IMU.cAngle(cimu::Wit_imu, cimu::Yaw));

    }

    if (portSetFollowUp())
    {
        if (Car.getPCarInEcd() - yaw.getEcd().total_angle > 180)Car.setPCarInEcd(-360); //加减2π
        if (yaw.getEcd().total_angle - Car.getPCarInEcd() > 180)Car.setPCarInEcd(360);//优弧劣弧处理
        yaw.MotorCtrl.c_PID.setPosTar(Car.getPCarInEcd());
        yaw.MotorCtrl.c_PID.setParam(0.6, 0, 22, 300);
        float chassisyaw = -yaw.MotorCtrl.c_PID.posLoop(yaw.getEcd().total_angle);
        setSpin(chassisyaw);//
    }


}

/**
 * 统一赋值控制量
 */
void Can_Send()
{
//    yaw.canSend(cMotor::ePid);
    yaw.canSend(cMotor::eExternal, (int16_t) yaw_mat.Out());
    pitch.canSend(cMotor::ePid);
//    fricR.canSend(cMotor::eAdrc);
//    fricL.canSend(cMotor::eAdrc);
//    rammc.canSend(cMotor::ePid);
    sendChassisYaw(-(yaw.getEcd().total_angle - Car.getPCarInEcd()));

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
        yaw.protect();
        pitch.protect();
        fricL.MotorCtrl.c_PID.setSpdTar(0);
        fricR.MotorCtrl.c_PID.setSpdTar(0);
        fricL.MotorCtrl.c_PID.spdLoop(fricL.getEcd().speed);
        fricR.MotorCtrl.c_PID.spdLoop(fricR.getEcd().speed);
        fricR.canSend(cMotor::ePid);
        fricL.canSend(cMotor::ePid);
        rammc.protect();
    }
    else
    {
        getCrtlData();

        Can_Calc();
        Can_Send();

    }

    usart_printf("%d,%d,%d\r\n", RC_GetDatas().key.W, RC_GetDatas().mouse.y, RC_GetDatas().mouse.z);
    CanMxg();

}

