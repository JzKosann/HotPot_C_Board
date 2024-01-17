//
// Created by ShiF on 2023/9/14.
//
#include "gimbalc.hpp"
#include "math.h"
#include "INS_task.h"
#include "IMUC.hpp"


/** Motor List **/
cMotor yaw, pitch;          //云台功能
cMotor fricR, fricL;
cMotor rammc;
/** algorithm **/
#define TIMpiece    0.002   //控制周期，时间切片
static float mouse_sense = TIMpiece * 2.5;

/** Car **/             //choose your car here
cCar Car(cCar::MECANUM);
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
    yaw.autoAim_filter.cLowPass.Init(5, 0.002);
    pitch.autoAim_filter.cLowPass.Init(10, 0.002);
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
    static uint16_t offline_record;
    offline_record++;
    if (offline_record > 2000)
    {
        if (RC_GetNewData.is_online)
        {
            RC_GetNewData.now_state = true;
        }
        else RC_GetNewData.now_state = false;
        offline_record = 0;
    }
    if (RC_GetNewData.now_state)
    {
        if (RC_GetDatas().rc.s[0] == 1) Car.is_protect = true;
        else Car.is_protect = false;
    }
    else Car.is_protect = true;
    RC_GetNewData.is_online = false;
    return Car.is_protect;
}


void portSetMove()
{
    float vx = 0;
    float vy = 0;
    switch (Car.CtrlMode)
    {

        case cCar::eRC:
            vx = RC_GetDatas().rc.ch[2] * 200.0f / 660.0f;
            vy = RC_GetDatas().rc.ch[3] * 200.0f / 660.0f;
            break;
        case cCar::eRC_Autoaim:
            vx = RC_GetDatas().rc.ch[2] * 200.0f / 660.0f;
            vy = RC_GetDatas().rc.ch[3] * 200.0f / 660.0f;
            break;
        case cCar::eKey:
            if (rc_ctrl.key.W)
                vx = rc_ctrl.key.W * 200.0f;
            else if (rc_ctrl.key.S)
                vx = -rc_ctrl.key.S * 200.0f;
            if (rc_ctrl.key.A)
                vx = rc_ctrl.key.A * 200.0f;
            else if (rc_ctrl.key.D)
                vx = -rc_ctrl.key.D * 200.0f;
            break;
    }


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
    bool mode_judge;
    switch (Car.CtrlMode)
    {

        case cCar::eRC:
            mode_judge = (RC_GetDatas().rc.s[0] == 3);
            break;
        case cCar::eRC_Autoaim:
            mode_judge = (RC_GetDatas().rc.s[0] == 3);
            break;
        case cCar::eKey:
            mode_judge = (rc_ctrl.key.F.Is_Click_Once != 1);
            break;
    }
    if (mode_judge)
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
                    yaw_mat.SetPara(2500, 1800, 0, 0, 30000,
                                    2.5, 0, 0, 175, 50, 0);
                    _tar_pos -= (float) RC_GetDatas().rc.ch[0] * portion * 360.0f / 660.0f;
                    tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;
                case cCar::eKey:
                    yaw_mat.SetPara(2500, 1800, 0, 0, 30000,
                                    2.5, 0, 0, 175, 50, 0);
                    _tar_pos -= (float) RC_GetDatas().mouse.x * mouse_sense;
                    tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;
                case cCar::eRC_Autoaim:
                    yaw_mat.SetPara(2000, 1400, 0, 0, 30000,
                                    0.8, 0, 0, 175, 200, 0);
                    if (yaw.autoaimflag)
                    {
                        tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + yaw.autoAim_filter.cLowPass.filter(vision_pkt.offset_yaw) * 1.0f;
                        yaw.autoaimflag = false;
                    }
                    break;
            }
            break;
        case cCar::MECANUM:
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    yaw_mat.SetPara(2200, 1200, 0, 0, 30000,
                                    1.8, 0, 0, 175, 50, 0);
                    _tar_pos -= (float) RC_GetDatas().rc.ch[0] * portion * 360.0f / 660.0f;
                    tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;
                case cCar::eKey:
                    switch (Car.ShootMode)
                    {
                        case cCar::eNormal:
                            yaw_mat.SetPara(2500, 1800, 0, 0, 30000,
                                            2.5, 0, 0, 175, 50, 0);
                            _tar_pos -= (float) RC_GetDatas().mouse.x * mouse_sense;
                            tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                            break;
                        case cCar::eAutoaim:
                            yaw_mat.SetPara(2000, 1400, 0, 0, 30000,
                                            0.8, 0, 0, 175, 200, 0);
                            if (yaw.autoaimflag)
                            {
                                _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + yaw.autoAim_filter.cLowPass.filter(vision_pkt.offset_yaw) * 1.0f;
                                tar_pos = _tar_pos;
                                yaw.autoaimflag = false;
                            }
                            break;
                    }
                    break;
                case cCar::eRC_Autoaim:
                    yaw_mat.SetPara(3300, 1000, 0, 0, 30000,
                                    0.65, 0.0, 0.03, 175, 200, 0);
                    if (yaw.autoaimflag)
                    {
                        _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + vision_pkt.offset_yaw;
//                        _tar_pos = tar_pos + yaw.autoAim_filter.cLowPass.filter(vision_pkt.offset_yaw) * 1.0f;
                        tar_pos = yaw.autoAim_filter.cLowPass.filter(_tar_pos);
                        yaw.autoaimflag = false;
                    }
                    break;
            }
            break;
        case cCar::UAV:

            break;
    }
    usart_printf("%.2f,%.2f,%.2f\r\n", tar_pos, IMU.cAngle(cimu::Wit_imu, cimu::Yaw), vision_pkt.offset_yaw);
    return tar_pos;
}

/**
 * 获取pitch速度，设置目标值
 */
float portSetPitch()
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
                    switch (Car.ShootMode)
                    {
                        case cCar::eNormal:
                            pitch.MotorCtrl.c_PID.setParam(800, 1, 0, 30000, 0,
                                                           1.8, 0.0, 5, 300, 50);
                            _tar_pos += (float) RC_GetDatas().rc.ch[1] * portion;
                            tar_pos = -pitch.RCcrtl_filter.ckalman.Calc(_tar_pos);
                            break;
                        case cCar::eAutoaim:
                            pitch.MotorCtrl.c_PID.setParam(1000, 2, 0, 30000,
                                                           2, 0.00, 0, 300);
                            if (pitch.autoaimflag)
                            {
                                tar_pos = -IMU.cAngle(cimu::Wit_imu, cimu::Pitch) + pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch) * 1.0f;
                                pitch.autoaimflag = false;
                            }
                            break;
                    }
                    break;
                case cCar::eKey:
                    switch (Car.ShootMode)
                    {
                        case cCar::eNormal:
                            pitch.MotorCtrl.c_PID.setParam(800, 1, 0, 30000, 0,
                                                           1.8, 0.0, 5, 300, 50);
                            _tar_pos += (float) RC_GetDatas().mouse.y * mouse_sense;
                            tar_pos = pitch.RCcrtl_filter.ckalman.Calc(_tar_pos);
                            break;
                        case cCar::eAutoaim:
                            break;
                    }

                    break;
                case cCar::eRC_Autoaim:
                    pitch.MotorCtrl.c_PID.setParam(700, 2, 0, 30000,
                                                   2, 0.00, 0, 300);
                    if (pitch.autoaimflag)
                    {
                        tar_pos = -IMU.cAngle(cimu::Wit_imu, cimu::Pitch) + pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch) * 1.0f;
                        pitch.autoaimflag = false;
                    }
                    break;
            }
            break;
        case cCar::MECANUM:
            switch (Car.CtrlMode)
            {
                case cCar::eRC:
                    pitch.MotorCtrl.c_PID.setParam(800, 1, 0, 30000, 0,
                                                   2, 0.0, 3, 300, 0);
                    _tar_pos += (float) RC_GetDatas().rc.ch[1] * portion;
                    tar_pos = -pitch.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;

                    break;
                case cCar::eKey:
                    switch (Car.ShootMode)
                    {
                        case cCar::eNormal:
                            pitch.MotorCtrl.c_PID.setParam(800, 1, 0, 30000, 0,
                                                           1.8, 0.0, 5, 300, 50);
                            _tar_pos += (float) RC_GetDatas().mouse.y * mouse_sense;
                            tar_pos = pitch.RCcrtl_filter.ckalman.Calc(_tar_pos);
                            break;
                        case cCar::eAutoaim:
                            pitch.MotorCtrl.c_PID.setParam(1000, 2, 0, 30000,
                                                           2, 0.00, 0, 300);
                            if (pitch.autoaimflag)
                            {
                                _tar_pos = -IMU.cAngle(cimu::Wit_imu, cimu::Pitch) + pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch) * 1.0f;
                                tar_pos = _tar_pos;
                                pitch.autoaimflag = false;
                            }
                            break;
                    }

                    break;
                case cCar::eRC_Autoaim:
                    pitch.MotorCtrl.c_PID.setParam(500, 1, 0, 30000,
                                                   1.2, 0.00, 0, 300);
                    if (pitch.autoaimflag)
                    {
                        tar_pos = -IMU.cAngle(cimu::Wit_imu, cimu::Pitch) + pitch.autoAim_filter.cLowPass.filter(vision_pkt.offset_pitch) * 1.0f;
                        pitch.autoaimflag = false;
                    }
                    break;
            }
            break;
        case cCar::UAV:
            break;
    }
    if (tar_pos >= 25) tar_pos = 25;
    else if (tar_pos <= -23) tar_pos = -23;
//    usart_printf("%.2f,%.2f,%.2f\r\n", tar_pos, IMU.cAngle(cimu::Wit_imu, cimu::Pitch), vision_pkt.offset_pitch);
    return tar_pos;
}

void portSetShoot()
{
    static bool is_shoot = false;
    switch (Car.CtrlMode)
    {
        case cCar::eRC_Autoaim:
        {
            static uint8_t is_fric_start = 0;
            uint16_t fric_count = 0;

            if (RC_GetDatas().rc.ch[4] <= -550 && is_fric_start == 0)
            {
                fricL.MotorCtrl.c_ADRC.setSpdTar(-6500);
                fricR.MotorCtrl.c_ADRC.setSpdTar(6500);
                is_shoot = true;
                for (fric_count = 0; fric_count >= 400; fric_count++)
                {
                }
                is_fric_start = 1;
            }
            else if (RC_GetDatas().rc.ch[4] <= -550 && is_fric_start == 1)
            {
                fricL.MotorCtrl.c_ADRC.setSpdTar(0);
                fricR.MotorCtrl.c_ADRC.setSpdTar(0);
                is_shoot = false;
                for (fric_count = 0; fric_count >= 400; fric_count++)
                {
                }
                is_fric_start = 0;
            }
            if (is_shoot)
            {
                if (vision_pkt.suggest_fire)
                {
                    rammc.MotorCtrl.c_PID.setSpdTar(-80);
                }
                else
                {
                    rammc.MotorCtrl.c_PID.setSpdTar(0);
                }
            }

        }
            break;
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
            static uint8_t is_fric_start = 0;
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
            portHandle(&rc_ctrl.mouse.press_r);
            if (rc_ctrl.mouse.press_r.Is_Click_Once)
            {
                is_shoot = true;
                fricL.MotorCtrl.c_ADRC.setSpdTar(-6500);
                fricR.MotorCtrl.c_ADRC.setSpdTar(6500);
            }
            else
            {
                is_shoot = false;
                fricL.MotorCtrl.c_ADRC.setSpdTar(0);
                fricR.MotorCtrl.c_ADRC.setSpdTar(0);
            }

            if (RC_GetDatas().mouse.press_l.Now_State && is_shoot)
            {
                rammc.MotorCtrl.c_PID.setSpdTar(-80);
            }
            else
                rammc.MotorCtrl.c_PID.setSpdTar(0);
            break;
    }


    static uint8_t stalled_count = 0;   //堵转处理
    static bool is_stalled = false;
    if (rammc.getEcd().torque_current < -6000)is_stalled = true;
    if (is_stalled)
    {
        rammc.MotorCtrl.c_PID.setSpdTar(150);
        stalled_count++;
        if (stalled_count >= 100)
        {
            stalled_count = 0;
            is_stalled = false;
        }
    }

//    usart_printf("%d,%d\r\n", RC_GetDatas().key.Q.Is_Click_Once, RC_GetDatas().mouse.press_r.Is_Click_Once);
}

float portSetLeft()
{
    switch (RC_GetDatas().rc.s[1])
    {
        case 1:
            Car.CtrlMode = cCar::eKey;
            break;
        case 2:
            Car.CtrlMode = cCar::eRC_Autoaim;
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
            portHandle(&rc_ctrl.key.F);
            if (rc_ctrl.key.F.Is_Click_Once)setSpin(50);
            else setSpin(0);
            break;
    }


    return 1;
}


/**
 * 获取所有遥控器数据
 */
void getCrtlData()
{

    yaw_mat.SetTar(portSetYaw());
    pitch.MotorCtrl.c_PID.setPosTar(portSetPitch());
    portSetLeft();
    portSetRight();
    portSetShoot();
    portSetMove();

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
    fricR.canSend(cMotor::eAdrc);
    fricL.canSend(cMotor::eAdrc);
    rammc.canSend(cMotor::ePid);
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
//    usart_printf("%.2f,%.2f,%.2f,%.2f\r\n", Bno085_IMU_Angle(0),);
    CanMxg();

}

