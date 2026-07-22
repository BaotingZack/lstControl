
/*
 * @Author: mapin 919808655@qq.com
 * @Date: 2025-09-12 08:27:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-01-17 09:08:11
 * @FilePath: /crane_robot/src/communication/ctrl_cmd_send/include/ctrl_cmd_send/lib/car_control.h
 * @Description: PLC控制库头文件
 */
#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // 基础连接
    int connect_to_plc(const char *ip); // 连接PLC，ip为NULL时用默认192.168.0.1
    void disconnect_plc();              // 断开PLC连接

    // 运动控制（带控制位）

    void BigCarCtrl(double velocity, const char *ip, uint16_t control_flag = 0x047F);   // 大车速度控制
    void SmallcarCtrl(double velocity, const char *ip, uint16_t control_flag = 0x047F); // 小车速度控制
    void liftctrl(double height, const char *ip);                                       // 吊钩高度控制

    bool SendPlcHeartbeat(const char *ip); // 心跳发布
    bool GetPlcHeartbeat();                // 心跳监测

    // 开关/安全控制
    void EmergencyBrake(bool clamp, const char *ip);                  // 紧急制动
    void ResetControl(bool clamp, const char *ip);                    // 复位

    // 吊具控制
    void GripperClampOnControl(bool clamp, const char *ip);    // 吊具夹紧
    void GripperClampOffControl(bool clamp, const char *ip);   // 吊具释放

    // 单独状态读取
    double GetActualLiftHeight();             // 读取实际吊钩高度
    int ReadPlcHeartbeatMonitor();            // 读取心跳值
    bool CheckPlcConnection();                // PLC连接检查
    bool CheckHeartbeatStatus();              // 心跳状态检查

#ifdef __cplusplus
}
#endif

#endif // CAR_CONTROL_H
