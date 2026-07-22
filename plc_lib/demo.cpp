/*
 * demo.cpp —— C++ 方式调用动态库的 PLC 控制测试程序
 *
 * 编译：
 *   g++ -o demo demo.cpp -I./lib -L./lib -lsscarctrl -lsnap7 -lstdc++ -lpthread
 *
 * 运行：
 *   export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
 *   ./demo
 */

#include <iostream>
#include <string>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <limits>
#include "ss_car_control.h"

/* =====================================================================
 * 心跳配置
 * ===================================================================== */
static const int HEARTBEAT_INTERVAL_MS = 100;  /* 100ms = 10Hz */
static const int HEARTBEAT_WARMUP_MS    = 500;  /* 连接后等 500ms 再开始 */
static const int MAX_CONSECUTIVE_FAILS  = 3;    /* 连续失败 3 次才算真断开 */

/* =====================================================================
 * 抓钩 / 复位控制配置
 * ---------------------------------------------------------------------
 * DB7 命令位布局（来自 libsscarctrl.so 反汇编）：
 *   GripperClampOnControl(v)  -> 把 v 写入 DB7.DBX48.0  (现场实测 1=释放)
 *   GripperClampOffControl(v) -> 把 v 写入 DB7.DBX48.1  (现场实测 1=夹紧)
 *   ResetControl(v)           -> 把 v 写入 DB7.DBX48.2  (=遥控"复位"键)
 *   EmergencyBrake(v)         -> 把 v 写入 DB7.DBX48.3  (急停)
 *
 * 重要：夹紧/释放位是【电平保持】，写 1 保持该动作、写 0 撤销该动作。
 * 因此绝对不能"写 1 后再写 0"当脉冲用 —— 那会让抓钩"夹一下又松开"。
 * 夹紧与释放互斥：置其一为 1 时必须把另一位清 0。
 *
 * "复位"位则是模拟遥控上的复位【按键】，属于瞬动信号，用脉冲(1->0)即可。
 * ===================================================================== */
static const int RESET_PULSE_MS = 300;  /* 软件复位脉冲宽度, 需 >= PLC 扫描周期 */

static std::atomic<bool> heartbeatRunning{true};

/* =====================================================================
 * 软件复位 —— 模拟遥控器"复位"键的一次瞬动按压 (置 1 -> 保持 -> 置 0)
 * ===================================================================== */
static void PlcResetPulse(const std::string& ip)
{
    ResetControl(true,  ip.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(RESET_PULSE_MS));
    ResetControl(false, ip.c_str());
}

/* =====================================================================
 * 抓钩夹紧 —— 电平保持
 * 现场实测极性: DB7.DBX48.1 (GripperClampOffControl) = 1 才是夹紧
 * ===================================================================== */
static void GripperClamp(const std::string& ip)
{
    GripperClampOnControl(false,  ip.c_str());  /* 释放位清 0 (互斥) */
    GripperClampOffControl(true,  ip.c_str());  /* 夹紧位置 1 并保持 */
}

/* =====================================================================
 * 抓钩释放 —— 电平保持
 * 现场实测极性: DB7.DBX48.0 (GripperClampOnControl) = 1 才是释放
 * ===================================================================== */
static void GripperRelease(const std::string& ip)
{
    GripperClampOffControl(false, ip.c_str());  /* 夹紧位清 0 (互斥) */
    GripperClampOnControl(true,   ip.c_str());  /* 释放位置 1 并保持 */
}

/* =====================================================================
 * 后台心跳线程 —— 独立运行，不受菜单阻塞影响
 * ===================================================================== */
static void heartbeatLoop(const std::string& ip)
{
    int failCount = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_WARMUP_MS));

    while (heartbeatRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_INTERVAL_MS));
        if (!heartbeatRunning) break;

        bool ok = SendPlcHeartbeat(ip.c_str());
        if (ok) {
            failCount = 0;
        } else {
            failCount++;
            std::cerr << "[心跳] 第 " << failCount << " 次失败"
                      << " (容忍 " << MAX_CONSECUTIVE_FAILS << " 次)" << std::endl;
            if (failCount >= MAX_CONSECUTIVE_FAILS) {
                std::cerr << "[心跳] 连续失败, PLC 断开" << std::endl;
                heartbeatRunning = false;
                break;
            }
        }
    }
}

/* =====================================================================
 * PLC 状态诊断（单次快照）
 * ===================================================================== */
static void ShowPlcStatus()
{
    std::cout << "\n"
              << "=========================================================\n"
              << "  PLC 状态诊断\n"
              << "=========================================================\n";

    bool conn = CheckPlcConnection();
    std::cout << "  连接状态:        " << (conn ? "正常" : "断开/异常") << "\n";

    bool hbStat = CheckHeartbeatStatus();
    std::cout << "  心跳状态:        " << (hbStat ? "正常" : "异常") << "\n";

    int hbCnt = ReadPlcHeartbeatMonitor();
    std::cout << "  PLC 心跳计数:    " << hbCnt << "\n";

    bool localHb = GetPlcHeartbeat();
    std::cout << "  本地方心跳:      " << (localHb ? "正常" : "异常") << "\n";

    double h = GetActualLiftHeight();
    std::cout << "  吊钩实际高度:    " << std::fixed << std::setprecision(2)
              << h << " m\n";

    int onStat = 0, offStat = 0;
    GetGripperOnStatus(&onStat);
    GetGripperOffStatus(&offStat);
    std::cout << "  抓钩夹紧到位:    " << (onStat ? "是" : "否") << "\n";
    std::cout << "  抓钩释放到位:    " << (offStat ? "是" : "否") << "\n";

    std::cout << "=========================================================\n"
              << std::endl;
}

/* =====================================================================
 * PLC 持续监控 —— 每秒刷新，回车停止
 * ===================================================================== */
static void MonitorPlcStatus()
{
    std::cout << "\nPLC 状态监控中 (每秒刷新, 按回车停止)...\n" << std::endl;

    std::atomic<bool> stop{false};
    std::thread t([&stop]() {
        while (!stop) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (stop) break;
            std::cout << "\r"
                      << "  连接:" << (CheckPlcConnection()   ? "OK" : "FAIL")
                      << " | 心跳:" << (CheckHeartbeatStatus() ? "OK" : "FAIL")
                      << " | 心跳计数:" << std::setw(6) << ReadPlcHeartbeatMonitor()
                      << " | 高度:" << std::fixed << std::setprecision(2)
                      << GetActualLiftHeight() << "m"
                      << "  [回车停止]  " << std::flush;
        }
        std::cout << std::endl;
    });

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
    stop = true;
    t.join();
}

/* =====================================================================
 * 菜单
 * ===================================================================== */
static void DispMainMenu()
{
    std::cout << "\n"
              << "---------------------------------------------------------\n"
              << "              linux -> snap7 -> plc                     \n"
              << "---------------------------------------------------------\n"
              << "  控制:\n"
              << "    1. big car ++         2. big car --\n"
              << "    3. small car ++       4. small car --\n"
              << "    5. hoist 0.6m         6. hoist 0.7m\n"
              << "    7. 抓钩夹紧           8. 抓钩释放\n"
              << "    9. STOP ALL (仅停运动, 不动抓钩)\n"
              << "   10. 复位(模拟遥控复位键)\n"
              << "  诊断:\n"
              << "   11. PLC 状态 (单次)   12. PLC 持续监控\n"
              << "---------------------------------------------------------\n"
              << "    0. Exit\n"
              << "---------------------------------------------------------\n"
              << "   >>> SELECT MENU : ";
}

/* =====================================================================
 * main
 * ===================================================================== */
int main()
{
    std::string ip = "192.168.0.1";

    int ret = connect_to_plc(ip.c_str());
    if (ret != 0) {
        std::cerr << "PLC 连接失败, 返回值=" << ret << std::endl;
        return 1;
    }
    std::cout << "PLC 连接成功, 启动后台心跳 (" << HEARTBEAT_INTERVAL_MS
              << "ms, " << (1000 / HEARTBEAT_INTERVAL_MS) << "Hz)..." << std::endl;

    std::thread heartbeat(heartbeatLoop, ip);

    while (true) {
        if (!heartbeatRunning) {
            std::cerr << "主程序退出（心跳断开）" << std::endl;
            break;
        }

        DispMainMenu();

        int no = 0;
        std::cin >> no;
        std::cout << std::endl;

        switch (no) {
        case 0:
            std::cout << "退出" << std::endl;
            heartbeatRunning = false;
            heartbeat.join();
            disconnect_plc();
            return 0;

        case 1:
            BigCarCtrl(0.1, ip.c_str(), 0x047F);
            std::cout << "send big car +0.1" << std::endl;
            break;

        case 2:
            BigCarCtrl(-0.1, ip.c_str(), 0x047F);
            std::cout << "send big car -0.1" << std::endl;
            break;

        case 3:
            SmallcarCtrl(0.1, ip.c_str(), 0x047F);
            std::cout << "send small car +0.1" << std::endl;
            break;

        case 4:
            SmallcarCtrl(-0.1, ip.c_str(), 0x047F);
            std::cout << "send small car -0.1" << std::endl;
            break;

        case 5:
            liftctrl(0.6, ip.c_str());
            std::cout << "send hoist 0.6m" << std::endl;
            break;

        case 6:
            liftctrl(0.7, ip.c_str());
            std::cout << "send hoist 0.7m" << std::endl;
            break;

        case 7:
            GripperClamp(ip);
            std::cout << "send 抓钩夹紧 (电平保持)" << std::endl;
            break;

        case 8:
            GripperRelease(ip);
            std::cout << "send 抓钩释放 (电平保持)" << std::endl;
            break;
            
        case 9:
            /* 只停运动, 不改抓钩位: 避免急停时松开抓钩掉载 */
            BigCarCtrl(0, ip.c_str(), 0x047F);
            SmallcarCtrl(0, ip.c_str(), 0x047F);
            liftctrl(0, ip.c_str());
            std::cout << "STOP ALL (运动已停, 抓钩状态保持)" << std::endl;
            break;
            
        case 10:
            PlcResetPulse(ip);
            std::cout << "send 复位脉冲 (模拟遥控复位键)" << std::endl;
            break;

        case 11:
            ShowPlcStatus();
            break;

        case 12:
            MonitorPlcStatus();
            break;

        default:
            std::cout << "无效选项" << std::endl;
            break;
        }
    }

    heartbeatRunning = false;
    heartbeat.join();
    disconnect_plc();
    return 1;
}
