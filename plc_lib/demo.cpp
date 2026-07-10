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

static std::atomic<bool> heartbeatRunning{true};

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
              << "    7. STOP ALL\n"
              << "  诊断:\n"
              << "    8. PLC 状态 (单次)    9. PLC 持续监控\n"
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
            BigCarCtrl(0, ip.c_str(), 0x047F);
            SmallcarCtrl(0, ip.c_str(), 0x047F);
            liftctrl(0, ip.c_str());
            std::cout << "STOP ALL" << std::endl;
            break;

        case 8:
            ShowPlcStatus();
            break;

        case 9:
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
