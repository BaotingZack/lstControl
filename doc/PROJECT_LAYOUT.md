# 项目目录说明

本仓库是**单包根运行**（`python main.py` 在项目根执行），Python 路径以根目录为准。新代码请按下面分区放置；根目录仅保留入口、配置与**兼容 shim**。

## 顶层一览

```
lstControl/
├── main.py                 # 入口：仿真 / Live Web / PLC
├── site_config.json        # 现场倾角仪、防摇、判稳参数（运行时加载）
├── site_config.py          # JSON 字段白名单与加载
├── pyproject.toml          # 工具链（ruff 等）
├── requirements*.txt
│
├── web/                    # 浏览器 Live 服务与页面（推荐 import web.*）
├── anti_sway/              # 可插拔防摇 / 消摆算法
├── doc/                    # 设计与说明文档
├── assets/                 # 静态图片等资源
├── tests/                  # pytest
├── tools/                  # 一次性维护脚本（非运行时）
├── 防摇方案/                 # 外部参考工程（只读对照，不参与 import）
│
├── crane_model.py          #  plant、PD 主循环 run_pd_control
├── settle_control.py       #  消摆 profile 封装
├── pd_controller.py        #  PD 核心（若被调度复用）
├── operation_scheduler.py  #  取放货调度
├── plc_interface.py        #  PLC / 真机执行器
├── ros_bridge.py           #  ROS 定位与倾角
├── coordinate_transform.py #  地图 ↔ 起重机坐标
├── calibration.py          #  标定
├── visualizer.py           #  命令行仿真出图
├── sim_inclinometer.py     #  仿真倾角
│
└── live_*.py, sway_*.py …  # 兼容 shim → 见下文
```

## `web/` — Live 与测试平台 UI

| 文件 | 作用 |
|------|------|
| `server.py` | HTTP 服务、API（goto / settle / control-state）、SSE |
| `operation_html.py` | 原三维标定 / 作业 Live 页 |
| `ptp_html.py` | 防摇测试平台（点到点） |
| `settle_html.py` | 消摆测试页 |
| `platform_nav.py` | 子页导航条 |
| `view_shared.py` | 共用 CSS / Canvas JS |
| `ptp_sway_canvas.py` | Hook Sway 画布脚本 |
| `run_log.py` | 运行日志 JSON 组装与 `logs/` 落盘 |
| `run_log_ui.py` | PTP / 消摆页「保存运行日志」按钮脚本 |

点到点或消摆**结束后**，侧栏「保存运行日志」会 POST `/api/run-log/save`：浏览器下载 JSON，同时在项目根 `logs/` 留一份副本（已加入 `.gitignore`）。

**导入约定**：新代码使用 `from web.server import serve_live_view` 等。  
根目录 `live_server.py` 等仅为旧 import 保留，内部 `from web.* import *`。

## `anti_sway/` — 防摇与消摆算法

| 文件 | 作用 |
|------|------|
| `registry.py` | 注册表；`build_strategy`；PTP / settle 选项列表 |
| `base.py` | `AntiSwayStepContext`、`correct_xy` 协议 |
| `ptp_config.py` | PTP 模式与 `site_config` 合并 |
| `incl.py` / `gantry.py` / `overhead.py` | **点到点**三种参考实现 |
| `baseline.py` / `crane_remote.py` / `gantry_settle.py` / `overhead_settle.py` | **消摆**专用 |
| `estimator.py` / `sim_feedback.py` / `rope.py` / `lqr_math.py` | θ 估计、仿真摆、绳长、LQR |
| `README.md` | 如何新增一种算法 |

控制环只在 `crane_model.run_pd_control` 里调用 `build_strategy` + `correct_xy`，**不要**在 `crane_model` 里按算法名分支。

## `doc/` — 文档

| 路径 | 内容 |
|------|------|
| `ptp_anti_sway.md` | PTP 防摇方案与参考对照 |
| `notes/` | 历史方案说明、杂项（原根目录 `方案*.md`、`说明`） |
| `PROJECT_LAYOUT.md` | 本文件 |

## `assets/` — 静态资源

仿真/说明用 PNG 等（原根目录 `crane_*.png`）。

## `tools/` — 维护脚本

以 `_` 开头的脚本用于拆分/重建 HTML 等**一次性**操作，不参与 `main.py` 运行。见 `tools/README.md`。

## `防摇方案/` — 外部参考

`CraneRemote-master`、`gantry-crane-anti-sway-control-main`、`OverheadCraneSwayControl-main` 等，**仅对照阅读**，Python 运行时不会 import 此目录。

## 根目录兼容 shim（逐步淘汰）

| Shim | 指向 |
|------|------|
| `live_*.py` | `web/` 对应模块 |
| `sway_estimator.py` | `anti_sway.estimator` |
| `sim_sway_feedback.py` | `anti_sway.sim_feedback` |
| `sway_controller.py` 等 | 旧名 re-export，新代码请 `import anti_sway` |

## 勿提交 / 可清理

- `__pycache__/`、`.pytest_cache/`（已在 `.gitignore`）
- `antisway/`：若仅有 `__pycache__` 无 `.py`，为历史实验残留，可整目录删除

## 新增代码放哪？

| 需求 | 位置 |
|------|------|
| 新防摇 / 消摆算法 | `anti_sway/<name>.py` + `registry.py` |
| 新 Live 页或 API | `web/server.py` + `web/*_html.py` |
| 新现场 JSON 字段 | `site_config.py` 白名单 + `site_config.json` |
| 调度 / PLC / 模型 | 根目录对应模块（或后续再抽 `control/` 包） |
