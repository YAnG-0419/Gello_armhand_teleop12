# GELLO Upper Body Teleop

本项目用于双 GELLO 增量遥操双 Franka FR3，并通过 MANUS 手套控制双侧
灵巧手。默认且保留的手部方案是双 O30i；Wuji、手动网页 UI 和 MoveIt
均为独立的可选模式，不会替换原有 O30i 控制链。

默认数据链路：

```text
双 GELLO -> 增量关节映射 -> UDP -> ROS 安全网关 -> 双 FR3
双 MANUS -> 手部重定向 -> LinkerHand 安全桥接 ------> 双 O30i
```

默认模式不启动相机或数据记录。

## 代码结构

```text
gello_upper_body_teleop/
├── teleop_runtime/                 统一 Operator 后端
│   └── cli.py                      组合机械臂、手、任务策略和控制 GUI 协议
├── adapters/                       输入设备与灵巧手适配层
│   ├── gello/                      GELLO 稳定入口
│   ├── pico/                       PICO 输入、共享遥操核心、仿真与测试
│   ├── vive/                       VIVE Tracker 输入、标定与测试
│   ├── manus/                      MANUS bridge、手部重定向、标定和工具
│   └── wuji/                       Wuji 重定向及 Wuji/Wuji Hand 2 后端
├── apps/                           面向操作员的交互应用
│   ├── operator_gui/               桌面 Operator GUI
│   └── hand_ui/                    O30i/G20 手动网页 UI
├── tasks/                          特殊任务，不改变默认遥操行为
│   └── powderweighing/             称粉手势策略、姿态配置和测试
├── config/                         仓库级配置
│   ├── workcell/                   FR3 地址与工作站拓扑
│   ├── modes/                      GELLO/PICO/VIVE/安全网关运行参数
│   └── calibration/                操作员和硬件标定结果
├── ros_ws/src/                     ROS 2 workspace 源码
│   ├── teleop_interfaces/          ArmCommand 等 ROS 消息契约
│   ├── teleop_core/                FR3 安全网关与统一 topic/joint contract
│   ├── pico_teleop_bridge/         UDP 输入到 ArmCommand 的 ROS bridge
│   ├── franka_fr3_arm_controllers/ 双 FR3 控制器和 Home 服务
│   ├── linker_hand_bridge/         手部命令校验、watchdog 与 slew limiter
│   ├── linker_hand_ros2_sdk/       O30i/G20 ROS 硬件节点
│   ├── lychee_fr3_description/     双 FR3 描述、网格和 RViz 配置
│   ├── lychee_fr3_moveit_config/   双 FR3 MoveIt 配置
│   └── teleop_data/                数据记录、回放与 LeRobot 导出
├── ops/                            日常运维入口
│   ├── run/                        启动遥操、Wuji、UI、MoveIt 和 preflight
│   ├── setup/                      环境、Docker/ROS、GELLO 驱动安装
│   └── diagnostics/                GELLO、O30i 和数据诊断工具
├── vendor/                         跨模块复用的第三方 SDK
│   ├── manus_sdk/
│   └── xrobotoolkit_sdk/
├── assets/                         LinkerHand 模型及重定向资源
├── docker/                         镜像、Compose 服务和本机环境模板
└── docs/                           部署、硬件、MoveIt 和交接文档
```

主要连接关系：

```text
ops/run ──> teleop_runtime + apps/operator_gui
                  │
                  ├──> adapters/gello|pico|vive ──UDP──> pico_teleop_bridge
                  ├──> adapters/manus ────────────────> linker_hand_bridge
                  ├──> adapters/wuji（仅显式选择 Wuji 时加载）
                  └──> tasks/powderweighing（仅传入任务配置时加载）

pico_teleop_bridge ──ArmCommand──> teleop_core 安全网关 ──> 双 FR3
linker_hand_bridge ──校验/限速后的命令──────────────────> O30i/G20
```

`apps` 只保存 UI，`tasks` 只保存特殊任务，硬件差异统一封装在 `adapters`。
所有机械臂来源最终都使用 `teleop_interfaces/ArmCommand`；只有
`teleop_core` 安全网关可以写入 FR3 命令总线。`ros_ws/build`、
`ros_ws/install`、`ros_ws/log` 以及各 Python `__pycache__` 都是生成物，不属于
源码结构，也不应提交到 Git。

## 功能与启动入口

| 模式 | 机械臂 | 灵巧手 | 启动命令 |
| --- | --- | --- | --- |
| 标准遥操（默认） | 双 GELLO -> 双 FR3 | MANUS -> 双 O30i | `./ops/run/start_teleop.sh` |
| 仅机械臂 | 双 GELLO -> 双 FR3 | 不启动 | `./ops/run/run_gello_arms_only.sh` |
| 手动网页 UI | 不启动 | 网页 UI -> O30i/G20 安全桥接 | `./ops/run/run_hand_ui.sh` |
| Wuji 遥操 | 双 GELLO -> 双 FR3 | MANUS -> Wuji | `./ops/run/start_wuji_teleop.sh ...` |
| MoveIt 假硬件 | MoveIt -> 双虚拟 FR3 | 不启动 | `./ops/run/run_moveit.sh --fake` |
| MoveIt 真机 | MoveIt -> 双 FR3 | 不启动 | `./ops/run/run_moveit.sh --real` |

## 安全约束

- GELLO/Operator 遥操模式启动后应保持双侧未 Engage，先测试一侧的小幅运动，
  再测试双侧。
- MoveIt 真机模式应先确认关节状态和规划结果，只执行小范围测试轨迹。
- GELLO 遥操与 MoveIt 不能同时控制 FR3；启动脚本会检查并拒绝冲突服务。
- Wuji 模式不会启动 `hand-control`，且会拒绝与 O30i/G20 服务同时运行。
- 手动网页 UI 通过 LinkerHand bridge 的范围校验、命令新鲜度 watchdog 和
  slew limiter，不直接写入厂商命令总线。
- 手动网页 UI 与正常 Operator 后端不能同时作为手部命令源；启动脚本会检查
  Operator 控制端口。
- 真机启动前确保工作区无人、急停可触及、Franka FCI 已解锁。

## 首次安装

```bash
cd /home/descfly/llx/gello_upper_body_teleop
cp docker/.env.example docker/.env
./ops/setup/build.sh
./ops/setup/setup_teleop_env.sh
adapters/manus/scripts/build.sh
GELLO_SOFTWARE_ROOT=/home/descfly/llx/gello_software \
  ./ops/setup/setup_gello_driver.sh
```

当前机器使用 Conda 环境 `gello-upper-body-teleop`。GELLO 的串口、舵机 ID、
符号和方向修正复用 `/home/descfly/llx/gello_franka` 的标定；硬件和装配未变化
时不需要重新标定。

如需使用 Wuji，再安装其可选依赖：

```bash
./ops/setup/setup_wuji_env.sh
```

## 启动前检查

标准模式、Wuji 模式和 MoveIt 真机模式会自动运行 preflight，也可以单独执行：

```bash
./ops/run/preflight.sh
```

仅检查机械臂环境：

```bash
./ops/run/preflight.sh --arms-only
```

preflight 检查 Docker、Compose、ROS workspace、Conda、GUI、GELLO 端口以及
MANUS 标定文件；检查过程不会发送机器人、灵巧手或 GELLO 电机命令。

## 标准遥操：GELLO + O30i

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./ops/run/start_teleop.sh
```

这是推荐且默认的入口。脚本依次启动：

```text
franka-control + teleop-control + gello-bridge + hand-control
                                     |
                                     +-> Operator 后端和 GUI
```

当前 `hand-control` 的左右手型号均为 `o30i`。关闭 GUI、Operator 后端退出或
按 `Ctrl-C` 后，脚本会停止它启动的机器人侧服务。

### 两个终端启动

需要持续观察 ROS 和 Franka 日志时，可分开启动。

终端 1：

```bash
./ops/run/run_robot_stack.sh
```

终端 2：

```bash
./ops/run/run_operator.sh
```

确认 `franka-control` 已接受碰撞阈值、`teleop-control` 显示 contact torque
gating active，且 GUI 状态正常后再 Engage。

## 仅启动 GELLO 双臂

```bash
./ops/run/run_gello_arms_only.sh
```

该入口仅启动 `franka-control`、`teleop-control` 和 `gello-bridge`，并以
`--hand-source none` 运行 Operator。它会先停止可能残留的 `hand-control`，
避免旧会话继续控制 O30i/G20。

## O30i/G20 手动网页 UI

```bash
./ops/run/run_hand_ui.sh
```

浏览器打开 <http://127.0.0.1:8080>。Compose 当前默认连接双 O30i；UI
也保留 G20 映射支持。该模式只启动 `hand-control` 和 `hand-ui`，不启动 FR3。

详细说明见 [apps/hand_ui/README.md](apps/hand_ui/README.md)。

## Wuji 手遥操

O30i 仍是标准入口的默认方案。使用 Wuji 时需走独立启动脚本，并显式提供
Wuji Hand 2 的左右地址，防止网络发现选错手：

```bash
./ops/run/start_wuji_teleop.sh \
  --wuji-left-address 192.168.1.111:50001 \
  --wuji-right-address 192.168.1.112:50001
```

Wuji 模式继续使用双 GELLO 控制 FR3，使用现有 MANUS bridge 和 Operator GUI
控制 Wuji 手。退出时会停止机械臂侧服务并尝试 disable、断开 Wuji 设备。

无需连接硬件即可测试正确的 `hand2_beta` 模型：

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right
```

原版 USB Wuji Hand、单侧运行和电流/控制增益参数见
[adapters/wuji/README.md](adapters/wuji/README.md)。首次真机验证应采用低电流、
单手、单侧 Engage。

## 双 FR3 MoveIt

先用假硬件验证规划和 RViz：

```bash
./ops/run/run_moveit.sh --fake
```

确认模型、规划组和控制器均正常后，才使用真机模式：

```bash
./ops/run/run_moveit.sh --real
```

当前 Compose 中的真机地址与 `config/workcell/current.yaml` 一致：左侧
`172.16.0.3`、右侧 `172.16.0.2`。修改工作站地址时需要同步更新两处。MoveIt
与正常 GELLO 安全网关是两套独立的机械臂控制模式，不能同时运行。

详细说明见 [docs/MOVEIT.md](docs/MOVEIT.md)。

## 特殊任务

特殊任务放在 `tasks/` 下，不属于通用遥操控制链。当前的称粉任务位于
`tasks/powderweighing`，只在显式传入任务配置时包装右手 MANUS/O30i
重定向策略：

```bash
./ops/run/run_operator.sh \
  --right-hand-strategy-config tasks/powderweighing/config/poses.json
```

不传该参数时，默认双 O30i/Sharpa 行为不变。标定、独立调试和触发参数见
[tasks/powderweighing/README.md](tasks/powderweighing/README.md)。

## 停止与故障日志

正常停止顺序：

1. 在 GUI 中执行 `DISENGAGE ALL`。
2. 关闭 GUI 或在启动终端按 `Ctrl-C`。
3. 两终端模式下，两个终端都按 `Ctrl-C`。
4. 确认 Compose 服务停止后关闭 Franka FCI。

发生 reflex、碰撞保护或其他异常时，在停止 Compose 前保存日志：

```bash
cd docker
docker compose logs franka-control teleop-control \
  > /tmp/franka_teleop_fault.log
```

查看当前服务：

```bash
cd docker
docker compose ps
```

## 关键配置

- `config/modes/gello.yaml`：左右 GELLO 身份、方向、`1.5 rad` 增量范围和
  `0.5 rad/s` GELLO 专用限速。
- `config/workcell/current.yaml`：双 FR3 地址、arm ID 和命名空间。
- `config/modes/teleop_control.yaml`：ROS 安全网关、限速和接触力矩 gating。
- `config/modes/pico.yaml`：共享 UDP 与控制周期配置；tracker 兼容入口仍保留。
- `docker/compose.yaml`：O30i、MoveIt、ROS 服务及硬件容器配置。

## 更多文档

- [硬件部署与故障处理](docs/HARDWARE_DEPLOY.md)
- [GELLO 控制语义](docs/GELLO_TELEOP.md)
- [MoveIt 模式](docs/MOVEIT.md)
- [手动灵巧手 UI](apps/hand_ui/README.md)
- [Wuji 手集成](adapters/wuji/README.md)
- [Powder weighing 特殊任务](tasks/powderweighing/README.md)
- [第三方代码与许可证](THIRD_PARTY_NOTICES.md)
