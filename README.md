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

## 功能与启动入口

| 模式 | 机械臂 | 灵巧手 | 启动命令 |
| --- | --- | --- | --- |
| 标准遥操（默认） | 双 GELLO -> 双 FR3 | MANUS -> 双 O30i | `./scripts/start_teleop.sh` |
| 仅机械臂 | 双 GELLO -> 双 FR3 | 不启动 | `./scripts/run_gello_arms_only.sh` |
| 手动网页 UI | 不启动 | 网页 UI -> O30i/G20 安全桥接 | `./scripts/run_hand_ui.sh` |
| Wuji 遥操 | 双 GELLO -> 双 FR3 | MANUS -> Wuji | `./scripts/start_wuji_teleop.sh ...` |
| MoveIt 假硬件 | MoveIt -> 双虚拟 FR3 | 不启动 | `./scripts/run_moveit.sh --fake` |
| MoveIt 真机 | MoveIt -> 双 FR3 | 不启动 | `./scripts/run_moveit.sh --real` |

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
./scripts/build.sh
./scripts/setup_pico_env.sh
teleop_sources/manus/scripts/build.sh
GELLO_SOFTWARE_ROOT=/home/descfly/llx/gello_software \
  ./scripts/setup_gello_driver.sh
```

当前机器使用 Conda 环境 `gello-upper-body-teleop`。GELLO 的串口、舵机 ID、
符号和方向修正复用 `/home/descfly/llx/gello_franka` 的标定；硬件和装配未变化
时不需要重新标定。

如需使用 Wuji，再安装其可选依赖：

```bash
./scripts/setup_wuji_env.sh
```

## 启动前检查

标准模式、Wuji 模式和 MoveIt 真机模式会自动运行 preflight，也可以单独执行：

```bash
./scripts/preflight.sh
```

仅检查机械臂环境：

```bash
./scripts/preflight.sh --arms-only
```

preflight 检查 Docker、Compose、ROS workspace、Conda、GUI、GELLO 端口以及
MANUS 标定文件；检查过程不会发送机器人、灵巧手或 GELLO 电机命令。

## 标准遥操：GELLO + O30i

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./scripts/start_teleop.sh
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
./scripts/run_robot_stack.sh
```

终端 2：

```bash
./scripts/run_operator.sh
```

确认 `franka-control` 已接受碰撞阈值、`teleop-control` 显示 contact torque
gating active，且 GUI 状态正常后再 Engage。

## 仅启动 GELLO 双臂

```bash
./scripts/run_gello_arms_only.sh
```

该入口仅启动 `franka-control`、`teleop-control` 和 `gello-bridge`，并以
`--hand-source none` 运行 Operator。它会先停止可能残留的 `hand-control`，
避免旧会话继续控制 O30i/G20。

## O30i/G20 手动网页 UI

```bash
./scripts/run_hand_ui.sh
```

浏览器打开 <http://127.0.0.1:8080>。Compose 当前默认连接双 O30i；UI
也保留 G20 映射支持。该模式只启动 `hand-control` 和 `hand-ui`，不启动 FR3。

详细说明见 [integrations/hand_ui/README.md](integrations/hand_ui/README.md)。

## Wuji 手遥操

O30i 仍是标准入口的默认方案。使用 Wuji 时需走独立启动脚本，并显式提供
Wuji Hand 2 的左右地址，防止网络发现选错手：

```bash
./scripts/start_wuji_teleop.sh \
  --wuji-left-address 192.168.1.111:50001 \
  --wuji-right-address 192.168.1.112:50001
```

Wuji 模式继续使用双 GELLO 控制 FR3，使用现有 MANUS bridge 和 Operator GUI
控制 Wuji 手。退出时会停止机械臂侧服务并尝试 disable、断开 Wuji 设备。

原版 USB Wuji Hand、单侧运行和电流/控制增益参数见
[integrations/wuji/README.md](integrations/wuji/README.md)。首次真机验证应采用低电流、
单手、单侧 Engage。

## 双 FR3 MoveIt

先用假硬件验证规划和 RViz：

```bash
./scripts/run_moveit.sh --fake
```

确认模型、规划组和控制器均正常后，才使用真机模式：

```bash
./scripts/run_moveit.sh --real
```

当前 Compose 中的真机地址与 `config/current_workcell.yaml` 一致：左侧
`172.16.0.3`、右侧 `172.16.0.2`。修改工作站地址时需要同步更新两处。MoveIt
与正常 GELLO 安全网关是两套独立的机械臂控制模式，不能同时运行。

详细说明见 [docs/MOVEIT.md](docs/MOVEIT.md)。

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

- `config/gello.yaml`：左右 GELLO 身份、方向、`1.5 rad` 增量范围和
  `0.5 rad/s` GELLO 专用限速。
- `config/current_workcell.yaml`：双 FR3 地址、arm ID 和命名空间。
- `config/teleop_control.yaml`：ROS 安全网关、限速和接触力矩 gating。
- `config/pico.yaml`：共享 UDP 与控制周期配置；tracker 兼容入口仍保留。
- `docker/compose.yaml`：O30i、MoveIt、ROS 服务及硬件容器配置。

## 更多文档

- [硬件部署与故障处理](docs/HARDWARE_DEPLOY.md)
- [GELLO 控制语义](docs/GELLO_TELEOP.md)
- [MoveIt 模式](docs/MOVEIT.md)
- [手动灵巧手 UI](integrations/hand_ui/README.md)
- [Wuji 手集成](integrations/wuji/README.md)
- [第三方代码与许可证](THIRD_PARTY_NOTICES.md)
