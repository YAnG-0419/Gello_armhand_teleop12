# GELLO Upper Body Teleop

双 GELLO 增量遥操双 Franka FR3，手部保持 MANUS 手套输入：左 G20、右
O30i。方案继承 `franka_upper_body_teleop` 的机器人、安全网关、灵巧手和
GUI，只把手臂输入从 tracker 换成 GELLO。目前不启动相机或数据记录。

```text
GELLO -> 增量关节映射 -> UDP -> ROS 安全网关 -> 双 FR3
MANUS -> 手部重定向 --------------------------> G20 / O30i
```

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

当前机器已经创建 Conda 环境 `gello-upper-body-teleop`。GELLO 的串口、
舵机 ID、符号和方向修正复用 `/home/descfly/llx/gello_franka` 的标定；
硬件与装配不变时无需重新标定。

## 启动前

确保双臂工作区无人、急停可触及、Franka FCI 已解锁，MANUS Core 和两只
手套已就绪。执行：

```bash
./scripts/preflight.sh
```

该检查不发送机器人、灵巧手或 GELLO 目标命令。

## 一条命令启动

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./scripts/start_teleop.sh
```

脚本依次启动双臂控制、安全网关、GELLO bridge、灵巧手 bridge、遥操后端
和 GUI。系统始终以双侧未 Engage 状态启动；关闭 GUI、后端退出或按
`Ctrl-C` 后，脚本会停止机器人侧服务。

## 只启动 GELLO 双臂

不启动 MANUS、G20 或 O30i 手部遥操时，使用：

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./scripts/run_gello_arms_only.sh
```

该入口只启动 `franka-control`、`teleop-control` 和 `gello-bridge`，并以
`--hand-source none` 启动后端和 GUI。为避免旧会话继续控制手部，它还会
停止已有的 `hand-control` 服务。退出时只停止上述双臂服务。

## 推荐：两个终端启动

需要持续观察 ROS/Franka 日志时使用此方式。

终端 1：

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./scripts/run_robot_stack.sh
```

终端 2：

```bash
cd /home/descfly/llx/gello_upper_body_teleop
./scripts/run_operator.sh
```

确认 `franka-control` 接受碰撞阈值、`teleop-control` 显示 contact torque
gating active，GUI 状态正常后，先只 Engage 一侧并做小幅运动，再测试双侧。

## 停止

先在 GUI 中 `DISENGAGE ALL`，再关闭 GUI 或按 `Ctrl-C`。两个终端模式下，
两个终端都按 `Ctrl-C`，最后关闭 Franka FCI。

发生 reflex 或异常时，停止 Compose 前保存日志：

```bash
cd docker
docker compose logs franka-control teleop-control > /tmp/franka_teleop_fault.log
```

## 关键配置

- `config/gello.yaml`：左右 GELLO 身份、方向、`1.5 rad` 增量范围和
  `0.5 rad/s` GELLO 专用限速。
- `config/current_workcell.yaml`：双 FR3 地址与命名空间。
- `config/teleop_control.yaml`：ROS 安全网关。
- `config/pico.yaml`：共享 UDP 和控制周期配置；tracker 兼容入口仍保留。

详细安全与故障流程见 [docs/HARDWARE_DEPLOY.md](docs/HARDWARE_DEPLOY.md)，
GELLO 控制语义见 [docs/GELLO_TELEOP.md](docs/GELLO_TELEOP.md)。
