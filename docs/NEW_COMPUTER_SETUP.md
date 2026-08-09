# 新电脑迁移与环境安装

本文用于把仓库部署到另一台操作电脑。不要复制现有 Conda 环境目录；
使用仓库根目录的 `environment.gello-upper-body-teleop.yml` 重新创建环境。

## 1. 适用平台

- 推荐使用 x86-64 Ubuntu 22.04。Docker 镜像基于 ROS 2 Humble，仓库内的
  XRoboToolkit 和 MANUS 原生库也是 Linux x86-64 二进制。
- ARM、Windows、macOS 或其他架构不能直接使用仓库内的原生 `.so`，需要从
  对应 SDK 源码重新编译或取得供应商提供的匹配版本。
- 安装 Git、Miniforge/Conda、Docker Engine、Docker Compose 插件、CMake、
  C++ 编译器和 `lsof`。主机不需要单独安装 ROS 2，机器人侧 ROS 在 Docker
  容器中运行。

## 2. 拉取仓库

```bash
git clone <仓库地址> gello_upper_body_teleop
cd gello_upper_body_teleop
```

本仓库当前没有 Git submodule 或 Git LFS 文件。普通 `git clone` 应取得仓库
内的 XRoboToolkit、MANUS SDK、机器人配置、URDF 和标定文件。

## 3. 创建 Conda 环境

```bash
conda env create -f environment.gello-upper-body-teleop.yml
conda activate gello-upper-body-teleop
```

安装仓库内的 Python 包：

```bash
python -m pip install ./third_party/xrobotoolkit_sdk
python -m pip install \
  -e ./ros_ws/src/teleop_core \
  -e ./teleop_sources/pico \
  -e ./teleop_sources/vive \
  -e ./teleop_sources/manus/python
```

环境文件只声明可从 Conda 获取的基础包。上述本地包必须在克隆仓库后安装，
其中 editable 安装会指向当前仓库路径；移动或删除仓库后需要重新安装。

当前 GUI 启动脚本使用 Conda `base` 环境的 Python，而不是
`gello-upper-body-teleop`，因此还需执行：

```bash
conda install -n base -c conda-forge pyside6
```

不要在创建此新环境后再运行 `scripts/setup_pico_env.sh`。该旧脚本读取
`teleop_sources/pico/environment.yml`，其中仍保留旧环境名和较少的依赖，
更新时使用 `--prune`，可能删除本文件新增的测试依赖。

## 4. 单独准备 GELLO 驱动

`gello_software` 不在本仓库中，必须单独克隆。不要复制旧环境中指向原电脑
路径的 editable 安装。

```bash
git clone https://github.com/wuphilipp/gello_software.git /path/to/gello_software
git -C /path/to/gello_software submodule update --init \
  third_party/DynamixelSDK

GELLO_SOFTWARE_ROOT=/path/to/gello_software \
  ./scripts/setup_gello_driver.sh
```

验证：

```bash
conda run -n gello-upper-body-teleop python -c \
  "from gello.dynamixel.driver import DynamixelDriver; print('GELLO driver ready')"
```

## 5. 构建不能由 Git 传输的产物

以下内容必须在新电脑重新生成：

```bash
cp docker/.env.example docker/.env
./scripts/build.sh
./teleop_sources/manus/scripts/build.sh
```

- `docker/.env` 被 Git 忽略，必须手动创建。
- `ros_ws/build`、`ros_ws/install`、`ros_ws/log` 和 MANUS `build/` 产物被
  Git 忽略，不能依赖旧电脑上传。
- Docker 镜像和容器也不在 Git 中，`scripts/build.sh` 会在新电脑重新构建。
- `teleop_sources/manus/config/Calibration_left.mcal` 和
  `Calibration_right.mcal` 已随 Git 管理；仍应确认它们与实际手套和操作员
  标定相匹配。

## 6. 修改新电脑的本地配置

打开 `docker/.env`，至少检查：

```text
TELEOP_DATA_ROOT=/path/to/franka_teleop_data
TELEOP_ROS_DOMAIN_ID=0
FRANKA_CPUSET=16-23
FRANKA_ROBOT_CONFIG=/workspace/franka_upper_body_teleop/config/current_workcell.yaml
```

注意：

- `TELEOP_DATA_ROOT` 是主机路径，目录需提前创建且当前用户可写。
- `FRANKA_CPUSET` 必须是新电脑实际存在的 CPU 编号；CPU 较少时不能照抄
  `16-23`。用 `lscpu -e` 查看后再设置。
- `FRANKA_ROBOT_CONFIG` 是容器内路径。Compose 会把当前仓库挂载为
  `/workspace/franka_upper_body_teleop`，这里不需要改成主机仓库名。
- `scripts/run_teleop.sh` 当前默认把诊断写到
  `/home/descfly/franka_teleop_data/diagnostics`。新电脑应在启动前设置
  `TELEOP_RUN_DIR` 为一个尚不存在的可写目录，或者先按新电脑用户名调整
  脚本中的默认路径。

例如：

```bash
export TELEOP_RUN_DIR="$HOME/franka_teleop_data/diagnostics/$(date +%Y%m%d_%H%M%S)"
```

## 7. 硬件和系统权限

### Docker

确保 Docker daemon 已启动，并让当前用户具备运行 Docker 的权限。用户组
变更后需要完整注销并重新登录。

### GELLO 串口

将操作用户加入 `dialout`：

```bash
sudo usermod -aG dialout "$USER"
```

完整注销并重新登录后再检查权限。`config/gello.yaml` 使用稳定的
`/dev/serial/by-id/...` 和设备序列号。连接硬件后执行：

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python scripts/check_gello_ports.py --config config/gello.yaml
```

如果使用的是同一对 USB 转换器，序列号通常不变。如果更换了转换器，应按
实际 `/dev/serial/by-id/` 更新配置并重新确认左右侧。不要根据 `ttyUSB0` /
`ttyUSB1` 的枚举顺序推断左右侧，也不要为绕过检查而降低串口、新鲜度、
跳变、限位或速度保护。

### Franka FR3

- 配置新电脑到机器人网段，核对 `config/current_workcell.yaml` 中的机器人
  IP 与实际设备一致。
- 确保 Franka FCI 已授权并解锁，主机到两台机器人网络可达。
- 实时性能、CPU 隔离和网卡设置属于新电脑系统配置，不会由 Git 或 Conda
  自动迁移。

### MANUS、灵巧手和可选追踪器

- 单独安装并启动与硬件匹配的 MANUS Core/运行时，完成手套配对。
- 确认 G20、O30i 的 USB/CAN/CAN-FD 设备权限和供应商运行库。
- 使用 VIVE 时需单独安装 SteamVR/OpenVR 运行时。
- 使用 PICO 时需单独安装并启动 XRoboToolkit PC Service。仓库内只包含
  Python binding 和 Linux x86-64 SDK 库。
- 不要同时启动两个 PICO 客户端、两个 MANUS 客户端，或同时运行多个会占用
  相同 UDP 端口的 bridge。

## 8. 数据不会随 Git 自动迁移

`TELEOP_DATA_ROOT` 下的录制数据、诊断日志和历史回放数据不在仓库中。若新
电脑需要这些内容，应单独使用硬盘、`rsync` 或其他受控方式迁移，并保留原
目录权限和足够磁盘空间。不要把大型数据或设备凭据直接提交到 Git。

## 9. 安装后验证

先运行不连接机器人命令输出的 Python 测试：

```bash
conda run -n gello-upper-body-teleop \
  pytest -q teleop_sources/pico/tests teleop_sources/vive/tests

conda run -n gello-upper-body-teleop \
  pytest -q ros_ws/src/teleop_core/test
```

然后检查环境：

```bash
conda run -n gello-upper-body-teleop python -c \
  "import numpy, scipy, pinocchio, mujoco, gello; print('Python environment ready')"

conda run -n base python -c \
  "import PySide6; print('GUI environment ready')"
```

硬件接好但保持机器人未 Engage 时执行：

```bash
./scripts/preflight.sh
```

`preflight.sh` 不发送机器人、灵巧手或 GELLO 目标命令，但会检查 Docker、
构建产物、GUI、GELLO 串口和配置。

## 10. 首次硬件启动安全要求

- 清空双臂工作区，急停保持可触及；GUI Disengage 不是物理断电。
- 系统应以两侧未 Engage 状态启动。
- 首次启动先观察 GELLO 实时读数和左右映射，再逐侧低速验证。
- 确认日志出现两侧碰撞阈值已接受，以及接触力矩门控已启用。
- GUI 丢失、输入过期、机器人状态过期或安全网关拒绝时必须保持自动
  Disengage；不要修改安全检查来消除报错。
- 只有安全网关可以发布 FR3 命令总线。
- Home、回放和手部诊断动作可能移动硬件。发生 reflex 或原因不明的故障后，
  先保存日志再停止服务。

完整启动和停机步骤见 [GELLO_TELEOP.md](GELLO_TELEOP.md) 与
[HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md)。
