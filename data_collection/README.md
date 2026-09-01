# Harvest 数据采集

该子系统只读取安全网关后的机械臂 action、实测状态、Operator 导出的 Wuji
telemetry 和三路相机。在线阶段只写 ROS bag；停止录制不会转换、上传或删除任何
bag。转换必须由操作者另行执行。

第一期只支持 **GELLO 臂 + Wuji 手 + 三相机**。现有
`./ops/run/start_wuji_teleop.sh` 入口不变；采集是独立的只读进程，故障不能阻塞
或接管遥操。

现有 `teleop_data`（O30i/G20 + 单路 `/camera/*`）保持独立，不要把它改成 Harvest
格式，也不要用 Harvest recorder 去订 `/cb_*`。

## 进程边界

```text
终端 1  现有 Operator（唯一 GELLO/MANUS/Wuji client）
  └─ 有界非阻塞队列 → 单向 UDP sender（默认 127.0.0.1:5602）
                         └─ teleop_hand_telemetry（仅 ROS publishers）

现有 safety gateway → /teleop/validated_arm_commands ─┐
双 FR3 实测状态 ───────────────────────────────────────┤
三台 Orbbec（cam0/cam1/cam2）──────────────────────────┼→ ros2 bag record
Wuji ROS telemetry ────────────────────────────────────┘

finalized ROS bag --操作者手动命令--> 临时 LeRobot v2 输出
                                      └─ 完整验证后原子发布
```

谁可以 publish：

| 进程 | 可以 publish | 禁止 |
| --- | --- | --- |
| safety gateway | `/teleop/validated_arm_commands`、FR3 command bus | 第二套控制源 |
| Operator / HandWorker | Wuji SDK 命令；UDP telemetry（本机） | ROS 手/臂命令 |
| `teleop_hand_telemetry` | `/teleop/wuji/{left,right}/command\|joint_states`、status | 任何硬件命令 |
| `teleop_camera_bringup` | `/cam{0,1,2}/color/image_raw`、`/cam0/depth/image_raw` | FR3/Wuji 命令 |
| `teleop_data_collector` | 无（只 `ros2 bag record`） | 任何 robot/hand 命令 |

采集容器不启动 Operator、GELLO/Pico bridge、safety gateway、controller 或任何
Wuji/Pico/MANUS SDK client。杀掉 recorder 或 telemetry receiver 后，控制进程
必须继续运行。

## 部署配置与 Orbbec 许可

先编辑 `data_collection/config/cameras.yaml`，为 cam0/cam1/cam2 填入三个唯一的
序列号和物理语义。任何 `REPLACE_*`、空白或重复 SN 都会阻止启动。不要把真实 SN
提交进 Git。

仓库里的 `ros_ws/src/orbbec_camera` 默认带 `COLCON_IGNORE`，以免 overlay 现有
Docker `orbbec` 单相机服务所用的 vendor 包。Orbbec wrapper 保留 Apache-2.0
LICENSE/NOTICE；SDK 2.8.6 的 LICENSE 和 EULA 位于
`ros_ws/src/orbbec_camera/SDK`。普通构建不会构建 SDK runtime。阅读并接受 EULA
后，必须显式构建：

```bash
./ops/setup/build.sh --accept-orbbec-eula
```

该参数只对本次构建有效。运行时 `start_recording.sh` 会读本机已 gitignore 的
`docker/.env`。在阅读 EULA 并接受后，把 `ORBBEC_SDK_LICENSE_ACCEPTED=YES`
写进该文件即可，不必每次录制前再 `export`。未写入则拒绝启动三相机。

Harvest 三相机 bringup 与 compose `orbbec`、`start_orbbec_viewer.sh` 互斥。
`start_recording.sh` 会拒绝 hand-control、compose `orbbec` 和 OrbbecViewer。

## 遥操与录制

终端一继续使用现有 Wuji 入口。Operator 默认把现有 Wuji 连接的只读 telemetry
发往本机 UDP 5602，采集未启动时不得影响控制：

```bash
ROS_DOMAIN_ID=1 TELEOP_ROS_DOMAIN_ID=1 ./ops/run/start_wuji_teleop.sh \
  --wuji-left-address LEFT_IP:PORT \
  --wuji-right-address RIGHT_IP:PORT
```

终端二在确认本机 `docker/.env` 已有 `ORBBEC_SDK_LICENSE_ACCEPTED=YES` 后启动
独立采集（不要焊进 `start_wuji_teleop.sh`）：

```bash
./ops/run/start_recording.sh \
  --data-root /home/user/franka_teleop_data
```

数据必须写在仓库外。`--data-root` 若落在 Git 仓库内会被拒绝。

- `SPACE`：开始/停止 episode。开始前检查 topic 名称和 ROS 类型；停止后读取整包，
  检查消息数、频率、150 ms gap、source header 单调性、joint name、有限值、双侧
  action、双手、三路 RGB、头部原始深度和 telemetry 丢包计数。
- `D`：将最近 episode 标记为 `discarded`，不删除源文件。
- `Ctrl-C`：active episode 标记为 `interrupted`。

状态只有 `recording`、`finalized`、`incomplete`、`interrupted`、`discarded`。
缺任一臂、手或相机流不得 `finalized`。

## 手动转换

仅 `finalized` bag 可转换：

```bash
./ops/run/convert_recording.sh \
  /home/user/franka_teleop_data/bags/gello/episode0 \
  /home/user/franka_teleop_data/datasets/episode0
```

脚本将源 bag 只读挂载；转换前后比较源文件大小和 mtime。输出写到相邻临时目录，
54/108 维、joint order、有限值、严格单调时间、三视频、头部 raw16 深度和
provenance 全部验证通过后才原子发布。失败时不改源 bag，也不暴露目标目录中的部分数据。

详细 topic、顺序和时间策略见
[`docs/DATA_FORMAT.md`](docs/DATA_FORMAT.md)。

代码已落地与仍待验收的条目见仓库根目录 [`采集进度.md`](../采集进度.md)。
