# Harvest / LeRobot v2 数据格式

机器可读契约是 `data_collection/contract/dataset_contract.yaml`；机器人 topic 和
joint name 的代码单一来源仍是 `teleop_core.contract`，测试会检查 YAML 与该来源的
关键映射。第一期 `teleoperator=gello`。Pico 臂若以后接入，只加
`record_pico.yaml`，LeRobot 特征不变，metadata 写 `teleoperator=pico`。

## ROS bag topic/type

| 数据 | Topic | ROS type |
|---|---|---|
| 双臂安全后 action | `/teleop/validated_arm_commands` | `sensor_msgs/msg/JointState` |
| 左/右臂实测 | `/left/franka/joint_states`, `/right/franka/joint_states` | `sensor_msgs/msg/JointState` |
| 左/右手已发送 command | `/teleop/wuji/{left,right}/command` | `sensor_msgs/msg/JointState` |
| 左/右手实测 state | `/teleop/wuji/{left,right}/joint_states` | `sensor_msgs/msg/JointState` |
| 手 telemetry 状态 | `/teleop/wuji/telemetry_status` | `teleop_interfaces/msg/HandTelemetryStatus` |
| 三路 RGB | `/cam{0,1,2}/color/image_raw` | `sensor_msgs/msg/Image` |
| 头部原始深度 | `/cam0/depth/image_raw` | `sensor_msgs/msg/Image` (`16UC1`) |
| 可选相机内参 | `/cam{0,1,2}/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 可选头部深度内参 | `/cam0/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |

训练用 arm action 只取 `/teleop/validated_arm_commands`。不要把
`/teleop/arm_commands` 或 `/{side}/gello/joint_states` 当作训练 action。
validated topic 可只含当前 active side；转换器按名字拆分
`left/right_fr3v2_joint*` 并归一化为稳定的 `left/right_fr3_joint*`。缺失的一侧
绝不复制、填零或伪造；超过默认 150 ms 后该时间点直接报告 unmatched 并丢弃，
整集没有完整帧则转换失败。

## 精确 feature 顺序

单位一律 radian。一律按 joint name 归一化，禁止依赖消息数组下标。

`action: float32[54]`：

1. left arm position：7，`left_fr3_joint1..7`
2. right arm position：7，`right_fr3_joint1..7`
3. left Wuji command：20，按 `teleop_core.contract.WUJI_LEFT_JOINT_NAMES`
4. right Wuji command：20，按 `teleop_core.contract.WUJI_RIGHT_JOINT_NAMES`

`observation.state: float32[108]`：

1. left arm position 7
2. left arm velocity 7
3. right arm position 7
4. right arm velocity 7
5. left hand position 20
6. left hand velocity 20
7. right hand position 20
8. right hand velocity 20

每个输入都按显式 joint name 重排。Wuji SDK 当前只提供位置反馈，因此 receiver 用
相邻、同名且 source monotonic timestamp 严格递增的样本做有限差分；首帧不发布
state，后续消息以 `velocity=finite_difference` 和 status 字段明确标记，绝不以零
冒充实测速度。

图像映射固定为：

```text
observation.images.cam0 <- /cam0/color/image_raw
observation.images.cam1 <- /cam1/color/image_raw
observation.images.cam2 <- /cam2/color/image_raw
observation.depths.cam0 <- /cam0/depth/image_raw (raw16)
```

动作、状态和数据集行使用 30 Hz 时间轴；cam1/cam2 RGB 原生 30 Hz，cam0 RGB
和原始深度原生 20 Hz。cam0 媒体帧各只存储一次，30 Hz 行按时间戳引用最近且不超过
150 ms 的源帧，不生成插值 RGB 或深度。深度不做 D2C 注册、不生成点云、不做去畸变。

cam0/cam1/cam2 的序列号及物理语义写入 `collection_state.json`，并随
`source_provenance` 进入 `meta/conversion_metadata.json`。

## 时间与完整性

ROS bag 同时保留原始消息 header timestamp 与 rosbag receive time。状态 sidecar
记录每条 required stream 的首末 receive time、source header 首末值、消息数、
频率、最大内部 gap、首尾 gap 和非单调计数。转换默认构造 30 Hz 确定性时间线，
仅选取不超过 150 ms 的最新历史样本，不使用未来样本；metadata 报告每个 source
的 unmatched 数和被跳过的不完整行数。

## LeRobot v2 输出

```text
<dataset>/
├── data/chunk-000/episode_000000.parquet
├── videos/chunk-000/observation.images.cam0/episode_000000.mp4
├── videos/chunk-000/observation.images.cam1/episode_000000.mp4
├── videos/chunk-000/observation.images.cam2/episode_000000.mp4
├── data/.../observation.depths.cam0  # Parquet 内联 raw16 字节与图像元数据
├── meta/info.json
├── meta/episodes.jsonl
├── meta/tasks.jsonl
├── meta/episodes_stats.jsonl
├── meta/conversion_metadata.json
├── README.md
└── dataloader.py
```

## 与 wuji-openpi 的已知差异

`/home/user/llx/harvest_ws/wuji-openpi` 当前与本契约不一致。本任务不修改
OpenPI。接入训练时必须用显式 loader/repack，禁止按数组位置或相似 feature 名
静默兼容。

| 项目 | 本仓库 Harvest/LeRobot v2 | wuji-openpi |
| --- | --- | --- |
| action 54 维顺序 | 左臂 7、右臂 7、左手 20、右手 20 | 左臂 7、左手 20、右臂 7、右手 20 |
| observation.state | 108 维（关节位 + 速度） | 默认约 54 维（关节位） |
| 相机 feature | `observation.images.cam{0,1,2}` | `cam_high` / `cam_left_wrist` / `cam_right_wrist` 一类 |
| 头部深度 | `observation.depths.cam0` raw16 | 默认 RGB 训练入口不消费 depth |
