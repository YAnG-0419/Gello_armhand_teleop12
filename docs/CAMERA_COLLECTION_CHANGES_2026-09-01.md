# 三相机采集变更记录（2026-09-01）

本文记录 Harvest 数据采集第一次现场联调中发现的问题、实施的修改和实测结果。
当前第一期数据格式只记录三路 RGB，不记录深度图或点云。

## 结论

- `start_recording.sh` 已可通过专用的特权 Compose service 启动，不再使用本机
  Compose 不支持的 `docker compose run --privileged` 参数。
- 三台相机均可按部署序列号连接。
- cam1、cam2 可稳定输出约 30 FPS；以太网 cam0 当前硬件 profile 最高只支持
  10 FPS。
- 相机 ROS publisher 为 **Reliable**。原 recorder 强制使用 **Best Effort**；在本机
  Cyclone DDS、1280x800 大图像和三相机环境下，Best Effort 订阅几乎只收到首帧。
  recorder 改为 Reliable 后，cam1、cam2 稳定达到 30 FPS。
- 相机配置已改为 RGB-only，关闭当前数据格式不使用的深度、点云、深度配准和
  实时彩色去畸变。

## 1. 采集容器入口

### 现场问题

本机版本为 Docker Compose v5.4.0，其 `docker compose run --help` 没有
`--privileged` 参数。原入口：

```bash
docker compose run --rm --privileged ... tools ...
```

会立即失败：

```text
unknown flag: --privileged
```

去掉特权后，网络 cam0 可以连接，但两台 USB 相机无法被 SDK 正常访问。因此采集
容器本身仍然需要特权设备访问，不能只删除参数。

### 修改

在 `docker/compose.yaml` 中新增独立的 `data-collection` service：

```yaml
data-collection:
  <<: *runtime
  profiles: [collection]
  privileged: true
  stdin_open: true
  tty: true
  stop_signal: SIGINT
  stop_grace_period: 30s
  volumes:
    - ..:/workspace/franka_upper_body_teleop
    - ${TELEOP_DATA_ROOT:?Missing TELEOP_DATA_ROOT in docker/.env}:/data
    - /dev:/dev
  command:
    - /workspace/franka_upper_body_teleop/ops/run/run_data_collection_container.sh
```

`ops/run/start_recording.sh` 改为：

```bash
docker compose run --rm ... data-collection
```

特权只属于显式启动的采集 service。现有 `tools`、机械臂控制、遥操和手控制 service
均未获得额外权限；普通 `docker compose up` 也不会自动启动 collection profile。

## 2. Reliable publisher 与 Best Effort recorder

### 现场现象

三台相机能够被 Orbbec SDK 识别，ROS topic 也存在，但短 bag 中的 RGB 消息数为
`0/0/1` 或接近零。单次 `ros2 topic echo --once` 可以从每台相机取得一帧，说明设备、
序列号和 topic 映射没有错误，但连续订阅异常。

通过 `ros2 topic info -v /cam0/color/image_raw` 检查到相机 publisher 的 QoS 为：

```text
Reliability: RELIABLE
History: KEEP_LAST (10)
Durability: VOLATILE
```

原 `data_collection/config/record_gello.yaml` 则强制 rosbag 使用：

```yaml
rosbag_record_default_qos:
  history: keep_last
  depth: 100
  reliability: best_effort
  durability: volatile
```

在本机 Cyclone DDS、1280x800 RGB 大消息和三相机环境下，Best Effort 轻量计数
15 秒的结果为：

```text
cam0: 2 frames / 15.01s = 0.13 Hz
cam1: 1 frame  / 15.01s = 0.07 Hz
cam2: 0 frames / 15.01s = 0.00 Hz
```

这与 bag 中几乎只有首帧的现象一致。

将同一订阅改为 Reliable 后，两台 30 FPS USB 腕部相机在 10 秒内实测：

```text
cam1: 29.95 Hz (300 frames)
cam2: 30.05 Hz (301 frames)
```

三路同时 Reliable 订阅 15 秒的最终结果为：

```text
cam0:  9.92 Hz (149 frames / 15.02s)
cam1: 29.89 Hz (449 frames / 15.02s)
cam2: 30.03 Hz (451 frames / 15.02s)
```

### 修改

`data_collection/config/record_gello.yaml` 已改为：

```yaml
rosbag_record_default_qos:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: volatile
```

这里必须保留 Reliable，不能为了“传感器通常使用 Best Effort”而恢复旧值。上述问题
是本机实际 ROS/DDS 路径上的复现结果，不是仅根据 QoS 名称作出的推断。

## 3. 相机流配置

当前 LeRobot 契约只消费：

```text
/cam0/color/image_raw -> observation.images.cam0
/cam1/color/image_raw -> observation.images.cam1
/cam2/color/image_raw -> observation.images.cam2
```

初始验收没有 depth image 或 point cloud feature。后续数据需求已明确为仅增加头部
cam0 原始深度；腕部深度和所有点云仍关闭。当前策略为：

```yaml
enable_color: true
enable_depth: true  # 仅 cam0；cam1/cam2 仍为 false
enable_point_cloud: false
enable_colored_point_cloud: false
depth_registration: false
enable_depth_undistortion: false
enable_color_undistortion: false
```

cam0 深度由本机 SDK 对序列号 `CP4N5630008Z` 实际枚举确认：1280x800 下支持
5/10 FPS，格式 Y16/Y12C4/RVL。采集采用 `1280x800@10, Y16`，以
`/cam0/depth/image_raw` 记录，转换为 `observation.depths.cam0` raw16；同时保存
`/cam0/depth/camera_info`，不做 D2C 注册。

当前彩色流参数：

| 相机 | 设备/连接 | 分辨率 | 配置 FPS | 实测 FPS |
| --- | --- | ---: | ---: | ---: |
| cam0 | Gemini 435Le / Ethernet | 1280x800 | 10 | 9.92 |
| cam1 | Gemini 305 / USB 3.2 | 1280x800 | 30 | 29.89 |
| cam2 | Gemini 305 / USB 3.2 | 1280x800 | 30 | 30.03 |

## 4. cam0 无法设置为 30 FPS

cam0 强制设置为 1280x800@30 时，驱动拒绝启动并打印其可用 profile。当前以太网
Gemini 435Le 在 1280x800、1280x720、800x600、640x400 和 640x360 等列出的
分辨率下均只提供 5 FPS 或 10 FPS，没有 30 FPS profile。因此降分辨率也不能在
当前连接方式下得到 30 FPS。

最终没有保留一个会导致节点退出的 30 FPS 配置，而是将 cam0 明确设为其硬件上限：

```yaml
color_width: 1280
color_height: 800
color_fps: 10
```

若要求三路都是真实 30 FPS，需要改变 cam0 的连接/设备方案，并重新查看新连接下
驱动实际公布的 profile。不能通过把 YAML 数值写成 30 来插值出真实相机帧。

## 5. 数据集时间轴

当前 `convert_gello_lerobot_v2.yaml` 仍配置：

```yaml
fps: 30
sampling:
  frequency_hz: 30
  max_staleness_ms: 150
```

转换器输出的 LeRobot v2 动作/状态时间轴仍为 30 FPS。cam0 RGB 和 raw depth
分别标记为 10 FPS，cam1/cam2 RGB 标记为 30 FPS。每个源媒体帧只存储一次；
30 Hz 数据行通过时间戳引用不超过 150 ms 的最近源帧，不生成插值帧。

不要把 cam0 媒体元数据伪标为 30 FPS；训练 loader 若按 30 Hz 读取动作，应按
Parquet 中的媒体 timestamp 解析 10 Hz 头部 RGB/depth 引用。

## 6. 验证与运行注意事项

- YAML 解析、Shell 语法、Compose 配置和 `git diff --check` 均通过。
- `teleop_data_collector` 非集成测试：`23 passed`。
- 测试结束后无采集或控制容器残留。
- `start_recording.sh` 会拒绝在 OrbbecViewer 或 compose `orbbec` service 运行时启动；
  它们会抢占相机设备。
- 相机-only 按 SPACE 会被 recorder preflight 拒绝，因为完整 episode 还要求双臂状态、
  validated action 和双手 telemetry。这属于数据契约保护，不是相机启动失败。
