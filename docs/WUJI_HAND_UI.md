# Wuji Hand 2 双手姿态与 MANUS 阈值 UI

本文说明双 Wuji Hand 2 网页 UI 的代码位置、启动方法、姿态 JSON 约定、
MANUS 阈值采集流程和真机验证边界。

## 1. 代码分布

网页应用主要位于 `apps/wuji_ui/`：

| 文件 | 作用 |
| --- | --- |
| `apps/wuji_ui/main.py` | NiceGUI 页面、双手关节滑块、姿态列表和 MANUS 阈值采集界面 |
| `apps/wuji_ui/runtime.py` | 单进程硬件所有者；读取实际反馈、切换失能/遥操、限速姿态回放、计算 MANUS 指尖距离 |
| `apps/wuji_ui/models.py` | 姿态和触发器数据校验、JSON 原子读写、阈值样本统计 |
| `apps/wuji_ui/test_models.py` | 姿态、触发器和 JSON 测试 |
| `apps/wuji_ui/test_runtime.py` | 双手模式、反馈和 MANUS 距离测试 |
| `apps/wuji_ui/README.md` | 应用级简要操作说明 |

Wuji 硬件与重定向代码位于 `adapters/wuji/`：

| 文件/目录 | 作用 |
| --- | --- |
| `adapters/wuji/backend.py` | Wuji SDK 连接、使能/失能、发送命令和 20 关节实际反馈订阅 |
| `adapters/wuji/pipeline.py` | MANUS → Wuji 重定向、optimizer/device 关节重排、模型关节限位 |
| `adapters/wuji/config/` | 左右手 MANUS 重定向 YAML |
| `adapters/wuji/models/hand2_beta/` | Wuji Hand 2 URDF、MJCF 和 mesh |
| `adapters/wuji/wuji_retargeting/` | 21 点手部关键点到 Wuji qpos 的求解器 |

MANUS 原生桥和手套标定位于：

- `adapters/manus/native/`：原生 MANUS SDK 桥；
- `adapters/manus/python/manus_teleop/`：Python MANUS frame 和 canonical landmarks；
- `adapters/manus/config/Calibration_left.mcal`；
- `adapters/manus/config/Calibration_right.mcal`。

启动与安装入口：

- `ops/run/start_wuji_ui.sh`：检查冲突服务、进入 Conda 环境并启动网页；
- `ops/setup/setup_wuji_env.sh`：安装 `wuji-sdk`、重定向依赖和 NiceGUI。

## 2. 启动

首次安装或环境更新：

```bash
./ops/setup/setup_wuji_env.sh
./adapters/manus/scripts/build.sh
```

启动左右两只 Wuji Hand 2：

```bash
./ops/run/start_wuji_ui.sh \
  --wuji-left-address 192.168.2.111:7447 \
  --wuji-right-address 192.168.2.112:7447
```

默认打开 <http://127.0.0.1:8082>。无人值守时可增加 `--no-browser`。

首次真机测试建议降低参数：

```bash
./ops/run/start_wuji_ui.sh \
  --wuji-left-address LEFT_IP:PORT \
  --wuji-right-address RIGHT_IP:PORT \
  --wuji-current-limit 0.5 \
  --wuji-kp 1.0 \
  --wuji-kd 0.1 \
  --pose-speed 0.3
```

启动前必须停止其他 Wuji、MANUS 和手部控制进程。一个设备只能有一个命令所有者。

## 3. 姿态记录与回放

两只手启动后均处于“手动（电机失能）”模式。

1. 手动摆好一只手，确认该侧 20 个“当前角度”持续刷新。
2. 点击“读取当前位置”。读取的是电机实际反馈，不是重定向目标值。
3. 输入该侧独立的姿态名称。
4. 可使用关节滑块或联动数字框调整“待保存角度”。
5. 点击“保存”。滑块编辑本身不会驱动真机。
6. 点击姿态列表只会选中和显示姿态。
7. “移动到选中姿态”需要二次确认，并使用 `--pose-speed` 限速执行。

滑块范围来自对应 Wuji Hand 2 模型的真实关节上下限，不使用统一的硬编码范围。
数字框单位为度，JSON 始终存弧度。

“启动 MANUS 遥操”可独立作用于左手或右手。切换回“手动 / 失能”后才能直接
触碰和摆动该手。页面顶部的“双手失能”会依次失能左右手，但它是软件停止，
不能代替硬件急停。

## 4. JSON 文件

默认输出：

```text
config/calibration/wuji_hand_2_poses.json
```

可通过 `--output PATH` 修改。文件采用原子替换写入，核心结构如下：

```json
{
  "format": "wuji-hand-2-hardcoded-poses",
  "version": 1,
  "units": "rad",
  "joint_order": "device",
  "joint_names": {
    "left": ["l_thumb_cmc_flex"],
    "right": ["r_thumb_cmc_flex"]
  },
  "poses": {
    "left": {
      "自然张开": [0.0]
    },
    "right": {
      "食指捏合": [0.0]
    }
  },
  "manus_triggers": {}
}
```

实际 `joint_names` 和每个 qpos 均包含 20 项。后续策略必须按关节名称重排，
不能假定 optimizer 顺序等于设备发送顺序。

## 5. MANUS 阈值采集

当前指标与 `tasks/powderweighing/strategy.py` 一致，使用 MANUS 骨骼中的
“拇指尖到目标指尖三维欧氏距离”，单位为米；页面显示毫米。可选择：

- 拇指—食指；
- 拇指—中指；
- 拇指—无名指；
- 拇指—小指。

采集流程：

1. 选择手侧、距离类型和已保存的对应姿态。
2. 点击“开始采集”。Wuji 电机可以保持失能；MANUS bridge 仍会读取手套。
3. 重复执行意图触发动作，并覆盖正常动作波动范围。
4. 点击“停止并计算”。至少需要 10 个序号不同的 MANUS frame。
5. 页面显示 `min / p05 / median / p95 / max`。
6. 推荐进入阈值为观测 `p95` 加小余量；推荐退出阈值更大，用于迟滞防抖。
7. 可手动修改进入、退出和 dwell 后，保存阈值映射。

保存示例：

```json
{
  "manus_triggers": {
    "right": {
      "index_pinch": {
        "finger": "index",
        "pose": "食指捏合",
        "metric": "thumb_tip_distance",
        "enter_max_m": 0.025,
        "exit_min_m": 0.032,
        "dwell_seconds": 0.15,
        "calibration": {
          "sample_count": 120,
          "p95_m": 0.023
        }
      }
    }
  }
}
```

如果重命名姿态，引用它的触发器会同步更新；仍被触发器引用的姿态不能直接删除。

## 6. 当前实现边界

### 复合手势（抓瓶等整体手型）

对于抓瓶、包覆抓取等动作，不应只使用一对指尖距离。页面中的
“MANUS 复合手势学习”同时提取 12 项与手在空间中的位置和朝向无关的特征：

- 拇指、食指、中指、无名指、小指的骨段弯曲量，共 5 项；
- 拇指到其余四个指尖的距离，共 4 项；
- 食指—中指、中指—无名指、无名指—小指距离，共 3 项。

七项距离都除以实时掌宽，因此整体放大、缩小不会改变特征。学习操作为：

1. 选择手侧和已经保存的目标 Wuji pose；
2. 点击“开始学习”；
3. 在希望触发的抓取范围内自然活动并重复 3～5 遍；
4. 至少采集 30 个序号不同的 MANUS 帧后停止；
5. 系统使用每项特征的 p05～p95，并增加小余量作为学习范围；
6. 继续做目标和非目标动作，观察实时 `命中项/12` 和每项 ✓/×；
7. 根据验证结果调整进入命中率、退出命中率和 dwell 后保存。

默认进入命中率为 80%，退出命中率为 60%。两级比例加 dwell 可避免边界附近
反复切换。模型保存在 JSON 的 `manus_gesture_triggers` 中：

```json
{
  "bottle_grasp": {
    "pose": "抓瓶",
    "metric": "composite_landmark_ranges_v1",
    "feature_ranges": {
      "index_curl_rad": {
        "min": 0.5,
        "median": 0.8,
        "max": 1.1
      }
    },
    "enter_match_fraction": 0.8,
    "exit_match_fraction": 0.6,
    "dwell_seconds": 0.2,
    "sample_count": 150
  }
}
```

实际文件包含全部 12 项范围。当前学习的是正样本范围，保存前应专门做几种
非目标动作验证误触发；若它们仍达到进入命中率，应重新采集更一致的目标动作，
或提高进入命中率。

当前已经实现：

- 双 Wuji Hand 2 连接与完整 20 关节反馈校验；
- 左右手独立手动失能和 MANUS 遥操；
- 姿态记录、关节滑块编辑、JSON 保存和限速回放；
- MANUS 实时指尖距离、统计阈值和 pose 映射保存。
- MANUS 复合手势学习、逐特征实时解释和整体命中率验证。

当前尚未实现：

- 根据保存的 MANUS 阈值自动切换到硬编码姿态；
- 多触发器冲突仲裁；
- 真机上的最终阈值验收和长时间运行验证。

因此 `manus_triggers` 目前是后续策略可直接读取的数据合同，不会自行驱动真机。

## 7. 首次真机验收顺序

1. 保持硬件停止手段可触达，清空双手周围空间。
2. 使用低 `kp`、低电流和低 `pose-speed` 启动。
3. 确认启动后两手均为失能模式，并收到完整反馈。
4. 先验证单手手动记录和 JSON，再验证另一手。
5. 单独启动一侧 MANUS 遥操，检查方向、限位和跟随。
6. 用小幅度姿态验证单手限速回放。
7. 两侧分别通过后才测试双手同时遥操。
8. 最后在电机失能状态采集 MANUS 阈值，检查实时毫米距离是否符合动作直觉。

没有完成上述真机验收前，不应把当前状态描述为生产验证完成。

## 8. 软件验证

运行相关测试：

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m pytest -q apps/wuji_ui adapters/wuji
```

当前覆盖姿态/触发器 JSON、关节反馈重排、独立手侧模式、模型关节顺序、
MANUS 指尖距离和阈值统计。
