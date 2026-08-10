# Operations

- `run/`：标准遥操、仅双臂、Wuji、手动 UI、MoveIt 等运行入口。
- `setup/`：Conda、本地包、Docker/ROS workspace 和 GELLO 驱动安装。
- `diagnostics/`：只读检查或显式诊断工具；可能移动硬件的工具会在自身文档中
  标明。

所有脚本都从自身位置解析仓库根目录，可以从任意当前工作目录调用。
