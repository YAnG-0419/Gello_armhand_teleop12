# Teleop runtime

`cli.py` 是统一 Operator 后端入口。它组合机械臂输入、手部输入、Operator
控制协议和硬件协调器；默认仍是 GELLO 双臂与 MANUS/O30i 双手。

Wuji 和 powderweighing 都是显式可选项。前者通过 `--hand-source wuji` 加载，
后者只有传入 `--right-hand-strategy-config` 时才包装右手策略，因此不会改变
默认 O30i 行为。
