# 中文导航

本项目已完成可复现的软件阶段验收：真实 MuJoCo 导航、同一物理世界中的移动 Panda 抓取/放置、C++ 核心、真实 ROS2/MoveIt2 轨迹执行、安全故障与恢复、接触阻抗/导纳控制。硬件尚未接入，目标平台 HIL 尚未测量。

- [英文首页：架构、运行命令、实测指标与边界](../README.md)
- [基线审计：旧底盘状态覆盖与逻辑夹爪问题](BASELINE_AUDIT.md)
- [仿真 Benchmark：种子、样本、统计方法和原始日志](SIMULATION_BENCHMARK.md)
- [状态估计：真实底盘受力仿真、虚拟编码器/IMU、纯估计反馈与丢包停机](STATE_ESTIMATION_ACCEPTANCE.md)
- [ROS2：真正编译、MoveIt2/OMPL、ros2_control、bag 与故障证据](ROS2_ACCEPTANCE.md)
- [硬件准备：未知参数、接线、标定、首周调试](HARDWARE_READINESS.md)
- [交付报告：逐项 PASS / PARTIAL / PENDING_HARDWARE](FINAL_DELIVERY_REPORT.md)
- [面试技术说明：数学公式、核心代码、方案取舍、追问](INTERVIEW_TECHNICAL_GUIDE.md)

真实仿真不等于真实硬件：物体是 MuJoCo 自由刚体，抓取依靠接触/摩擦；没有焊接物体或改写其坐标来伪造抓取。移动底盘是受力驱动的平面约束模型，未实现轮胎打滑或特定底盘动力学。既有抓取任务使用明确标注的仿真真值；新增独立底盘实验采用虚拟编码器/IMU 的 EKF 反馈，真值仅用于离线评分，尚未接入整机移动抓取或 ROS 任务。
