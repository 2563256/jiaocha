# 快速运行与速度上限

本轮实测旧运行的仿真实时倍率约 0.061，因此画面慢主要来自仿真时间推进慢。当前生效上限为 light 0.84、heavy 0.54、omni 0.66、G1 0.45 m/s。

新增修改：light 上限 1.00、heavy 0.65、omni 0.80 m/s；G1 保留已完成步态物理验证的 0.45 m/s。调度档案、MPPI、速度平滑器和轮式控制器同步，局部地图范围随预测轨迹调整。加速度及碰撞停止保留。

RViz 帧率改为 12 FPS。新增 fast.launch.py，不启动 Gazebo GUI，保留 RViz；彩色地图资源仍在项目中，普通 bringup.launch.py 仍可启动 Gazebo GUI。fast 模式也可用 rviz:=false 关闭全部可视化。

需 Ctrl+C 停止旧运行后重启，禁止同时启动第二套同 ROS_DOMAIN_ID 的仿真：

```bash
cd /home/xuzhijie/hetero_fleet_geomap123_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hetero_fleet_bringup fast.launch.py
```

参数一致性与预测范围检查通过，description/bringup 构建通过，快速入口参数解析通过。当前用户运行没有停止；新配置的整场实时倍率及新速度下完整 20 任务尚未实测。速度上限不是保证所有路段都会保持该速度。
