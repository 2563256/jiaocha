> G1 沿墙约束更新：局部地图已加入静态层，padding 8 cm、避障余量 18 cm，需重启应用。见 [G1_WALL_GUARD.md](G1_WALL_GUARD.md)。

> 2026-10-12：导航计时、G1 转向、五台速度与 omni 示例模型已更新；最新设置、验证范围和重启指令见 [NAVIGATION_FIXES_20261012.md](NAVIGATION_FIXES_20261012.md)。

# G1 人形机器人接入（2026-10-11）

参考 `/home/xuzhijie/hetero_fleet_geomap_ws` 的 G1 模型和原生步态实现，参考目录只读取。当前车队保留 light_1、light_2、heavy_1、omni_1，新增 g1_1。地图、墙体颜色、默认贪心调度及其他车辆控制保持当前项目版本，CBS 默认关闭。

G1 使用 0.9 缩放模型、十二个腿部关节力矩控制和原生 LSTM 行走策略；上身固定在训练姿态，无轮式底座。策略与许可在 `src/hetero_fleet_gz/policies/g1`。导航步态轮廓为 0.60×0.52 m，padding 为 0.04 m，最高速度 0.40 m/s。物理步长改为 1 ms，整场仿真运行速度可能比原来的 5 ms 步长低。

G1 出生点和充电位为 `(2.70,-32.00,0)`，world 中新增 charger_E。导航底盘坐标保持平面，真实身体高度和倾斜通过 base_footprint→base_link 反馈给雷达。队友感知、碰撞停止和 RViz 模型显示覆盖五台机器人。跌倒后原生控制器锁定停机；调度器需要新鲜的 locomotion_ready 和可用导航服务器才给 G1 派单，失去就绪状态会取消导航。G1 不通过运动学模拟冒充任务完成。

## 任务

- mixed_20 保留 20 个任务、20 组不同取送组合。其中 t08、t14、t15 为 G1 专属任务，载荷分别为 1、2、3 kg。
- demo_10 保留 10 个任务，其中 task_004、task_005 为 G1 专属任务，载荷为 1、2 kg。
- 专属能力为 humanoid_transport，G1 最大调度载荷 5 kg；北部房间的任务仍要求 light_payload。
- 装卸沿用调度仿真，尚未实现手臂抓取和货物物理挂载，也未验证额外载荷下的步态。
- 任务数量并未按实测利用率重新配平；应从下一次完整运行结果判断五台机器人的实际负载。

## 运行

```bash
cd /home/xuzhijie/hetero_fleet_geomap123_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hetero_fleet_bringup bringup.launch.py
```

另开终端发送 20 个任务：

```bash
cd /home/xuzhijie/hetero_fleet_geomap123_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/mixed_20.yaml --interval 8
```

等待五台 Nav2 导航栈启动后再发送，启动约需 170 秒墙钟时间。G1 的步态就绪不代表导航器已经就绪。原任务结果已保留，不用旧终端中的其他项目 overlay 启动本项目。

若首次编译从 Conda 找到不兼容的 protobuf，在干净 ROS 终端使用系统 PATH，并对 GZ 包清理 CMake 缓存重新配置：

```bash
export PATH=/usr/bin:/bin:/opt/ros/jazzy/bin
colcon build --symlink-install --packages-select hetero_fleet_gz \
  --cmake-clean-cache --cmake-args -DCMAKE_IGNORE_PREFIX_PATH=/home/xuzhijie/condaecho
colcon build --symlink-install
```

## 本项目验证结果

- 四个修改包编译通过；130 项 Python 回归检查通过，9 项北部房间非轻型路径检查按能力限制跳过。
- 原生策略 CTest 通过，五台车出生/工作点净空与连通性检查通过；队友停驻时的保守进出路径检查通过。
- ROS_DOMAIN_ID=87、独立 Gazebo partition 中五台导航栈激活，G1 平衡就绪，720 束雷达扫描可用。
- 修正队友感知话题并重新配置碰撞监控后，短距离 NavigateToPose 成功；G1 净位移约 1.20 m，记录约 1.51 m 的身体位移轨迹，导航状态 SUCCEEDED，结束时步态仍就绪。该轨迹包含身体摆动，不等同于运输任务里程。
- 尚未完成全套 20 任务及 G1 返充实测，不能据此宣称利用率均衡或全场无卡滞。
- 比对参考项目 src、tools 和接入说明共 201 个文件的哈希，无变化。独立测试进程已停止。
