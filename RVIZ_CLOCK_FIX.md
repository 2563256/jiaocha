# RViz 模型与代价地图显示修复（2026-10-12）

本轮诊断发现上次 Gazebo PID 80708 遗留，与当前 PID 90656 同场景同时运行。ROS /clock 没有数据，G1 joint_states 有数据，但十二个活动腿部连杆缺少 map TF，代价地图没有正常刷新。所有 G1 mesh 文件存在。

处理：清理确认属于本项目的旧仿真；将 bridge.yaml 中 ROS /clock 的 Gazebo 来源明确改为 /world/geomap_fleet/clock。当前桥接替换后重新收到地图且所有 G1 link 的 map TF 可解析。后续顶层启动默认生成独立 GZ_PARTITION，用户显式指定的分区仍保留。

RViz 补齐全 fleet 的局部与全局 costmap，统一可靠/Transient Local 订阅，关闭未使用的 CBS 显示。全局代价地图启用周期完整地图发布，支持重开 RViz 后接收完整底图。重载当前 RViz；任务与 Nav2 未重启。两个替换进程由 watcher 绑定本次主 launch PID，在其退出后关闭。

构建通过；bringup 12 项测试通过。G1 所有连杆 TF 和 mesh 检查通过；本轮实际接收 G1 局部及全局地图，以及其余机器人局部地图。显示配置覆盖全部十个代价地图。

补充：两台 light 的全局地图通过 GetCostmap 均为 649×1684、含有效障碍。针对本次时钟切换后完整地图发布延迟，使用真实 GetCostmap 数据做一次完整 OccupancyGrid 刷新供 RViz 显示；没有构造空白地图，也未替换 Nav2 原始代价数据。
