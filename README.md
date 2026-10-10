## 2026-10-10 x=2 分散充电位

light_1=(2.00,-11.50)，light_2=(2.00,-21.50)，heavy_1=(2.00,-38.00)，omni_1=(2.00,-69.50)，yaw 均为 0。
出生位与充电位同步，Gazebo 标记同步。四车分别验证其他三车停放时到允许的各任务点的可达性；
北部点仍仅轻型车执行。停放会车效果需重启后动态验证；加载新出生位和充电标记需重启整套仿真。

## 2026-10-10 按车辆独立返充电位

完成任务后先接续本车能执行的待办任务；没有本车可执行任务时，默认空闲 2 秒后返自己的充电位。
_drive_home 的二次检查由车队级 _work_remains 改为 _work_for(robot)，其他车辆仍在执行任务不再阻止返航。
开始返航时再次检查本车是否已有可执行任务，避免新任务到达与返航启动竞争。
恢复默认保持启用，充电行程继续记录里程/能耗。需重启 fleet_manager（建议整套仿真）后生效。

## 2026-10-10 放宽停滞检测

调度器停滞窗口由 15 s 改为 30 s；目标误差改善 0.05 m 即算进展（原 0.10 m）；
绕行相对上次进展位置移动 0.40 m 即刷新检测窗口（原 1.20 m），允许缓慢、暂时远离目标的绕行。
Nav2 进度检查由 30 s 内移动 0.20 m 放宽为 45 s 内移动 0.15 m。
恢复次数、到达判定和低频 0.12 m 倒退不变。配置在节点启动时读取，需重启仿真后生效。

## 2026-10-10 恢复原彩色 Gazebo 外观

撤销灰色 walls_smoothed 渲染替换，model.sdf 再次使用原 geomap_combined12_noroof.obj 及 MTL/atlas 贴图。
地图与碰撞边缘从原始备份重新做 0.10 m 轻微平滑（970 格去毛刺、211 格填浅槽），
避免累积上次 0.15 m 处理。通道净宽和连通性保护保持；原扫描的可见外形与颜色不修改。
GPU 雷达重新读取原扫描可见几何，导航/碰撞几何与其在局部细节上仍可能存在小差异。
Nav2 调优与任务、车型限制保持。需重启 Gazebo 才能恢复已加载场景的彩色外观。

## 2026-10-10 墙体平滑与 Nav2 避障调优

原地图备份：tools/data/geomap_before_wall_smoothing.pgm。tools/smooth_wall_map.py 对全图做 0.15 m 边缘平滑，
保留薄墙、独立家具和通道中心净宽；去除 1492 格毛刺、填补 337 格浅槽，可通行面积净增 2.8875 m²。
0.20/0.30/0.50/0.55/0.70 m 净空区域均未损失已有通行格与连通关系，20223 个中心线样本净空未减小。
地图生成器增加默认 --wall-smooth-radius 0.15，之后重生成仍会保留此平滑步骤；设为 0 可禁用。
Gazebo 实体墙与 GPU 雷达渲染墙同步由地图构造。原彩色扫描网格仍保留为资源，当前场景改为统一灰色墙面，
避免旧扫描毛刺被 GPU 雷达重新标记为障碍。该渲染变化不影响机器人配色。
Nav2 三车型：软膨胀梯度半径 0.50 m、衰减系数 3.0，Smac2D cost_travel_multiplier=4.0，
MPPI 全车型 consider_footprint=true，PathAlignCritic 权重 8.0，critical_weight=25.0，碰撞余量 0.07 m。
速度上限保持。低频 12 cm 倒退恢复保留；CBS 默认关闭；北部任务仍仅轻型。
verify_scene 使用原有“内切半径 + 0.30 m”净空标准；软膨胀梯度半径不等于实体障碍厚度。
111 项测试通过，6 项因北部点不允许重型/omni 而跳过。动态避障与 20 任务成功率需重启后实跑对比。
必须停止任务发布程序并重启整套仿真，才能加载新地图、墙体、传感器几何和导航参数。

## 2026-10-10 北部房间轻型专用任务

新增 P6=(5.00,-17.50)、P7=(12.00,-12.50)、D6=(10.00,-21.00)。
20 个任务仍使用不同的起终点组合。t01、t02、t03、t10、t11、t13 涉及北部点，
明确 required_capability=light_payload，载重均不超过 20 kg，只有 light_1/light_2 可执行。
其他任务保持原有能力限制。demo_10 的相关任务同步修改。CBS 调度保持默认关闭。
新的任务若使用 P6/P7/D6，也必须指定 light_payload；当前限制由任务能力字段执行。
需重启仿真加载新场景标记、重新发布任务文件；旧任务不会被自动改写。

## 2026-10-10 室内分散站点与 20 个不同任务组合

S1=(12.00,-10.20)、S2=(12.00,-26.00)，充电位与出生点同步移动。
P1=(13.00,-30.00)、P2=(2.97,-30.25)、P3=(9.00,-53.50)、P4=(10.00,-33.80)、P5=(5.00,-67.00)。
D1=(8.00,-25.60)、D2=(8.00,-43.60)、D3=(10.22,-37.50)、D4=(10.20,-49.20)、D5=(6.00,-70.00)。
任务与场景标记同步：mixed_20 的 20 个有向起终点组合均不同，每个取货/送货点各使用 4 次。
任务载重、能力、优先级和截止时间保持。demo_10 也更新为新坐标。
四车到十个工作点、其他三车停在出生位的 40 项保守离线规划检查通过；不启用 CBS 调度。
地图连通与间距检查通过。共用门口和通道仍会重叠；实际会车与任务完成情况需重启仿真实跑验证。
需停止旧任务发布程序并重启整套仿真，再发布新版任务，才能使用新位置。

## 2026-10-09 heavy 防倾与低频短倒退

实测 heavy 在 (2.97,-23.06) 处倾斜约 17.6 度，导航有速度指令但车身不移动。
增加后部防倾球支撑：正常离地约 8.9 mm，约 2.4 度后倾时接地，保持原外形轮廓。
停滞恢复先重规划；连续第二次停滞才允许 BackUp，默认 0.12 m、0.06 m/s、4 秒动作预算，
每段最多一次、每车至少 120 秒冷却。后方碰撞检测仍启用；普通行为树不加入循环 BackUp。
更新涉及 Gazebo 模型和行为插件，须重启整套仿真。84 项测试通过；完整动态任务仍待重启实测。

## 当前默认：恢复原调度算法（2026-10-09）

默认 `use_cbs:=false`：按车型能力筛选，结合距离、负载、能耗等成本进行贪心任务分配，
每辆车通过 Nav2 NavigateToPose 独立导航（SMAC2D 全局规划、MPPI 局部避障）。
CBS 默认关闭，保留为可选功能。车体尺寸、膨胀半径、队友障碍感知以及移除后退恢复的修改保持。
重新启动仿真后生效；已有节点不会自动读取代码修改。启动时请移除旧命令中的 `use_cbs:=true`。

# hetero_fleet_geomap123_ws — 异构机器人车队 Gazebo + Nav2 + 贪心调度仿真（实景扫描地图版）

## CBS 取消竞态修复（2026-10-09）

实际运行中一个 FollowPath 刚刚成功、另一个仍在取消时，已完成动作会返回空 goals_canceling。
旧代码错误地查看了整个车队是否还有未完成结果，导致将正常取消误判为失败并关闭 CBS 执行器。
现在以每个取消请求完成及所有动作到达终态作为重规划屏障，空 goals_canceling 本身不代表失败。
补充真实取消响应形态的混合终态回归测试。关闭的执行器明确报 cbs_executor_stopped，区别于车辆已有任务的 cbs_busy。
此修改须重启 fleet_manager 生效；已失败任务不会自动恢复，重跑前需备份 metrics。


## 2026-10-09 CBS 导航失败修复

本轮日志中 t01–t05 的失败发生在下发 Nav2 动作前：`fallback did not reach every goal`。
复现发现：旧停车队列中的 light_2 和 heavy_1 在其余车辆静止时被 CBS 的安全占用区封住；
先前仅检查 light_1 和四车全动，漏掉了单独派车的情况。保留放大车体与安全间距，将出生点和充电位同步改为
light_1 (-2.40,-11.00)、light_2 (0.30,-14.40)、heavy_1 (2.97,-32.25)、omni_1 (8.22,-37.50)，
场景标记与生成器同时更新；这四处不占用取送货点，也不会封住其通路。
CBS/优先规划的搜索时间步上限现在依据实际绕行路线长度计算，避免直线距离短而绕行很长时误报无路。
新增任务导致联合规划失败时，只拒绝新任务，已接纳任务取消旧动作后重新规划继续执行，避免整批连带失败。
新增真实地图回归检查：4 台车各自到 6 个任务点，其他 3 台静止，24 项均需可达且目标偏移不超过 0.20 m。
新配置必须完整重启仿真生效；动态 20 任务结果仍需新一轮验证。


## 2026-10-09 队友避障修复

实时采样确认 heavy 与 omni 的 robot_obstacles 发布、局部代价地图及 collision_monitor 订阅均正常；采样时两车已分开，因此没有复现用户此前遇到的阻挡现场。此前全局地图不接收合成队友障碍，且队友圆形代理小于实际矩形轮廓。现在按车型长宽与实时朝向进行射线/矩形相交，同时处理扫描 ±pi 边界。全局、局部地图增加独立 robot_layer，使用 marking + clearing + inf_is_valid 清除旧障碍，避免清除真实墙体图层；omni 的 MPPI 启用完整轮廓检查。几何与配置检查通过，重启后仍需验证真实会车、窄通道和被队友占据的任务点（可能等待或规划失败）。体积、0.30 m 膨胀半径、无减速区和无倒退恢复保持上次设置。


## 2026-10-09 第二次调整

所有车型在上次修改基础上体积再增加 50%（相对最初体积为 2.25 倍，线性尺寸约 1.310371 倍）；膨胀半径保持 0.30 m，保留停车保护。heavy 的 MPPI 改为完整矩形轮廓碰撞检查；全局障碍层只声明有配置的激光源，队友源继续用于局部避障；雷达偏移从放大后的模型同步到车型配置。调度器移除倒退恢复，卡死后取消目标并从当前位置重试；两个 Nav2 默认导航行为树均移除 BackUp，行为服务也移除 backup 插件。普通导航仍允许短暂负向速度。46 项调度测试与静态场景检查通过，动态避障效果待实跑验证。


## 2026-10-09 参数调整

- 所有车型按体积增加 50% 缩放：长宽高乘以 `1.5 ** (1/3)`（约 1.144714）。视觉、碰撞、质量/惯量随 xacro 缩放，轮径、轮距和导航轮廓同步调整。
- 全局及局部代价地图膨胀半径从 0.35 m 改为 0.30 m。
- 删除 Collision Monitor 的 PolygonSlow，仅保留随车体放大的 PolygonStop。MPPI 障碍物评分仍保留。
- 碰撞轮廓（长 × 宽）：light 0.432702 × 0.339981 m；heavy 0.537100 × 0.404771 m；omni 0.505506 × 0.442318 m。
- 配置解析、模型生成和场景净空/连通检查通过；动态会车效果待重启仿真验证。修改后重新编译并重启整套仿真。


四台车、三种构型（`light_1` / `light_2` / `heavy_1` / `omni_1`）在一张**由点云扫描重建的
室内地图**上协同完成「取货 → 送货」任务：Gazebo 负责物理与传感器，每台车跑自己的一套
Nav2，`fleet_manager` 用可解释的成本函数决定「谁去做哪个任务」。

这是 `hetero_fleet_ws`（仓库地图版）的**换地图分支**：世界换成一张 25.0 × 84.2 m 的实景
扫描楼（扫描模型 `geomap_combined12_noroof`，主走廊 ~3 m 宽、两侧挂房间），车型按通道
净空整体缩小；调度层比仓库版多了两件事——**对峙/卡死处理**（standoff watchdog）和
**空闲回充电位**。

> 换地图、缩车型、物理引擎从 Bullet 换到 DART 的全过程记在 [MAP_SWAP.md](MAP_SWAP.md)，
> 那份文档是**当时的历史记录**，其中若干数字已被后续调整覆盖——**当前状态以本文件与代码
> 为准**。日常命令速查见 [RUNBOOK.md](RUNBOOK.md)。

```
  /task_request ─► fleet_manager ─► /<robot>/navigate_to_pose (Action)
                       │                     │
                       │                     └─ 每车一套 Nav2（独立命名空间）：
                       │                        map_server + AMCL + SmacPlanner2D
                       │                        + MPPIController + velocity_smoother
                       │                        + collision_monitor（安全兜底）
                       │                             │ /<robot>/cmd_vel
                       │                             └─ cmd_vel_stamper ─► diff_drive_controller
                       │                                                  / mecanum_drive_controller
                       ▼
        /fleet_status · /task_markers · metrics/metrics.json + metrics/tasks.csv

  物理层  Gazebo Sim 8.11（DART，5 ms 步长，见 §12）+ worlds/geomap_fleet.sdf + 4 个 URDF 实例
  地图层  assets/scan_sources/ 里的原始扫描 → 处理成 models/geomap_combined12_noroof
          （1 个视觉网格 + 2409 个盒体碰撞），栅格化成 maps/geomap.pgm
  桥接层  ros_gz_bridge(/clock, 4×scan, /gz/dynamic_pose) + 4× 真值里程计 relay
```

> 验证状态：本文档的命令与数字都在本机（ROS 2 Jazzy / Gazebo 8.11.0 / 2026-09-24）实跑过，
> 未验证的项目在 §13 单独列出。

---

## 1. 环境要求

| 组件 | 版本 | 说明 |
| --- | --- | --- |
| ROS 2 | Jazzy | `/opt/ros/jazzy` |
| Gazebo Sim | 8.11.0（Harmonic） | `gz sim`，物理用 **DART**（见 §12） |
| Nav2 | 全套 | 实际用到 `nav2_mppi_controller`、`nav2_smac_planner`、`nav2_collision_monitor`、`nav2_amcl` |
| ros_gz | `ros_gz_sim` / `ros_gz_bridge` | |
| ros2_control | `gz_ros2_control`、`diff_drive_controller`、`mecanum_drive_controller` | |
| Python | PyYAML、**numpy**、Pillow | `tools/` 下的地图处理脚本要 numpy + Pillow；运行时节点只要 PyYAML（Pillow 可选） |

### ⚠️ 本机特有：conda 环境会破坏编译

本机 conda 环境 `condaecho`（Python 3.14）会遮蔽 `/usr/bin/python3`，而 ROS 2 Jazzy 的
`rosidl` 需要系统 Python 3.12（`empy` 装在那里）。CMake 的 `FindPython3` 会优先选
`$CONDA_PREFIX`，生成接口时报 `ModuleNotFoundError: No module named 'em'`。

**工作空间根目录的 `colcon_defaults.yaml` 已经修好了这个问题**，不需要手动加参数：

```yaml
{ "build": { "cmake-args": ["-DPython3_EXECUTABLE=/usr/bin/python3"] } }
```

> 该文件只对从**工作空间根目录**执行的 `colcon build` 生效。

---

## 2. 编译

```bash
cd /home/xuzhijie/hetero_fleet_geomap123_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

实测输出：`Summary: 5 packages finished`（全量约 1.5 s，只有改动时才真正重编）。

> 每个新终端都要 `source install/setup.bash`，否则 `ros2 run hetero_fleet_scheduler ...`
> 会报找不到包。

单元测试（39 个：成本函数 + 对峙/充电行为；需要先 source，因为要导入 `hetero_fleet_msgs`）：

```bash
cd /home/xuzhijie/hetero_fleet_geomap123_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
cd src/hetero_fleet_scheduler
/usr/bin/python3 -m pytest test/ -v      # 实测 39 passed
```

---

## 3. 启动

### 3.1 一键启动全部（推荐）

```bash
cd /home/xuzhijie/hetero_fleet_geomap123_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch hetero_fleet_bringup bringup.launch.py
```

启动时序（timer 值来自 `bringup.launch.py` / `nav2.launch.py`）：

| 时刻 | 动作 |
| --- | --- |
| 0 s | Gazebo + `geomap_fleet.sdf` + 4× `robot_state_publisher` + 4× `ros_gz_sim create` |
| 8 s | `bridge.launch.py`：ros_gz bridge（`/clock`、4× `<ns>/scan`、`/gz/dynamic_pose`）+ `scan_frame_relay` + 4× 真值里程计 relay + 4× 静态 `map→<ns>/odom` |
| 13 s | 8 个 controller spawner：每车先 `joint_state_broadcaster`、1 s 后驱动控制器，车与车再错开 2 s |
| 30 s | `nav2.launch.py`：4 套 Nav2，套与套之间再错开 30 s（实测第 1/2/3/4 套的 map_server 在 +30 / +60 / +90 / +120 s 拉起） |
| 150 s | `fleet_manager`（实测 +152 s） |
| 160 s | RViz2（`rviz:=false` 时不启动） |
| 180 s | 根命名空间的 `map_server`（实测 +180 s）：只为 RViz 与 `ros2 topic echo /map` 提供 `/map`，导航用的是每个命名空间自己那份 |
| 205 s | `map_server_watchdog`（一次性；manager 正常工作时 no-op） |
| 215 s | `nav2_stack_watchdog`（一次性） |

整套激活完约 **2.5–3 分钟**，之后才能稳定接任务。启动完自检：

```bash
ros2 action list | grep navigate_to_pose        # 应恰好 4 条
ros2 lifecycle get /omni_1/bt_navigator         # active [3]
ros2 control list_controllers -c /omni_1/controller_manager
#   joint_state_broadcaster   ...  active
#   mecanum_drive_controller  ...  active
```

常用参数：

```bash
# 无显示器 / 省资源：关掉 Gazebo GUI 和 RViz（长跑建议，扫描图渲染很吃 GPU）
ros2 launch hetero_fleet_bringup bringup.launch.py headless:=true rviz:=false

# 只起仿真，不起调度器（手动喂任务）
ros2 launch hetero_fleet_bringup bringup.launch.py scheduler:=false
```

`bringup.launch.py` 自身的参数是 `headless` / `rviz` / `use_sim_time` / `scheduler`；
还透传 include 进来的 `world`、`map`、`controllers_dir`、`params_dir`、`autostart`、
`start_controllers`。全部可用 `ros2 launch ... --show-args` 查看。

### 3.2 长时间运行的资源提示

这张扫描图（25 × 84 m）+ 2409 个盒体碰撞 + 4 套 Nav2 会让整机负载明显高于仓库地图版。
实测建议：`headless:=true rviz:=false`；不要在同一个 `GZ_PARTITION` 上同时开第二个
`gz sim`（见 §14「两个 gz sim 实例」条目）。

物理步长已经为此调过一次：`make_geomap_world.py` 的 `PHYSICS_STEP`。同一台机器上实测
（headless、DART、4 车 5 个雷达）——2.5 ms 步长时窗口 RTF 只有 **0.35**（去掉碰撞盒、
只留视觉网格则回到 1.00，说明代价来自盒体数量而不是网格或雷达），改成 **5 ms（200 Hz）**
后回到 **1.00**（3013 个盒子时 1.00，2365 个时也是 1.00）。
**注意盒子数一路在涨**：门楣过滤 3013 → 3900 → 填空白圈 2344 → 放开封住通道的悬浮家具
**4430**。按线性模型（5 ms 步长、单实例）4430 盒约 **0.83**，所以现在 RTF 不再是稳 1.00，
要压回去有三个旋钮：`PHYSICS_STEP` 调到 0.007（≈143 Hz，仍 >1 个控制器周期）、
`make_collision_boxes.py --block 2`（盒数减半，但每面墙最多胖 5 cm）、
或者把 `--fill-unreachable`/`--free-floating` 的范围缩一缩。

---

## 4. 注入任务

唯一入口是话题 `/task_request`（`hetero_fleet_msgs/msg/Task`）。`task_publisher` 会等订阅者
出现后连发 4 次以对抗丢包；`fleet_manager` 按 `id` 去重。

**方式 A：命令行发单个任务**

```bash
ros2 run hetero_fleet_scheduler task_publisher --single \
  --capability heavy_payload --payload 120 \
  --pickup 2.97 -30.25 --dropoff -1.53 -39.65 --priority 4 --deadline 420
```

**方式 B：回放预设场景**（10 个任务，含 1 个故意不可完成的 200 kg）

```bash
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/demo_10.yaml --interval 8
```

**方式 C：20 任务统计场景**

```bash
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/mixed_20.yaml --interval 6 --shuffle
```

**方式 D：直接发消息**（`id` 必须唯一）

```bash
ros2 topic pub --once /task_request hetero_fleet_msgs/msg/Task \
"{id: 'adhoc_1', type: 'transport', pickup: {x: -1.68, y: -19.00}, dropoff: {x: -1.68, y: -24.95}, payload_kg: 12.0, required_capability: 'light_payload', priority: 2, deadline_sec: 600.0}"
```

字段约定：

* `required_capability`：`light_payload` | `heavy_payload` | `narrow_aisle` |
  `precise_docking` | `omnidirectional` | `""`（空 = 不限）
* 车队与载重：`light_1` / `light_2` 20 kg；`heavy_1` 150 kg；`omni_1` 90 kg
* 任务点坐标（`geomap_fleet.sdf` 里的 10 个 zone 标记，`tools/pick_zones.py` 选、
  `tools/verify_scene.py` 校验）：

| | A | B | C |
| --- | --- | --- | --- |
| pickup | (-1.68, -19.00) | (2.97, -30.25) | (-1.73, -51.05) |
| dropoff | (-1.68, -24.95) | (-1.53, -39.65) | (10.22, -37.50) |

* 充电位（每台车出生点就是自己的充电位）：`charger_A` (-1.60, -11.00) → light_1、
  `charger_B` (-1.60, -12.70) → light_2、`charger_C` (-1.60, -14.40) → heavy_1、
  `charger_D` (-1.60, -16.10) → omni_1

---

## 5. 观察

```bash
ros2 topic echo /fleet_status --once        # 车队快照：状态、电量、队列、makespan、reschedule
ros2 topic echo /task_markers --once        # RViz 面板与任务标签的数据源
ros2 topic echo /light_1/odom               # 真值里程计（Nav2 用的就是它）
ros2 topic echo /light_1/scan               # 激光
ros2 action list                            # 应含 4 条 navigate_to_pose
ros2 topic echo /gz/dynamic_pose            # Gazebo 真值位姿数组（见下方说明）
```

> ⚠️ `bridge.yaml` 里虽然给每台车写了 `/<ns>/gz_pose` 与 `/<ns>/gz_odom`，但 Gazebo 侧的
> `/model/<ns>/pose`、`/model/<ns>/odometry` 在本世界里**不发布数据**，这两组话题实测
> `ros2 topic hz` 无输出。真值来源是 `/gz/dynamic_pose`（`Pose_V → TFMessage`），
> 4 个 relay 从里面按出生点挑出自己那台车，转成 `/<ns>/odom` 和 `odom→base_footprint`。

RViz（`src/hetero_fleet_bringup/rviz/fleet.rviz`）里可以看到：`/map`、TF、4 台车的
RobotModel、4 路激光、规划路径与代价地图，以及 `task_markers`——任务线段（颜色随状态变化）、
任务编号/状态，以及四角固定的每车状态块（电量、载荷、当前任务；颜色与车体一致）。

状态块的位置可以不改代码调（默认值已按这张 25 × 84 m 的图放大：四个角在楼外
~4 m，字号 1.6 m）：

```bash
ros2 run hetero_fleet_scheduler fleet_manager --ros-args \
  -p status_panel_x_min:=-12 -p status_panel_x_max:=25 \
  -p status_panel_y_min:=-82 -p status_panel_y_max:=17 \
  -p status_panel_inset_m:=4.0 \
  -p status_panel_pos_overrides:=['omni_1:6.0,0.5']
```

---

## 6. 分层调试

```bash
# 只起 Gazebo + 4 台车（控制器默认不启动，要显式打开）
ros2 launch hetero_fleet_gz gz_sim.launch.py start_controllers:=true

# 只起 Nav2（假设仿真已在跑）
ros2 launch hetero_fleet_bringup nav2.launch.py

# 不重启仿真，只重启调度器并自动回放场景
ros2 launch hetero_fleet_scheduler scheduler.launch.py \
  tasks_file:=$PWD/src/hetero_fleet_scheduler/config/tasks/demo_10.yaml \
  autostart_tasks:=true task_interval_sec:=8

# 绕过调度器，单测某台车的导航
ros2 action send_goal /light_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 8.10, y: -0.43}, orientation: {w: 1.0}}}}"

# 只问规划器「能不能到」，不真的开车
ros2 action send_goal /heavy_1/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
"{goal: {header: {frame_id: map}, pose: {position: {x: 11.0, y: 4.22}, orientation: {w: 1.0}}}, start: {header: {frame_id: map}}, use_start: false}"
```

`tools/` 下的诊断脚本（详见 §7 与各自 docstring）：

```bash
# 场景静态自检：地图 × 车型 × 任务点
python3 tools/verify_scene.py
# 两台车实际靠多近（车队分离度）
python3 tools/fleet_separation.py
# 记录「指令 vs 实际运动」，用于判断车是不是真的卡了
python3 tools/drive_trace.py --robot omni_1 --seconds 60
# TF 说法 vs 真实位姿
python3 tools/tf_probe.py --robot light_1
```

---

## 7. 项目结构

```
hetero_fleet_geomap123_ws/
├── colcon_defaults.yaml               # 固定系统 Python（见 §1）
├── README.md / RUNBOOK.md / MAP_SWAP.md
├── assets/scan_sources/               # ★ 原始扫描模型（管线第 1 步的输入，见 §12）
│   └── geomap_combined12_noroof/      #   当前地图的源（OBJ 14.2 MB + 贴图 atlas + model.sdf）
├── metrics/                           # 每次运行落盘的指标（§11，不进 git）
├── generated/                         # 每次 launch 渲染出的配置（不进 git）
│   ├── controllers/                   #   每车一份 ros2_control 控制器 YAML
│   └── params/                        #   每车一份 Nav2 参数（已改写命名空间）
├── tools/                             # ★ 离线工具（地图处理 + 诊断），不属于任何 ROS 包
│   ├── prepare_map_mesh.py            #   原始扫描 → 可驱动的地图模型（第 1 步）
│   ├── make_nav2_map.py               #   地图模型 → 占据栅格 maps/geomap.pgm（第 2 步）
│   ├── make_collision_boxes.py        #   栅格 → 盒体碰撞（第 3 步）
│   ├── pick_zones.py                  #   从栅格挑出生点/任务点/充电位
│   ├── verify_scene.py                #   地图 × 车型 × 任务点 的静态自检
│   ├── map_defects.py                 #   ★ 体检：门楣/麻点/门洞瓶颈宽度（换图先跑）
│   ├── measure_footprint.py           #   从 URDF 量出真实足迹（robot_profiles 的来源）
│   ├── fleet_separation.py            #   车队最小间距统计
│   ├── drive_trace.py / chain_probe.py / topic_probe.py / tf_probe.py / omni_probe.py
│   ├── analyze_map.py / map_common.py #   栅格/网格分析，被上面几个脚本复用
│   └── （脚本的临时输出写到 generated/，不落 /tmp）
└── src/
    ├── hetero_fleet_msgs/             # 接口（Task / RobotState / FleetStatus / DispatchTask）
    ├── hetero_fleet_description/      # 模型与能力参数
    │   ├── urdf/common.xacro          #   共享宏（ros2_control / 摩擦 / 材质）
    │   ├── urdf/*_amr.xacro           #   三种机器人（几何 = 原设计 × 缩放系数）
    │   ├── config/*_controllers.yaml  #   每型的 ros2_control 控制器模板
    │   └── config/robot_profiles.yaml #   ★ 唯一参数真源（车队/能力/权重/充电位）
    ├── hetero_fleet_gz/               # 场景
    │   ├── models/geomap_combined12_noroof/ # 处理后的地图模型：视觉网格 + 2409 个盒体碰撞
    │   ├── models/geomap_walls/       #   备用墙体模型（当前世界不用）
    │   ├── worlds/source/             #   ★ 桌面上的 viewer 世界（原始输入）
    │   ├── worlds/make_geomap_world.py#   生成 geomap_fleet.sdf（第 4 步）
    │   ├── worlds/geomap_fleet.sdf    #   生成结果（已入库）
    │   └── launch/gz_sim.launch.py    #   Gazebo + spawn 4 车 + robot_state_publisher
    ├── hetero_fleet_bringup/          # 桥接 / 导航 / 可视化
    │   ├── config/bridge.yaml         #   ros_gz 话题映射
    │   ├── config/nav2_params_*.yaml  #   ★ 三套差异化 Nav2 参数
    │   ├── launch/bringup.launch.py   #   总入口
    │   ├── launch/nav2.launch.py      #   每车一套 Nav2（含命名空间改写）
    │   ├── maps/geomap.{pgm,yaml}     #   由扫描网格栅格化的占据地图（501 × 1684 @ 0.05 m，25.0 × 84.2 m）
    │   ├── rviz/fleet.rviz
    │   └── scripts/                   #   运行时节点：scan_frame_relay / omni_odom_relay /
    │                                  #   cmd_vel_stamper / 两个 watchdog
    └── hetero_fleet_scheduler/        # 调度
        ├── hetero_fleet_scheduler/
        │   ├── task_model.py          #   任务/机器人/指标数据类（无 ROS）
        │   ├── cost.py                #   ★ 可解释成本函数（无 ROS）
        │   ├── fleet_manager.py       #   ROS 节点 + 状态机 + 对峙处理 + 回充电位 + 指标
        │   ├── metrics.py             #   指标记录/导出（JSON + CSV）
        │   └── task_publisher.py      #   任务注入
        ├── config/tasks/*.yaml        #   任务场景（demo_10 / mixed_20）
        └── test/                      #   test_cost.py + test_standoff.py（39 个用例）
```

---

## 8. 异构性体现在哪

**三类机器人参数**（`robot_profiles.yaml`；四台车 = 2 light + 1 heavy + 1 omni）：

| | light_diff_amr | heavy_diff_amr | omni_mecanum_amr |
| --- | --- | --- | --- |
| 实例 | `light_1`, `light_2` | `heavy_1` | `omni_1` |
| max_speed | 0.70 m/s | **0.45 m/s** | 0.55 m/s |
| max_accel / max_omega | 1.0 m/s² / 1.5 rad/s | 0.5 / 0.8 | 0.9 / 1.2 |
| payload | 20 kg | **150 kg** | 90 kg |
| footprint（真实外形） | 0.378 × 0.297 | **0.469 × 0.354** | 0.442 × 0.386 |
| battery | 480 Wh | 1200 Wh | 640 Wh |
| 运动学 | 差速 | 差速 | **mecanum（可侧移）** |
| 能力 | `light_payload`, `narrow_aisle` | `heavy_payload` | `precise_docking`, `omnidirectional` |
| 每米能耗 | 0.85 + 0.012·kg Wh/m | 1.80 + 0.020·kg Wh/m | 1.10 + 0.015·kg Wh/m |

（这些尺寸是把原设计整体缩放后的结果，缩放系数与实测过程见 `MAP_SWAP.md` 第 3 节；
`tools/measure_footprint.py` 能量出 URDF 的真实外接尺寸，就是这张表的来源。
2026-09-28 按用户要求把整车**放大 10%**：`s` 由 0.41 / 0.42 / 0.31 调到
0.45 / 0.46 / 0.34，控制器里的轮径轮距（`*_controllers.yaml`）与 Nav2 足迹多边形
同步更新，`verify_scene.py` 仍通过。）

**三套 Nav2 参数**（`nav2_params_{light,heavy,omni}.yaml`）：

| 参数 | light | heavy | omni |
| --- | --- | --- | --- |
| 足迹多边形（半长 × 半宽） | 0.189 × 0.149 | **0.235 × 0.177** | 0.221 × 0.193 |
| `footprint_padding` | 0.06 | 0.03 | 0.02 |
| `inflation_radius` | 0.35 | 0.35 | 0.35 |
| `cost_scaling_factor` | 3.0 | 1.5 | 2.5 |
| `vx_max` | 0.70 | **0.45** | 0.55 |
| `vy_max`（侧移） | 0.0（禁用） | 0.0（禁用） | **0.55** |
| `wz_max` | 1.5 | 0.8 | 1.2 |
| MPPI `motion_model` | DiffDrive | DiffDrive | DiffDrive（见下） |

和三型差异有关的两点，容易踩：

* `inflation_radius` 三型**相同（0.35）**，差异化在 footprint、`footprint_padding`、
  `cost_scaling_factor` 与速度上限上。旧文档里那张 0.17/0.20/0.21 的表已经不适用。
* `omni_1` 的 MPPI 故意仍用 `DiffDrive`：在这个尺度的仿真底盘上，纯侧移指令只能实现
  一部分，用 `Omni` 模型会让 MPPI 预测出底盘做不出的横移并反复修正。横向能力保留在
  底盘里——`mecanum_drive_controller` 仍接受显式 `linear.y`（`cmd_vel_stamper` 负责
  话题名与 y 符号，仿真底盘对 +y 指令是反向响应）。

**谁走哪条道不是特判，而是参数的后果**：`tools/verify_scene.py` 用同一套 footprint +
inflation 去检查每个任务点的净空，`tools/pick_zones.py` 也是在这个约束下挑点的——所以
「重型车进不去的窄处」在离线阶段就已经被排除掉了。

---

## 9. 任务模型、状态机与两个附加行为

任务字段见 `msg/Task.msg`；场景 YAML 写法：

```yaml
- id: task_004
  type: transport
  pickup: [10.60, -0.43, 0.0]    # x, y, yaw（pickup_A）
  dropoff: [5.40, 4.22, 0.0]     # dropoff_B
  payload_kg: 120.0
  required_capability: heavy_payload
  priority: 4                     # 1..5
  deadline_sec: 420.0             # 从任务到达算起的软截止
```

状态机：

```
PENDING → ASSIGNED → TO_PICKUP → LOADING → TO_DROPOFF → COMPLETED
                          ↘ FAILED / REQUEUED
```

* 导航失败会**重排队一次**（`REQUEUED` → `PENDING`），再失败才 `FAILED`。
* 电量低于 `min_battery_pct`（默认 15%）的车不会被派新任务。
* 没有任何车能满足的任务会一直留在 PENDING 队列，并在日志给出原因，例如
  `no robot for task_010: ... payload 200.0kg > capacity 150.0kg`。
* 只有 IDLE 的车会被派活；同一个 zone 被两车同时使用时，按 1.0 m 间距换一个 approach
  slot（`zone_slot_spacing`，比仓库版的 1.4 m 小，因为这张图的走廊窄，1.4 m 的偏移
  经常直接落到墙里）。

**① 对峙 / 卡死处理（这个工作空间特有的）**

扫描地图的走廊只够两台车勉强错车，所以「两台车对着顶」和「车已经站在任务点上但 Nav2
一直不宣布成功」是这里的常见故障。`fleet_manager` 用一个停滞看门狗处理：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `stall_timeout_sec` | 15 s | 多久没「更接近目标」就算卡住 |
| `stall_move_eps_m` | 0.10 m | 算作进展的最小改善量 |
| `stall_yaw_weight` | 0.30 m/rad | 把「转正朝向」也算成进展（否则终点原地对齐会被误判成卡死） |
| `stall_wander_m` | 1.20 m | 绕路时离上次进展点超过这个距离 → 重新计时，不算卡死 |
| `stall_arrive_radius_m` | 0.30 m | 卡住时离目标这么近，直接**接受到达**而不是倒退 |
| `stall_backoff_m` | 0.55 m | 解对峙时后退的距离；每次重试按 `stall_backoff_growth`(0.6) 递增，上限 1.30 m |
| `stall_max_recoveries` | 3 | 同一段路最多救几次，之后判失败（任务重排队，车从充电位重新出发） |

**② 空闲回充电位**

每台车的出场点就是它的充电位（`robot_profiles.yaml` 里的 `charger:` 键，与 `spawn:` 相同）。
车做完活**不会立刻回**——任务流每几秒就来一条，中途回家会让每条任务都变成 5–15 m 的
空驶；而是：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `return_to_charger` | true | 总开关 |
| `dock_when_idle_sec` | 20 s | 这台车**手上没活可接**且空闲这么久才回桩 |
| `clear_task_point_sec` | 5 s | 但若它正停在 pickup/dropoff 点上，阈值缩短到这个值（不能占着任务点） |
| `clear_task_point_radius_m` | 1.2 m | 判定「停在任务点上」的半径 |
| `charger_arrive_radius_m` | 0.35 m | 离桩这么近就算已到家，不再折腾控制器 |

充电本身在仿真里是"停着不耗电"的语义（电量由 `battery_drain_per_meter_pct` 按里程扣），
没有实现充电功率曲线。

---

## 10. 调度算法：可解释成本函数

```python
cost(robot, task) = w_travel * 行驶时间
                  + w_energy * 能耗
                  + w_queue  * 排队等待
                  + w_late   * 迟到惩罚
                  + w_cap    * 能力/载荷/电量不满足（大惩罚）
```

权重在 `robot_profiles.yaml` 的 `scheduler.weights` 下：

| 权重 | 当前值 | 说明 |
| --- | --- | --- |
| `travel_time` | 1.5 | 主项：谁近谁去 |
| `energy` | 0.02 | 保留小项，让轻车在小件任务上排在重车前（0.85 vs 1.80 Wh/m） |
| `queue_wait` | 0.30 | **当前无效**：只有 IDLE 车会被派活，`busy_until` 恒等于 now |
| `capability_mismatch` | 1000.0 | 事实上的硬约束 |
| `lateness` | 2.0 | 再乘优先级系数 `1 + 0.25·(priority-1)` |

不可行组合用大惩罚而不是无穷大，所以仍会返回一个数并附带原因，日志里能直接看到
「为什么这台车被拒绝」：

```
ASSIGN light_1 -> task_001: total=... [travel=... energy=... wait=0.00 late=0.00]
no robot for task_005: omni_1 INFEASIBLE for task_005: lacks capability 'heavy_payload'
                       (has ['precise_docking', 'omnidirectional']); payload 95.0kg > capacity 90.0kg
```

**替换点**（后续演进，当前未实现）：`cost.py` 里的 `select_robot()` 是唯一决策入口，
换成拍卖 / OR-Tools CP-SAT / 滚动时域只改这一个函数，ROS 侧不动。

---

## 11. 指标

`fleet_manager` 每 `report_period_sec`（默认 15 s）打印一行汇总，并**每次都落盘**
（中途崩溃也留得下数据；`Ctrl-C` 时还会补写一次）：

```bash
cat metrics/metrics.json    # summary（含 per_robot）+ 每个任务明细
cat metrics/tasks.csv       # 逐任务一行，直接喂 pandas/matplotlib
```

**输出目录默认是 `<workspace>/metrics`**，即
`/home/xuzhijie/hetero_fleet_geomap123_ws/metrics/`（与 `src/`、`build/`、`install/` 同级，
已被 `.gitignore` 忽略）。三种改法：

```bash
# 1) 环境变量（对整个 launch 生效）
HETERO_FLEET_METRICS_DIR=/tmp/run_a \
  ros2 launch hetero_fleet_bringup bringup.launch.py
# 2) 只重启调度器时传参
ros2 launch hetero_fleet_scheduler scheduler.launch.py metrics_dir:=/tmp/run_b
# 3) 直接起节点
ros2 run hetero_fleet_scheduler fleet_manager --ros-args -p metrics_dir:=/tmp/run_c
```

记录的量：任务完成率、平均完成时间、makespan、总行驶距离、总能耗、机器人利用率、
重调度次数、迟到任务数，以及每车的完成数 / 距离 / 能耗 / 电量 / 忙碌时间。

**本机一次实测**（`demo_10.yaml`，间隔 8 s 注入，headless，Nav2 真的在开车）——
⚠️ 这组数字是在**上一张图 `geomap_final_noroof`（26 × 10 m 实验室）**上测的；
换成 `geomap_combined12_noroof` 之后**还没有**端到端跑过任务（见 §13「尚未验证」），
新图的对比数字要跑完 `demo_10` / `mixed_20` 之后补：

```
completed=8/10 rate=80% mean_t=141.4s makespan=308.6s
dist=91.8m energy=128.5Wh util=64% resched=19 tardy=1
```

| robot | tasks | distance | energy | battery | busy |
| --- | --- | --- | --- | --- | --- |
| light_1 | 3 | 32.0 m | 29.3 Wh | 92.0% | 298.6 s |
| light_2 | 2 | 29.0 m | 25.8 Wh | 92.8% | 193.4 s |
| heavy_1 | 2 | 21.1 m | 61.4 Wh | 94.7% | 196.3 s |
| omni_1 | 1 | 9.8 m | 12.1 Wh | 97.6% | 103.1 s |

怎么读这组数字：

* **能力/载重过滤是对的**：120 kg 的 `task_004` 只可能给 `heavy_1`；95 kg 的 `task_005`
  明确以 `payload 95.0kg > capacity 90.0kg` 拒掉 `omni_1`；200 kg 的 `task_010`
  从头到尾没被派出去，运行结束时队列里剩的就是它。
* **能耗差异明显**：`heavy_1` 走 21.1 m 花 61.4 Wh，`light_1` 走 32.0 m 只花 29.3 Wh——
  这正是能量模型想要的（重车装重货，每米能耗高一个量级）。
* **迟到统计生效**：`task_008`（60 s 截止、优先级 5）被判 tardy。
* **8/10 不是"两条没人干"**：`task_007` 是 `omni_1` 卡在离 dropoff_C 只有
  0.39–0.44 m 的地方，停滞看门狗连续触发 `recovery 1/3`、`2/3`，最后 requeue 了一次。
  也就是 **9 条可完成任务里完成 8 条**，剩下那条卡在这张窄图的角落——与 §13 记的
  「吞吐/堵车」是同一个已知限制。

> 没有 Nav2 时（例如只想跑调度实验），`fleet_manager` 会退化成按 `距离/最大速度` 的
> 运动学估算，任务仍会「完成」——**那种数字不是导航结果，不要混用**。

**新图 `geomap_combined12_noroof` 上的实测**（4 套 Nav2 全部 active 之后手工注入 3 条
单任务，headless；这是目前新图唯一的真实导航数据，正式场景还没跑）：

```
completed=3/3 rate=100% mean_t=114.5s makespan=193.7s
dist=75.8m energy=153.6Wh util=44% resched=3 tardy=0

light_2  8 kg   pickup_A -> dropoff_A  12.3 m   65.4 s
heavy_1  120 kg pickup_A -> dropoff_B  25.3 m   91.2 s
omni_1   40 kg  pickup_B -> dropoff_C  26.1 m  187.0 s   (中途近目标停滞 0.45 m，
                                                          看门狗 2 次 recovery +
                                                          0.62 m 后退解对峙后完成)
```

任务清空后三台车依次 `returning to charger`，`light_2: docked at charger` —— 新图的
充电排（x=−1.60，y −11.0~−16.1）在真机物理下能停进去。

**异构比例对比**：改 `robot_profiles.yaml` 的 `fleet:` 列表增删实例（例如 2 heavy），
重跑同一份 `mixed_20.yaml`，对比 `metrics.json` 即可。注意增删实例后要同步
`tools/pick_zones.py` 挑出的出生点数量。

---

## 12. 地图与世界的生成链（全部输入都在仓库内）

扫描地图是**四步**生成的，顺序不能反；前三步的输入现在是仓库里的
`assets/scan_sources/`，不再依赖桌面或别的项目目录。

**下面这组参数就是仓库里现有产物的参数**（用它们重跑，视觉网格、碰撞网格、占据栅格、
`model.sdf` 四份产物都会与入库文件一致）：

```bash
cd /home/xuzhijie/hetero_fleet_geomap123_ws

# 1) 原始扫描 → 可驱动的地图模型（拟合地板、剔除地板面、烘焙水平变换）
python3 tools/prepare_map_mesh.py \
  assets/scan_sources/geomap_combined12_noroof/meshes/geomap_combined12_noroof.obj \
  src/hetero_fleet_gz/models/geomap_combined12_noroof \
  --name geomap_combined12_noroof --keep-above 0.40 --drop-above 2.00

# 2) 模型 → 占据栅格（Nav2 用的地图）+ 报告
python3 tools/make_nav2_map.py \
  src/hetero_fleet_gz/models/geomap_combined12_noroof/meshes/geomap_combined12_noroof.obj \
  src/hetero_fleet_gz/models/geomap_combined12_noroof/meshes/geomap_combined12_noroof_collision.obj \
  src/hetero_fleet_bringup/maps/geomap --report --smooth 1 --structure-dilate 0 \
  --squash-mesh 0.15 --squash-max 2.00 --fill-outlines --min-width 0 \
  --fill-pits 0.15 --fill-pockets 0.5

# 3) 栅格 → 盒体碰撞（写回 model.sdf；DART 不能用 20 万三角面，见下）
#    必须在仓库根目录执行：--map 默认是相对路径 src/hetero_fleet_bringup/maps/geomap
python3 tools/make_collision_boxes.py \
  --model src/hetero_fleet_gz/models/geomap_combined12_noroof/model.sdf

# 4) 世界文件（含 10 个 zone 标记）
python3 src/hetero_fleet_gz/worlds/make_geomap_world.py

# 附带：重新挑点 / 静态自检
python3 tools/pick_zones.py src/hetero_fleet_description/config/robot_profiles.yaml \
  --model-name geomap_combined12_noroof --spawns 4 --zones 6 \
  --spacing 3.0 --spot-margin 0.25 --seed -1.53 -39.65
python3 tools/verify_scene.py
```

四个容易踩的点：

* `--keep-above` **不是越小越安全**。`1.0`（按"机器人能否从下面钻过去"来选）会让雷达
  扫到大量地图里没有的低矮家具，Nav2 在空旷处直接报 `no valid path found`；
  `0.40`（贴近雷达高度）才是这张图在用的值。
* `--drop-above` 是这条链上**另一头**的同一把尺子，缺了它门洞会被封死：俯视投影
  分不出"墙"和"门楣"。门楣/过梁/管道/吊牌（最低点 2.0~2.4 m）投到栅格里跟墙一模一样，
  于是它悬挂的那个门洞在地图上就是一条实心黑带，RViz 里看就是"门框被压成不可通行"。
  **但阈值不能按"车队高度"来定**：这份扫描把一部分家具只扫到了**顶面**（x 5~10、
  y −31~−32 那张工作台，顶面 1.5~2.4 m，下面几乎没有几何），阈值取 1.5 m 会把整张
  工作台从地图上抹掉。实测四个取值的取舍是：

  | drop-above | 工作台框内障碍 | 可通行 | 最大连通区 | 门洞 |
  | --- | --- | --- | --- | --- |
  | 关 | 7.55 m² | 717.4 m² | 541.1 m² | 关 |
  | 1.5 | 4.05 m²（工作台被删） | 814.0 m² | 714.1 m² | 开 |
  | **2.0（在用）** | **6.03 m²** | 760.2 m² | 669.6 m² | **开** |
  | 2.2 | 6.03 m² | 750.4 m² | 617.0 m² | 关 |

  也就是说 2.0 m 才是分界：只丢掉"完全在 2 m 以上"的东西（门楣 2.0~2.4 m、吊顶残料），
  任何有几何低于 2 m 的家具都留下。这个过滤器**只会删障碍**（实测"原本可通行、被新堵
  上"的格子是 0.0 m²），换个扫描想重新定这个值时，跑
  `tools/map_defects.py --doorway` 会把同样的取舍打出来。
* 第 1 步的**输出目录名会成为 SDF 里的模型名**（`model://geomap_combined12_noroof`），
  所以别把它改成别的目录名，否则第 4 步生成的世界会指向一个不存在的模型。
* `pick_zones.py` 只负责**给出候选**（这份地图上它会挑出沿主走廊散开的 10 个宽敞点），
  世界里的 zone 表是人工按"充电位排成一行、作业点沿走廊铺开、别堵门洞"整理过的，
  所以两者数字不完全相同；`verify_scene.py` 校验的是**世界里的那一份**。
* 第 2 步的三个 `--fill-*` 是**收尾**（都跑在 `--smooth` 之后，因为 3x3 中值投票本身
  会在拐角重新造出几个单格麻点）：
  `--fill-pits 0.15` 填掉"四周全是可通行地面"的孤立障碍小块——**值要小**，因为扫描把
  一些家具也打成了小块（工作台那一片就吃过这个亏，0.5 m² 会再削掉 0.53 m²）；
  `--fill-pockets 0.5` 填掉被障碍包住、比机器人还小的自由小口袋；
  `--fill-unreachable 20` 填掉**不属于最大可通行连通区**的自由块（这份图有 28 块、
  合计 90.5 m²，最大 13.8 m²）——它们是真实地面但车队到不了，在 RViz 里就是满屏
  "空白圈"。填完之后 `free components: 1`，地图上只剩一片能走的白区。
* 第 2 步的 `--fill-notches 1.0` 是**把家具块的豁口填平**：对障碍做形态学闭运算，
  凡是宽度小于 1 m 的凹口（扫描漏掉的家具缝、桌面上被啃掉的一角）都补成实心，
  凸出来的形状不受影响。**上限就是 1 m**：实测 1.2 m 会把那道 1.4 m 的翼楼门洞
  一起封上（净空直接掉到 0）。这份图上 1.0 填掉约 51 m² 的窄缝，`verify_scene.py`
  里所有任务点的净空**一个都没变**（翼楼门洞仍是 1.40 m）。
  **闭运算还必须带"通道守卫"**：凹口和门洞在栅格上长得一样，一版没加守卫时它把
  **7 处通道**补成了实心——最狠的一处只有 0.11 m²，却把一间 **17.7 m² 的房间**和主区
  切断（切断了再被 `--fill-unreachable` 填成黑，看上去就是"整个房间不见了"）。
  现在的做法：对闭运算新加的每一块障碍，检查它两侧的自由格在闭运算前是否连通，
  若连通则说明那是通道而不是豁口，**整块撤回**。实测封堵量 53.3 → 50.7 m²、
  被误封通道 **7 → 0**，可通行面积也从 618.8 回到 **643.0 m²**。
  排查脚本：`tools/map_defects.py`（外加这份工作留下的 `find_sealed.py` 思路）。
* **扫描毕竟不是 CAD，最后还有一张手动编辑表**：
  `src/hetero_fleet_bringup/maps/map_edits.yaml`。里面按 `erase` / `fill` 两类列了
  世界坐标的框（`[x0, y0, x1, y1]`），由 `make_nav2_map.py` 在自动流程之后、填空白圈
  之前套用，**每条都要写理由**。这样"这里多了一块 / 这里少了一块"不用手改 pgm，
  重跑四步链还是同一张图。现有的条目：`(2, −11)` 那块的楔形突出物——扫描只扫到它
  0.4~1.5 m 的顶面（只有 0.18 m² 的几何低到机器人高度），投影出来却是一根扎进房间的
  楔子，机器人本来就能从下面过，所以 `erase`。
* **`--free-floating`：只放开"把通道封死的悬浮家具"**，这是这张图最关键的一条。
  俯视投影分不出"墙"和"只扫到顶面的家具"：这份扫描把成片家具记成 0.5~1.5 m 的悬浮面，
  投影之后把它们**下面所有能走的地方**都堵死了——地图里建筑内部有 637 m² 障碍，
  而 Gazebo 的网格在那个高度根本没有几何。判据和"通道守卫"完全一致：一块悬浮几何
  **只有把原本连通的自由空间切成两半时**才删掉；屋子中间挡视线但绕得过去的家具
  （比如用户要求保留的工作台）仍然留在地图上。实测放开 **283.3 m²** 封堵几何，
  可通行 643.0 → **953.2 m²**，东侧翼楼从"整片黑"变成"家具块 + 通道"。
  对照检查工具见 `tools/map_defects.py`（它会把网格里"跨过机器人高度带 0.05~0.45 m"
  的面栅格化，再看地图里哪些障碍落在这个集合之外）。
* **`--rectangularize 0.53`：把家具补成实心矩形**（2026-09-28 用户换网格后要求
  "地图太破碎，只留通道、家具区域变成不可通行矩形"）。扫描留下的家具是一堆小碎斑，
  读起来像麻子：这一步把每个"厚于 0.53 m"的紧凑障碍块用它的**外接矩形**填实，
  薄长的（墙）不动。**硬约束：只填净空 < 0.53 m 的格子**——那种格子本来就没有任何
  车型能占用，所以补矩形永远不会吃掉通道中心线（第一版没有这条约束，直接把
  `pickup_C` 压到 0.40 m、`dropoff_B` 压到 0.20 m，`verify_scene.py` 立刻报错）。
  配合这套"简化档"使用时：`--min-width 1.00`（比 1 m 窄的可走空间填掉）、
  `--fill-notches 1.6`（<1.6 m 的家具豁口封实）、`--fill-pockets 2.0`、
  `--fill-unreachable 80`。实测内部障碍孤块 **39 → 12**，工作台区域变成整块矩形，
  通道全部保留（`verify_scene.py` 通过，各点净空 0.81~1.45 m）。
* **`--squash-mesh 0.15`：把 Gazebo 地图直接压平**（用户 2026-09-28 的最终要求）。
  不再只投影碰撞网格里“高于 0.4 m”的那些面，而是**把可视网格里一切高于地面 0.15 m
  的面整体俯视投影**（墙、家具顶、门楣、杂物都算，只排除地面本身）。地图于是长成
  “Gazebo 从上往下看”的样子，而不是只剩几块被过滤保留下来的表面碎片。
  压平之后再跑 `--free-floating` 把**堵住通道**的部分打开（这一版放开 429.4 m²），
  通道就留出来了。实测：障碍 1759.0 m²、可通行 **973.3 m²**、**1 个连通块**、
  `verify_scene.py` 通过、盒体碰撞 4300。
  取舍：地图忠实于 Gazebo（工作台/家具都在），只有**确实把可走空间切成两半**的
  悬浮几何才被放开——这就是“把工作站保留、把通道留出来”的落点。
* **`--fill-outlines`：把扫描出来的“描边”家具填成实心块**（同一轮加的）。这份扫描把多数
  家具记成**表面壳**（书桌只有一个矩形轮廓），俯视投影就是一团描边，而 Gazebo 里看是
  实心家具。把结构掩膜里每个闭合轮廓的内部填实（+84.1 m²），地图才像 Gazebo 的样子；
  闭合轮廓内部本来就到不了，所以不会吃掉任何可走空间。
  这一轮的最终档：`--squash-mesh 0.15 --fill-outlines --free-floating --min-width 1.20
  --fill-pits 0.15 --fill-pockets 2.0 --fill-unreachable 40 --fill-notches 1.6`。
  实测：障碍块 **39 → 21**、周长/面积 0.639 → **0.416**（越接近矩形越干净）、
  可通行 709.4 m²、1 个连通块、各任务点净空 1.00~1.64 m、盒体碰撞 2777。

### 第六轮（2026-09-28 晚）：RViz 地图 vs Gazebo 的保真度修复

用户看 RViz 与 Gazebo 并排，报了三件事：**部分通道被封死**、**部分家具消失**、
**家具里出现可通行区域（应该是一块不可通行的矩形）**。量出来确实很大：把可视网格
按"地面以上 0.15 m 以上、最低点低于 2 m"投影成参照真值后，旧图有
**140.0 m² 的障碍压在 Gazebo 明明开着的地面上**（其中一条 28.7 m² 直接把通道切断）、
**94.8 m² 的家具在图上不见了**、**84.9 m² 的可通行格子被家具包着**（RViz 里就是
"家具中间一个洞"）。三个成因都在参数上：

* `--free-floating` 会**整块删掉**"把自由空间切成两半"的悬浮几何。它确实打开了通道，
  但删的是整块家具——于是家具消失、或者家具中间被切出一条可走的路。**这就是第 2、3 条。**
* `--min-width 1.20` + `--fill-notches 1.6` + `--fill-unreachable 40` 会**填掉窄缝**、
  封掉家具豁口、把不连通的自由块涂黑。**这就是第 1 条（封死的通道）。**
* `--squash-mesh 0.15` 只设了下限、没有上限，**门楣/横梁/管道也被压平**，把它悬挂的
  门洞整条封死（`prepare_map_mesh.py --drop-above` 治过碰撞网格，但压平用的是可视网格，
  所以要在这一侧再治一次）。

改法（新增一个上限开关，其余全是关掉过度的后处理）：

```bash
python3 tools/make_nav2_map.py <obj> <collision.obj> src/hetero_fleet_bringup/maps/geomap \
  --report --smooth 1 --structure-dilate 0 \
  --squash-mesh 0.15 --squash-max 2.00 --fill-outlines --min-width 0 \
  --fill-pits 0.15 --fill-pockets 0.5
```

* **`--squash-max 2.00`（新）**：压平时**丢掉完全位于 2 m 以上的面**（门楣、横梁、
  吊牌、管道）。房间从上方看是什么样，地图就是什么样；门洞不再被它头顶的东西封死。
  和 `prepare_map_mesh.py --drop-above` 是同一条判据，只是作用在可视网格上。
* **不再用 `--free-floating`**：它正是"家具消失/家具中间多一条路"的来源。悬浮几何现在
  老老实实按 Gazebo 的样子留在地图上——车开不进桌子底下，是因为桌子本来就在那儿。
* **`--min-width 0`、`--fill-notches 0`、`--fill-unreachable 0`**：这三条都会把
  Gazebo 里开着的地方涂黑。窄缝交给代价地图的膨胀层和 SMAC 的真足迹校验去挡，地图本身
  保持忠实。
* `--fill-outlines` 保留（描边家具填成实心块）、`--fill-pits 0.15`、`--fill-pockets 0.5`
  保留（只去噪点）。

| 指标（对 Gazebo 可视网格） | 旧图 | **现在** |
| --- | --- | --- |
| 封死的通道（地图挡、Gazebo 开着） | 140.0 m² / 29 处切断 | **7.8 m² / 2 处** |
| 消失的家具（Gazebo 有、地图可走） | 94.8 m² | **1.2 m²** |
| 家具内部的"洞" | 84.9 m² / 27 块 | **0.0 m²** |
| 实车雷达实测：地图挡、雷达看得穿 | 194 束 | **36 束** |
| 实车雷达实测：地图可走、雷达打到东西 | 25 束 | 26 束（多数是波束擦到扫描地板） |
| 可通行面积 / 连通块 | 709.4 m² / 1 | 748.0 m² / 12 |

> 连通块从 1 变成 12 是**有意的**：那 11 块是墙外、家具缝隙里 Gazebo 本来就能看见的
> 地面，在 RViz 里是白色小块。以前靠 `--fill-unreachable 40` 把它们涂黑来"好看"，
> 代价就是把真实地面说成墙。要用旧口径就把 `--fill-unreachable 20` 加回去。

两个配套检查工具（这轮新加）：

* `python3 tools/map_fidelity.py [--png …]`：把上面三条缺陷**按面积和坐标**打出来，
  不依赖 Gazebo，直接拿可视网格当真值。改参数后先跑它。
* `python3 tools/scan_vs_map.py [--png …]`（仿真起来后跑，需要 `/usr/bin/python3`）：
  把**小车真实激光**和地图逐束对比，分成"地图漏了障碍 / 地图凭空挡路 / 挡的是家具
  （正常）/ 打到别的车（正常）"四类。注意 4 台车相距 1.7 m，37% 的光束是打到邻车的。

改了世界或地图记得 `colcon build`（`models/`、`worlds/`、`maps/` 都要装进 `install/`）。

为什么必须是盒体碰撞：地图的三角网格有 29.7 万面，本机 DART 直接用网格碰撞会段错误，
而 Bullet-Featherstone 虽然不崩，却**无法让轮式底盘原地转向**（轮子按指令反向转、车身
yaw 不动，所有需要转向的目标都以 `Failed to make progress` 结束）。现在改成：DART +
由占据栅格生成的 4430 个轴对齐盒体，物理与代价地图描述的是同一个房间。
完整排查过程见 `MAP_SWAP.md` 第 6 节。

盒体数量就是物理步长的预算：这张楼比上一张 26 × 10 m 的实验室图**碎得多**（障碍物
1293 m²，要靠 2409 个盒子才能贴住 5 cm 的占据栅格），实测代价几乎与盒体数成正比，
所以 `PHYSICS_STEP` 从 2.5 ms 放到了 5 ms（§3.2 有数据）。想再压时间的话，优先动
`make_collision_boxes.py`：`--margin 0.15` 能减 9%（只去掉机器人在物理上够不到的死角，
几何不变，安全）；`--block 2` 能减到 ~1740（便宜，但每面墙最多胖 5 cm，可能让机器人
卡在规划器认为能过的地方，非必要别用）。

---

## 13. 已验证 / 未验证

**换成 `geomap_combined12_noroof` 时实测通过（2026-09-24，独立 `GZ_PARTITION` 的干净实例）**

* `colcon build` → `Summary: 5 packages finished`（8.3 s）
* 单元测试 **39 passed**（成本函数 + 对峙/充电行为）
* 地图链四步全部重跑成功，产物与本节数字一致：147053 顶点 / 294660 面的扫描 →
  视觉网格 + 碰撞网格 → 占据栅格 → `model.sdf`（1 个视觉网格 + **2409 个盒体碰撞**）
* 地图：**649 × 1684 @ 0.05 m = 32.4 × 84.2 m**，origin `[-6.1045, -74.4206]`
  （2026-09-28 用户把扫描网格往东扩到 x=25.7 之后重新生成：147053 顶点 / 294660 面）
  障碍 2048.7 m²，可通行 **683.6 m²**，**1 个连通块**，内部障碍孤块 **39 → 12**
  （大面积家具已补成实心矩形，通道全部保留）
  （门楣过滤前是 718.3 m² / 24 块 / 541.1 m²；只做门楣过滤时是 816.4 / 33 块 / 716.5 m²）
* 世界：world `geomap_fleet`，1 个 include（`model://geomap_combined12_noroof`）+ 地面 +
  10 个 zone 标记（3 pickup / 3 dropoff / 4 charger），插件 Physics(DART) / Sensors(ogre) /
  SceneBroadcaster / UserCommands / Contact，`xmllint` 通过
* `tools/verify_scene.py` → `RESULT: scene is consistent`：4 台车的出生点 + 10 个 zone
  全在自由格上、净空 ≥ 各车型需求（最宽的 omni 需 0.53 m）、且**都在同一个连通区**
  （region 1），任务点两两直线距离 5.9~26.1 m；门楣过滤后 `charger_B` 的净空还从
  0.97 m 涨到 1.35 m
* **门洞不再被门楣封死**（2026-09-28 修）：碰撞网格里"最低点 ≥ 2.0 m"的面降到 **0**
  （阈值取 1.5 m 会连工作台一起删掉，见 §12 那张表），106 m² 的翼楼通过一道 1.40 m
  宽的门洞重新接进来；同时**没有任何原本可通行的格子被新堵上**（0.0 m²）
* **工作台已恢复**：x 5~10、y −31~−32 那块在阈值 1.5 m 时被当成门楣整片删掉
  （框内障碍 7.55 → 3.52 m²），改成 2.0 m 后回到 **6.03 m²**
* **空白圈填平**：`--fill-unreachable 20` 填掉 28 块不可达自由块（合计 90.5 m²，
  最大 13.8 m²），地图变成 `free components: 1`；`--fill-pits 0.15` 之后孤立障碍小块
  最小 4.84 m²（都是真家具，不再有扫描噪点）
* Gazebo（headless，DART）：世界加载无报错，4 台车**精确落在配置的出生位姿**上
  （z = 0.117 = 地面驱动平面），8 个控制器全部 `Configured and activated`
* 物理实时率（3013 盒、门楣过滤前）：`/stats` 的 `real_time_factor` 稳定在 **1.00**
  （5 ms 步长）；同样条件换成 2.5 ms 步长只有 0.35，去掉盒体只留视觉网格则回到 1.00
  —— 代价来自盒体数量
* 运动实测：给 `light_1` 发 `-0.35 m/s` 共 6 s，从 (-1.600, -11.000) 直线走到
  (-3.073, -11.000)，y/z 不变 —— 轮子在新地图的盒体碰撞上正常滚动，没有穿地
* 门楣过滤（2.0 m）+ 填空白圈之后重跑同一套检查：4 车仍精确落在出生位姿
  （z ≈ 0.112~0.117 ≈ 驱动平面），`light_1` 再走一次直线 6 s 位移 -1.486 m、y 不变；
  2365 个盒子下 `real_time_factor` 10 次采样**全部 1.00**（盒子数从 3013 降到 2365，
  把之前 0.94 的损耗也还回去了）
* **端到端跑通 3 条真实任务**（`bringup.launch.py headless:=true rviz:=false`，4 套 Nav2
  全部 active 后注入）：`light_2` 完成 8 kg（pickup_A→dropoff_A，12.3 m / 65.4 s）、
  `heavy_1` 完成 120 kg（pickup_A→dropoff_B，25.3 m / 91.2 s）、`omni_1` 完成 40 kg
  不限能力（pickup_B→dropoff_C，26.1 m / 187.0 s）—— **3/3 完成、0 失败**，
  makespan 193.7 s、总里程 75.8 m、总能耗 153.6 Wh、利用率 44%、重调度 3 次
* **空闲回充电位在新地图上生效**：任务清空后三台车各自 `returning to charger`，
  `light_2: docked at charger, battery 93.9%`，最终停在新的充电排
  (-1.52,-12.79)≈ charger_B (-1.60,-12.70)
* `omni_1` 在 pickup_B 前 0.45 m 处触发过一次已知的近目标停滞：停滞看门狗
  `recovery 1/3` → `2/3`，再由对峙逻辑 `reversing 0.62 m to break the standoff`
  解开，随后正常取货送货 —— 和上一张图 §11 记的 `task_007` 是同一种（已兜住的）行为
* 物理实时率：只有 Gazebo + 4 车时 `real_time_factor` 稳定 **1.00**；整套
  （4 套 Nav2 + 调度器）跑起来后平均 **0.77**（最低 0.58），此时 load average 19/16 核
  —— 瓶颈是 Nav2 的 CPU 占用，不是仿真步长

**尚未验证 / 存疑**

* **正式场景（`demo_10` / `mixed_20`）还没在这张图上跑过**：上面那 3 条是手工注入的
  单任务，足以证明"新地图能导航、能取送、能回充"，但 §11 里的
  makespan / 利用率 / 里程统计仍是上一张图的，跑完预设场景再补。
* **GUI 路径**：本次全程 `headless:=true rviz:=false`；扫描图的颜色需要 `ogre`(1) 引擎，
  这条（`gz_sim.launch.py` 已传 `--render-engine ogre`）没有在图形界面里复核。
* **吞吐/堵车**：这张图的主走廊 ~3 m 宽、门洞净空 1.0~1.6 m，比上一张图宽敞，但车队
  作业点沿 40 m 走廊铺开，仍可能出现会车对峙；对峙看门狗会解，但会增加单任务耗时。
* **长跑稳定性**：只跑了单次会话。
* `models/geomap_walls/` 当前世界不用，是换图前的遗留（`geomap_colour_noroof` /
  `geomap_final_noroof` 两个旧地图模型已从本工作空间移出，原工作空间里仍在）。

**已知但无害的小问题**

* `light_*` 的 `planner_server` 会打印 `Inflation layer either not found or inflation is
  not set sufficiently ...` 的 ERROR：`inflation_radius` 0.35 相对足迹偏小。实测不影响
  规划（能正常出路径），想消掉就把它抬到足迹最大半宽（≈0.18）以上更多一些。
* `bt_navigator` 会打印 `Error_code parameters were not set`，用默认值即可。

---

## 14. 常见问题

**`ModuleNotFoundError: No module named 'em'`**
→ 见 §1。确认在**工作空间根目录**编译，且 `colcon_defaults.yaml` 存在。

**`Unable to parse the value of parameter robot_description as yaml`**
→ 已修复。xacro 输出的是 XML 不是 YAML，必须用
`ParameterValue(robot_desc, value_type=str)` 包裹。

**`Failed to load system plugin [gz_ros2_control-system]`**
→ 已修复两处：① 用 `ros_gz_sim` 自带的 `gz_sim.launch.py`（它会设
`GZ_SIM_SYSTEM_PLUGIN_PATH`），不要自己 `ExecuteProcess(["gz","sim",...])`；
② URDF 里 `<gazebo><plugin>` 必须是系统插件类名
`gz_ros2_control::GazeboSimROS2ControlPlugin`，不是 `gz_ros2_control::GazeboSimSystem`。

**`The 'type' param was not defined for 'diff_drive_controller'`**
→ 控制器 YAML 里每个控制器必须同时有 `type:` 和 `ros__parameters:`，同级。

**`Parameter 'left_wheel_names' cannot be empty`**
→ 参数层级放深了，必须在 `<控制器名>/ros__parameters/` 下。

**`Switch controller timed out after 5 seconds`**
→ spawner 起太早，`/clock` 还没通。`bringup.launch.py` 在 13 s 时统一启动；
单独用 `gz_sim.launch.py` 时要加 `start_controllers:=true`（否则控制器不启动）。

**`Failed to acquire lock in 20 seconds` / spawner 连环超时**
→ **不是 `/dev/shm` 的问题**。spawner 之间靠
`~/.ros/locks/ros2-control-controller-spawner.lock` 串行化（源码
`controller_manager/spawner.py`：等 20 s、重试 5 次）。第一个 spawner 长时间占锁
（通常是 `controller_manager` 在等 `/clock`）就会连环超时。真正的根因在工作空间之外：
见下一条。

**两个 `gz sim` 实例 / 车不动 / TF 报 `two or more unconnected trees`，
日志出现 `Found additional publishers on /clock`**
→ 这台机器上常有**别的会话遗留的 `gz sim`**（本机确实有一个来自
`~/Documents/Codex/2026-09-18/pcd-gazebo` 的实例）。两个实例共用同一个 Gazebo
transport 分区时，`/clock`、`/stats`、`/world/*/dynamic_pose/info` 会互相污染：
本空间的表现是 4 个里程计 relay 收到 `transforms: []` → `/<ns>/odom` 与 TF 全空 →
Nav2 一直 `Timed out waiting for transform`、spawner 连环超时。处理：

```bash
ps -eo pid,etimes,args | grep "gz sim"     # etimes 很长 = 上个会话的残留
pkill -INT -f "gz sim"                     # 确认不是自己正在用的实例
```

或者给本次运行一个独立分区：

```bash
export GZ_PARTITION=geomap_$(date +%s)
ros2 launch hetero_fleet_bringup bringup.launch.py headless:=true rviz:=false
```

**RViz 里门洞被一条黑带封住 / 明明是门的地方不能走**
→ 门楣（过梁）投到栅格上了：俯视投影分不出"墙"和"挂在门洞上方的结构"。
`tools/prepare_map_mesh.py` 的 `--drop-above` 就是治它的——碰撞网格里**最低点**高于该值的
面（门楣、横梁、管道、吊牌）整片丢掉。这张图用 **2.0 m**：取 1.5 m 会连只扫到顶面的
家具（工作台）一起删掉，取 2.2 m 又有门洞重新被关上（§12 有四个取值的实测表）。
重建顺序见 §12 四步链；只改参数不改代码时记得 `colcon build`
（`models/`、`maps/`、`worlds/` 都要装进 `install/`）。
注意门洞通了不代表机器人一定能过：还要看**净空**够不够（本车队最宽的 omni 要 0.53 m，
即门净宽 ≥ ~1.1 m 才舒服），`tools/verify_scene.py` 会报每个点的净空。

**地图上有一粒粒黑点/白点，或者空地中间有个小坑**
→ 扫描噪点，和第 2 步的去麻点收尾有关：`--fill-pits` 填掉四周全是可通行地面的孤立障碍
小块（用 0.15 m²——**别调大**：扫描把一些家具也打成了小块，0.5 m² 会削掉工作台的一角），
`--fill-pockets` 填掉被障碍包住的自由小口袋。

**RViz 里到处是"空白圈"（墙里面一个个白块），看着很乱**
→ 那是**不可达的自由地面**：扫描把家具之间的空隙也扫出来了，但车队过不去，于是成了主
连通区之外的一块块白斑。`tools/make_nav2_map.py --fill-unreachable 20` 把不属于最大
可通行连通区的自由块填成障碍（这张图填了 28 块、90.5 m²，之后 `free components: 1`）。
想保留大房间、只填小圈就把数字调小（例如 `--fill-unreachable 3`）。
这几个 `--fill-*` 必须和 `pick_zones.py` 用的一致，否则挑点的栅格和 Nav2 加载的不是同一张。

**Gazebo 里明明是通的，RViz 地图上却是一条黑带（通道被堵）**
→ 俯视投影把"只扫到顶面的家具"当成了墙。这份扫描把成片家具记成 0.5~1.5 m 的悬浮面，
投影下来把它们**下面所有能走的地方**都堵死了。`make_nav2_map.py --free-floating`
只删掉**把原本连通的自由空间切成两半**的悬浮几何（判据与填豁口用的"通道守卫"一致），
屋子中间绕得过去的家具照旧保留。实测放开 283.3 m² 封堵几何、可通行 643 → 953 m²。
想看"地图哪一块其实没几何挡着"，用 `tools/map_defects.py` 或把网格里
"跨过 0.05~0.45 m 高度带"的面栅格化后和 `geomap.pgm` 逐格对比。

**家具块的边上有一个个豁口 / 矩形被啃掉一角，想填平**
→ `make_nav2_map.py --fill-notches 1.0`：对障碍做形态学闭运算，宽度小于 1 m 的凹口
全部补上。**不要超过 1 m**——1.2 m 会把那道 1.4 m 的门洞一起封死（实测净空掉到 0）。
闭运算自带"通道守卫"：新补的每一块都会检查它是否把原本连通的自由空间切成两半，
是就撤回（第一版没这道检查，把 7 处通道补成了实心，其中一处 0.11 m² 的塞子切掉了
一间 17.7 m² 的房间）。所以看到"某条通道被莫名其妙堵上"，先跑一遍
`--fill-notches 0` 对比，或者直接看下面的 `map_edits.yaml`。

**某一处障碍明显是扫描的错（多了一块 / 少了一块），想单独改**
→ 用 `src/hetero_fleet_bringup/maps/map_edits.yaml`，别手改 `geomap.pgm`：

```yaml
erase:                      # 强制变成可通行
  - [0.15, -11.75, 2.25, -10.80]   # (2,-11) 那根楔子：只扫到顶面，机器人能从下面过
fill:                       # 强制变成障碍
  - [x0, y0, x1, y1]
```

坐标是世界米（x 东、y 北），`make_nav2_map.py` 在自动流程之后套用，重跑四步链即可复现。
对照 [generated/map_coords/](generated/map_coords/) 里那五张坐标图报坐标给我也行。

**仿真跑得比实时慢很多（`gz topic -e -t /stats` 里 `real_time_factor` 0.3~0.7）**
→ 看碰撞盒数量，不看机器人。这张图的 `models/geomap_combined12_noroof/model.sdf` 里有
2409 个盒体碰撞（上一张图只有 477 个），DART 每步都要为每个机器人形状过一遍它们，
代价几乎与盒体数成正比：实测 3013 盒 + 2.5 ms 步长 → RTF 0.35，同样条件下的视觉网格
单独（无碰撞盒）→ 1.00，5 ms 步长 + 2365 盒 → 1.00。所以世界默认用
`PHYSICS_STEP = 0.005`（在 `worlds/make_geomap_world.py` 里，改完要重跑第 4 步并
`colcon build`）。要再快就减盒子（`make_collision_boxes.py --margin 0.15`，几何不变），
不要用 `--block 2`——它让物理墙比代价地图胖，机器人会卡在"规划器认为能过"的地方。

**车只会直走 / 原地转不动 / 任务总是 `Failed to make progress`**
→ 先确认物理引擎：

```bash
grep -n "engine" src/hetero_fleet_gz/worlds/geomap_fleet.sdf   # 必须是 dartsim
```

Bullet-Featherstone 在本机无法让轮式底盘原地转向（连 Gazebo 官方 diff-drive 示例
强制 Bullet 也一样）。改了引擎或地图后要 `make_geomap_world.py` 重生成世界 + `colcon build`。
另外**不要用 `/joint_states` 判断轮子转没转**（本机 gz_ros2_control 的关节反馈会回显指令）：
用 `gz model -m light_1 -p` 读模型位姿，或用 `tools/drive_trace.py` 对比指令与实测。

**`ros2 run hetero_fleet_scheduler task_publisher` 找不到包**
→ 忘了 `source install/setup.bash`。本工作空间安装后**没有** `install/.../bin/`，
脚本在 `install/hetero_fleet_scheduler/lib/hetero_fleet_scheduler/` 下；实测
`ros2 run` 正常（`ros2 pkg executables` 能列出 `fleet_manager`、`task_publisher`）。

**车悬浮 / 陷进地面**
→ 扫描地板已校平到 z≈0，世界的驱动面在 z = 1.117e-01（见 `make_geomap_world.py`
的 `MAP_Z` / `GROUND_Z`）。换新图时重跑 §12 的四步即可，第 1 步会自己拟合地板平面。

**想用 SLAM 建图替代离线栅格化**
→ 当前 `maps/geomap.pgm` 是把扫描网格按"机器人高度以下"投影得到的。若要真实 SLAM：
起 Nav2 时加 `slam:=true`（需自行加 slam_toolbox 节点），遥控一台车走一圈，
`ros2 run nav2_map_server map_saver_cli -f maps/geomap`。

---

## 15. 后续演进路线

1. **交通 / 让行**：当前调度器不看路况，多车会挤同一角落。**已按 CBS 实现了一版
   （见 §16），规划部分完成、执行部分待接通**——把 §16 的执行层做完，这一条就可以划掉。
2. **充电语义**：现在回桩只是"停着不掉电"，可以加充电功率曲线与 `CHARGING` 状态联动。
3. **时间窗 / 容量 / 任务依赖**：`cost.py` 加约束项，或整体替换为 OR-Tools CP-SAT
   （`select_robot()` 是唯一入口）。
4. **动态插单 / 故障重调度**：滚动时域 = 周期性重跑 `_try_assign_all()` 并允许抢占。
5. **换新图**：把新扫描放进 `assets/scan_sources/`，按 §12 四步重跑，再
   `tools/pick_zones.py` 重新挑点即可（脚本都已参数化）。

---

## 16. CBS 多机路径规划（规划已完成，执行待接通）

**为什么做这件事**：以前每台车各自规划——Nav2 的 SMAC 只躲静态地图，队友靠运行时的
反应式机制处理（`robot_obstacles` 把队友注入局部代价地图 + 碰撞监控刹车）。反应式
避让解决不了**结构性冲突**：两台车要从同一个 1.0~1.6 m 门洞对向通过，各自都会停在
对方前面等着，局部怎么重规划都出不来——它们必须**先谈好谁先走**。这是多机路径
规划（MAPF）问题，CBS 是标准最优解。

### 16.1 规划器：`hetero_fleet_scheduler/cbs.py`

两层结构，和文献里的 CBS 一致：

* **底层**：单车 A\*，状态是 `(格子, 时间)`，遵守两类约束——"某时刻不能在某格"
  （顶点约束）和"某时刻不能从 A 格走到 B 格"（边约束，用于消解对向交换）。
* **顶层**：约束树。找第一处冲突 → 分两支（分别给冲突双方加约束）→ 各自重规划 →
  代价最小者优先展开，直到无冲突。加了两处工程优化：
  **bypass**（子节点代价不变就继续下潜，不必进开表；40 m 走廊对向从"上千个排队节点"
  降到 5 次展开）和**最小间距约束**（见下）。

针对这张图专门做的四件事：

1. **降采样**：MAPF 网格 0.20 m（163×421），CBS 是时空搜索，在 0.05 m 的导航栅格
   （109 万格）上跑不动。粗格子里只要有一个细格子是障碍就整格判障，所以降采样不会
   打通墙。
2. **按车型膨胀**：每台车用自己半径（半长 0.19/0.23/0.22 m）膨胀障碍，heavy 比
   light 宽 25% 这件事在规划里是真的。
3. **最小间距 `sep_margin`**：规划"不冲突"不等于"能开"。局部代价地图会把队友膨胀
   0.35 m，实测 0.76 m 中心距时 `collision_monitor` 一直报 `PolygonSlow`（降速 40%）
   最后 `Failed to make progress`。所以两台车的中心距被强制拉开到
   `r1 + r2 + 0.50 m`（0.20 m 格子 → 5 格 = 1.0 m）。
4. **停着的车是不可移动障碍**：不能让 CBS"请"一台停在充电位的车挪开再回来——没人会
   给它发目标，计划就成了假的。停着的车直接烘进其他车的可通行掩码。同理，两台车被
   派到**同一个作业点**时目标必须错开到 ≥ 间距要求，否则 MAPF 意义下无解（先到的不走，
   后到的永远进不来），CBS 会白烧完整个预算。

超预算（CBS 是 NP-hard）时回退到**优先规划 + 迭代精修**（按"停着的优先、路线长的优先"
排序，逐个避开已规划的路，再迭代几轮），有硬时限，保证返得回来。

### 16.2 接入：`traffic.py` + `fleet_manager`

每次有车要开新的一段（充→取 / 取→卸 / 回充），就**对全车队重规划一次**：在跑的车用
当前位置和当前目标，停着的车当障碍。每条车的新路线用 Nav2 的 **FollowPath** 动作下发
（局部控制器、代价地图、碰撞监控全部不变，所以"计划之外的事"仍有安全网兜着）。路线
同时发到 `/<robot>/cbs_plan`，RViz 配置里已加 4 条粗线显示，可以直接和 Nav2 自己的
`/<robot>/plan` 对比。某台车的路线被新计划改动了，就打个"脏"标记，由它自己的 leg
线程换上新路线（每台车只有一个 goal 属主，不会被两个线程抢）。

开关：`use_cbs`（默认 **false**）。

### 16.3 当前接入（2026-10-09）

以下为之前 CBS 接入记录；当前 `use_cbs` 默认 **false**。CBS 负责多车的路径和通行顺序，车型能力/成本函数继续负责任务分配。
`cbs_execution.py` 将 CBS 通行顺序转换为共享路段的进度依赖。无冲突路段通过一条连续
FollowPath 执行，保留折点，仅在交叉或共用路段等待另一辆车实际离开；不再逐格执行或全车同步。
终点容差仍为 0.04 m，最终朝向使用任务目标的 yaw。无冲突的新任务直接加入；
与已执行路线冲突的新任务等待预留路段释放，不取消现有车辆动作。取消或失败只停止存在依赖的车辆。
CBS 规划失败、缺少实测位姿、动作失败或超时都会明确失败，不会转独立导航或模拟成功。

超过 CBS 搜索预算时仍允许优先规划回退，但结果必须再次通过冲突和到达目标检查；
有残余冲突或未抵达全部目标时不会执行。停放车辆始终是不可移动障碍。
车型半径按完整矩形外接圆加 0.06 m 余量计算，静态障碍 margin 为 0.30 m。
同一目标被占用时不能通过大幅挪动目标宣告成功，超出一个粗网格的目标偏移会拒绝。

```bash
# 默认原调度算法（CBS 关闭）
ros2 launch hetero_fleet_bringup bringup.launch.py
# 明确选择 CBS
ros2 launch hetero_fleet_bringup bringup.launch.py use_cbs:=true
# 对照实验：独立 Nav2
ros2 launch hetero_fleet_bringup bringup.launch.py use_cbs:=false
# 单独启动调度器：默认原调度算法（不要与已有 fleet_manager 同时运行）
ros2 launch hetero_fleet_scheduler scheduler.launch.py use_cbs:=false
```

可选 CBS 模式采用连续路段与共享路段进度依赖。`cbs_step_timeout_sec` 默认 30 秒，
`cbs_time_limit_sec` 默认 3 秒；执行等待不会触发旧停滞检测器，超时由 CBS 执行器处理。
原来的减速区和卡死后倒退恢复保持关闭，MPPI 和近距离停车保护仍在。
53 项测试、实际地图单车/四车规划以及编译通过；尚未验证新执行器在 Gazebo 的完整 20 任务运行。

### 16.4 待验证

重启加载新的 Nav2 到位检查器与调度器后，先测试单车，再测试窄通道会车和完整 mixed_20。
观察 `CBS coordinated` 日志、各车 `/cbs_plan`、停车区以及 metrics/metrics.json、metrics/tasks.csv。
无解、目标被停放车辆占用、物理卡住与动作超时仍可能导致任务失败；CBS 不提供无法通过的通道。
