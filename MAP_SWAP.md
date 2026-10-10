# 换地图说明：warehouse → geomap_colour_noroof → geomap_final_noroof → geomap_combined12_noroof

> **注意：本文是换地图当时的记录。** 第 1~3 版里的若干数字和命令后来又被调整过（例如
> `prepare_map_mesh.py` 的 `--keep-above` 最终用的是 `0.40`，而第 1 版命令块仍写着
> `1.0`；车队也已经从 3 台恢复成 4 台）。**当前状态与可复现的命令以
> [`README.md`](README.md) 为准**，本文只作背景阅读。

> **又一批数字过时了（2026-09-28 晚）**：本文第四版收尾用的
> `--free-floating --min-width 1.20 --fill-notches 1.6 --fill-unreachable 40`
> 后来被判定为"RViz 地图和 Gazebo 差太多"的根源（通道被封死、家具消失、家具中间
> 多出可通行区域），已换成
> `--squash-mesh 0.15 --squash-max 2.00 --fill-outlines --min-width 0`。
> 推导、前后实测数字和两个新的保真度检查工具（`tools/map_fidelity.py`、
> `tools/scan_vs_map.py`）见 [`README.md` §12 第六轮](README.md)。

---

## 更新（第四版，最新）：换成 `geomap_combined12_noroof`（25 × 84 m 的楼）

工作空间现在跑的是桌面 `~/Desktop/geomap_combined12_noroof.sdf` 指向的扫描图
（模型 `geomap_combined12_noroof`，来自 `Documents/Codex/2026-09-18/pcd-gazebo` 里
`cleaned/geomap_combined12_noroof.pcd` —— 两张 0923 地图合并、去天花板、手工删点、
再整体上抬使最低点落在 z=0 的那一版）。车队仍是 **4 台**（light ×2 / heavy / omni），
因为新图比上一张宽敞得多，不需要再缩车型。

```
本次换图：   /home/xuzhijie/hetero_fleet_geomap123_ws      （就地换图，非新建工作空间）
上一张图：   /home/xuzhijie/hetero_fleet_geomap_ws           （未改动）
仓库地图版： /home/xuzhijie/hetero_fleet_ws                  （未改动）
世界文件来源：/home/xuzhijie/Desktop/geomap_fleet.sdf           （未改动）
模型来源：   .../pcd-gazebo/models/geomap_combined12_noroof    （未改动）
```

> **落地方式**：这次不是在旁边新建工作空间，而是把 `hetero_fleet_geomap123_ws`
> 里原来的 `geomap_final_noroof` 图**就地换掉**（模型、世界、Nav2 栅格、出生点、
> 任务点、工具链一起换，换完 `src/` 与上一张图的 `hetero_fleet_geomap_ws` 不再相同）。
> 旧图和它的资产在 `git` 历史里、以及 `~/hetero_fleet_geomap_ws` 里都还在。

因为输入模型（148310 顶点 / 297154 面）比上一张图大 14 倍，处理链的参数**只有一处**
要改（模型名），其余全部沿用：[`README.md` §12](README.md) 的四步链 + `verify_scene.py`。

| | 上一张 `geomap_final_noroof` | **这张 `geomap_combined12_noroof`** |
| --- | --- | --- |
| 网格 | 521 × 207 @ 0.05 m（26.0 × 10.3 m） | **501 × 1684 @ 0.05 m（25.0 × 84.2 m）** |
| 可通行面积 | 180.0 m² | **718.3 m²**（障碍 1390.9 m²） |
| 最大连通块 | 177.5 m²（99%） | **541.1 m²（75%）**，共 24 块 |
| 空间形态 | 一整块开阔大厅 | **一条 ~3 m 宽的主走廊，长 ~65 m，东侧挂房间** |
| 碰撞盒 | 477 个 | **3013 个** |
| 任务点间距 | 3.4~9.5 m | **5.9~26.1 m** |
| 数据来源 | `geomap_20260917_224521.pcd` | 两张 0923 地图合并 + 手工删点（combined12） |

三个和上一张图不同的地方（都是这次才踩到的）：

1. **地板要先看清水面在哪。** `prepare_map_mesh.py` 的地板拟合是"面积加权直方图取峰"，
   而这张图的网格里除了行走面（原始 z≈+0.197）还带着楼板的**下表面**（原始 z≈0）——
   两层的水平面积都很大。拟合正确地选中了行走面（z 残差 p5..p95 = −0.4..−0.0 cm），
   于是 `level_transform` 的 `dz = −0.197 m` 把行走面放到 z=0、楼板下表面落到 z=−0.203。
   这个下表面在 `--keep-above 0.40` 的过滤里自然被丢掉，不影响碰撞。
   **判据**：模型里 `|法向 z| > 0.9` 且**朝上**的面，在 z≈0 有 940 m²，在 z≈−0.2 是 0；
   反向的面（天花板/楼板底）才在 z≈−0.2 有 1343 m²。
2. **碰撞盒数量决定物理步长，不决定于机器人。** 同一台机器上实测（headless、DART、
   4 车 5 个雷达）：3013 个盒子 + 2.5 ms 步长 → 窗口 RTF **0.35**；把盒子去掉只留
   视觉网格 → RTF **1.00**；改成 5 ms（200 Hz）步长 → RTF **1.00**。
   所以 `make_geomap_world.py` 的 `PHYSICS_STEP` 从 2.5 ms 调到了 5 ms
   （仍是 100 Hz 控制器周期的 2 倍）。想再快只能从盒子数想办法：`--margin 0.15`
   减 9%（纯收益，去掉的是机器人物理上够不到的死角），`--block 2` 减到 ~1740 但每面墙
   最多胖 5 cm（会与代价地图不一致，非必要别用）。
3. **点位按"走廊 + 门洞"重排。** 充电位不再贴墙排，而是排在走廊最北端那间房间
   （x=−1.60，y −11.0/−12.7/−14.4/−16.1，间距 1.7 m）；作业点沿走廊向南铺开，
   并且特意**不放进门洞**（走廊虽然 ~3 m 宽，但东侧房间的门洞净空只有 1.0~1.6 m，
   车停在那儿就把南北通道封了——这正是上一张图 42 次卡死里 25 次的成因）。
   点位由 `pick_zones.py` 给候选、人工整理、再用 `verify_scene.py` 校验：
   4 个出生点 + 10 个 zone 全部在自由格上、净空 ≥ 0.53 m（最宽的 omni）、且同在
   一个连通区（脚本里是 `region 3`）。

### 补记（2026-09-28）：门楣把门洞压死了，后来补了一刀

第一版这张图跑起来后，RViz 里能看到"门框被压成不可通行区域"，地面也有一粒粒坑点。
根因是**俯视投影分不出墙和门楣**：`prepare_map_mesh.py` 原来只筛"贴地以上 0.4 m"，
于是最低点在 2.0~2.4 m 的门楣/横梁/管道投出来跟墙一模一样，把它悬挂的门洞整条封死。

修法都在同一条链上、不需要手工改栅格：

* `prepare_map_mesh.py --drop-above 2.00`：丢掉**完全**位于 2 m 以上的面；
* `make_nav2_map.py --fill-pits 0.15 --fill-pockets 0.5 --fill-unreachable 20`：收尾
  去麻点 + 填掉车队到不了的空白块（跑在 `--smooth` 之后，因为 3x3 中值投票本身会在
  拐角重新造出几个单格麻点）。

同一张图、同一套参数实测：

| | 修前 | 只做门楣过滤(1.5 m) | **现在(2.0 m + 填空白圈)** |
| --- | --- | --- | --- |
| 碰撞网格里"最低点 ≥ 阈值"的面 | 65924（1.5 m） | 0 | **0** |
| 占据栅格障碍面积 | 1390.9 m² | 1292.8 m² | **1439.6 m²** |
| 可通行面积 | 718.3 m² | 816.4 m² | **669.6 m²** |
| 连通块数 / 最大连通区 | 24 块 / 541.1 m² | 33 块 / 716.5 m² | **1 块 / 669.6 m²** |
| 自由小口袋 < 0.5 m² | 3 个 | 0 个 | **0 个** |
| 孤立障碍小块最小值 | 0.05 m²（噪点） | 1.00 m² | **4.84 m²（真家具）** |
| x 5~10 / y −31~−32 工作台 | 7.55 m² | 3.52 m²（**被误删**） | **6.03 m²（已恢复）** |
| 盒体碰撞 | 3013 | 3900 | **2365** |
| RTF（GZ 只带 4 车、5 ms 步长） | 1.00 | 0.94 | **1.00** |

关键性质：这个高度过滤器**只会删障碍**，实测"原本可通行、修完被新堵上"的格子是
**0.0 m²**，所以不会出现"地图比房间更严"把机器人困住的情况。被重新接通的 172.3 m²
里最大一块是 106 m² 的翼楼，门洞净宽 1.40 m，三种构型都能过；另有几道 0.7~1.0 m 的
窄门（按瓶颈净空算只够 light 车，或在 0.53 m 需求以下完全不能用）。
`tools/map_defects.py` 会把这几项都打出来，换下一张图时先跑它。

**第二天补的两刀**（同一次会话里用户看图后提的）：

1. **阈值不能按车队高度取**。这份扫描把一部分家具只扫到了顶面：x 5~10、y −31~−32 的
   工作台顶面在 1.5~2.4 m、下面几乎没有几何，`--drop-above 1.5` 就把它整片删了。
   四个取值的实测取舍：关 → 工作台 7.55 m² 但门洞关；1.5 → 门洞开但工作台只剩
   4.05/3.52 m²；**2.0 → 门洞开且工作台 6.03 m²**；2.2 → 门洞又关。
2. **不可达自由块填掉**。车队到不了的自由地面（家具之间的空隙）在 RViz 里是一堆
   "空白圈"，共 28 块、90.5 m²、最大 13.8 m²。`--fill-unreachable 20` 把它们填成
   障碍，地图变成 `free components: 1`；副作用只是可通行面积 816.4 → 669.6 m²（本来就
   走不到的部分），盒体碰撞反而从 3900 降到 2365，RTF 回到 1.00。

**第三天补的两刀**（用户继续指着图提的）：

3. **家具豁口填平**：`make_nav2_map.py --fill-notches 1.0`，对障碍做闭运算，宽度
   <1 m 的凹口补成实心。注意 `binary_closing` 必须带 `border_value=1`，否则侵蚀那一步
   会沿整张图边界吃掉 0.25 m 宽的墙（实测少掉 50 多 m² 障碍，还看不出原因）。
   上限 1.0 m：1.2 m 会把 1.4 m 的翼楼门洞一起封上（净空 1.40 → 0.00）。
4. **手动编辑表**：`src/hetero_fleet_bringup/maps/map_edits.yaml`（`erase` / `fill`
   两组世界坐标框，每条写理由），由 `make_nav2_map.py` 在自动流程后套用。
   现有条目：`(2, −11)` 的楔形突出物——扫描只扫到 0.4~1.5 m 的顶面（到机器人高度的
   几何只有 0.18 m²），投影却是扎进房间的一根楔子，机器人本来能从下面过 → `erase`。

最终（第四天定稿）这张图的数字：结构 445.5 m²、可通行 **618.8 m²**、**1 个连通块**、
**2153 个盒体碰撞**、`verify_scene.py` 通过、各任务点净空不变（翼楼门洞 1.40 m）。

**第五天：闭运算把通道当成豁口补掉了**。用户报"有些通道被错误地封上"。
把 `--fill-notches 0` 和 `1.0` 两版地图逐格对比、再看闭运算新增的每块障碍两侧的
自由格在闭运算前是否连通，查出 **7 处通道被补成实心**：

| 塞子面积 | 位置 | 切断了什么 |
| --- | --- | --- |
| 1.15 m² | (11.58, −60.65) | 南段 6.3 m² ↔ 1.7 m² |
| 0.58 m² | (16.55, −45.95) | 南段 7.5 m² ↔ 2.3 m² |
| 0.33 m² | (12.76, −61.79) | 南段 1.7 m² ↔ 1.6 m² |
| 0.26 m² | (6.02, −26.90) | 主区 ↔ 0.8 m² |
| **0.11 m²** | **(11.32, −29.71)** | **主区 ↔ 17.7 m² 的房间** |
| 0.10 m² | (1.68, −23.27) | 主区 ↔ 1.3 m² |
| 0.06 m² | (−3.45, −34.89) | 主区 ↔ 3.8 m² |

修法：闭运算加"通道守卫"——对新增的每一块障碍，检查它两侧的自由格在闭运算前是否
属于同一连通区；是则整块撤回（那是通道，不是豁口）。结果：封堵量 53.3 → 50.7 m²，
误封通道 **7 → 0**，可通行面积 618.8 → **643.0 m²**，仍保持 **1 个连通块**。
当前定稿数字：结构 445.5 m²、可通行 **643.0 m²**、**2344 个盒体碰撞**、
`verify_scene.py` 通过。

**第六天：还有通道被堵——根因是"只扫到顶面的家具"被判成墙**。用户要求"对照 Gazebo
检查"。做法：把可视网格里"跨过机器人高度带 0.05~0.45 m"的面栅格化成 `solid`，
再逐格比对 `geomap.pgm`——地图里建筑内部有 **637 m²** 的障碍在 `solid` 之外，
也就是 Gazebo 在那个高度根本没有几何。东侧翼楼整片是黑，实际是成片家具只扫到
0.5~1.5 m 的顶面。

修法是 `--free-floating`：和"通道守卫"同一套判据，一块悬浮几何**只有在它把原本连通的
自由空间切成两半时才删掉**（这样屋子中间绕得过去的工作台仍然保留），实测放开
**283.3 m²** 封堵几何：

| | 修前 | 修后 |
| --- | --- | --- |
| 障碍面积 | 1466.2 m² | **1156.0 m²** |
| 可通行面积 | 643.0 m² | **953.2 m²** |
| 连通块 | 1 | 1 |
| 工作台（x5~10, y−31~−32） | 7.04 m² | **5.64 m²（保留）** |
| 盒体碰撞 | 2344 | **4430** |

东侧翼楼从"整片黑"变成"家具块 + 通道"；`verify_scene.py` 仍通过（任务点净空不变，
pickup_A 1.00 m、dropoff_C 1.27 m、翼楼门洞 1.40 m）。成本是障碍边缘变碎、盒子数
翻倍：单实例 5 ms 步长下按线性模型约 RTF 0.83，需要的话调 `PHYSICS_STEP` 或
`make_collision_boxes.py --block 2`。

**第七天：用户换了扫描网格，并要求"地图别太破碎，只留通道、家具变矩形"**。
新网格 147053 顶点 / 294660 面，往东扩到 x = 25.7（栅格 649 × 1684 = 32.4 × 84.2 m）。
在原来那套过滤之上加了"简化档"：

```bash
--min-width 1.00 --fill-pits 0.50 --fill-pockets 2.0 \
--fill-unreachable 80 --fill-notches 1.6 --free-floating --rectangularize 0.53
```

`--rectangularize` 是这次新写的：把每个"厚于 0.53 m"的紧凑障碍块用**外接矩形**填实
（薄长的墙不动），并且**只允许填净空 < 0.53 m 的格子**——那种格子本来就没有车型能占用，
所以补矩形不可能吃掉通道。第一版没这条约束时，`pickup_C` 被压到 0.40 m、
`dropoff_B` 压到 0.20 m，`verify_scene.py` 立刻报错，加了约束后通过。

| | 用户换网格后自己跑的 | 这次的"简化档" |
| --- | --- | --- |
| 障碍面积 | 1772.5 m² | **2048.7 m²** |
| 可通行 | 959.8 m² | **683.6 m²** |
| 内部障碍孤块 | 39（12 个 <1 m²） | **12（1 个 <1 m²）** |
| 自由连通块 | 1 | 1 |
| 盒体碰撞 | — | **2409** |
| 任务点净空 | — | 0.81~1.45 m，`verify_scene.py` 通过 |

---

这次换图**验证到哪一步**（详见 [`README.md` §13](README.md)）：地图四步链重跑、
`verify_scene.py` 通过（4 出生点 + 10 点全在同一连通区）、`colcon build` 通过、
39 个单元测试通过、Gazebo 里 4 车精确出生且 8 个控制器全部激活、`light_1` 直线行驶
1.47 m（y/z 不变）证明轮子在新碰撞盒上正常滚动、只有仿真时 RTF 稳定 1.00（整套 Nav2
跑起来后平均 0.77）。**端到端也跑通了**：4 套 Nav2 active 后注入 3 条真实任务，
3/3 完成（12.3 / 25.3 / 26.1 m），任务清空后三台车自动回新的充电排并报
`docked at charger`。`omni_1` 中途出现过一次上一张图也有的"近目标 0.45 m 停滞"，
被停滞看门狗 + 对峙后退逻辑解开后正常完成。**还没跑**预设的 `demo_10` / `mixed_20`
统计场景。

---

## 更新（第三版）：又换成 `geomap_final_noroof`，车队加到 4 台

工作空间现在跑的是桌面 `~/Desktop/model.sdf` 指向的那张新扫描图
（模型名 `geomap_final_noroof`，来自 `geomap_20260917_224521.pcd`），
车队在原来的 3 台基础上加了一台 **light_2**（light/light/heavy/omni）。

**新地图长什么样**：一整块开阔大厅，不是旧图的回型走廊。

> **关键教训（换这张图时踩到的）**：`keep-above` 不能只按"机器人能不能从下面钻过去"
> 来定，还要对齐**雷达所在的高度**。本工作空间里雷达渲染的是**视觉网格**（不是碰撞体），
> 所以 `keep-above=1.0` 时雷达能看到大量"地图里没有"的低矮家具：实测 114 条波束在
> 1 m 内看到东西、而地图说该方向 3 m 内是空地 —— 代价地图会被这些幻影障碍填满，
> 规划器在空旷处直接报 `no valid path found`。改成 `keep-above=0.40`（贴合雷达高度
> 0.26~0.36 m）后这个数字降到 **0**。旧图恰好没暴露这个问题，是因为那条走廊在雷达
> 高度上本来就是空的。

| | 旧图 geomap_colour_noroof | **新图 geomap_final_noroof** |
| --- | --- | --- |
| 网格 | 632 × 443 @ 0.05 m | **521 × 207 @ 0.05 m**（26.0 × 10.3 m） |
| 自由区 | 142.4 m² | **180.0 m²** |
| 最大连通区 | 108.6 m² | **177.5 m²（占 99%）** |
| 最宽处净空半径 | 1.42 m | **3.60 m** |
| 碰撞盒 | 2010 个 | **477 个** |
| 任务点净空 | 1.00~1.15 m | **1.30~2.25 m** |

处理链和之前完全一样（脚本都已参数化，换图只要换一个 `--model-name`）：

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws
python3 tools/prepare_map_mesh.py \
  ~/Documents/Codex/2026-09-18/pcd-gazebo/models/geomap_final_noroof/meshes/geomap_final_noroof.obj \
  src/hetero_fleet_gz/models/geomap_final_noroof --name geomap_final_noroof --keep-above 1.0
python3 tools/make_nav2_map.py \
  src/hetero_fleet_gz/models/geomap_final_noroof/meshes/geomap_final_noroof.obj \
  src/hetero_fleet_gz/models/geomap_final_noroof/meshes/geomap_final_noroof_collision.obj \
  src/hetero_fleet_bringup/maps/geomap --report --smooth 1 --structure-dilate 0
python3 tools/make_collision_boxes.py --model src/hetero_fleet_gz/models/geomap_final_noroof/model.sdf
python3 tools/pick_zones.py src/hetero_fleet_description/config/robot_profiles.yaml --model-name geomap_final_noroof --spacing 2.5 --spot-margin 0.25
python3 src/hetero_fleet_gz/worlds/make_geomap_world.py     # MODEL 已改成 geomap_final_noroof
python3 tools/verify_scene.py
```

新点位（由 `pick_zones.py` 选出，出生点=全图最宽敞的 4 处）：

| 出生点 | 坐标 | 净空 |
| --- | --- | --- |
| light_1 | (-7.47, 2.16) | 2.46 m |
| light_2 | (-5.27, 0.81) | 2.89 m |
| heavy_1 | (-2.82, 1.61) | 3.60 m |
| omni_1 | ( 1.48, 0.56) | 2.32 m |

| 任务点 | 坐标 | 净空 |
| --- | --- | --- |
| pickup_A | (-4.72, 3.26) | 1.91 m |
| pickup_B | (-0.97,-0.09) | 1.76 m |
| pickup_C | (-0.62, 2.96) | 2.25 m |
| dropoff_A | ( 4.93,-0.59) | 1.30 m |
| dropoff_B | ( 5.13, 3.06) | 2.10 m |
| dropoff_C | ( 8.18, 3.26) | 1.75 m |
| charger_A/B/C | (8.53,-0.64) / (11.28,3.31) / (11.63,-0.19) | 仅作标记 |

状态字面板的默认矩形也按新图放大到 x[-15,18]、y[-5,9]（4 台车各占一角，
离最近的标签 ≥6.5 m）。下面是第一、二版（旧图）的推导，保留作参考。


## 更新（第二版）：恢复成"回型走道实验室"

第一版把 **z < 0.6 m 的所有几何**都当障碍，于是扫描里的椅子、箱子、地面杂物把
走道切成 60 多个小口袋——地图"太破碎"。真实房间是**回型（环形）走道**：

* 现在只把**高过 1.0 m 的结构**（墙、隔断、高柜）当障碍。碰撞网格由
  `prepare_map_mesh.py --keep-above 1.0` 生成，地图再由**同一份网格**投影得到，
  物理与代价地图天然一致。
* 自由区 **142.4 m²**（早期版本是 120.9 m²）。放大来自两处：阈值 0.8 → 1.0，
  以及**去掉栅格化时那层多余的膨胀**——实测雷达比地图预测的中位远 0.35 m，
  说明墙被画胖了，去掉 1 格膨胀后墙变薄 5 cm/边。
* 三种机型的需求净空现在是 **0.31 / 0.36 / 0.39 m**（车体经三轮缩小到
  `s` = 0.41 / 0.31 / 0.42，见第 3 节）。满足该净空的最大连通区：
  **61.1 m²**（上一版是 50.1 m²），跨度 x[-8.0, 12.9]、y[-1.5, 5.8]，
  仍是同一条环形走廊。
* 出生点与 9 个任务点重新由 `tools/pick_zones.py` 选出（选点改为"分散 × 宽敞"
  加权），任务直线距离 3.4~9.5 m；`heavy_1` 的出生点是**手工**挪到卸货区旁边的
  (8.00, 4.00)（净空 0.95 m，距 dropoff_B 1.64 m），重跑 `pick_zones.py` 会覆盖；
  `tools/verify_scene.py` 输出 `RESULT: scene is consistent`。
* 低矮杂物仍然**可见**（视觉网格未改动），只是不再阻挡通行——取舍理由见 6.0 节。

重跑顺序：`prepare_map_mesh.py --keep-above 1.0` → `make_nav2_map.py`
（`--smooth 1 --structure-dilate 0`）→ `make_collision_boxes.py`（把栅格变成盒体碰撞，见 6.0）→
`make_geomap_world.py` → `verify_scene.py`。
下面第 1~6 节保留第一版的推导与实测数据，具体数字以本节为准。

本工作空间是从 `/home/xuzhijie/hetero_fleet_ws`（仓库地图）复制出来的一份，
**原工作空间没有被改动**。改动只有一件事：把 Gazebo 世界换成
`~/Desktop/geomap_colour_noroof.sdf` 用的那张点云扫描地图，车队缩到 3 台
（三种机型各一台），机器人尺寸按新地图重新标定。

```
新工作空间： /home/xuzhijie/hetero_fleet_geomap_ws
原工作空间： /home/xuzhijie/hetero_fleet_ws                     （未改动）
地图来源：   /home/xuzhijie/Desktop/geomap_colour_noroof.sdf      （未改动）
模型来源：   /home/xuzhijie/Documents/Codex/2026-09-18/pcd-gazebo/models/geomap_colour_noroof （未改动）
```

---

## 1. 这张地图到底长什么样（先看结论）

原始 OBJ 是把 `geomap_20260917_200017.pcd` 体素化成 0.1 m 表面网格得到的，
128445 顶点 / 257912 三角面，彩色贴图由图集 PNG 提供。把它量出来以后有三个
关键事实，后面所有决定都由这三个事实推出来：

1. **地板是斜的。** 对朝上的面做面积加权平面拟合，倾角 **1.14°**
   （roll +0.97°、pitch +0.61°）。Gazebo 的地面只能是水平的，所以先把整张网
   旋转到地板水平、并平移到 z≈0（残差 σ≈4.6 cm，见下）。
2. **地板比墙面矮不了多少，但多余几何很多。** 面朝下的天花板在
   `_noroof` 版本里已经被切掉，可是扫描里高于机器人身高的几何（桌子、柜顶、
   管线）非常多：把整张网俯视投影，一楼通道会被这些"头顶上的东西"整条封死。
   所以占据栅格只把 **z < 0.6 m** 的几何当成障碍。
3. **可通行区域不是整栋楼，而是一条走廊。** 在一体化连通性检查里，整张图
   155 m² 的空白地面被家具/隔断切成 60 多个小块，其中只有一条约
   **13 m × 2.5–4 m（约 40 m²，中心 (5.7, -3.8)）** 的通路在三种机器人的
   净空要求下仍然连通。任务点、出生点因此都放在这条走廊里。

> 说得直白一点：这张扫描图不是仓库，是一层办公室/实验室。房间之间在网格上
> 是不连通的（门/隔断/家具把通道堵死），所以 Nav2 只能在上述走廊内调度。
> 想要更大活动范围，见本文第 6 节。

---

## 2. 地图处理链（`tools/`）

所有脚本都是普通 python3 + numpy/scipy/PIL，不依赖 ROS，可单独重跑。

| 脚本 | 作用 |
| --- | --- |
| `tools/map_common.py` | 共用几何：OBJ 读取、面法向/面积、地板平面拟合、俯视栅格化（含竖直面"退化成线段"的处理） |
| `tools/prepare_map_mesh.py` | 生成 Gazebo 模型：把地板校平后写出**视觉网格**和**去地板碰撞网格** |
| `tools/make_nav2_map.py` | 生成 Nav2 占据栅格 `src/hetero_fleet_bringup/maps/geomap.{pgm,yaml}` |
| `tools/make_collision_boxes.py` | 把占据栅格转成 ~890 个轴对齐盒体，写进 `model.sdf`（DART 用；替代三角网碰撞） |
| `tools/pick_zones.py` | 在连通可通行区域里自动挑出生点和任务点 |
| `tools/verify_scene.py` | 交叉校验地图 × 机器人尺寸 × 世界里的任务点标记 |

完整重建命令（顺序不能换，后一个依赖前一个的产物）：

```bash
cd <这个工作空间>

# 1) 地图模型：校平 + 视觉/碰撞分离
python3 tools/prepare_map_mesh.py \
  /home/xuzhijie/Documents/Codex/2026-09-18/pcd-gazebo/models/geomap_colour_noroof/meshes/geomap_colour_noroof.obj \
  src/hetero_fleet_gz/models/geomap_colour_noroof

# 2) Nav2 占据栅格
python3 tools/make_nav2_map.py \
  src/hetero_fleet_gz/models/geomap_colour_noroof/meshes/geomap_colour_noroof.obj \
  src/hetero_fleet_gz/models/geomap_colour_noroof/meshes/geomap_colour_noroof_collision.obj \
  src/hetero_fleet_bringup/maps/geomap --report

# 3) 盒体碰撞：把占据栅格变成 DART 能吃的盒子，写进地图模型的 model.sdf
#    （必须排在 make_nav2_map.py 之后；换引擎的原因见本文 6.0 节）
python3 tools/make_collision_boxes.py --report

# 4) 世界文件（任务点标记写在这里的表里，改完坐标要重跑）
python3 src/hetero_fleet_gz/worlds/make_geomap_world.py

# 5) 自检
python3 tools/verify_scene.py
```

### 2.1 为什么要拆成"视觉网格 + 去地板碰撞网格"

`models/geomap_colour_noroof/meshes/` 下有两个 OBJ：

* `geomap_colour_noroof.obj` —— 视觉用。**逐行复制原文件，只改 `v` 行的三个数**，
  所以 `vt`（贴图坐标）和 `f v/vt` 索引一个都没动，彩色图集照旧正确。
* `geomap_colour_noroof_collision.obj` —— 碰撞用。把"近似水平且整体低于
  z=0.12 m"的面（也就是地板那一层体素）删掉，剩下的墙面/家具就是真正的障碍。

为什么删地板：扫描地板只有 ±6 cm 的平整度，如果让它参与碰撞，轮子会一路
跟着噪声起伏，同时还要和世界里的水平地面"打架"。删掉之后世界里的
`ground_plane`（z = -0.01）就是行驶面，机器人走在平地上；视觉地板仍在，
肉眼看到的地板和轮子接触面最多差 6 cm。

### 2.2 为什么障碍只算 z < 0.6 m

三种机器人的雷达高度分别是 0.31 m（light）、0.34 m（omni）、0.42 m（heavy），
都在 0.6 m 以内。让地图只标记"机器人身体高度以下"的几何，好处是两头都对得上：

* 桌子/柜子上方的板子不会在地图上继续"长"出一条墙，
* 雷达看到的东西一定在地图里（地图标记的是超集），
* 机器人可以合理地从高处结构下方通过。

仓库版本里用的是反过来的补丁（shelf blockers，把地图上的机架投影在物理上
补实）；这里方向相反，所以那个脚本已经删除。

### 2.3 生成的地图

```
src/hetero_fleet_bringup/maps/geomap.pgm   632 x 443 @ 0.05 m（31.6 x 22.1 m）
src/hetero_fleet_bringup/maps/geomap.yaml  origin [-13.54, -14.53], negate 0
```

极性沿用 map_server 约定：**白色 = 可走，黑色 = 障碍**（`negate: 0`）。
扫描没覆盖到的地方一律记为障碍，否则规划器会把路线画到楼外面去。

---

## 3. 机器人尺寸是怎么定的

> **这一节的值改过三轮**：计划值 0.70 / 0.72 / 0.53、第一版 0.60 / 0.61 / 0.45、
> 第二版 0.48 / 0.49 / 0.36，**现在用的是 0.41 / 0.42 / 0.31**。控制器 YAML 的
> 轮径/轮距、`robot_profiles.yaml` 的 footprint 都按实际值算，下表由
> `tools/verify_scene.py` 复核。

原设计是仓库尺寸，缩小系数是逐机型定的（不是统一一个数）：

| 机型 | 仓库尺寸 | URDF 车体 | footprint（含轮子） | 缩小系数 s | inflation | 需要净空 |
| --- | --- | --- | --- | --- | --- | --- |
| light_diff_amr | 0.80 × 0.54 | 0.328 × 0.221 | 0.344 × 0.271 | **0.41** | 0.17 | **0.31 m** |
| omni_mecanum_amr | 0.90 × 0.70 | 0.378 × 0.294 | 0.403 × 0.353 | **0.42** | 0.21 | **0.39 m** |
| heavy_diff_amr | 1.32 × 0.86 | 0.409 × 0.267 | 0.428 × 0.322 | **0.31** | 0.20 | **0.36 m** |

（"URDF 车体" 不含保险杠和护角；"footprint" 是所有碰撞几何——含保险杠、护角
和驱动轮——在地面上的真实外接矩形，由 `tools/measure_footprint.py` 量出来。
**最宽的是轮子**：light 的轮子外缘到 y = ±0.135 m。）

> **这一节修过两次。**
> 1. 早期版本的 footprint 是 0.43 × 0.29 / 0.50 × 0.39 / 0.54 × 0.35，比它们
>    描述的车体小 15–25%——Nav2 按偏乐观的车体规划，`verify_scene.py` 的净空
>    结论也用同一个偏小的数字。改成真实外形后，靠 `pick_zones.py` 重挑了出生点
>    和任务点，任务场景 YAML 的坐标也一起改了。
> 2. 改成真实外形之后暴露了更根本的问题：环形走廊最窄可通行处只有 ~1.1 m，
>    而当时最宽的 omni 车体 0.512 m，扣掉膨胀后横向余量只剩十几厘米，MPPI
>    在窄处频繁 `Optimizer fail to compute path`（每轮 13–52 次）、车队会卡死。
>    于是连续缩了两轮（→ 0.48/0.36/0.49，再 → 0.41/0.31/0.42），每次都重挑
>    任务点。第一轮之后 MPPI 报错降到 0、利用率 47–53% → 74%。
> 3. 地图一侧：`keep-above` 0.8 → 1.0，并去掉栅格化时那层多余的 5 cm 膨胀，
>    自由区 120.9 → 142.4 m²，omni 可用的连通区 50.1 → 61.1 m²（+22%）。

膨胀半径（inflation_radius）取"footprint 内切半径 + 0.03 m"：

* Nav2 **硬性**要求 `inflation_radius ≥ 内切半径`（0.158 / 0.206 / 0.187），
  否则 `planner_server` 拒绝启动；
* 多出来的 3 cm 让膨胀层有个能用的衰减环 —— 只取等于内切半径的话，
  整个膨胀环不到一个代价地图格子宽，代价函数等于一堵硬墙；
* 所以用**多边形 footprint** 而不是 `robot_radius`：后者会被当成内切半径解释，
  直接触发上面那条启动失败。
* 代价地图里另加 `footprint_padding: 0.03`：不加的话"致命半径"正好等于
  "物理接触距离"，机器人蹭一下墙就落在自己的致命格里，MPPI 会算不出可行轨迹。

同步改动：

* `.xacro`：所有尺寸乘以 `s`，质量乘 `s³`，惯量乘 `s⁵`，外壳 STL 用
  `<mesh scale="s s s">`。三台车的雷达高度仍在 0.6 m 以内（见上）。
* `config/*_controllers.yaml`：`wheel_radius` / `wheel_separation`
  （omni 是 `sum_of_robot_center_projection_on_X_Y_axis`）按新几何更新。
* `config/nav2_params_*.yaml`：`footprint` 多边形、`inflation_radius`、
  速度/加速度上限、碰撞监控多边形一起缩放。
* `config/robot_profiles.yaml`：footprint、速度、`inflation_radius`
  （供 `tools/pick_zones.py` 和 `verify_scene.py` 使用）。

> 为什么三种机型都用**多边形** footprint 而不是 `robot_radius`：Nav2 会拒绝
> 启动 `inflation_radius < 内切半径` 的配置，而 `robot_radius` 被当作**内切**
> 半径解释。用多边形时内切半径是真实的半宽（light 0.17 m），0.20 m 膨胀才合法。

**车队组成：3 台，每种一台**（原来是 light×2 + heavy + omni）：

```yaml
light_1  spawn [1.09, -3.45]     # 黄，走廊中段
heavy_1  spawn [9.69, -2.95]     # 蓝，走廊东段
omni_1   spawn [5.69, -4.40]     # 红，走廊中段
```

---

## 4. 任务点与自检

`tools/pick_zones.py` 在连通可通行区域里按"最远点采样"挑出 12 个点
（3 出生点 + 9 个任务点），要求每个点的净空 ≥ 最宽机器人需求（0.39 m）+ 0.15 m，任务点还额外要求
离 light_1（黄）/ omni_1（红）状态标签 ≥ 6 m，免得标签压住任务点：

| 名称 | 坐标 | 净空 |
| --- | --- | --- |
| pickup_A | (  4.00, -0.50) | 1.00 m |
| pickup_B | (  3.14,  4.40) | 1.01 m |
| pickup_C | (  5.94,  4.30) | 1.05 m |
| dropoff_A | (  7.44, -0.50) | 1.15 m |
| dropoff_B | (  9.64,  4.00) | 1.03 m |
| dropoff_C | ( 11.79,  0.50) | 1.15 m |
| charger_A | ( -6.46,  5.00) | 0.96 m |
| charger_B | (  0.24,  4.40) | 1.08 m |
| charger_C | ( -4.86,  0.15) | 0.94 m |

出生点取**全图最宽敞的三个点**（`pick_zones.py` 现在按"分散 × 宽敞"加权选点，
spawn 再从中挑净空最大的三个）：`light_1 (0.39, 0.10)` 净空 1.42 m、
`heavy_1 (3.84, -0.25)` 1.25 m、`omni_1 (9.74, -0.60)` 1.24 m。

`tools/verify_scene.py` 会读 `geomap.pgm` + `robot_profiles.yaml` +
`geomap_fleet.sdf`，对每种机型检查：出生点和 9 个任务点都在自由格上、净空足够、
并且落在**同一个连通区域**里（否则任务之间根本不可达）。当前输出：

```
RESULT: scene is consistent
   zone to zone straight-line distances (m):
                dropoff_A dropoff_B dropoff_C
      pickup_A       3.44      7.22      7.85
      pickup_B       6.52      6.51      9.49
      pickup_C       5.03      3.71      6.98
```

任务场景 `config/tasks/demo_10.yaml`、`mixed_20.yaml` 的坐标已按上表重写。

---

## 5. 世界文件：为什么不直接用 Desktop 上那个 sdf

`~/Desktop/geomap_colour_noroof.sdf` 是"看图用"的世界：只有地面、一盏灯和地图
模型，缺少跑车队必须的东西，而且它的网格碰撞在本机默认物理引擎下不可用。
`src/hetero_fleet_gz/worlds/make_geomap_world.py` 重新生成
`geomap_fleet.sdf`（原文件不动），补上：

* **DART 物理** + **盒体地图碰撞**：DART 在 20 万面的三角网碰撞上会段错误，所以
  地图的碰撞几何不再用三角网，而是由 `tools/make_collision_boxes.py` 从
  Nav2 占据栅格生成 ~890 个轴对齐盒子（见 6.0 节，那里解释了为什么必须换引擎）；
* **sensors system**：没有它 GPU 雷达不会出数据，Nav2 没有代价地图；
* **`<render_engine>ogre</render_engine>`**：本机 ogre2 会把带材质的网格渲成
  纯白，彩色扫描图必须用 ogre 一代渲染（`gz_sim.launch.py` 里同时加了
  `--render-engine ogre`）；
* **水平地面**（z = -0.01）：扫描地板只用于视觉，行驶面是这块水平面；
* **2.5 ms 物理步长**（400 Hz）：DART 每个步长都要把机器人形状和近千个地图
  盒子做碰撞检测，1 ms 会把整个仿真压到 0.35 倍实时；换 2.5 ms 后回到 1.00
  （实测窗口 RTF）。控制器仍然跑 100 Hz，最快的轮子每步只转 0.011 rad。
* **9 个任务点标记**：坐标来自 `pick_zones.py`。

模型通过 `model://geomap_colour_noroof` 引用，安装到
`share/hetero_fleet_gz/models/`；`gz_sim.launch.py` 把该 share 目录加进
`GZ_SIM_RESOURCE_PATH`（漏掉这一步 Gazebo 会报
`Unable to find uri[model://geomap_colour_noroof]`，然后加载出一个没有地图的空世界）。

---

## 6. 已知限制 / 想再调怎么办

### 6.0 已解决：差速车不会转向 = 物理引擎（Bullet）问题

**现象**（换 DART 之前）：给 `light_1` 发纯旋转指令（`angular.z = 0.6`，持续 8 s）：

* 驱动轮**确实反向旋转**：直接读 Gazebo 里连杆姿态，实测 `left=-1.638,
  right=+1.638 rad/s`，正是 `w = 0.6` 对应值（注意别用 `/joint_states` 判断——
  本机 gz_ros2_control 的关节反馈会回显指令）；
* 但车体 `yaw` 变化 **0.0002 rad**（期望 4.8 rad），`x/y` 也不动；前进是正常的，
  所以任务只要需要转向就卡在 `Failed to make progress`，重排队后 FAILED。
  `heavy_1` / `omni_1` 更怪：纯旋转指令让它们**向前平移** 0.3~0.9 m，yaw 不变。

**定位过程**（每一步都可复现）：

1. 换到"只有水平地面"的空世界 → 现象一样 ⇒ 与点云地图无关；
2. 用 Gazebo 自带 `gz-sim-diff-drive-system` 插件绕过 ros2_control → 现象一样
   ⇒ 与控制器/Nav2/调度器无关；
3. 用 Gazebo 官方参考模型 `/usr/share/gz/gz-sim9/worlds/diff_drive.sdf`：
   默认引擎（DART）纯旋转正常（yaw 变化 1.20 rad），**强制 Bullet-Featherstone
   后同样纹丝不动**（0.0014 rad）⇒ **变量是引擎，不是机器人模型**；
4. 本项目世界换成 DART → 同一台车同一套控制器立刻能转。

期间排除掉的模型侧因素（全部实测无效）：`fdir1`、`max_contacts`、`kp/kd`、
轮子 mu（0 / 1.5 / 3）、万向轮摩擦、球轮 vs 圆柱轮、spawn 高度。

**修复**：`make_geomap_world.py` 改用 `gz-physics-dartsim-plugin`，并把地图碰撞
从 209k 面三角网换成 `tools/make_collision_boxes.py` 从占据栅格生成的约 890 个
轴对齐盒子（DART 处理盒子是原生操作，且物理与代价地图逐格一致）。

**修复后实测**（本项目世界，`light_1`）：

| 指令 | 实测 | 说明 |
| --- | --- | --- |
| `wz = +0.6` | `+0.48 rad/s` | 方向正确（缩小前是 `+0.663`） |
| `wz = -0.6` | 反向对称 | 双向一致 |
| `vx = +0.4` | `0.397~0.399 m/s` | 直线跟踪 99% |
| Nav2 `NavigateToPose` 15 m（走廊全长） | 到位误差 0.018 m | 含绕环走廊 + 终点原地对齐 |

三台车都能原地旋转：`light_1` 跟踪约 80%、`heavy_1` 约 78%（缩小后实测），
`omni_1` 跟踪较差（指令 +0.5 → 实测 +0.12~0.18）且会附带 0.01~0.04 m/s 的平移
——这是麦轮摩擦模型在小尺度下的已知偏差，`nav2_params_omni.yaml` 已把它按
DiffDrive 处理，闭环由 Nav2 补偿。

**再犯时怎么查**：`gz topic -e -t /world/geomap_fleet/pose/info -n 1` 读连杆姿态
（不要信 `/joint_states`），或直接 `gz model -m light_1 -p`。若 yaw 又不跟随，
先确认世界里的 `<engine>` 还是不是 dartsim。

* **活动范围只有一条走廊。** 这是扫描图本身的几何决定的（家具与隔断把房间
  相互隔开），不是参数没调好。想扩大：把 `--z-limit` 调小（例如 0.35，允许
  从更多低矮结构下方通过），或把机器人再缩小，然后重跑第 2 节的 2)–4) 步，
  再跑 `tools/pick_zones.py` 挑新点。
* **换回仓库地图**：把 `gz_sim.launch.py` 的 `world:=` 指回原 `warehouse_fleet.sdf`、
  `bringup.launch.py` 的 `map:=` 指回原 `warehouse.yaml` 即可；原文件都在
  `/home/xuzhijie/hetero_fleet_ws` 里。
* **换一张新地图**：同样的四步流程可以复用——`prepare_map_mesh.py` 会自己拟合
  地板平面，不依赖本文里的具体数字。
* **渲染成白色 / 地图没出来**：先查 `--render-engine ogre` 和
  `GZ_SIM_RESOURCE_PATH`（见第 5 节）。

---

## 更新（2026-09-28）：地图本身又手工修过一遍

点云在 `~/Documents/Codex/2026-09-18/pcd-gazebo` 里用 `edit.sh`（点云编辑器）又清理/补过一版：

* 编辑后点云：`cleaned/geomap_combined12_noroof_edit_0928_1452.pcd`（23,807,110 点，
  比上一版少 1,066,650 点，另用 `fill column` 补了约 18 万点）
* 新模型（名字仍是 `geomap_combined12_noroof`，工程里的引用不用改）：
  `pcd-gazebo/models/geomap_combined13_noroof`（147,053 顶点 / 294,660 面）

本工作空间里 `assets/scan_sources/geomap_combined12_noroof/meshes/` 已换成这一版，
并重跑了 README §12 的四步链：

| 产物 | 结果 |
| --- | --- |
| 视觉网格 | 147,053 顶点 / 294,660 面，地面拟合残差 σ=2.3 cm，`dz = -0.197`（行走面落在 z=0） |
| 碰撞网格 | 135,639 面 |
| Nav2 栅格 | 649 × 1684 @ 0.05 m，障碍 1772.5 m² / 自由 959.8 m² |
| 碰撞盒 | 4,049 个（改图前文档记的是 4,430 个） |
| `verify_scene.py` | `RESULT: scene is consistent`（4 出生点 + 10 个 zone 全在同一连通区、净空达标） |

> 注意：本工作空间的地图文件是**未纳入 git 的新文件**，所以替换是原地覆盖的；
> 上一版（未手工编辑）的点云与模型仍在 pcd-gazebo 里（`cleaned/geomap_combined12_noroof.pcd`
> + `models/geomap_combined12_noroof`），随时可以重跑这四步回到那一版。
