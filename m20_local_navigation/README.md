# M20 局部感知与路径规划

本仓库汇集 M20 轮足机器人已经完成实机联调的局部导航链路，包括 AOS 104 上的时序障碍点云证据融合、NVIDIA NX 上的地形通行性分析，以及 A* 全局引导、Lookahead 局部目标生成和 FIRST 局部轨迹规划。

## 项目摘要

轮足机器人在坡面、遮挡和动态环境中运行时，单帧点云容易闪烁，固定帧拼接又容易产生运动拖影。本系统首先在 104 上利用 IMU 重力对齐和 ODOM 运动补偿维护局部证据体素地图，再由 NX 将稳定点云转化为通行性障碍表示。规划端使用 A* 生成全局引导路径，从路径前方选取 2.5 m 局部目标，最后由 FIRST 在 4641 条预生成运动轨迹中进行碰撞检查和代价选择，以 10 Hz 输出局部路径与速度指令。

整套系统的核心不是单独追求“点云更密”或“路径更短”，而是让感知状态、空间可通行性和机器人可执行轨迹保持一致：障碍出现时快速建立，短暂漏点时稳定保留，明确自由时及时清除；局部规划则只从经过静态碰撞检查的轨迹中选择，并在全部候选路径受阻时主动停车。

## 系统链路

```text
/LIDAR/POINTS + /IMU + /lio/robo/odom
                  ↓
104 Evidence Voxel Mapping
                  ↓
/m20/local_mapping/evidence_cloud
                  ↓
NX Traversability Mapping
                  ↓
/m20/local_mapping/traversability/obstacle_cloud
                  ↓
             FIRST V2
                  ↓
/m20/v2/local_plan + /m20/v2/track_path + /cmd_vel
```

全局目标到局部引导的链路为：

```text
/goal_pose + /map + /lio/robo/odom
                  ↓
             A* Global Planner
                  ↓
/m20/navigation/global_path
                  ↓
         2.5 m Path Lookahead
                  ↓
              /local_goal
                  ↓
              FIRST V2
```

## 目录结构

```text
.
├── perception/
│   └── evidence_mapping_104/
│       ├── drdds_current/          # 104 当前 DrDDS 证据点云实现
│       ├── ros2_shadow/            # 早期 ROS 2 参考实现
│       └── deployment/             # 104 systemd 服务
├── traversability/
│   └── m20_traversability_mapping/ # 地面、净空、坡度、粗糙度和台阶分析
└── planning/
    ├── m20_first_planner_v2/       # FIRST C++ 局部规划器
    ├── m20_navigation_integration/ # A*、Lookahead 和集成 launch
    ├── m20_first_paths/             # 4641 条轨迹与碰撞对应表
    └── start_v2_astar_navigation.sh
```

## 两部分怎样衔接

FIRST 不直接订阅原始 `/NAV_POINTS`，也不直接把 104 的 `evidence_cloud` 当作最终障碍输入。104 输出的证据点云先在 NX 上完成地形和通行性分类，再生成 `/m20/local_mapping/traversability/obstacle_cloud`。FIRST 将该点云视为已经分类的语义障碍单元，每个有效点都可以参与候选轨迹碰撞检查。

这层接口隔离使感知与规划各自保持清晰职责：104 负责稳定空间证据，NX 通行性模块负责判断哪些区域不可通行，FIRST 负责在机器人真实可执行的轨迹集合中选择控制方案。

## 推荐阅读顺序

1. [104 点云算法中文说明](perception/evidence_mapping_104/README.md)
2. [NX 通行性分析中文说明](traversability/m20_traversability_mapping/README_中文导读.md)
3. [FIRST 局部规划中文导读](planning/README_中文导读.md)
4. [证据地图核心实现](perception/evidence_mapping_104/drdds_current/src/evidence_map.cpp)
5. [通行性栅格核心实现](traversability/m20_traversability_mapping/src/traversability_grid.cpp)
6. [A* 全局规划器](planning/m20_navigation_integration/scripts/m20_astar_global_planner.py)
7. [Lookahead 局部目标生成](planning/m20_navigation_integration/scripts/m20_path_lookahead.py)
8. [FIRST C++ 核心实现](planning/m20_first_planner_v2/src/m20_first_planner_v2_cpp.cpp)
9. [FIRST 实机参数](planning/m20_first_planner_v2/config/first_planner_v2.yaml)

## 运行边界

仓库中的启动脚本仍使用 NX 上的 `/home/nvidia/m20_v2_nx` 绝对路径，并依赖 ROS 2 Humble、Fast DDS、PCL、Eigen、厂商 DrDDS、通行性服务及机器狗运动桥。代码快照适合学习、回放与二次开发，不代表在任意机器上解压后即可直接驱动机器人。

真实机器人测试前，应依次验证传感器时间同步、TF、ODOM 连续性、局部障碍点云、轨迹碰撞检查和零速度保护。未完成回放与现场急停测试前，不应启用 `/cmd_vel` 到机器狗的运动转发。

## 快照来源

- 104 点云源码于 2026-08-31 从 `user@10.21.31.104` 只读复制并完成 SHA256 对照；
- NX 通行性分析来自本机保存的 `m20_traversability_mapping` 完整源码，核心算法 11 项单元测试通过；
- FIRST、A* 与 Lookahead 来自本机保存的 NVIDIA NX 导航源码快照；
- FIRST 轨迹库来自完整 NX 部署目录；
- 巡检手册记录该感知—规划链路于 2026-08-06 完成两轮 13 点实机巡检。
