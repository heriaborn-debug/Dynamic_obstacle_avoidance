# M20 FIRST 局部路径规划源码包

本目录整理自本机保存的 NVIDIA NX 导航代码快照，用于学习和复盘 M20 的 A* 全局引导、前视目标生成和 FIRST 局部轨迹选择。

## 数据链路

```text
/goal_pose + /map + /lio/robo/odom
        ↓
A* 全局规划：/m20/navigation/global_path
        ↓
2.5 m Lookahead：/local_goal
        ↓
FIRST：4641 条预生成轨迹的碰撞检查与代价选择
        ↓
/m20/v2/local_plan + /m20/v2/track_path + /cmd_vel
```

FIRST 使用的局部障碍输入为：

```text
/m20/local_mapping/traversability/obstacle_cloud
```

该点云由 104 的 `/m20/local_mapping/evidence_cloud` 经过 NVIDIA NX 通行性分析产生。当前正式链路不直接使用 `/NAV_POINTS` 进行 FIRST 碰撞检查。

## 目录说明

- `m20_first_planner_v2`：FIRST C++ 局部规划器、实机配置、launch、RViz 和测试；
- `m20_navigation_integration`：A* 全局规划、2.5 m 前视点生成和整套集成 launch；
- `m20_first_paths`：4641 条预生成轨迹及四层体素—轨迹对应关系；
- `start_v2_astar_navigation.sh`：NX 上的正式启动入口。

建议依次阅读：

1. `m20_navigation_integration/config/m20_astar.yaml`；
2. `m20_navigation_integration/scripts/m20_astar_global_planner.py`；
3. `m20_navigation_integration/scripts/m20_path_lookahead.py`；
4. `m20_first_planner_v2/config/first_planner_v2.yaml`；
5. `m20_first_planner_v2/src/m20_first_planner_v2_cpp.cpp`；
6. `m20_first_planner_v2/README.md`。

## 快照说明

规划源码采用本机保存的较新 `m20_v2_nx_snapshot`；轨迹库补充自完整部署目录 `m20_v2_nx_deploy/m20_first_paths`。打包日期为 2026-08-31。

代码中的绝对路径仍以 NX 的 `/home/nvidia/m20_v2_nx` 为基准。在其他机器编译或运行前，需要修改路径、ROS 2 环境、DDS 配置和机器狗运动桥。未经回放与实机安全测试，不要直接启用 `/cmd_vel` 转发。
