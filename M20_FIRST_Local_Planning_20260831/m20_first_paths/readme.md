# 数据说明

## 文件概览
- path: 4641  points: 46410  X * Y = 90 * 80

## 文件详细说明

### 1. pathList.ply
- **用途**: 存储DWA候选轨迹的采样点数据，包含4641条轨迹，每条轨迹10个采样点
- **调用位置**: `ReadPathData::readStartPaths()` (dwa_planner.h:127)
- **存储位置**: `path_data_.startPaths[groupID]` 和 `path_data_.paths[groupID]`
- **数据格式**: 每行包含 `x y z groupID` (4个值)
- **存储结构**:
  - **轨迹数量**: 4641条（groupID从0到4640）
  - **每条轨迹采样点数**: 10个点
  - **总点数**: 46410个点（4641 × 10）
  - **数据组织方式**: 按groupID分组，连续存储。每条轨迹的10个点按顺序存储，代表该轨迹在时间上的离散采样
  - **坐标系统**: 所有点坐标相对于机器人初始位置（base_link坐标系）
  - **Z坐标**: 通常为0（2D平面路径）
- **数据示例**:
  ```
  -0.05 -0.06 0 0          # 轨迹0的第1个采样点
  -0.104635 -0.115812 0 0  # 轨迹0的第2个采样点
  ...
  -0.661519 -0.375306 0 0 # 轨迹0的第10个采样点
  -0.05 -0.06 0 1          # 轨迹1的第1个采样点
  ...
  1.16124 1.0614 0 4640    # 轨迹4640的第10个采样点
  ```
- **使用说明**: 
  - 这些采样点用于路径碰撞检测（`judgePath()`函数）
  - 每个采样点代表机器人在该时刻的预期位置
  - 在碰撞检测时，会检查每个采样点位置上的机器人包围盒是否与障碍物重叠

### 2. path_end.ply
- **用途**: 存储DWA候选轨迹的终点数据和速度信息，包含4641条轨迹的终点坐标、yaw角和速度命令
- **调用位置**: `ReadPathData::readPathList()` (dwa_planner.h:159)
- **存储位置**: 
  - `path_data_.vel[pathID]` (速度信息，存储为point.x/y/z，分别对应x_vel, y_vel, yaw_vel)
  - `path_data_.endDirPathList_x/y/yaw` (终点坐标和朝向)
- **数据格式**: 每行包含 `endX endY endZ endYaw point.x point.y point.z pathID` (8个值)
  - `endX, endY, endZ`: 路径终点坐标（base_link坐标系）
  - `endYaw`: 路径终点朝向角度（弧度）
  - `point.x, point.y, point.z`: 速度命令（x方向线速度、y方向线速度、角速度）
  - `pathID`: 路径编号（0-4640）
- **存储结构**:
  - **轨迹数量**: 4641条（pathID从0到4640）
  - **每条轨迹数据**: 1行（每条轨迹只有终点和速度信息）
  - **总行数**: 4641行
  - **数据组织方式**: 按pathID顺序存储，每条轨迹一行
  - **速度特性**: **每条路径的速度都是匀速**（每条路径只有一个速度值，在整个轨迹执行过程中保持不变）
  - **坐标系统**: 所有坐标相对于机器人初始位置（base_link坐标系）
- **数据示例**:
  ```
  -0.661519 -0.375306 0 -0.8 -0.5 -0.6 -0.8 0    # 轨迹0: 终点(-0.66, -0.38), yaw=-0.8, 速度(-0.5, -0.6, -0.8)
  -0.648002 -0.40722 0 -0.7 -0.5 -0.6 -0.7 1     # 轨迹1: 终点(-0.65, -0.41), yaw=-0.7, 速度(-0.5, -0.6, -0.7)
  -0.5 -0.6 0 0 -0.5 -0.6 0 8                    # 轨迹8: 终点(-0.5, -0.6), yaw=0, 速度(-0.5, -0.6, 0)
  ...
  1.16124 1.0614 0 0.8 1.5 0.6 0.8 4640          # 轨迹4640: 终点(1.16, 1.06), yaw=0.8, 速度(1.5, 0.6, 0.8)
  ```
- **使用说明**: 
  - 速度信息用于远距离路径跟随阶段，直接作为控制指令输出（`CmdCalculate()`函数）
  - 终点坐标和yaw用于路径评分计算（`pathScoreAndSelect()`函数）
  - 每条路径在整个执行过程中保持恒定速度（匀速运动）
- **特殊轨迹类型**：
  - **原地转向轨迹**：存在16条原地转向轨迹（路径ID: 1207-1214, 1216-1223）
    - 终点位置：(0, 0) - 位置未改变
    - Yaw角度范围：-0.8 ~ 0.8 rad（-45.84° ~ 45.84°），步长0.1 rad
    - 速度命令：x_vel=0, y_vel=0, yaw_vel=±[0.1, 0.2, ..., 0.8] rad/s
    - 用途：用于机器人原地调整朝向，不改变位置
    - 注意：路径1215是静止轨迹（x=0, y=0, yaw=0, 所有速度=0）
  - 速度值存储在`vel[pathID]->points[0]`中，通过`points[0].x/y/z`访问

- **四个文件（correspondences.ply, correspondences_4.ply, correspondences_5.ply, correspondences_6.ply）的信息存储格式与结构说明如下：**

#### - **格式说明：**
  - 每一行对应一个栅格(gridVoxelID)，格式如下：
    ```
    <gridVoxelID> <pathID1> <pathID2> ... <pathIDn> -1
    ```
    - `gridVoxelID`: 栅格的唯一编号（通常为从0开始的整数递增）
    - `pathID1, pathID2, ...`: 该栅格对应的路径编号列表（**一个栅格可以对应多条轨迹**）
    - `-1`: 结束标记，表示该栅格的路径列表结束

  - **重要特性**：
    - **一个栅格可以对应多条轨迹**（常见情况）
    - 路径ID可能重复出现（表示该路径多次经过该栅格）
    - 若栅格没有对应路径，格式为：`<gridVoxelID> -1`
    - 文件以纯文本方式保存，没有表头

#### - **结构示例：**
  ```
  0 -1                    # 栅格0没有对应路径
  1 -1                    # 栅格1没有对应路径
  672 2 3 -1              # 栅格672对应路径2和3
  750 4 5 22 -1           # 栅格750对应路径4、5、22
  751 3 3 4 21 22 -1      # 栅格751对应路径3（2次）、4、21、22
  755 0 0 1 2 18 19 20 37 221 -1  # 栅格755对应多条路径
  ```

#### - **四个文件的区别（适用机器人尺寸不同）：**
| 文件名                | robotlength | robotwidth | 具体用途（惩罚层级）  |
|-----------------------|-------------|------------|---------------------|
| correspondences.ply   | 0.9         | 0.4        | 基础碰撞/计数        |
| correspondences_4.ply | 1.1         | 0.5        | 惩罚层1（penalty1）  |
| correspondences_5.ply | 1.2         | 0.7        | 惩罚层2（penalty2）  |
| correspondences_6.ply | 1.4         | 0.8        | 惩罚层3（penalty3）  |

- 每个文件的gridVoxelID数量：7200个（gridVoxelNum_参数）
- pathID 取值范围：0~4640（与全局轨迹数量一致），若未分配则为-1
- **统计信息**：在correspondences.ply中，有2732个栅格对应多条不同路径，最多有64个栅格对应全部4641条路径

> **总结**：四个correspondences文件存储格式为"每行第一个数字是栅格ID，后面跟着多个路径ID，最后以-1结束"，一个栅格可以对应多条轨迹。


### 3. correspondences.ply
- **用途**: 存储栅格与路径的对应关系，用于基础碰撞检测
- **对应数据**: robotlength = 0.9, robotwidth = 0.4, t = 1.0
- **调用位置**: `ReadPathData::readCorrespondences(4)` → 当 `num=1` 时 (dwa_planner.h:201)
- **存储位置**: `path_data_.correspondences[gridVoxelID]`
- **使用位置**: `DwaPlanner::updatePathPenalty()` (dwa_planner.cpp:763) - 用于增加障碍物计数

### 4. correspondences_4.ply
- **用途**: 存储栅格与路径的对应关系，用于第一层障碍物惩罚评分
- **对应数据**: robotlength = 1.1, robotwidth = 0.5, t = 1.0
- **调用位置**: `ReadPathData::readCorrespondences(4)` → 当 `num=2` 时 (dwa_planner.h:202)
- **存储位置**: `path_data_.penalty1[gridVoxelID]`
- **使用位置**: `DwaPlanner::updatePathPenalty()` (dwa_planner.cpp:764) - 用于PathListadd1标记

### 5. correspondences_5.ply
- **用途**: 存储栅格与路径的对应关系，用于第二层障碍物惩罚评分
- **对应数据**: robotlength = 1.2, robotwidth = 0.7, t = 1.0
- **调用位置**: `ReadPathData::readCorrespondences(4)` → 当 `num=3` 时 (dwa_planner.h:203)
- **存储位置**: `path_data_.penalty2[gridVoxelID]`
- **使用位置**: `DwaPlanner::updatePathPenalty()` (dwa_planner.cpp:765) - 用于PathListadd2标记

### 6. correspondences_6.ply
- **用途**: 存储栅格与路径的对应关系，用于第三层障碍物惩罚评分
- **对应数据**: robotlength = 1.4, robotwidth = 0.8, t = 1.0
- **调用位置**: `ReadPathData::readCorrespondences(4)` → 当 `num=4` 时 (dwa_planner.h:204)
- **存储位置**: `path_data_.penalty3[gridVoxelID]`
- **使用位置**: `DwaPlanner::updatePathPenalty()` (dwa_planner.cpp:766) - 用于PathListadd3标记

## 加载流程

所有文件在 `ReadPathData` 构造函数中按以下顺序加载：
1. `readStartPaths()` → 加载 `pathList.ply`
2. `readPathList()` → 加载 `path_end.ply`
3. `readCorrespondences(4)` → 依次加载 `correspondences_6.ply`, `correspondences_5.ply`, `correspondences_4.ply`, `correspondences.ply`

## 使用说明

- `correspondences.ply` 系列文件用于多层障碍物检测和路径评分
- 不同尺寸的机器人对应不同的 correspondences 文件
- 在 `updatePathPenalty()` 函数中，这些对应关系用于标记经过障碍物栅格的路径
- 在 `pathScoreAndSelect()` 函数中，penalty1/2/3 用于计算路径评分权重
