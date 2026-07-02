include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- ==================== 坐标系配置 (Frames) ====================
  map_frame = "map",                              -- 地图坐标系，SLAM发布的核心原点，默认 "map"
  tracking_frame = "base_footprint",              -- 追踪坐标系，算法将所有传感器数据投影到该坐标系下。通常有IMU时设为imu_link，但2D建图设为 base_footprint 最稳定
  published_frame = "odom",                       -- 算法发布TF的子坐标系。如果有里程计，发布到 odom，TF树将是 map -> odom -> base_footprint
  odom_frame = "odom",                            -- 里程计坐标系，默认 "odom"
  provide_odom_frame = false,                     -- 是否由 Cartographer 发布 odom 到 base_footprint 的 TF。因为 Gazebo 驱动插件已经发布了该 TF，此处必须为 false，否则会发生 TF 冲突
  publish_frame_projected_to_2d = true,           -- 将发布的坐标变换投影到2D平面中（强制 roll/pitch 为 0，防止Z轴微小飘移造成地图倾斜）
  use_pose_extrapolator = true,                   -- 使用位姿外推器，可以结合里程计和IMU更好地平滑估计运动位姿
  
  -- ==================== 传感器通道配置 (Inputs) ====================
  use_odometry = true,                            -- 是否使用里程计数据。仿真环境中里程计数据（/odom）非常干净，开启可以极大提升建图的鲁棒性
  use_nav_sat = false,                            -- 是否使用 GPS/导航卫星数据。室内/仿真建图设为 false
  use_landmarks = false,                          -- 是否使用路标数据，设为 false
  num_laser_scans = 1,                            -- 激光雷达（2D）的输入通道数量。小车只有1个单线雷达，设为 1
  num_multi_echo_laser_scans = 0,                 -- 多回波雷达通道数，设为 0
  num_subdivisions_per_laser_scan = 1,            -- 每一帧雷达数据分割的数量，单线雷达一般设为 1
  num_point_clouds = 0,                           -- 3D点云输入通道数量，2D建图设为 0
  
  -- ==================== 坐标变换与发布周期配置 (Timeouts & Periods) ====================
  lookup_transform_timeout_sec = 0.2,             -- 查找 TF 变换的超时时间（秒）
  submap_publish_period_sec = 0.3,                -- 子图（Submap）的发布周期（秒），用于 RViz 实时显示建图过程
  pose_publish_period_sec = 5e-3,                 -- 位姿发布周期（秒），例如 5ms 对应 200Hz
  trajectory_publish_period_sec = 30e-3,          -- 轨迹发布周期（秒），用于控制轨迹更新频率
  
  -- ==================== 传感器数据采样比例配置 (Sampling Ratios) ====================
  rangefinder_sampling_ratio = 1.,                -- 雷达数据采样率（1.0表示全部使用）
  odometry_sampling_ratio = 1.,                   -- 里程计数据采样率
  fixed_frame_pose_sampling_ratio = 1.,           -- 固定帧位姿采样率
  imu_sampling_ratio = 1.,                        -- IMU数据采样率
  landmarks_sampling_ratio = 1.,                  -- 路标数据采样率
}

-- 启用2D SLAM轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true

-- ==================== 2D局部地图构建参数 (Local SLAM) ====================
TRAJECTORY_BUILDER_2D.min_range = 0.1             -- 雷达最小有效距离（过滤掉车身自身的反射噪点）
TRAJECTORY_BUILDER_2D.max_range = 8.0             -- 雷达最大有效距离（小车单线雷达量程限制，通常设为 8-10 米）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0 -- 当雷达未探测到物体时，光束的标称距离（用于清除障碍物栅格）
TRAJECTORY_BUILDER_2D.use_imu_data = false        -- 是否在2D SLAM中使用IMU。由于仿真环境中IMU存在较大的高频抖动和累积偏差，设为 false 可以让建图更加平滑，完全依靠雷达+里程计进行扫描匹配

-- 匹配器（Scan Matcher）参数
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 开启实时相关性扫描匹配（无IMU时极大提高首次匹配精度，防丢）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- 触发子图更新的最小旋转弧度

-- ==================== 后端位姿图优化参数 (Global SLAM) ====================
POSE_GRAPH.constraint_builder.min_score = 0.65     -- 闭环检测约束匹配的最小得分门槛（越高越严格，避免错误闭环造成的地图错位）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 -- 全局重定位约束匹配的最小得分门槛

return options
