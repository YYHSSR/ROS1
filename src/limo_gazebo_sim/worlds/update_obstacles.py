#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil

# ====================================================================
#                      📍 坐标配置接口 (Obstacle Coordinates Config)
# ====================================================================
# 您可以在下方直接修改各障碍物的中心位置坐标(x, y, z)以及尺寸参数：

# 1. 双向对称斜坡 (Slope)
SLOPE_CONFIG = {
    "x": 1.32,
    "y": -0.4,
    "z": 0.0,
    "height": 0.2,      # 斜坡最大高度 (米)
    "flat_len": 1.0,    # 顶部平坦段长度 (米)
    "slope_len": 0.6,   # 两侧坡道长度 (米)
    "width": 0.96        # 宽度 (米)
}

# 2. 红绿灯 (Traffic Light)
TRAFFIC_LIGHT_CONFIG = {
    "x": -1.7,
    "y": -0.4,
    "z": 0.0
}

# 3. 树木列表 (Trees - 可在此增减树木数量及修改坐标)
# 注意：即使在这里写了相同的 name，脚本底层的自动排重逻辑也会自动为其加上 _1, _2 等后缀。
TREES_CONFIG = [
    # 体育馆区域 (2 棵树)
    {"name": "tree_teaching_2", "x": -1.3, "y": 1.7},
    {"name": "tree_teaching_2", "x": -2, "y": 1.7},
    
    # 教学楼区域 (4 棵树)
    {"name": "tree_library_1", "x": 1.1, "y": 0.4},
    {"name": "tree_library_2", "x": 2.0, "y": 0.4},
    {"name": "tree_library_3", "x": 1.1, "y": 2.15},
    {"name": "tree_library_4", "x": 2.0, "y": 2.15},
    
    # 实验楼区域 (2 棵树)
    {"name": "tree_gym_1", "x": -1.7, "y": -1.7},
    {"name": "tree_experimental_1", "x": -1.7, "y": -1.0},

    # 图书馆区域 (3 棵树)
    {"name": "tree_experimental_1", "x": 2.0, "y": -1.8},
    {"name": "tree_experimental_1", "x": 1.35, "y": -1.8},
    {"name": "tree_experimental_1", "x": 0.7, "y": -1.8},
]
# ====================================================================

# 路径定义
current_dir = os.path.dirname(os.path.abspath(__file__))
world_file = os.path.join(current_dir, "limo_sandtable.world")
backup_file = os.path.join(current_dir, "limo_sandtable.world.backup")

def main():
    if not os.path.exists(backup_file):
        print(f"错误: 找不到干净的备份文件 {backup_file}，请确保该备份文件存在！")
        return

    # 从干净的备份恢复 world 文件，防止重复堆叠写入
    shutil.copyfile(backup_file, world_file)
    print("已成功从原版备份恢复 limo_sandtable.world")

    xml_models = "\n    <!-- ================= 自定义生成障碍物开始 ================= -->"

    # 1. 构造斜坡 XML
    slope_x = SLOPE_CONFIG["x"]
    slope_y = SLOPE_CONFIG["y"]
    slope_z = SLOPE_CONFIG["z"]
    sh = SLOPE_CONFIG["height"]
    fl = SLOPE_CONFIG["flat_len"]
    sl = SLOPE_CONFIG["slope_len"]
    sw = SLOPE_CONFIG["width"]
    
    # 计算坡道中点相对位置及倾斜角 (利用 pitch 旋转 Y 轴实现 X 轴对齐)
    import math
    angle = math.asin(sh / sl) if sh <= sl else 0.0
    relative_x_offset = fl / 2.0 + sl / 2.0
    half_sh = sh / 2.0
    
    xml_models += f"""
    <!-- Slope Model (Double-sided symmetric ramp/bridge, X-aligned) -->
    <model name='slope_obstacle'>
      <static>true</static>
      <pose>{slope_x} {slope_y} {slope_z} 0 0 0</pose>
      <link name='link'>
        <!-- Flat Top -->
        <collision name='flat_collision'>
          <pose>0 0 {half_sh} 0 0 0</pose>
          <geometry>
            <box>
              <size>{fl} {sw} {sh}</size>
            </box>
          </geometry>
        </collision>
        <visual name='flat_visual'>
          <pose>0 0 {half_sh} 0 0 0</pose>
          <geometry>
            <box>
              <size>{fl} {sw} {sh}</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        
        <!-- West Slope (Up) -->
        <collision name='west_collision'>
          <pose>-{relative_x_offset} 0 {half_sh} 0 -{angle:.4f} 0</pose>
          <geometry>
            <box>
              <size>{sl} {sw} 0.01</size>
            </box>
          </geometry>
        </collision>
        <visual name='west_visual'>
          <pose>-{relative_x_offset} 0 {half_sh} 0 -{angle:.4f} 0</pose>
          <geometry>
            <box>
              <size>{sl} {sw} 0.01</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        
        <!-- East Slope (Down) -->
        <collision name='east_collision'>
          <pose>{relative_x_offset} 0 {half_sh} 0 {angle:.4f} 0</pose>
          <geometry>
            <box>
              <size>{sl} {sw} 0.01</size>
            </box>
          </geometry>
        </collision>
        <visual name='east_visual'>
          <pose>{relative_x_offset} 0 {half_sh} 0 {angle:.4f} 0</pose>
          <geometry>
            <box>
              <size>{sl} {sw} 0.01</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""

    # 2. 构造红绿灯 XML (灯头面朝西侧-面向马路和小车)
    tl_x = TRAFFIC_LIGHT_CONFIG["x"]
    tl_y = TRAFFIC_LIGHT_CONFIG["y"]
    tl_z = TRAFFIC_LIGHT_CONFIG["z"]
    
    xml_models += f"""
    <!-- Traffic Light Model (placed inside Experimental Building green area facing West) -->
    <model name='traffic_light_obstacle'>
      <static>true</static>
      <pose>{tl_x} {tl_y} {tl_z} 0 0 0</pose>
      <link name='link'>
        <!-- Base -->
        <collision name='base_collision'>
          <pose>0 0 0.01 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.02</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='base_visual'>
          <pose>0 0 0.01 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.02</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <!-- Pole -->
        <collision name='pole_collision'>
          <pose>0 0 0.27 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.015</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='pole_visual'>
          <pose>0 0 0.27 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.015</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <!-- Box Head -->
        <collision name='head_collision'>
          <pose>0 0 0.67 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1 0.1 0.3</size>
            </box>
          </geometry>
        </collision>
        <visual name='head_visual'>
          <pose>0 0 0.67 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1 0.1 0.3</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Black</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <!-- Red Light (Sphere facing West) -->
        <visual name='red_light_visual'>
          <pose>-0.05 0 0.77 0 0 0</pose>
          <geometry>
            <sphere>
              <radius>0.025</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Red</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <!-- Yellow Light (Sphere facing West) -->
        <visual name='yellow_light_visual'>
          <pose>-0.05 0 0.67 0 0 0</pose>
          <geometry>
            <sphere>
              <radius>0.025</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Yellow</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <!-- Green Light (Sphere facing West) -->
        <visual name='green_light_visual'>
          <pose>-0.05 0 0.57 0 0 0</pose>
          <geometry>
            <sphere>
              <radius>0.025</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Green</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""

    # 3. 构造树木列表 XML (加入自动去重防错逻辑)
    seen_names = {}
    for tree in TREES_CONFIG:
        base_name = tree["name"]
        if base_name in seen_names:
            seen_names[base_name] += 1
            name = f"{base_name}_{seen_names[base_name]}"
        else:
            seen_names[base_name] = 1
            name = base_name
        tx = tree["x"]
        ty = tree["y"]
        xml_models += f"""
    <!-- Tree Model: {name} (placed at X={tx}, Y={ty}) -->
    <model name='{name}'>
      <static>true</static>
      <pose>{tx} {ty} 0 0 0 0</pose>
      <link name='link'>
        <!-- Pot -->
        <collision name='pot_collision'>
          <pose>0 0 0.075 0 0 0</pose>
          <geometry>
            <box>
              <size>0.25 0.25 0.15</size>
            </box>
          </geometry>
        </collision>
        <visual name='pot_visual'>
          <pose>0 0 0.075 0 0 0</pose>
          <geometry>
            <box>
              <size>0.25 0.25 0.15</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <!-- Trunk -->
        <collision name='trunk_collision'>
          <pose>0 0 0.25 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='trunk_visual'>
          <pose>0 0 0.25 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Wood</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <!-- Foliage (Sphere) -->
        <collision name='foliage_collision'>
          <pose>0 0 0.55 0 0 0</pose>
          <geometry>
            <sphere>
              <radius>0.25</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name='foliage_visual'>
          <pose>0 0 0.55 0 0 0</pose>
          <geometry>
            <sphere>
              <radius>0.25</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Green</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""
    xml_models += "\n    <!-- ================= 自定义生成障碍物结束 ================= -->\n"

    # 读取并修改世界文件
    with open(world_file, 'r') as file:
        content = file.read()

    if "</world>" in content:
        content = content.replace("  </world>", xml_models + "  </world>")
        with open(world_file, 'w') as file:
            file.write(content)
        print("🎉 成功: 世界地图已根据您的配置坐标重新生成！")
    else:
        print("错误: 未能在世界文件中定位到 </world> 结束标签！")

if __name__ == "__main__":
    main()
