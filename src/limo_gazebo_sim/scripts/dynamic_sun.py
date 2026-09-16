#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
动态日光与动态阴影控制器 (Dynamic Sun & Shadows Controller)
功能:
  在保持 100% 白天的前提下，让太阳方位角与高度角持续平滑动态运动，
  使沙盘上所有建筑物、树木、行人以及小车的物理阴影随时间产生连续的旋转与伸缩，
  既生动自然，又绝不会进入黑夜（保证单目/深度相机视觉算法始终处于良好光照下）。
"""

import math
import rospy
from gazebo_msgs.srv import SetLightProperties, SetLightPropertiesRequest
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA

def dynamic_sun_controller():
    rospy.init_node('dynamic_sun_controller', anonymous=False)

    # 可配置参数
    light_name = rospy.get_param('~light_name', 'sun')
    # 太阳旋转一周的周期 (秒，默认 120 秒完成一次 360 度日影循环)
    cycle_period = float(rospy.get_param('~cycle_period', 120.0))
    # 更新频率 (Hz，10Hz 即可实现肉眼非常丝滑的影子转动)
    update_rate = float(rospy.get_param('~update_rate', 10.0))
    # 最低太阳高度角 (度，默认 35 度，保证太阳始终高悬于地平线上，永远是明亮白天)
    min_elevation = float(rospy.get_param('~min_elevation_deg', 35.0))
    # 最高太阳高度角 (度，默认 75 度，接近正午天顶)
    max_elevation = float(rospy.get_param('~max_elevation_deg', 75.0))
    # 运动模式: 'orbit' (360度环绕旋转), 'oscillate' (在早中晚之间平滑往返)
    mode = rospy.get_param('~mode', 'orbit')

    min_elev_rad = math.radians(min_elevation)
    max_elev_rad = math.radians(max_elevation)
    mid_elev_rad = (min_elev_rad + max_elev_rad) / 2.0
    amp_elev_rad = (max_elev_rad - min_elev_rad) / 2.0

    service_name = '/gazebo/set_light_properties'
    rospy.loginfo(f"[dynamic_sun] 等待 Gazebo 光照服务 [{service_name}] 就绪...")

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
            break
        except rospy.ROSException:
            rospy.loginfo(f"[dynamic_sun] 正在等待 Gazebo 启动并加载服务...")

    if rospy.is_shutdown():
        return

    set_light = rospy.ServiceProxy(service_name, SetLightProperties, persistent=True)
    rospy.loginfo(f"[dynamic_sun] 光照服务已连接！开始运行动态白天光照系统 (周期: {cycle_period}s, 模式: {mode})")

    rate = rospy.Rate(update_rate)
    start_time = rospy.Time.now().to_sec()

    req = SetLightPropertiesRequest()
    req.light_name = light_name
    req.cast_shadows = True
    req.specular = ColorRGBA(0.2, 0.2, 0.2, 1.0)
    req.attenuation_constant = 0.9
    req.attenuation_linear = 0.01
    req.attenuation_quadratic = 0.001
    req.pose = Pose(Point(0.0, 0.0, 25.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed = current_time - start_time

        # 归一化相位 0 ~ 2*pi
        phase = (2.0 * math.pi * (elapsed % cycle_period)) / cycle_period

        if mode == 'oscillate':
            # 往返摆动: 方位角在 -45度 ~ +45度之间来回摆动
            azimuth = 0.785 * math.sin(phase)
            elevation = mid_elev_rad + amp_elev_rad * math.cos(phase)
        else:
            # 默认 orbit: 太阳方位角 360 度连续平滑旋转
            azimuth = phase
            elevation = mid_elev_rad + amp_elev_rad * math.sin(phase)

        # 根据极坐标计算射向地面的平行光方向向量 (Light Direction Vector)
        # 光线从空中射向地面，因此 Z 分量必须恒为负数，且绝对值较大以保证永远是白天
        cos_elev = math.cos(elevation)
        dx = -cos_elev * math.cos(azimuth)
        dy = -cos_elev * math.sin(azimuth)
        dz = -math.sin(elevation)

        req.direction = Vector3(dx, dy, dz)

        # 动态微调光照色彩 (在温和暖白与明亮日光之间呼吸式自然微变)
        diffuse_val = 0.88 + 0.08 * math.sin(phase)
        req.diffuse = ColorRGBA(diffuse_val, diffuse_val * 0.98, diffuse_val * 0.94, 1.0)

        try:
            resp = set_light(req)
            if not resp.success:
                rospy.logwarn_throttle(10, f"[dynamic_sun] 光照更新提示: {resp.status_message}")
        except rospy.ServiceException as e:
            # 当仿真重置或暂停时尝试重连
            rospy.logwarn_throttle(5, f"[dynamic_sun] 服务调用异常 (可能正在重置): {e}")
            try:
                set_light = rospy.ServiceProxy(service_name, SetLightProperties, persistent=True)
            except Exception:
                pass

        rate.sleep()

if __name__ == '__main__':
    try:
        dynamic_sun_controller()
    except rospy.ROSInterruptException:
        pass
