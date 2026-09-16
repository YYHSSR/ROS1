#include <cmath>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

/**
 * @brief Gazebo 原生世界插件 (WorldPlugin) - 动态白天日光与移动物理阴影
 * 
 * 直接嵌入在 .world 文件的 <world> 标签内，无需启动外部 ROS 节点：
 * 1. 恒定维持在明亮白天 (太阳仰角 >= 35°，绝不进入黑夜)
 * 2. 太阳方位角 360° 连续平滑旋转，路面所有物体物理阴影产生平滑回旋与伸缩效果
 */
class DynamicSunPlugin : public WorldPlugin {
public:
    DynamicSunPlugin()
        : cycle_period_(180.0),
          min_elevation_(35.0),
          max_elevation_(75.0),
          update_interval_(0.2),
          light_name_("sun") {}

    virtual ~DynamicSunPlugin() {}

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
        this->world_ = _world;

        if (_sdf->HasElement("cycle_period")) {
            this->cycle_period_ = _sdf->Get<double>("cycle_period");
        }
        if (_sdf->HasElement("min_elevation")) {
            this->min_elevation_ = _sdf->Get<double>("min_elevation");
        }
        if (_sdf->HasElement("max_elevation")) {
            this->max_elevation_ = _sdf->Get<double>("max_elevation");
        }
        if (_sdf->HasElement("update_interval")) {
            this->update_interval_ = _sdf->Get<double>("update_interval");
        }
        if (_sdf->HasElement("light_name")) {
            this->light_name_ = _sdf->Get<std::string>("light_name");
        }

        // 获取物理世界中的光源指针 (若已就绪)
        this->light_ = this->world_->LightByName(this->light_name_);

        // 初始化 Gazebo 内部传输节点，直接发布 ~/light/modify 消息驱动物理渲染引擎更新阴影
        this->node_ = transport::NodePtr(new transport::Node());
        this->node_->Init(this->world_->Name());
        this->light_pub_ = this->node_->Advertise<msgs::Light>("~/light/modify");

        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DynamicSunPlugin::OnUpdate, this, _1));

        gzmsg << "[DynamicSunPlugin] 成功加载到世界仿真中！目标光源: [" << this->light_name_
              << "], 旋转周期: " << this->cycle_period_ << "s, 仰角范围: "
              << this->min_elevation_ << "° ~ " << this->max_elevation_
              << "°, 更新步长: " << this->update_interval_ << "s" << std::endl;
    }

    void OnUpdate(const common::UpdateInfo &_info) {
        common::Time sim_time = _info.simTime;
        // 节流平滑更新 (默认 5Hz / 0.2s)，既保证人眼观察极其平滑丝滑，又避免高频重建阴影贴图导致 OGRE 渲染卡顿
        if ((sim_time - this->last_update_time_).Double() < this->update_interval_) {
            return;
        }
        this->last_update_time_ = sim_time;

        if (!this->light_) {
            this->light_ = this->world_->LightByName(this->light_name_);
        }

        double t = sim_time.Double();
        double phase = (2.0 * M_PI * std::fmod(t, this->cycle_period_)) / this->cycle_period_;

        double min_elev_rad = this->min_elevation_ * M_PI / 180.0;
        double max_elev_rad = this->max_elevation_ * M_PI / 180.0;
        double mid_elev_rad = (min_elev_rad + max_elev_rad) / 2.0;
        double amp_elev_rad = (max_elev_rad - min_elev_rad) / 2.0;

        double azimuth = phase;
        double elevation = mid_elev_rad + amp_elev_rad * std::sin(phase);

        // 光源射向地面的方向矢量 (向下入射，dz 始终为负且绝对值较大，保持高悬白天)
        double cos_elev = std::cos(elevation);
        double dx = -cos_elev * std::cos(azimuth);
        double dy = -cos_elev * std::sin(azimuth);
        double dz = -std::sin(elevation);

        msgs::Light msg;
        if (this->light_) {
            this->light_->FillMsg(msg);
        } else {
            msg.set_name(this->light_name_);
            msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(0, 0, 20, 0, 0, 0));
        }

        // 核心修复: 严谨显式指定定向平行光与阴影投射标志，防止 SDF 解析重构时丢失阴影或退化为点光源
        msg.set_name(this->light_name_);
        msg.set_type(msgs::Light_LightType_DIRECTIONAL);
        msg.set_cast_shadows(true);
        msgs::Set(msg.mutable_direction(), ignition::math::Vector3d(dx, dy, dz));

        double diffuse = 0.88 + 0.08 * std::sin(phase);
        msg.mutable_diffuse()->set_r(diffuse);
        msg.mutable_diffuse()->set_g(diffuse * 0.98);
        msg.mutable_diffuse()->set_b(diffuse * 0.94);
        msg.mutable_diffuse()->set_a(1.0);

        msg.mutable_specular()->set_r(0.2);
        msg.mutable_specular()->set_g(0.2);
        msg.mutable_specular()->set_b(0.2);
        msg.mutable_specular()->set_a(1.0);

        // 同步物理引擎中的光源状态
        if (this->light_) {
            this->light_->ProcessMsg(msg);
        }

        // 发布给渲染引擎更新阴影
        this->light_pub_->Publish(msg);
    }

private:
    physics::WorldPtr world_;
    physics::LightPtr light_;
    transport::NodePtr node_;
    transport::PublisherPtr light_pub_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;

    double cycle_period_;
    double min_elevation_;
    double max_elevation_;
    double update_interval_;
    std::string light_name_;
};

GZ_REGISTER_WORLD_PLUGIN(DynamicSunPlugin)

}
