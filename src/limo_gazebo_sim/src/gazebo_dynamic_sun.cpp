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
        : cycle_period_(120.0),
          min_elevation_(35.0),
          max_elevation_(75.0),
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
        if (_sdf->HasElement("light_name")) {
            this->light_name_ = _sdf->Get<std::string>("light_name");
        }

        // 初始化 Gazebo 内部传输节点，直接发布 ~/light/modify 消息驱动物理渲染引擎更新阴影
        this->node_ = transport::NodePtr(new transport::Node());
        this->node_->Init(this->world_->Name());
        this->light_pub_ = this->node_->Advertise<msgs::Light>("~/light/modify");

        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DynamicSunPlugin::OnUpdate, this, _1));

        gzmsg << "[DynamicSunPlugin] 成功加载到世界仿真中！目标光源: [" << this->light_name_
              << "], 旋转周期: " << this->cycle_period_ << "s, 仰角范围: "
              << this->min_elevation_ << "° ~ " << this->max_elevation_ << "°" << std::endl;
    }

    void OnUpdate(const common::UpdateInfo &_info) {
        common::Time sim_time = _info.simTime;
        // 每 0.05 秒仿真时间更新一次 (20Hz)，保证人眼观察极其平滑丝滑
        if ((sim_time - this->last_update_time_).Double() < 0.05) {
            return;
        }
        this->last_update_time_ = sim_time;

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
        msg.set_name(this->light_name_);
        msgs::Set(msg.mutable_direction(), ignition::math::Vector3d(dx, dy, dz));

        double diffuse = 0.88 + 0.08 * std::sin(phase);
        msg.mutable_diffuse()->set_r(diffuse);
        msg.mutable_diffuse()->set_g(diffuse * 0.98);
        msg.mutable_diffuse()->set_b(diffuse * 0.94);
        msg.mutable_diffuse()->set_a(1.0);

        this->light_pub_->Publish(msg);
    }

private:
    physics::WorldPtr world_;
    transport::NodePtr node_;
    transport::PublisherPtr light_pub_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;

    double cycle_period_;
    double min_elevation_;
    double max_elevation_;
    std::string light_name_;
};

GZ_REGISTER_WORLD_PLUGIN(DynamicSunPlugin)

}
