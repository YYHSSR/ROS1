#include <cmath>
#include <algorithm>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

/**
 * @brief Gazebo 仿真沙盘光照强度量化控制插件 (WorldPlugin)
 * 
 * 嵌入在 ros_sandbox_simulation.world 的 <world> 标签中。
 * 用户只需在 .world 文件中修改 <light_percent>50</light_percent> (0 ~ 100 的任意数字)，
 * 保存并重启 Gazebo 世界，即可自动将环境光 (ambient)、太阳直射光 (diffuse) 和高光 (specular)
 * 按照基准比例精准缩放，彻底告别手动计算 RGB 浮点数！
 */
class WorldLightPlugin : public WorldPlugin {
public:
    WorldLightPlugin()
        : light_percent_(100.0),
          light_name_("sun"),
          apply_countdown_(30) {}

    virtual ~WorldLightPlugin() {}

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
        this->world_ = _world;

        // 从 SDF 读取用户填写的百分比 (0 ~ 100)
        if (_sdf->HasElement("light_percent")) {
            this->light_percent_ = _sdf->Get<double>("light_percent");
        }
        if (_sdf->HasElement("light_name")) {
            this->light_name_ = _sdf->Get<std::string>("light_name");
        }

        // 初始化 Gazebo 内部传输节点
        this->node_ = transport::NodePtr(new transport::Node());
        this->node_->Init(this->world_->Name());

        this->light_pub_ = this->node_->Advertise<msgs::Light>("~/light/modify");
        this->scene_pub_ = this->node_->Advertise<msgs::Scene>("~/scene");

        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&WorldLightPlugin::OnUpdate, this, _1));

        gzmsg << "=================================================================" << std::endl;
        gzmsg << "💡 [WorldLightPlugin] 成功加载世界光照插件！" << std::endl;
        gzmsg << "   目标光源: [" << this->light_name_ << "], 用户设定光照强度: 【 " 
              << this->light_percent_ << "% 】" << std::endl;
        gzmsg << "=================================================================" << std::endl;
    }

    void OnUpdate(const common::UpdateInfo & /*_info*/) {
        // 在仿真启动初期连续更新前 30 帧 (确保渲染客户端 GUI 连接后 100% 收到光照调整)
        if (this->apply_countdown_ <= 0) {
            return;
        }
        this->apply_countdown_--;

        double scale = std::max(0.0, this->light_percent_) / 100.0;

        // 1. 更新环境光 (Scene Ambient & Background)
        msgs::Scene scene_msg;
        scene_msg.set_name(this->world_->Name());
        double amb = 0.25 * scale;
        scene_msg.mutable_ambient()->set_r(amb);
        scene_msg.mutable_ambient()->set_g(amb);
        scene_msg.mutable_ambient()->set_b(amb);
        scene_msg.mutable_ambient()->set_a(1.0);
        
        // 柔和天际背景颜色缩放
        double bg_r = std::max(0.02, 0.70 * scale * 0.8 + 0.02);
        double bg_g = std::max(0.02, 0.80 * scale * 0.8 + 0.03);
        double bg_b = std::max(0.03, 0.90 * scale * 0.8 + 0.04);
        scene_msg.mutable_background()->set_r(bg_r);
        scene_msg.mutable_background()->set_g(bg_g);
        scene_msg.mutable_background()->set_b(bg_b);
        scene_msg.mutable_background()->set_a(1.0);
        this->scene_pub_->Publish(scene_msg);

        // 2. 更新太阳主光源 (Sun Light Diffuse & Specular)
        if (!this->light_) {
            this->light_ = this->world_->LightByName(this->light_name_);
        }

        msgs::Light light_msg;
        if (this->light_) {
            this->light_->FillMsg(light_msg);
        } else {
            light_msg.set_name(this->light_name_);
            msgs::Set(light_msg.mutable_pose(), ignition::math::Pose3d(0, 0, 50, 0, 0, 0));
        }

        light_msg.set_name(this->light_name_);
        light_msg.set_type(msgs::Light_LightType_DIRECTIONAL);
        light_msg.set_cast_shadows(true);
        msgs::Set(light_msg.mutable_direction(), ignition::math::Vector3d(-0.6, 0.6, -0.53));

        // 100% 基准标准日光漫反射与高光
        double d_r = 1.00 * scale;
        double d_g = 0.98 * scale;
        double d_b = 0.95 * scale;
        double spec = 0.20 * scale;

        light_msg.mutable_diffuse()->set_r(d_r);
        light_msg.mutable_diffuse()->set_g(d_g);
        light_msg.mutable_diffuse()->set_b(d_b);
        light_msg.mutable_diffuse()->set_a(1.0);

        light_msg.mutable_specular()->set_r(spec);
        light_msg.mutable_specular()->set_g(spec);
        light_msg.mutable_specular()->set_b(spec);
        light_msg.mutable_specular()->set_a(1.0);

        if (this->light_) {
            this->light_->ProcessMsg(light_msg);
        }
        this->light_pub_->Publish(light_msg);
    }

private:
    physics::WorldPtr world_;
    physics::LightPtr light_;
    transport::NodePtr node_;
    transport::PublisherPtr light_pub_;
    transport::PublisherPtr scene_pub_;
    event::ConnectionPtr update_connection_;

    double light_percent_;
    std::string light_name_;
    int apply_countdown_;
};

GZ_REGISTER_WORLD_PLUGIN(WorldLightPlugin)

} // namespace gazebo
