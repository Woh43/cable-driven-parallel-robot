#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ropes_interfaces/msg/cable_tensions.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ropes_simulation
{
namespace
{
using gazebo::physics::ModelPtr;
using geometry_msgs::msg::PointStamped;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;
using ropes_interfaces::msg::CableTensions;
using sensor_msgs::msg::JointState;

constexpr std::size_t kCableCount = 3;
constexpr double kLengthEpsilon = 1e-9;
constexpr double kPredictionHorizonS = 0.035;
constexpr double kDrumResponseTimeS = 0.020;
constexpr char kBaseFrame[] = "base_link";
constexpr char kFrameModel[] = "ropes_frame";
constexpr std::array<const char *, 3> kCartesianJoints = {
  "end_effector_x_joint", "end_effector_y_joint", "end_effector_z_joint"};

struct Cable
{
  std::string name;
  Vector3d anchor;
  double neutral_length = 0.0;
  double free_length_limit = 0.0;
  double payout_coefficient = 0.0;
  double preload_m = 0.0;
  double min_angle = 0.0;
  double max_angle = 0.0;
  std::string drum_model_name;
  Vector3d drum_axis;
  ModelPtr drum;
  Pose3d drum_origin;
  std::string span_visual_name;
  gazebo::msgs::Visual span_visual;
  double angle = 0.0;
  double tension = 0.0;
};

struct DrumCommand
{
  std::array<double, kCableCount> angles{};
  std::array<double, kCableCount> rates{};
  double stamp = 0.0;
  bool has_rates = false;
};
}  // namespace

class CablePlugin : public gazebo::ModelPlugin
{
public:
  void Load(ModelPtr model, sdf::ElementPtr sdf) override
  {
    auto row = sdf->GetElement("cable");
    for (auto & cable : cables_) {
      if (!row) {
        gzerr << "Cable dynamics requires exactly three cables\n";
        return;
      }
      cable.name = row->Get<std::string>("joint");
      cable.anchor = row->Get<Vector3d>("anchor");
      cable.neutral_length = row->Get<double>("neutral_length");
      cable.free_length_limit = row->Get<double>("free_length_limit");
      cable.payout_coefficient = row->Get<double>("payout_coefficient");
      cable.preload_m = row->Get<double>("preload_m");
      const double minimum_payout_angle =
        (cable.preload_m + kLengthEpsilon - cable.neutral_length) / cable.payout_coefficient;
      const double maximum_payout_angle =
        (cable.free_length_limit - cable.neutral_length) / cable.payout_coefficient;
      cable.min_angle = std::min(minimum_payout_angle, maximum_payout_angle);
      cable.max_angle = std::max(minimum_payout_angle, maximum_payout_angle);
      cable.drum_model_name = row->Get<std::string>("drum_model");
      cable.drum_axis = row->Get<Vector3d>("drum_axis").Normalized();
      cable.span_visual_name = row->Get<std::string>("span_visual");
      row = row->GetNextElement("cable");
    }
    if (row) {
      gzerr << "Cable dynamics requires exactly three cables\n";
      return;
    }

    world_ = model->GetWorld();
    payload_ = model->GetLink("end_effector");
    if (!payload_) {
      gzerr << "Cable dynamics requires link end_effector\n";
      return;
    }
    use_sim_time_ = sdf->Get<bool>("use_sim_time");
    visual_zero_ = sdf->Get<Vector3d>("description_home");
    stiffness_ = sdf->Get<double>("stiffness_n_m");
    damping_ = sdf->Get<double>("damping_ns_m");
    max_drum_speed_ = sdf->Get<double>("max_drum_speed_rad_s");
    publish_period_ = gazebo::common::Time(1.0 / sdf->Get<double>("state_rate_hz"));

    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_) {
      gzerr << "Cable dynamics could not create its ROS node\n";
      return;
    }
    node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time_));
    poses_ = node_->create_publisher<PointStamped>("end_effector_pose", 10);
    states_ = node_->create_publisher<JointState>("joint_states", 10);
    tensions_ = node_->create_publisher<CableTensions>("cable_tensions", 10);
    commands_ = node_->create_subscription<JointState>(
      "drum_command", rclcpp::QoS(5),
      std::bind(&CablePlugin::ReceiveCommand, this, std::placeholders::_1));

    transport_.reset(new gazebo::transport::Node());
    transport_->Init(world_->Name());
    visuals_ = transport_->Advertise<gazebo::msgs::Visual>("~/visual");
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&CablePlugin::Update, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Gazebo cable dynamics ready");
  }

  void Reset() override
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_ = DrumCommand{};
    }
    for (auto & cable : cables_) {
      cable.angle = 0.0;
      cable.tension = 0.0;
    }
    last_update_ = gazebo::common::Time::Zero;
    next_publish_ = gazebo::common::Time::Zero;
  }

private:
  void ReceiveCommand(const JointState & message)
  {
    std::string error;
    DrumCommand command;
    command.has_rates = !message.velocity.empty();
    const auto & stamp = message.header.stamp;
    if (stamp.sec >= 0 && stamp.nanosec < 1000000000u) {
      command.stamp = static_cast<double>(stamp.sec) + 1e-9 * stamp.nanosec;
    }
    if (message.name.size() != message.position.size() ||
      std::unordered_set<std::string>(message.name.begin(), message.name.end()).size() !=
      message.name.size())
    {
      error = "Joint names and positions must match without duplicates";
    } else if (command.has_rates && message.velocity.size() != message.name.size()) {
      error = "Joint velocities must be empty or match the joint names";
    } else {
      for (std::size_t index = 0; index < cables_.size(); ++index) {
        const auto & cable = cables_[index];
        auto it = std::find(message.name.begin(), message.name.end(), cable.name);
        if (it == message.name.end()) {
          error = "Missing joint " + cable.name;
          break;
        }
        const auto offset = it - message.name.begin();
        const double angle = message.position[offset];
        const double length = cable.neutral_length + cable.payout_coefficient * angle;
        if (!std::isfinite(length) || length <= cable.preload_m + kLengthEpsilon ||
          length > cable.free_length_limit + kLengthEpsilon)
        {
          error = "Free length beyond drum travel for joint " + cable.name;
          break;
        }
        command.angles[index] = angle;
        if (command.has_rates) {
          const double rate = message.velocity[offset];
          if (!std::isfinite(rate)) {
            error = "Nonfinite drum velocity for joint " + cable.name;
            break;
          }
          command.rates[index] = std::clamp(rate, -max_drum_speed_, max_drum_speed_);
        }
      }
    }
    if (!error.empty()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), wall_clock_, 2000, "%s", error.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = command;
  }

  void Update(const gazebo::common::UpdateInfo & info)
  {
    if (info.simTime < last_update_) {
      Reset();
    }
    const double dt = (info.simTime - last_update_).Double();
    last_update_ = info.simTime;
    DrumCommand command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = command_;
    }
    const double now = use_sim_time_ ? info.simTime.Double() : wall_clock_.now().seconds();
    const double age = command.stamp > 0.0 ? now - command.stamp : -1.0;
    const double forecast = std::clamp(age, 0.0, kPredictionHorizonS);
    const bool fresh = age >= 0.0 && age < kPredictionHorizonS;
    const double max_step = max_drum_speed_ * dt;
    const Vector3d position = payload_->WorldPose().Pos();
    const Vector3d velocity = payload_->WorldLinearVel();
    for (std::size_t index = 0; index < cables_.size(); ++index) {
      auto & cable = cables_[index];
      const double target = std::clamp(
        command.angles[index] + command.rates[index] * forecast, cable.min_angle, cable.max_angle);
      double correction = target - cable.angle;
      if (command.has_rates) {
        // Smooth sampled trajectory targets while keeping their analytic payout rate.
        correction *= std::min(dt / kDrumResponseTimeS, 1.0);
        correction += fresh ? command.rates[index] * dt : 0.0;
      }
      const double next_angle = std::clamp(
        cable.angle + std::clamp(correction, -max_step, max_step), cable.min_angle, cable.max_angle);
      const double step = next_angle - cable.angle;
      cable.angle += step;
      if (!cable.drum) {
        cable.drum = world_->ModelByName(cable.drum_model_name);
        if (cable.drum) {
          cable.drum_origin = cable.drum->WorldPose();
        }
      }
      if (cable.drum && step != 0.0) {
        const Quaterniond spin(cable.drum_axis, cable.angle);
        const Pose3d pose(cable.drum_origin.Pos(), cable.drum_origin.Rot() * spin);
        cable.drum->SetWorldPose(pose);
      }

      // Rope can pull only. Spread preload according to static support at the home point.
      const Vector3d toward_anchor = cable.anchor - position;
      const double distance = toward_anchor.Length();
      const double rest_length =
        cable.neutral_length + cable.payout_coefficient * cable.angle - cable.preload_m;
      const double extension = distance - rest_length;
      cable.tension = 0.0;
      if (extension > 0.0 && distance > kLengthEpsilon) {
        const Vector3d direction = toward_anchor / distance;
        const double payout_speed = dt > 0.0 ? cable.payout_coefficient * step / dt : 0.0;
        const double extension_speed = -direction.Dot(velocity) - payout_speed;
        cable.tension = std::max(0.0, stiffness_ * extension + damping_ * extension_speed);
        payload_->AddForce(cable.tension * direction);
      }
    }

    if (info.simTime >= next_publish_) {
      Publish(info.simTime, position);
      next_publish_ = info.simTime + publish_period_;
    }
  }

  void Publish(const gazebo::common::Time & sim_time, const Vector3d & position)
  {
    PointStamped point;
    point.header.frame_id = kBaseFrame;
    if (use_sim_time_) {
      point.header.stamp.sec = sim_time.sec;
      point.header.stamp.nanosec = sim_time.nsec;
    } else {
      point.header.stamp = wall_clock_.now();
    }
    point.point.x = position.X();
    point.point.y = position.Y();
    point.point.z = position.Z();
    poses_->publish(point);

    JointState state;
    state.header = point.header;
    CableTensions tensions;
    tensions.header = point.header;
    for (std::size_t index = 0; index < cables_.size(); ++index) {
      auto & cable = cables_[index];
      state.name.push_back(cable.name);
      state.position.push_back(cable.angle);
      tensions.joint_names[index] = cable.name;
      tensions.tension_newtons[index] = cable.tension;
      if (visuals_->HasConnections()) {
        PublishSpan(cable, position);
      }
    }
    const Vector3d offset = position - visual_zero_;
    for (std::size_t index = 0; index < kCartesianJoints.size(); ++index) {
      state.name.emplace_back(kCartesianJoints[index]);
      state.position.push_back(offset[index]);
    }
    states_->publish(state);
    tensions_->publish(tensions);
  }

  void PublishSpan(Cable & cable, const Vector3d & position)
  {
    if (!cable.span_visual.has_id()) {
      const auto frame = world_->ModelByName(kFrameModel);
      const auto link = frame ? frame->GetLink(kBaseFrame) : nullptr;
      if (!link) {
        return;
      }
      const auto & visuals = link->Visuals();
      const auto visual = std::find_if(visuals.begin(), visuals.end(), [&cable](const auto & row) {
        return row.second.name() == cable.span_visual_name;
      });
      if (visual == visuals.end()) {
        return;
      }
      cable.span_visual = visual->second;
      cable.span_visual.clear_scale();
    }
    const Vector3d span = position - cable.anchor;
    const Vector3d center = cable.anchor + span * 0.5;
    const Quaterniond rotation(
      0.0, std::atan2(std::hypot(span.X(), span.Y()), span.Z()), std::atan2(span.Y(), span.X()));
    gazebo::msgs::Set(cable.span_visual.mutable_pose(), Pose3d(center, rotation));
    cable.span_visual.mutable_geometry()->mutable_cylinder()->set_length(span.Length());
    visuals_->Publish(cable.span_visual);
  }

  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr payload_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Clock wall_clock_{RCL_SYSTEM_TIME};
  Vector3d visual_zero_;
  bool use_sim_time_ = true;
  double stiffness_ = 0.0;
  double damping_ = 0.0;
  double max_drum_speed_ = 0.0;
  gazebo::common::Time last_update_;
  gazebo::common::Time next_publish_;
  gazebo::common::Time publish_period_;
  std::array<Cable, kCableCount> cables_;
  std::mutex command_mutex_;
  DrumCommand command_;
  rclcpp::Publisher<PointStamped>::SharedPtr poses_;
  rclcpp::Publisher<JointState>::SharedPtr states_;
  rclcpp::Publisher<CableTensions>::SharedPtr tensions_;
  rclcpp::Subscription<JointState>::SharedPtr commands_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::PublisherPtr visuals_;
  gazebo::event::ConnectionPtr update_;
};

GZ_REGISTER_MODEL_PLUGIN(CablePlugin)
}  // namespace ropes_simulation
