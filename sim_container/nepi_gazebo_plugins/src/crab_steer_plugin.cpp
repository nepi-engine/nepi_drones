// Per-wheel steering+spin animator AND body driver for a 4-wheel-
// independent-steering ("crab steering" / "wheel independence") rover.
//
// Drives the rover's actual body physics itself now (SetLinearVel/
// SetAngularVel on the model, in the world frame, from the same cmd_vel
// Twist -- body-frame x/y rotated by the model's current yaw, angular.z
// applied directly) -- see "Why this plugin drives the body itself, not
// libgazebo_ros_planar_move" below for why that stock plugin was dropped.
// Also makes the wheels themselves visually steer to point the direction
// of travel and spin at a rate consistent with the commanded speed, so the
// rover LOOKS like it's crab-steering instead of skidding sideways with
// static wheels.
//
// base_link is kinematic, not dynamic (2026-09-23) -- see Load's own
// SetKinematic(true) comment. The chassis is immune to gravity/contact forces and passes
// straight through static obstacles as a result (an accepted, explicitly
// requested tradeoff, "for now"); the wheels are ordinary dynamic links and
// still physically react to the ground and anything the rover's path runs
// into.
//
// Steering joints are driven with Joint::SetPosition (a kinematic
// teleport-to-angle) and spin joints with Joint::SetParam("vel"/"fmax", ...)
// (ODE's built-in velocity motor, the same mechanism
// libgazebo_ros_diff_drive itself already uses elsewhere in this codebase)
// -- deliberately, since the goal is a correct-looking animation of wheel
// corners that steer independently of the chassis, not a physically-accurate
// steering suspension model.
//
// Why this plugin drives the body itself, not libgazebo_ros_planar_move
// (2026-09-23): confirmed live, under gdb, that respawning (delete+spawn --
// the SAME mechanism camera offset/FOV changes already use, see
// sim_bridge_node.py's own extensive comments on why that's the only viable
// way to reposition a camera here) a model that has BOTH planar_move and
// this plugin loaded crashes gzserver outright -- SIGABRT, "boost: mutex
// lock failed in pthread_mutex_lock: Invalid argument" -- and the crash's
// own backtrace is squarely inside libgazebo_ros_planar_move.so's OWN
// GazeboRosPlanarMove::QueueThread/ros::CallbackQueue::callAvailable, not
// anywhere in this file. Confirmed this plugin's own thread/NodeHandle
// teardown is NOT implicated: it was present and undisturbed in both
// crash captures, only planar_move's own thread aborted. Since
// libgazebo_ros_planar_move.so is a stock, unmodified system library (not
// something this repo builds or can patch), the only fix actually
// available here is to stop depending on it -- this plugin already has a
// proven-safe ROS NodeHandle/thread lifecycle (verified across every
// respawn in this same investigation), so it now also does planar_move's
// one job itself: apply body-frame cmd_vel as a world-frame model velocity,
// every tick, right alongside the wheel animation it already did.
// generate_model_sdf.py's buildRoverSdf no longer emits a
// planar_move_controller plugin block for a wheel-independence-enabled
// rover -- see that function's own comment.
//
// robotNamespace matters here: without it, this plugin's ros::NodeHandle()
// resolves "cmd_vel" against the GLOBAL namespace ("/cmd_vel"), not
// "/rover/cmd_vel" -- confirmed live via debug logging before this was
// found: this plugin's own OnUpdate read vx=vy=0 on every call, no matter
// what was actually published. See generate_model_sdf.py's own
// crab_steer_controller plugin block for the matching <robotNamespace>.

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <string>
#include <vector>

namespace gazebo
{

// See OnUpdate's own comment on the active yaw hold this gates.
static const double YAW_HOLD_EPS = 0.01;
static const double YAW_HOLD_KP = 2.0;

class CrabSteerPlugin : public ModelPlugin
{
public:
  CrabSteerPlugin() : wheel_radius_(0.1), vx_(0.0), vy_(0.0) {}

  ~CrabSteerPlugin() override
  {
    update_connection_.reset();
    if (rosnode_)
    {
      queue_.clear();
      queue_.disable();
      rosnode_->shutdown();
      callback_queue_thread_.join();
    }
  }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override
  {
    model_ = parent;
    // See OnUpdate's own comment on why body velocity is set on THIS link
    // specifically, not model_->SetLinearVel/SetAngularVel.
    base_link_ = model_->GetLink("base_link");
    if (!base_link_)
    {
      gzerr << "[nepi_crab_steer_plugin] could not find link 'base_link' on model '"
            << model_->GetName() << "' -- body drive will not work\n";
    }
    else
    {
      // Kinematic, not dynamic, for the chassis specifically (requested
      // live, 2026-09-23, after the chassis was still visibly "fighting"
      // OnUpdate's own commanded velocity: "it still needs to react to
      // objects while being independent... as long as the wheels mainly
      // react to the objects, its fine for now"). A kinematic link ignores
      // gravity and any contact/friction force acting ON it -- it moves only
      // by the velocity OnUpdate sets on it every tick -- which
      // removes the last of the residual wheel-ground-reaction coupling
      // the yaw-hold correction further down was only ever a partial
      // band-aid for. Accepted tradeoff, exactly as requested: the chassis
      // itself now passes straight through static obstacles instead of
      // colliding with them. The wheels are unaffected by this -- they
      // stay ordinary dynamic links (see WheelJoints below) jointed to
      // this one, so they still physically contact the ground and any
      // object in the rover's path.
      base_link_->SetKinematic(true);
    }

    std::string command_topic = "cmd_vel";
    if (sdf->HasElement("commandTopic"))
    {
      command_topic = sdf->Get<std::string>("commandTopic");
    }
    std::string robot_namespace = "";
    if (sdf->HasElement("robotNamespace"))
    {
      robot_namespace = sdf->Get<std::string>("robotNamespace");
    }
    if (sdf->HasElement("wheelRadius"))
    {
      wheel_radius_ = sdf->Get<double>("wheelRadius");
    }
    if (sdf->HasElement("spinMaxForce"))
    {
      spin_max_force_ = sdf->Get<double>("spinMaxForce");
    }
    // Same field names/defaults as libgazebo_ros_planar_move's own SDF
    // params -- this plugin now publishes the odometry that stock plugin
    // used to (see this file's own header comment), so buildRoverSdf's
    // <plugin> block for this can keep passing the exact same tags.
    if (sdf->HasElement("odometryTopic"))
    {
      odometry_topic_ = sdf->Get<std::string>("odometryTopic");
    }
    if (sdf->HasElement("odometryFrame"))
    {
      odometry_frame_ = sdf->Get<std::string>("odometryFrame");
    }
    if (sdf->HasElement("robotBaseFrame"))
    {
      robot_base_frame_ = sdf->Get<std::string>("robotBaseFrame");
    }
    if (sdf->HasElement("odometryRate"))
    {
      const double rate = sdf->Get<double>("odometryRate");
      odometry_period_ = (rate > 0.0) ? (1.0 / rate) : 0.0;
    }

    // Repeated <wheel steerJoint="..." spinJoint="..."/> elements, one per
    // wheel corner -- looked up by name against joints this SAME model
    // already declares (the steer-hub revolute + the pre-existing wheel
    // spin revolute, see buildRoverSdf), not created here.
    if (sdf->HasElement("wheel"))
    {
      sdf::ElementPtr wheelElem = sdf->GetElement("wheel");
      while (wheelElem)
      {
        std::string steerName = wheelElem->Get<std::string>("steerJoint");
        std::string spinName = wheelElem->Get<std::string>("spinJoint");
        physics::JointPtr steer = model_->GetJoint(steerName);
        physics::JointPtr spin = model_->GetJoint(spinName);
        if (steer && spin)
        {
          WheelJoints wj;
          wj.steer = steer;
          wj.spin = spin;
          // Body-frame offset from base_link's own origin -- see
          // generate_model_sdf.py's own x/y attribute comment. Defaults to
          // (0,0) if missing (an older-generated SDF without them), which
          // just means this wheel gets no tangential correction during
          // rotation -- the same behavior this plugin always had before.
          wj.x = wheelElem->HasElement("x") ? wheelElem->Get<double>("x") : 0.0;
          wj.y = wheelElem->HasElement("y") ? wheelElem->Get<double>("y") : 0.0;
          wheels_.push_back(wj);
        }
        else
        {
          gzerr << "[nepi_crab_steer_plugin] could not find joint(s) '"
                << steerName << "'/'" << spinName << "' on model '"
                << model_->GetName() << "'\n";
        }
        wheelElem = wheelElem->GetNextElement("wheel");
      }
    }

    if (wheels_.empty())
    {
      gzerr << "[nepi_crab_steer_plugin] no valid <wheel> entries found -- "
            << "plugin will do nothing\n";
      return;
    }

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace));

    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        command_topic, 1,
        boost::bind(&CrabSteerPlugin::cmdVelCallback, this, _1),
        ros::VoidConstPtr(), &queue_);
    vel_sub_ = rosnode_->subscribe(so);
    odom_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    callback_queue_thread_ = boost::thread(boost::bind(&CrabSteerPlugin::QueueThread, this));

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CrabSteerPlugin::OnUpdate, this));
  }

private:
  struct WheelJoints
  {
    physics::JointPtr steer;
    physics::JointPtr spin;
    // Body-frame offset from base_link's own origin -- see Load's own
    // comment on the SDF <wheel x="" y=""> attributes this comes from.
    double x = 0.0;
    double y = 0.0;
    // Per-wheel now, not a single value shared by all four -- see
    // OnUpdate's own comment for why a rotating rover needs each wheel to
    // hold its OWN tangential angle, not one angle applied identically
    // everywhere (only correct for pure translation).
    double last_steer_angle = 0.0;
  };

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vyaw_ = msg->angular.z;
  }

  void OnUpdate()
  {
    double vx, vy, vyaw;
    {
      boost::mutex::scoped_lock lock(mutex_);
      vx = vx_;
      vy = vy_;
      vyaw = vyaw_;
    }

    // Body drive -- see this file's own header comment ("Why this plugin
    // drives the body itself, not libgazebo_ros_planar_move") for why this
    // moved here instead of staying in that stock plugin. Same kinematics
    // planar_move itself used: body-frame (vx,vy) rotated into the world
    // frame by the model's CURRENT yaw, angular.z applied directly (yaw
    // rate is frame-independent for a planar mover). SetLinearVel/
    // SetAngularVel are world-frame velocity commands, re-applied every
    // tick -- the same "kinematic, not force-based" style already used
    // below for the steering joints.
    //
    // On base_link_ specifically, NOT model_ (confirmed live, 2026-09-23,
    // the actual root cause of a serious bug: a plain rotate-in-place
    // command left the rover severely under-rotated AND drifting sideways
    // it should never have moved at all, and combined with a real,
    // sustained turn during the manual-motor stress test, threw the whole
    // rover into the air tumbling). Model::SetLinearVel/SetAngularVel set
    // the IDENTICAL velocity vector on every link in the model, including
    // the wheel links -- which is only correct for angular velocity ZERO;
    // for a truly rotating rigid body each link's linear velocity should
    // differ by (angular_velocity x offset_from_reference), which this
    // Gazebo API does not compute. Every wheel link sits offset from
    // base_link's own origin (see buildRoverSdf's wheel poses), so any real
    // angular.z produced a velocity field inconsistent with what the
    // steer/spin JOINTS connecting those same wheels back to base_link
    // physically require -- exactly the kind of constraint conflict that
    // can make ODE's solver diverge. Setting velocity on base_link_ ALONE
    // and letting the steer/spin joints' own physics propagate a
    // consistent velocity out to each wheel (the normal, correct way an
    // articulated rigid body works) removes that conflict entirely.
    const ignition::math::Pose3d pose = base_link_->WorldPose();
    const double yaw = pose.Rot().Yaw();
    const double world_vx = vx * std::cos(yaw) - vy * std::sin(yaw);
    const double world_vy = vx * std::sin(yaw) + vy * std::cos(yaw);
    base_link_->SetLinearVel(ignition::math::Vector3d(world_vx, world_vy, 0.0));

    // Active yaw hold, not just a bare "commanded angular velocity is 0"
    // (requested live, 2026-09-23: "position changes keep the base locked
    // to whatever angle they're at while moving the wheels"). Confirmed
    // live that SetAngularVel(0) ALONE still let the base drift ~10 degrees
    // over a few seconds of pure translation -- residual coupling from the
    // wheels' own ground contact/steering forces between ticks, which a
    // one-shot zero velocity command doesn't cancel. Below YAW_HOLD_EPS
    // (no real rotation commanded), servo back toward whatever yaw this
    // was at the moment it stopped being actively rotated, instead of
    // just accepting the drift.
    double commanded_vyaw = vyaw;
    if (std::fabs(vyaw) < YAW_HOLD_EPS)
    {
      if (!yaw_held_valid_)
      {
        held_yaw_ = yaw;
        yaw_held_valid_ = true;
      }
      const double yaw_err = std::atan2(std::sin(held_yaw_ - yaw), std::cos(held_yaw_ - yaw));
      commanded_vyaw = YAW_HOLD_KP * yaw_err;
      // Clamped -- confirmed live (2026-09-23) that an unclamped P
      // correction here is genuinely dangerous, not just a tuning nit: a
      // sustained manual drive command (motor ratios left nonzero, no
      // active goto to bound it) let real yaw error grow large enough that
      // YAW_HOLD_KP * yaw_err commanded several rad/s of angular velocity
      // while the steering joints were still kinematically pinned to
      // last_steer_angle_ via SetPosition -- that mismatch between a
      // rapidly-spinning chassis and wheels frozen at the wrong angle threw
      // the rover into the air and tumbling across the world (position/
      // orientation both diverged wildly, confirmed via /rover/odom). This
      // cap keeps the correction fast enough for the few-degree drift it
      // exists to fix while never commanding a rotation rate large enough
      // to produce that mismatch.
      const double MAX_YAW_HOLD_RATE = 1.0;
      commanded_vyaw = std::max(-MAX_YAW_HOLD_RATE, std::min(MAX_YAW_HOLD_RATE, commanded_vyaw));
    }
    else
    {
      // Actively rotating (a real gotoPose/angular.z command) -- drop the
      // held anchor so the next time commands go quiet, a fresh one gets
      // captured at wherever the rover actually stopped turning, not
      // wherever it was idle before this rotation (same reasoning as
      // sim_bridge_node.py's own cmdCb/held_pose for position).
      yaw_held_valid_ = false;
    }
    base_link_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, commanded_vyaw));

    // No explicit SetWorldPose integration here: ODE advances a kinematic
    // link by the velocity set above on its own. Doing both (tried
    // 2026-09-23) applied every motion twice -- measured 90 deg/s against a
    // 45 deg/s max_angular_rate_dps cap, and doubled translation speed too.
    PublishOdometry(pose);

    // Per-wheel tangential velocity, not one shared (speed, angle) applied
    // identically to all four wheels -- confirmed live (2026-09-23) that
    // sharing one angle/speed is only correct for pure translation
    // (vyaw == 0). The instant a real rotation is commanded, each wheel's
    // own required velocity is v_center + omega x r (standard rigid-body
    // point velocity, r = this wheel's own body-frame offset from
    // base_link's origin, see generate_model_sdf.py's own x/y attributes):
    // omega x r = (-vyaw*wy, vyaw*wx). Before this, every wheel was steered
    // straight ahead (or held at whatever the last translation pointed)
    // and spun at a rate derived only from vx/vy, completely ignoring
    // vyaw -- driving the CHASSIS to rotate via base_link_->SetAngularVel
    // while the wheels' own kinematically-pinned steer/spin DOFs held them
    // as if nothing was rotating at all. That mismatch is exactly what let
    // ground contact all but cancel a commanded rotation (confirmed live:
    // a 1 rad/s angular.z command for 3s produced almost no yaw change at
    // all) and, combined with a real sustained turn, threw the rover into
    // the air tumbling during the manual-motor stress test.
    for (auto &w : wheels_)
    {
      const double wheel_vx = vx - vyaw * w.y;
      const double wheel_vy = vy + vyaw * w.x;
      const double speed = std::hypot(wheel_vx, wheel_vy);
      // Hold the last commanded steering angle at rest instead of
      // re-deriving atan2(0, 0) == 0 every tick -- a wheel pointed sideways
      // shouldn't visibly snap back to straight-ahead just because the
      // rover paused.
      if (speed > 0.01)
      {
        w.last_steer_angle = std::atan2(wheel_vy, wheel_vx);
      }
      // Negated -- confirmed live: this joint's own measured Position()
      // comes out inverted relative to a plain atan2(vy, vx) (a commanded
      // +90 degrees, i.e. pure +Y travel, measured back as ~-90 degrees),
      // most likely ODE's own child-relative-to-parent convention for this
      // axis/parent-child pairing, not a bug in the atan2 math itself.
      // Calibrated empirically against the real simulator, not assumed
      // from SDF axis semantics on paper.
      w.steer->SetPosition(0, -w.last_steer_angle);
      w.spin->SetParam("vel", 0, speed / wheel_radius_);
      w.spin->SetParam("fmax", 0, spin_max_force_);
    }
  }

  void PublishOdometry(const ignition::math::Pose3d &pose)
  {
    // odometrySource=world, matching the planar_move config this replaces
    // (see this file's own header comment) -- ground-truth pose/twist read
    // straight back from Gazebo's own model state, no dead-reckoning
    // integration to drift or need resetting on a respawn.
    const gazebo::common::Time now = model_->GetWorld()->SimTime();
    if (odometry_period_ > 0.0 &&
        (now - last_odom_publish_time_).Double() < odometry_period_)
    {
      return;
    }
    last_odom_publish_time_ = now;

    const ignition::math::Vector3d linear_world = base_link_->WorldLinearVel();
    const ignition::math::Vector3d angular_world = base_link_->WorldAngularVel();
    // Twist is body-frame (REP 105) -- rotate world-frame velocity back by
    // -yaw, the inverse of the rotation OnUpdate applies going the other
    // way from a commanded body-frame Twist.
    const double yaw = pose.Rot().Yaw();
    const double body_vx = linear_world.X() * std::cos(yaw) + linear_world.Y() * std::sin(yaw);
    const double body_vy = -linear_world.X() * std::sin(yaw) + linear_world.Y() * std::cos(yaw);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(now.sec, now.nsec);
    odom.header.frame_id = odometry_frame_;
    odom.child_frame_id = robot_base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = body_vx;
    odom.twist.twist.linear.y = body_vy;
    odom.twist.twist.linear.z = linear_world.Z();
    odom.twist.twist.angular.z = angular_world.Z();
    odom_pub_.publish(odom);
  }

  void QueueThread()
  {
    static const double timeout = 0.01;
    while (rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  event::ConnectionPtr update_connection_;

  boost::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Subscriber vel_sub_;
  ros::Publisher odom_pub_;
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  boost::mutex mutex_;

  std::vector<WheelJoints> wheels_;
  double wheel_radius_;
  double spin_max_force_ = 5.0;
  double vx_;
  double vy_;
  double vyaw_ = 0.0;
  double held_yaw_ = 0.0;
  bool yaw_held_valid_ = false;

  std::string odometry_topic_ = "odom";
  std::string odometry_frame_ = "odom";
  std::string robot_base_frame_ = "base_link";
  double odometry_period_ = 1.0 / 30.0;
  gazebo::common::Time last_odom_publish_time_;
};

GZ_REGISTER_MODEL_PLUGIN(CrabSteerPlugin)

}  // namespace gazebo
