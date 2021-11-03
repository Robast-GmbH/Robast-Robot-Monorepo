#include "autonomous_actor/AutonomousActorPlugin.hh"

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(AutoActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
AutoActorPlugin::AutoActorPlugin()
{
}

/////////////////////////////////////////////////
void AutoActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&AutoActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Added by brucechanjianle
  // Read in multiple targets
  if (_sdf->HasElement("targets"))
  {
    // Obtain targets with element pointer
    sdf::ElementPtr local_targets = _sdf->GetElement("targets")->GetElement("target");

    // Extract target
    while(local_targets)
    {
      this->targets.push_back(local_targets->Get<ignition::math::Vector3d>());
      local_targets = local_targets->GetNextElement("target");
    }

    // Set index to zero
    this->idx = 0;
  }

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  // This needs to be done because std::find cannot find the last entry in list. (check HandleObstacles func)
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement( "model");
    }
  }

  // Added by brucechanjianle
  // Read in target tolerance
  if (_sdf->HasElement("target_tolerance"))
    this->tolerance = _sdf->Get<double>("target_tolerance");
  else
    this->tolerance = 1.5;

}

/////////////////////////////////////////////////
void AutoActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  this->target = this->targets.at(this->idx);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void AutoActorPlugin::ChooseNewTarget()
{ 
  // Increase index number in sequence
  this->idx++;

  //reset the Target index
  if((this->idx > this->targets.size()))
  {
    this->idx = 0;
  }

  // Set next target
  this->target = this->targets.at(this->idx);
}

/////////////////////////////////////////////////
ignition::math::Vector3d AutoActorPlugin::HandleObstacles(ignition::math::Vector3d &tf_target_to_pose)
{
  // gzdbg << "this->world->ModelCount():" << this->world->ModelCount() << std::endl;
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    // std::find returns an iterator to the first element satisfying the condition or last if no such element is found.
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset_model_actor = model->WorldPose().Pos() - this->actor->WorldPose().Pos();
      double dist_to_model = offset_model_actor.Length();
      if (dist_to_model < 4.0)
      {
        double inv_dist_to_model = this->obstacleWeight / dist_to_model;
        offset_model_actor.Normalize();
        offset_model_actor *= inv_dist_to_model;
        return tf_target_to_pose - offset_model_actor;
      }
    }
  }
  return tf_target_to_pose;
}

/////////////////////////////////////////////////
void AutoActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d offset_target_to_pose = this->target - pose.Pos();
  ignition::math::Vector3d pose_rpy = pose.Rot().Euler();

  double distance = offset_target_to_pose.Length();
  
  // Choose a new target position if the actor has reached its current target.
  if (distance < this->tolerance)
  {
    this->ChooseNewTarget();
    offset_target_to_pose = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  offset_target_to_pose = offset_target_to_pose.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  ignition::math::Vector3d offset_target_to_model = this->HandleObstacles(offset_target_to_pose);

  // Compute the yaw orientation
  // 1.5707 = 90°
  ignition::math::Angle yaw = atan2(offset_target_to_model.Y(), offset_target_to_model.X()) + 1.5707 - pose_rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, pose_rpy.Z()+yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += offset_target_to_model * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, pose_rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  // pose.Pos().X(std::max(-100.0, std::min(50.5, pose.Pos().X())));
  // pose.Pos().Y(std::max(-100.0, std::min(50.0, pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
