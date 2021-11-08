#include "autonomous_actor/AutonomousActorPlugin.hh"

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <ctgmath>
#include <fstream>
#include <string>

//#include "gazebo_person_detection/actor_vel.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(AutoActorPlugin)

#define WALKING_ANIMATION "walking"
#define SPEED_BASE 0.8
#define SPEED_STOP 0.0
//#define DEBUG_

std::ofstream myfile;

/////////////////////////////////////////////////
AutoActorPlugin::AutoActorPlugin()
{
}

/////////////////////////////////////////////////
void AutoActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  debugging_counter = 0; //TODO: Delete after debugging
  gzdbg << "Start load!" << std::endl;

  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin( std::bind(&AutoActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Read in multiple targets
  if (_sdf->HasElement("targets"))
  {
    gzdbg << "Targets beeing read!" << std::endl;

    // Obtain targets with element pointer
    sdf::ElementPtr local_targets = _sdf->GetElement("targets")->GetElement("target");

    // Extract target
    while (local_targets)
    {
      this->targets.push_back(local_targets->Get<ignition::math::Vector3d>());
      local_targets = local_targets->GetNextElement("target");
    }

    // Set index to zero
    this->idx = 0;
  }

   // Read in multiple complexe Objects, that contain childs with their own bounding boxes
  if (_sdf->HasElement("complex_models"))
  {
    gzdbg << "Complex Objects beeing read!" << std::endl;

    // Obtain targets with element pointer
    sdf::ElementPtr local_complex_models = _sdf->GetElement("complex_models")->GetElement("complex_model");

    // Extract target
    while (local_complex_models)
    {
      this->complex_models.push_back(local_complex_models->Get<std::string>());
      local_complex_models = local_complex_models->GetNextElement("complex_model");
    }
    //this->complex_models.push_back(local_complex_models->);
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
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Read in the other obstacles to ignore (Bacchin Alberto)
  if (_sdf->HasElement("actor_awareness"))
  {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("actor_awareness")->GetElement("model");
    while (modelElem)
    {
      this->otherActors.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Moving obstacles will be treated with larger security bounds (Bacchin Alberto)
  if (_sdf->HasElement("dynamic_obstacles"))
  {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("dynamic_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->dynObstacles.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Read in target tolerance
  if (_sdf->HasElement("target_tolerance"))
    this->tolerance = _sdf->Get<double>("target_tolerance");
  else
    this->tolerance = 0.3;

  //Initialize the obstacle map
  ignition::math::Vector3d init_pose;
  if (_sdf->HasElement("init_pose"))
    init_pose = _sdf->Get<ignition::math::Vector3d>("init_pose");
  else
    init_pose = {0, 0, 0};

  check_pose = init_pose;

  direction = (this->target - init_pose).Normalize();

  // Set index to zero
  this->idx = 0;
}

/////////////////////////////////////////////////
void AutoActorPlugin::Reset()
{
  gzdbg << "RESET CALLED!!!!!" << std::endl;

  this->velocity = SPEED_BASE; //[m/s]
  gzdbg << "this->velocity: " << this->velocity << std::endl;
  //Add rotational velocity for gracefully rotations
  this->rot_velocity = this->velocity.Length() * 4.375; //[rad/s]
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("targets"))
  {
    this->target = this->targets.at(this->idx);
  }
  else
  {
    this->target = ignition::math::Vector3d(0, -5, room_z);
  }

  gzdbg << "TARGET AFTER RESET: " << target[0] << " " << target[1] << " " << target[2] << " " << std::endl;

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
  gzdbg << "ChooseNewTarget is called!" << std::endl;
  // Increase index number in sequence
  this->idx++;

  gzdbg << "this->idx: " << this->idx << std::endl;
  gzdbg << "this->targets.size() " << this->targets.size() << std::endl;
  //reset the Target index
  if((this->idx >= this->targets.size()))
  {
    this->idx = 0;
  }
  gzdbg << "this->idx: " << this->idx << std::endl;

  // Set next target
  this->target = this->targets.at(this->idx);
}

ignition::math::Vector3d AutoActorPlugin::Perpendicular(ignition::math::Vector3d &_pos, bool dir_pi_2)
{
  ignition::math::Vector3d rot;
  rot.Z(0);
  _pos.Normalize();
  //rotation of pi/2 counter clockwise
  if (dir_pi_2)
  {
    rot.X(-_pos.Y());
    rot.Y(_pos.X());
  }
  else //rotation of pi/2 clockwise
  {
    rot.X(_pos.Y());
    rot.Y(-_pos.X());
  }

  return rot;
}

/////////////////////////////////////////////////
void AutoActorPlugin::HandleObstacles(ignition::math::Vector3d &offset_pose_to_target)
{
  ignition::math::AxisAlignedBox Bbox;
  ignition::math::Vector3d other_actor_pose;
  ignition::math::Vector3d scaling_actor(0.5, 0.5, 1.2138);

  //List of observed obstacles within the perception_box
  std::vector<ignition::math::AxisAlignedBox> current_observed_obstacles;

  // Set velocity to SPEED_BASE. In case there is an obstacle in the way, the velocity is set to zero later in this func
  this->velocity = SPEED_BASE;
  this->rot_velocity = this->velocity.Length() * 4.375; //[rad/s]

  //Cycle over the models of the world
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);

    //Ignore models indicated in the .world file, like ground
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(), model->GetName()) == this->ignoreModels.end())
    {
      if (std::find(this->complex_models.begin(), this->complex_models.end(), model->GetName()) != this->complex_models.end())
      {
        for (int i=0; i<model->GetChildCount(); i++)
        {
          Bbox = model->GetLink(model->GetChild(i)->GetName())->BoundingBox();
          this->SteerAroundObstacle(model, Bbox, offset_pose_to_target, current_observed_obstacles);
        }
      }
      else
      {
        //Actors bounding box handling (the method BoundingBox() dose not work properly for actors)
        if (std::find(this->otherActors.begin(), this->otherActors.end(), model->GetName()) == this->otherActors.end())
        {
          Bbox = model->BoundingBox();
        }

        else
        {
          other_actor_pose = model->WorldPose().Pos();
          Bbox = ignition::math::AxisAlignedBox(other_actor_pose - scaling_actor, other_actor_pose + scaling_actor);
        }
        this->SteerAroundObstacle(model, Bbox, offset_pose_to_target, current_observed_obstacles);
      }
    }
  }
  // Save current_observed_obstacles for the next round so that in each intersection check all nearby obstacles are known
  observed_obstacles = current_observed_obstacles;
}

/////////////////////////////////////////////////
void AutoActorPlugin::SteerAroundObstacle(physics::ModelPtr model, ignition::math::AxisAlignedBox Bbox, ignition::math::Vector3d &offset_pose_to_target, std::vector<ignition::math::AxisAlignedBox> current_observed_obstacles)
{  
  std::tuple<bool, double> obs_intersection;
  ignition::math::Vector3d actor_pose = this->actor->WorldPose().Pos();
  
  //Dilation of the bounding box of the model to improve the awareness
  ignition::math::AxisAlignedBox security_box, perception_box;
  ignition::math::Vector3d security_range(0.2, 0.2, 0);
  ignition::math::Vector3d perception_range(1.0, 1.0, 0);

  //Parameters of the function that compute the correction to the trajectory
  double distance_dependend_evasion_factor = 0.1;

  //Compute a security distance of perception_range meters
  perception_box = ignition::math::AxisAlignedBox(Bbox.Min() - perception_range, Bbox.Max() + perception_range);
  //Project the actor pose on the 2D plane
  actor_pose.Z(std::max(Bbox.Min().Z(), 0.0));

  //A collision is handled when the obstacle is closer then 2 meters
  if (perception_box.Contains(actor_pose))
  {
    // Add current Bbox to list of observed obstacles, because it is within the perception_box
    current_observed_obstacles.push_back(Bbox);

    //projection in a 2D plane
    offset_pose_to_target.Z(Bbox.Min().Z());

    //Dynamic ostacles are treated with larger security bounds
    if (std::find(this->dynObstacles.begin(), this->dynObstacles.end(),
                  model->GetName()) == this->dynObstacles.end())
    {
      security_range.Set(0.2, 0.2, 0.0);
      distance_dependend_evasion_factor = 1.0;
    }
    else
    {
      security_range.Set(0.7, 0.7, 0.0);
      distance_dependend_evasion_factor = 2.0;
    }

    security_box = ignition::math::AxisAlignedBox(Bbox.Min() - security_range, Bbox.Max() + security_range);

    //Check if there are any intersection in the direction of motion
    obs_intersection = security_box.IntersectDist(actor_pose, direction, 0.01, 5.99);

    if (std::get<0>(obs_intersection))
    {
      this->CorrectPath(offset_pose_to_target, distance_dependend_evasion_factor, obs_intersection);
    }
  }
}

/////////////////////////////////////////////////
void AutoActorPlugin::CorrectPath(ignition::math::Vector3d &offset_pose_to_target, double distance_dependend_evasion_factor, std::tuple<bool, double> obs_intersection)
{
  ignition::math::Vector3d correction;
  //Correction have 2 components:

  //First: a force in the direction opposite to the actual motion
  correction = -offset_pose_to_target;
  correction.Normalize();

  //Second: a force in perpendicular direction to the motion, propotional to the cos(angle<_pos,correction>)
  correction = correction + Perpendicular(offset_pose_to_target, true);

  correction.Normalize();

  if (std::get<1>(obs_intersection) < 0.1 && std::get<1>(obs_intersection) > 0)
  {
    //this->ChooseNewTarget();
    this->velocity = SPEED_STOP;
    this->rot_velocity = 0;
  }
  else
  {
    //Correction strength depends on the distance from the obstacle
    double b = distance_dependend_evasion_factor;
    double a = 2.0; // factor to weight the correction
    offset_pose_to_target = offset_pose_to_target + (a * exp(-b * std::get<1>(obs_intersection))) * correction;
    offset_pose_to_target.Normalize();
  }
}

void AutoActorPlugin::StuckCheck(double distance) {
  //Try to unstuck the actor
  double check_dist = (check_pose - this->actor->WorldPose().Pos()).Length();

  if (check_dist > 0.1)
  {
    stuck_count = 0;
    check_pose = this->actor->WorldPose().Pos();
  }
  // In case the velocity is set to 0, the actor is supposed to stand still
  else if (!this->velocity.Equal(ignition::math::Vector3d (SPEED_STOP, SPEED_STOP, SPEED_STOP)))
  {
    stuck_count++;
  }
  if (stuck_count > 750)
  {
    gzdbg << "Calling ChooseNewTarget from StuckCheck now! Distance: " << distance << std::endl;
    this->ChooseNewTarget();
    stuck_count = 0;
  }
}

/////////////////////////////////////////////////
void AutoActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();
  // Make sure time delta is not too big, which is usually  the case in the first call of OnUpdate
  if (dt > 1) {
    gzdbg << "Skipping OnUpdate because of too large value for dt: " << dt << std::endl;
    this->lastUpdate = _info.simTime;
    return;
  }  

  //gazebo_person_detection::actor_vel msg;

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d offset_pose_to_target = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = offset_pose_to_target.Length();

  if (debugging_counter > 400) {
    gzdbg << "Still alive!"<< std::endl;
    debugging_counter = 0;
  }
  debugging_counter++;

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < this->tolerance)
  {
    gzdbg << "Calling ChooseNewTarget now!" << std::endl;
    this->ChooseNewTarget();
    offset_pose_to_target = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  offset_pose_to_target = offset_pose_to_target.Normalize(); //* this->targetWeight;

  // Adjust the direction vector by avoiding obstacles

  this->HandleObstacles(offset_pose_to_target);

  ignition::math::Vector3d pos = offset_pose_to_target;
  direction = pos; // TODO: Fix this (change pos to offset variable)

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  pose.Pos() = pose.Pos() + pos * this->velocity * dt;
  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian() * this->rot_velocity * dt);

  // Make sure the actor stays within bounds
  // pose.Pos().X(std::max(this->room_x[0], std::min(this->room_x[1], pose.Pos().X())));
  // pose.Pos().Y(std::max(this->room_y[0], std::min(this->room_y[1], pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking animation
  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->StuckCheck(distance);

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;

}
