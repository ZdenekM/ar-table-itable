// Copyright 2016 Robo@FIT

#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include "objects.h"
#include <std_msgs/String.h>

// adapted from
// https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/baxter_pick_place/src/block_pick_place.cpp

#ifndef ART_PR2_GRASPING_PR2GRASP_H
#define ART_PR2_GRASPING_PR2GRASP_H

namespace art_pr2_grasping
{

class artPr2Grasping
{
private:

  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  moveit_simple_grasps::GraspFilterPtr grasp_filter_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  ros::Publisher grasped_object_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;

  const std::string default_target_;
  const std::string gripper_state_topic_;

protected:

  ros::NodeHandle nh_;

  const std::string group_name_;

  boost::shared_ptr<Objects> objects_;
  boost::shared_ptr<TObjectInfo> grasped_object_;

  float getGripperValue() {

      pr2_controllers_msgs::JointControllerStateConstPtr msg =
              ros::topic::waitForMessage<pr2_controllers_msgs::JointControllerState>(gripper_state_topic_, ros::Duration(1));

      if (msg) return msg->process_value;
      else return 1000; // TODO NaN / exception?
  }


public:
  artPr2Grasping(boost::shared_ptr<tf::TransformListener> tfl, boost::shared_ptr<Objects> objects, std::string group_name, std::string default_target, std) : nh_("~"), group_name_(group_name), default_target_(default_target), gripper_state_topic_(gripper_state_topic)
  {
    tfl_ = tfl;
    objects_ = objects;

    move_group_.reset(new move_group_interface::MoveGroup(group_name_));
    move_group_->setPlanningTime(30.0);
    move_group_->allowLooking(false);  // true causes failure
    move_group_->allowReplanning(true);
    move_group_->setGoalTolerance(0.005);
    move_group_->setPlannerId("RRTConnectkConfigDefault");

    ROS_INFO_STREAM_NAMED(group_name_, "Planning frame: " << getPlanningFrame());

    // Load grasp generator
    if (!grasp_data_.loadRobotGraspData(nh_, group_name_))
      ros::shutdown();

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools(getPlanningFrame(), "markers"));
    // visual_tools_->setFloorToBaseHeight(0.0);
    visual_tools_->loadEEMarker(grasp_data_.ee_group_, group_name_);
    visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->setLifetime(10.0);

    visual_tools_->setMuted(false);

    grasped_object_.reset();

    simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_));

    grasped_object_pub_ = nh_.advertise<std_msgs::String>("/art/pr2/" + group_name_ + "/grasped_object", 1, true);

    grasped_object_pub_.publish(std_msgs::String(""));

  }

  bool transformPose(geometry_msgs::PoseStamped &ps)
  {
    try
    {
      if (tfl_->waitForTransform(getPlanningFrame(), ps.header.frame_id, ps.header.stamp, ros::Duration(5)))
      {
        tfl_->transformPose(getPlanningFrame(), ps, ps);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(group_name_, "Transform between" << getPlanningFrame() << "and " << ps.header.frame_id << " not "
                                                                                                      "available!");
        return false;
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_NAMED(group_name_, "%s", ex.what());
      return false;
    }

    return true;
  }

  bool getReady()
  {

    move_group_->clearPathConstraints();
    if (!move_group_->setNamedTarget(default_target_)) {

        ROS_ERROR_STREAM_NAMED(group_name_, "Unknown default target name: " << default_target_);
        return false;
    }

    if (!move_group_->move())
    {
      ROS_WARN_NAMED(group_name_, "Failed to get ready.");
      return false;
    }

    return true;
  }

  bool place(const geometry_msgs::Pose &ps, double z_axis_angle_increment = 0.0, bool keep_orientation = false)
  {
    if (!hasGraspedObject())
    {
      ROS_ERROR_NAMED(group_name_, "No object to place.");
      return false;
    }

    manipulation_in_progress_ = true;

    lookAt(ps.position);

    std::vector<moveit_msgs::PlaceLocation> place_locations;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = getPlanningFrame();
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = ps;

    shape_msgs::SolidPrimitive bb = grasped_object_->type.bbox;

    visual_tools_->publishBlock(ps,
                                moveit_visual_tools::ORANGE,
                                bb.dimensions[0],
                                bb.dimensions[1],
                                bb.dimensions[2]);

    if (z_axis_angle_increment < 0)
      z_axis_angle_increment *= -1.0;  // only positive increment allowed
    if (z_axis_angle_increment == 0)
      z_axis_angle_increment = 2 * M_PI;  // for 0 we only want one cycle (only given orientation)

    // Create 360 degrees of place location rotated around a center
    for (double angle = 0; angle < 2 * M_PI; angle += z_axis_angle_increment)
    {
      pose_stamped.pose = ps;

      // Orientation
      Eigen::Quaterniond quat(Eigen::AngleAxis<double>(static_cast<double>(angle), Eigen::Vector3d::UnitZ()));
      pose_stamped.pose.orientation.x = quat.x();
      pose_stamped.pose.orientation.y = quat.y();
      pose_stamped.pose.orientation.z = quat.z();
      pose_stamped.pose.orientation.w = quat.w();

      // Create new place location
      moveit_msgs::PlaceLocation place_loc;

      place_loc.place_pose = pose_stamped;

      // Approach
      moveit_msgs::GripperTranslation pre_place_approach;
      pre_place_approach.direction.header.stamp = ros::Time::now();
      pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_;  // The distance the origin
                                                                                         // of a robot link needs
                                                                                         // to travel
      pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_;  // half of the desired? Untested.
      pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = -1;  // Approach direction
                                                   // (negative z axis)  // TODO:
                                                   // document this assumption
      place_loc.pre_place_approach = pre_place_approach;

      // Retreat
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      // todo is box_z always height of the object?
      // assume that robot holds the object in the middle of its height
      double des_dist =
          std::max(grasp_data_.approach_retreat_desired_dist_,
                   0.05 + 0.5 * objects_[grasped_object_->object_id].bb.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);

      post_place_retreat.desired_distance = des_dist;  // The distance the origin of a robot link needs to travel
                                                       // -> depends on the object size
      post_place_retreat.min_distance = grasp_data_.approach_retreat_min_dist_;  // half of the desired? Untested.
      post_place_retreat.direction.header.frame_id = grasp_data_.base_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = 1;  // Retreat direction (pos z axis)
      place_loc.post_place_retreat = post_place_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

      place_locations.push_back(place_loc);
    }

    if (keep_orientation)
    {
      ROS_INFO_NAMED(group_name_, "Applying orientation constraint...");

      moveit_msgs::OrientationConstraint ocm;
      ocm.link_name = move_group_->getEndEffectorLink();
      ocm.header.frame_id = getPlanningFrame();
      ocm.orientation = move_group_->getCurrentPose().pose.orientation;
      ocm.absolute_x_axis_tolerance = 0.2;
      ocm.absolute_y_axis_tolerance = 0.2;
      ocm.absolute_z_axis_tolerance = M_PI;
      ocm.weight = 1.0;

      moveit_msgs::Constraints c;
      c.orientation_constraints.push_back(ocm);

      move_group_->setPathConstraints(c);
    }

    // Prevent collision with table
    // move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME)

    if (move_group_->place(grasped_object_->object_id, place_locations))
    {
      grasped_object_.reset();
      grasped_object_pub_.publish(std_msgs::String(""));
      move_group_->clearPathConstraints();
      return true;
    }

    ROS_WARN_NAMED(group_name_, "Failed to place");
    return false;
  }

  std::string getPlanningFrame()
  {
    return move_group_->getPlanningFrame();
  }

  bool hasGraspedObject()
  {
    return grasped_object_;
  }

  bool pick(const std::string &object_id)
  {

    if (hasGraspedObject())
    {
      ROS_ERROR_NAMED(group_name_, "Can't grasp another object.");
      return false;
    }

    TObjectInfo obj = objects_.getObject(object_id);

    if (obj == TObjectInfo()) {

        ROS_ERROR_NAMED(group_name_, "Unknown object_id: " + object_id);
        return false;
    }

    std::vector<moveit_msgs::Grasp> grasps;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = getPlanningFrame();
    p.header.stamp = ros::Time::now();
    p.pose = obj.pose.pose;

    // visualization only -> published by publishCollisionBB
    // visual_tools_->publishBlock(objects_[id].p, moveit_visual_tools::ORANGE,
    // objects_[id].bb.dimensions[0], objects_[id].bb.dimensions[1],
    // objects_[id].bb.dimensions[2]);

    if (!simple_grasps_->generateShapeGrasps(obj.type.bbox, true, true, p, grasp_data_, grasps))
    {
      ROS_ERROR_NAMED(group_name_, "No grasps found.");
      manipulation_in_progress_ = false;
      return false;
    }

    // todo fix this (No kinematic solver found)
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;  // save each grasps ik solution for visualization
    if (!grasp_filter_->filterGrasps(grasps, ik_solutions, true, grasp_data_.ee_parent_link_, group_name_))
    {
      ROS_ERROR_NAMED(group_name_, "Grasp filtering failed.");
      manipulation_in_progress_ = false;
      return false;
    }

    if (grasps.size() == 0)
    {
      ROS_ERROR_NAMED(group_name_, "No feasible grasps found.");
      manipulation_in_progress_ = false;
      return false;
    }

    // visualization only - takes time
    /*if (!groups_[group]->visual_tools_->publishAnimatedGrasps(grasps,
    groups_[group]->grasp_data_.ee_parent_link_, 0.02)) {

        ROS_WARN("Grasp animation failed");
    }*/

    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(object_id);

    // Add this list to all grasps
    for (std::size_t i = 0; i < grasps.size(); ++i)
    {
      grasps[i].allowed_touch_objects = allowed_touch_objects;
    }

    grasped_object_ = boost::make_shared(obj);
    grasped_object_->object_id = object_id;

    if (!move_group_->pick(object_id, grasps))
    {
        ROS_WARN_NAMED(group_name_, "Failed to pick");
        grasped_object_.reset();
        return false;
    }

    float gripper = getGripperValue();

    if (gripper < 0.005) {

      grasped_object_.reset();
      ROS_ERROR_NAMED(group_name_, "Gripper is closed - object missed or dropped :-(");
      return false;
    }

    ROS_INFO_NAMEd(group_name_, "Picked the object.");
    grasped_object_pub_.publish(std_msgs::String(object_id));
    return true;

  }
};

}  // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_PR2GRASP_H
