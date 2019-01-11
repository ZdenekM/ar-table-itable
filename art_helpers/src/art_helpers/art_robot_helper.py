#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


class UnknownRobot(Exception):
    pass


class RobotParametersNotOnParameterServer(Exception):
    pass


class ArtRobotHelper(object):

    def __init__(self, robot_ns="/art/robot"):
        self.robot_ns = robot_ns
        self.arms = []
        self.robot_parameters = rospy.get_param(self.robot_ns, None)
        self.capabilities = [cap for cap in self.robot_parameters.get('capabilities', {})]
        if self.robot_parameters is None:
            raise RobotParametersNotOnParameterServer

        self.robot_type = rospy.get_param(self.robot_ns + "/robot_id", "")

        for arm_id, parameters in self.robot_parameters.get("arms", []).iteritems():
            self.arms.append(ArtRobotArmHelper(robot_ns=robot_ns, arm_id=arm_id, parameters=parameters))

        # this is mainly due to gui, to make it able to display arms in the right order
        self.arms.sort(key=lambda x: x.idx)

    def get_robot_type(self):
        return self.robot_type

    def get_robot_arms_ids(self):
        return [arm.arm_id for arm in self.arms]

    def get_robot_arms(self):
        return self.arms

    def get_robot_arm(self, arm_id):
        for arm in self.arms:
            if arm.arm_id == arm_id:
                return arm

    def look_at_enabled(self):
        return "look_at" in self.capabilities

    def get_emergency_stop_service_name(self):
        return self.robot_ns + "/stop"

    def get_robot_restore_service_name(self):
        return self.robot_ns + "/restore"

    def get_prepare_for_calibration_service_name(self):
        return self.robot_ns + "/prepare_for_calibration"

    def get_look_at_topic_name(self):
        return self.robot_ns + "/look_at" if self.look_at_enabled() else None

    def get_look_at_default_service_name(self):
        return self.robot_ns + "/look_at/default" if self.look_at_enabled() else None


class ArtRobotArmHelper(object):

    def __init__(self, robot_ns, arm_id, parameters):
        self.arm_id = arm_id
        self.robot_ns = robot_ns
        self.capabilities = []
        self.gripper_link = parameters.get("gripper_link", None)
        self.camera_link = parameters.get("camera_link", None)  # TODO make it mandatory for pick from feeder capability
        self.base_link = parameters.get("base_link", None)  # where the arm is mounted
        self.range = (parameters.get("min_range", 0), parameters.get("max_range", 0))
        self.idx = parameters.get("idx", None)
        self._name = parameters.get("name", {})

        # TODO wouldn't it be better to just do transform from gripper_link to world frame?
        self._gripper_pose_sub = rospy.Subscriber(
            '/art/robot/' +
    self.arm_id +
            '/gripper/pose',
            PoseStamped,
            self._gripper_pose_cb)
        self._gripper_pose = None

        if not self._name:
            rospy.logwarn("Arm " + self.arm_id + " has not readable name!")

        for cap in parameters.get("capabilities", {}):
            self.capabilities.append(cap)

        if not self.capabilities:
            rospy.logwarn("Arm " + self.arm_id + " has no capabilities.")

    def _gripper_pose_cb(self, msg):

        self._gripper_pose = msg

    def name(self, loc):

        try:
            return self._name[loc]
        except KeyError:
            return self.arm_id

    def get_pose(self):

        return self._gripper_pose

    def pick_place_enabled(self):
        return "pick_place" in self.capabilities

    def manipulation_enabled(self):
        return "manipulation" in self.capabilities

    def interactive_mode_enabled(self):
        return "interactive_mode" in self.capabilities

    def drill_enabled(self):
        return "drill" in self.capabilities

    def move_to_user_enabled(self):
        return "move_to_user" in self.capabilities

    def get_ready_enabled(self):
        return "get_ready" in self.capabilities

    def get_capabilities(self):
        return self.capabilities

    def get_pick_place_action_name(self):
        return self.robot_ns + "/" + self.arm_id + "/pp" if self.pick_place_enabled() else None

    def get_holding_object_topic_name(self):
        return self.robot_ns + "/" + self.arm_id + "/grasped_object" if self.pick_place_enabled() else None

    def get_manipulation_action_name(self):
        return self.robot_ns + "/" + self.arm_id + "/manipulation" if self.manipulation_enabled() else None

    def get_move_to_user_service_name(self):
        return self.robot_ns + "/" + self.arm_id + "/move_to_user" if self.move_to_user_enabled() else None

    def get_get_ready_service_name(self):
        return self.robot_ns + "/" + self.arm_id + "/get_ready" if self.get_ready_enabled() else None

    def get_interaction_on_service_name(self):
        return self.robot_ns + "/" + self.arm_id + "/interaction/on" if self.interactive_mode_enabled() else None

    def get_interaction_off_service_name(self):
        return self.robot_ns + "/" + self.arm_id + "/interaction/off" if self.interactive_mode_enabled() else None

    def get_arm_reinit_service_name(self):
        return self.robot_ns + "/" + self.arm_id + "/reinit"

    def get_arm_prepare_for_calibration_service_name(self):
        return self.robot_ns + "/" + self.arm_id + "/prepare_for_calibration"

    def get_interaction_state_topic_name(self):
        return self.robot_ns + "/" + self.arm_id + "/interaction/state" if self.interactive_mode_enabled() else None

    def get_gripper_link(self):
        return self.gripper_link

    def get_reinit_service_name(self):
        return self.robot_ns + "/" + self.arm_id + "/reinit"
