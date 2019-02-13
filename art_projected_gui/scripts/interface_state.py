#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import LearningRequestAction, LearningRequestGoal
from art_helpers import InterfaceStateManager, ProgramHelper
from art_utils import ArtApiHelper
import actionlib
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped

INST_ID = 3
DELAY = 3.0


class TestUI(object):

    def __init__(self):

        self.state_manager = InterfaceStateManager("testui", self.sm_callback)
        self.ph = ProgramHelper()
        self.timer = None
        self.api = ArtApiHelper()
        self.ph = None

        rospy.loginfo("Waiting for /art/brain/learning_request")
        self.learning_action_cl = actionlib.SimpleActionClient(
            '/art/brain/learning_request', LearningRequestAction)
        self.learning_action_cl.wait_for_server()

        print "Edit instruction " + str(INST_ID) + " in GUI."

    def sm_callback(self, old_state, new_state, flags):

        if not self.ph:

            self.ph = ProgramHelper()
            self.ph.load(self.api.load_program(new_state.program_id))

        if new_state.program_current_item.id == INST_ID and new_state.edit_enabled and not self.timer:

            print "Starting timer"
            self.timer = rospy.Timer(rospy.Duration(3.0), self.timer_cb, oneshot=True)

    def timer_cb(self, event):

        # set parameters to the current instruction
        self.state_manager.state.timestamp = rospy.Time.now()
        self.state_manager.state.program_current_item.object = ["profile_20_60"]
        poly = PolygonStamped()
        poly.header.frame_id = "marker"
        poly.polygon.points.append(Point32(x=0.245, y=0.245))
        poly.polygon.points.append(Point32(x=0.354, y=0.245))
        poly.polygon.points.append(Point32(x=0.354, y=0.354))
        poly.polygon.points.append(Point32(x=0.245, y=0.354))
        self.state_manager.state.program_current_item.polygon = [poly]
        self.state_manager.send()
        rospy.sleep(DELAY)

        self.learning_action_cl.send_goal_and_wait(LearningRequestGoal(request=LearningRequestGoal.DONE))

        while self.state_manager.state.edit_enabled:
            rospy.sleep(0.1)

        # switch to the next instruction
        self.state_manager.state.program_current_item = self.ph.get_item_msg(
            self.state_manager.state.block_id, INST_ID + 1)
        self.state_manager.state.timestamp = rospy.Time.now()
        self.state_manager.send()
        rospy.sleep(DELAY)

        self.learning_action_cl.send_goal_and_wait(LearningRequestGoal(request=LearningRequestGoal.GET_READY))

        rospy.sleep(DELAY)

        while not self.state_manager.state.edit_enabled:
            rospy.sleep(0.1)

        # ...and set its parameters
        ps = PoseStamped()
        ps.header.frame_id = "marker"
        ps.pose.position.x = 0.436
        ps.pose.position.y = 0.367
        ps.pose.position.z = 0.025
        ps.pose.orientation.x = 0.707
        ps.pose.orientation.w = 0.707
        self.state_manager.state.timestamp = rospy.Time.now()
        self.state_manager.state.program_current_item.pose = [ps]
        self.state_manager.send()
        rospy.sleep(DELAY)

        self.learning_action_cl.send_goal_and_wait(LearningRequestGoal(request=LearningRequestGoal.DONE))

        print "Done!"
        sys.exit()


def main():

    rospy.init_node('test_brain')
    TestUI()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
