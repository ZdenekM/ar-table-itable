from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
import rospy
from art_msgs.msg import InstancesArray
from geometry_msgs.msg import PoseStamped
from collections import deque
import tf
# from art_projected_gui.items import PlaceItem
from art_msgs.msg import InstancesArray
from math import sqrt

translate = QtCore.QCoreApplication.translate

ARM_MOVE_TH = 0.005


class PickFromFeeder(GuiInstruction):

    NAME = translate("PickFromFeeder", "Pick from feeder")

    def __init__(self, *args, **kwargs):

        super(PickFromFeeder, self).__init__(*args, **kwargs)

    @staticmethod
    def get_text(ph, block_id, item_id):

        text = "\n"

        if ph.is_pose_set(block_id, item_id):
            text += translate("PickFromFeeder", "     Pose stored.")
        else:
            text += translate("PickFromFeeder", "     Pose has to be set.")

        return text


class PickFromFeederLearn(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederLearn, self).__init__(*args, **kwargs)

        self.timer = None

        if self.editable:

            self.timer = QtCore.QTimer()
            self.timer.timeout.connect(self.timer_tick)
            self.timer.start(500)

            self.arm_position = {}
            self.last_move = {}  # timestamp of the last move for each arm

            self.ui.notif(translate("PickFromFeeder", "Point robot's gripper on desired object in feeder."))

    def learning_done(self):

        self.timer.stop()

    def timer_tick(self):

        now = rospy.Time.now()

        for idx, arm in enumerate(self.ui.rh.get_robot_arms()):

            if arm not in self.arm_position:
                self.arm_position[arm] = deque(maxlen=3)

            ap = arm.get_pose()

            if not ap:
                continue

            self.arm_position[arm].append(ap)

            if len(self.arm_position[arm]) < 3:
                continue

            for m in ('x', 'y', 'z'):

                d1 = abs(getattr(self.arm_position[arm][-2].pose.position,
                                 m) - getattr(self.arm_position[arm][-1].pose.position, m))

                d2 = abs(getattr(self.arm_position[arm][-3].pose.position,
                                 m) - getattr(self.arm_position[arm][-2].pose.position, m))

                if d1 > ARM_MOVE_TH or d2 > ARM_MOVE_TH:
                    self.last_move[arm] = now
                    break

        last_arm, last_arm_ts = None, None

        # find which arm moved last
        for idx, arm in enumerate(self.ui.rh.get_robot_arms()):

            if arm not in self.last_move:
                continue

            if not last_arm or last_arm_ts < self.last_move[arm]:
                last_arm, last_arm_ts = arm, self.last_move[arm]

        if not last_arm:
            return

        if now - last_arm_ts < rospy.Duration(1.0):
            return

        # if there is already saved pose and if the current arm pose is same as the saved one, do nothing
        if self.ui.ph.is_pose_set(*self.cid):

            cp = self.arm_position[last_arm][-1].pose.position
            sp = self.ui.ph.get_pose(*self.cid)[0][0].pose.position

            for m in ('x', 'y', 'z'):

                d = abs(getattr(cp, m) - getattr(sp, m))

                if d > 5 * ARM_MOVE_TH:
                    break
            else:
                return

        # find closest object
        # TODO hmm, objects should be taken from gui (without waiting)...
        objects = rospy.wait_for_message('/art/object_detector/object_filtered', InstancesArray, timeout=1)

        closest_obj, closest_obj_dist = None, None

        for obj in objects.instances:

            if obj.on_table:
                continue

            ps = PoseStamped()
            ps.header.frame_id = objects.header.frame_id
            ps.pose = obj.pose

            try:
                ps = self.ui.tfl.transformPose(last_arm.gripper_link, ps)
            except tf.Exception:
                rospy.logerr("Failed to transform detection to gripper frame.")
                continue

            p = ps.pose.position

            d = sqrt(p.x**2 + p.y**2 + p.z**2)

            if d > 0.2:
                continue

            if not closest_obj_dist or closest_obj_dist > d:
                closest_obj, closest_obj_dist = obj, d

        if not closest_obj:
            return

        rospy.loginfo("Distance to object: " + str(closest_obj_dist))

        # self.ui.program_vis.clear_poses()
        self.ui.program_vis.set_object(closest_obj.object_type)
        self.ui.program_vis.set_pose(self.arm_position[last_arm][-1])

        self.ui.notif(translate("PickFromFeeder", "Gripper pose stored."), temp=True)
        self.ui.notify_info()


class PickFromFeederRun(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederRun, self).__init__(*args, **kwargs)

        ps = self.ui.ph.get_pose(*self.cid)[0][0]

        if ps.pose.position.x < 1.5 / 2.0:  # TODO this is not nice solution!
            self.ui.notif(
                translate("PickFromFeeder", "Picking object from feeder on my right."))
        else:
            self.ui.notif(
                translate("PickFromFeeder", "Picking object from feeder on my left."))


class PickFromFeederVis(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederVis, self).__init__(*args, **kwargs)

        self.ui.select_object_type(self.ui.ph.get_object(*self.cid)[0][0])
