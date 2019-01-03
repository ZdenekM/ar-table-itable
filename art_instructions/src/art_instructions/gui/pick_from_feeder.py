from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
import rospy
from art_msgs.msg import InstancesArray
from geometry_msgs.msg import PoseStamped
from collections import deque
import tf

translate = QtCore.QCoreApplication.translate


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

        self.grasp_dialog_timer = None
        self.obj_raw_sub = None

        if self.editable:

            QtCore.QObject.connect(self, QtCore.SIGNAL('objects_raw'), self.object_raw_cb_evt)

            self.grasp_dialog_timer = QtCore.QTimer()
            self.grasp_dialog_timer.timeout.connect(self.grasp_dialog_timer_tick)
            self.grasp_dialog_timer.start(500)

            # TODO delegate getting detections per frame to object helper
            self.obj_raw_sub = rospy.Subscriber(
                '/art/object_detector/object', InstancesArray, self.object_raw_cb, queue_size=1)

            self.objects_by_sensor = {}

            self.arm_camera_position = {}
            self.last_end_of_interaction = None  # TODO last_move = {}  -> for each arm

        # TODO notifications
        self.ui.notif(
            translate("PickFromFeeder", "TBD."))

    def learning_done(self):

        self.obj_raw_sub.unregister()
        self.grasp_dialog_timer.stop()

    def object_raw_cb_evt(self, msg):

        for obj in msg.instances:

            # this mainly serves for detection of objects in feeder so let's count only objects not on table
            o = self.ui.get_object(obj.object_id)

            if o and o.on_table:
                continue

            if msg.header.frame_id not in self.objects_by_sensor:
                self.objects_by_sensor[msg.header.frame_id] = {}

            ps = PoseStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.pose = obj.pose

            for idx, arm in enumerate(self.ui.rh.get_robot_arms()):

                if arm.camera_link != msg.header.frame_id:
                    continue

                try:
                    ps = self.ui.tfl.transformPose(arm.gripper_link, ps)
                except tf.Exception:
                    rospy.logerr("Failed to transform detection to gripper frame.")
                    continue

                # TODO rather use euclid dist?
                if ps.pose.position.x > 0.3 or ps.pose.position.y > 0.1 or ps.pose.position.z > 0.1:
                    continue

                self.objects_by_sensor[msg.header.frame_id][obj.object_id] = [msg.header.stamp, obj]

    def grasp_dialog_timer_tick(self):

        now = rospy.Time.now()

        # delete outdated objects
        for cam_link, obj_dict in self.objects_by_sensor.iteritems():

            to_delete = []

            for obj_id, obj_arr in obj_dict.iteritems():

                if now - obj_arr[0] > rospy.Duration(2.0):

                    to_delete.append(obj_id)

            for td in to_delete:
                del obj_dict[td]

        for idx, arm in enumerate(self.ui.rh.get_robot_arms()):

            current = PoseStamped()
            current.header.stamp = rospy.Time(0)
            current.header.frame_id = arm.camera_link
            current.pose.orientation.w = 1

            try:
                current = self.ui.tfl.transformPose("marker", current)
            except tf.Exception:
                rospy.logerr("Failed to transform from arm-camera to world")
                continue

            if arm.camera_link not in self.arm_camera_position:
                self.arm_camera_position[arm.camera_link] = deque(maxlen=3)

            self.arm_camera_position[arm.camera_link].append(current.pose.position)

            if len(self.arm_camera_position[arm.camera_link]) < 3 or arm.camera_link not in self.objects_by_sensor:
                continue

            for m in ('x', 'y', 'z'):

                d1 = abs(getattr(self.arm_camera_position[arm.camera_link][0],
                                 m) - getattr(self.arm_camera_position[arm.camera_link][1], m))
                d2 = abs(getattr(self.arm_camera_position[arm.camera_link][1],
                                 m) - getattr(self.arm_camera_position[arm.camera_link][2], m))

                if d1 > 0.005 or d2 > 0.005:
                    self.last_end_of_interaction = None
                    break

            else:

                self.last_end_of_interaction = arm, now

        if self.last_end_of_interaction is not None:

            arm, when = self.last_end_of_interaction

            # if now - when > rospy.Duration(5.0):
            #    self.last_end_of_interaction = None
            #    self.ui.notif(translate("PickFromFeeder", "No suitable part was found."), temp=True)  # TODO for which arm
            #    return

            obj_type = None

            for obj_arr in self.objects_by_sensor[arm.camera_link].values():

                if not obj_type:
                    obj_type = obj_arr[1].object_type
                    continue

                if obj_arr[1].object_type != obj_type:
                    # TODO if this happens, show it once at the end of interval
                    # self.ui.notif(translate("PickFromFeeder", "Can see more types of parts."), temp=True)
                    obj_type = None
                    break

            if not obj_type:
                return

            if obj_type != self.ui.ph.get_object(*self.cid)[0][0]:
                rospy.loginfo("new object type: " + obj_type)
                self.ui.program_vis.clear_poses()

                self.ui.program_vis.set_object(obj_type)
                # self.ui.select_object_type(obj.object_type.name)

                # TODO check if pose is different from the saved one and allow to change
                # pose without changing object type
                ps = arm.get_pose()

                if ps:

                    self.ui.notif(translate("PickFromFeeder", "Gripper pose stored."), temp=True)
                    self.ui.notify_info()
                    self.ui.program_vis.set_pose(ps)

                else:

                    self.ui.notif(translate("PickFromFeeder", "Failed to get gripper pose."), temp=True)
                    self.ui.notify_warn()

    def object_raw_cb(self, msg):

        self.emit(QtCore.SIGNAL('objects_raw'), msg)


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
