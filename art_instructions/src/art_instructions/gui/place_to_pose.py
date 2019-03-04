from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore, QtGui
from art_projected_gui.items import Item, ObjectItem, PlaceItem
from geometry_msgs.msg import PoseStamped
import tf
from math import sqrt
from art_msgs.srv import NotifyUserRequest
import rospy
from functools import reduce

translate = QtCore.QCoreApplication.translate


# https://stackoverflow.com/a/21068791
def frange(start, stop, step):
    '''Helper float generator'''
    while start < stop:
        yield start
        start += step


def painterPathArea(path, precision=10):  # low precision is ok in this case...
    '''QPainterPath area calculation'''
    points = [(point.x(), point.y()) for point in (path.pointAtPercent(perc) for perc in frange(0, 1, 1.0 / precision))]
    points.append(points[0])

    return 0.5 * abs(reduce(
        lambda sum, i: sum + (points[i][0] * points[i + 1][1] - points[i + 1][0] * points[i][1]),
        xrange(len(points) - 1),
        0
    ))


class RangeVisItem(Item):

    def __init__(self, scene, x, y, min_range=0.1, max_range=0.3):

        super(RangeVisItem, self).__init__(scene, x, y)

        self.min_range = self.m2pix(min_range)
        self.max_range = self.m2pix(max_range)

        self.setZValue(-500)

    def boundingRect(self):
        # robot is in the middle
        return QtCore.QRectF(-self.max_range, -self.max_range, 2 * self.max_range, 2 * self.max_range)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        painter.setBrush(QtCore.Qt.NoBrush)

        dr = self.max_range - self.min_range

        pen = QtGui.QPen()
        pen.setWidth(dr)
        pen.setColor(QtCore.Qt.white)
        # pen.setCosmetic(True)
        # pen.setJoinStyle(QtCore.Qt.RoundJoin)
        painter.setPen(pen)

        painter.setOpacity(0.2)

        # it would be more sophisticated to use QtGui.QPainterPath()...
        painter.drawEllipse(QtCore.QPointF(0, 0), self.max_range - dr / 2, self.max_range - dr / 2)


class PlaceToPose(GuiInstruction):

    NAME = translate("PlaceToPose", "Place to pose")

    def __init__(self, *args, **kwargs):

        super(PlaceToPose, self).__init__(*args, **kwargs)

        self.place = None
        self.hidden_objects = []
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timer_tick)
        self.timer.start(500)

    def get_name(self, block_id, item_id):

        name = self.ui.ph.get_name(block_id, item_id)

        if name:
            return name

        return translate("PlaceToPose", "PLACE POSE")

    def timer_tick(self):

        if self.place and self.place.hover:
            return

        for obj in self.ui.get_scene_items_by_type(ObjectItem):
            for place_pose in self.ui.get_scene_items_by_type(PlaceItem):

                if obj.object_type != place_pose.object_type or not obj.collidesWithItem(place_pose):
                    continue

                pp_shape = place_pose.mapToScene(place_pose.shape())
                intersected = obj.mapToScene(obj.shape()).intersected(pp_shape)

                rat = painterPathArea(intersected) / painterPathArea(pp_shape)

                if rat > 0.5:
                    if obj not in self.hidden_objects:
                        obj.set_enabled(False, also_set_visibility=True)
                        self.hidden_objects.append(obj)
                        self.logdebug("Object ID " + obj.object_id + " hidden (" + str(rat) + ")")
                    break
            else:
                if obj in self.hidden_objects:
                    obj.set_enabled(True, also_set_visibility=True)
                    self.hidden_objects.remove(obj)
                    self.logdebug("Showing object ID " + obj.object_id)

    def cleanup(self):

        self.timer.stop()

        for obj in self.hidden_objects:
            obj.set_enabled(True, also_set_visibility=True)

        return ()


class PlaceToPoseLearn(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseLearn, self).__init__(*args, **kwargs)

        self.range_ind = []
        self.arms_pos = {}
        self.selected_arm = None
        self.out_of_reach = False

        if not self.ui.ph.is_object_set(*self.cid):

            (obj_arr, ref_id) = self.ui.ph.get_object(*self.cid)

            self.ui.notif(translate(
                "PlaceToPose", "Select object to be picked up in instruction %1").arg(ref_id))
            self.notified = True
            return

        if self.editable:

            # get current poses of arms' base links (used to display range)
            for arm in self.ui.rh.get_robot_arms():

                if not arm.base_link:
                    self.logwarn("Arm base_link not defined!")
                    continue

                p = PoseStamped()
                p.header.frame_id = arm.base_link

                # TODO get world frame from ui (where it should be read from param)
                try:
                    self.ui.tfl.waitForTransform("marker", p.header.frame_id, p.header.stamp, rospy.Duration(5.0))
                    p = self.ui.tfl.transformPose("marker", p)
                except tf.Exception as e:
                    self.logerr(str(e))
                    continue

                self.arms_pos[arm] = p.pose.position

            if self.ui.hololens_learning:
                self.ui.notif(
                    translate(
                        "PlaceToPose",
                        "Put part on desired place. Then use blue sphere to set orientation."))
            else:
                self.ui.notif(
                    translate(
                        "PlaceToPose",
                        "Drag object outline to set place pose. Use blue point to set orientation."))
            self.notified = True

        else:

            if self.ui.ph.is_pose_set(*self.cid):
                self.ui.notif(
                    translate(
                        "PlaceToPose",
                        "You are done. However, place pose might be adjusted if necessary."))
                self.notified = True

        # loop through all instructions in block in order to display current and other place poses
        # ...other place poses are displayed using dashed outline
        for it_id in self.ui.ph.get_items_ids(self.block_id):

            if self.ui.ph.get_item_msg(self.block_id, it_id).type != "PlaceToPose":
                continue

            self.show_place_for_instruction(it_id)

    def show_place_for_instruction(self, it_id):

        object_type = None
        object_id = None

        if self.ui.ph.is_object_set(self.block_id, it_id):
            object_type = self.ui.art.get_object_type(self.ui.ph.get_object(self.block_id, it_id)[0][0])

        if it_id == self.instruction_id:

            pick_msg = self.ui.ph.get_ref_pick_item_msg(*self.cid)

            if self.editable:

                add_only_one = pick_msg.type in self.ui.ih.properties.using_pose and self.ui.ph.is_pose_set(
                    self.block_id, pick_msg.id)

                if add_only_one and self.arms_pos:

                    closer_arm, closer_dist = None, None
                    lpos = self.ui.ph.get_pose(self.block_id, pick_msg.id)[0][0].pose.position

                    for arm, pos in self.arms_pos.iteritems():

                        dist = sqrt((pos.x - lpos.x) ** 2 + (pos.y - lpos.y) ** 2 + (pos.z - lpos.z) ** 2)

                        if not closer_arm or dist < closer_dist:
                            closer_arm, closer_dist = arm, dist

                    self.selected_arm = closer_arm
                    pos = self.arms_pos[closer_arm]
                    self.range_ind.append(RangeVisItem(self.ui.scene, pos.x, pos.y, *closer_arm.range))

                else:

                    for arm, pos in self.arms_pos.iteritems():
                        self.range_ind.append(RangeVisItem(self.ui.scene, pos.x, pos.y, *arm.range))

            if self.ui.ph.is_pose_set(self.block_id, it_id):

                if object_type is not None:

                    ps = self.ui.ph.get_pose(self.block_id, it_id)[0][0]

                    self.place = self.ui.add_place(
                        self.get_name(self.block_id, it_id),
                        ps,
                        object_type,
                        object_id,
                        place_cb=self.place_pose_changed,  # TODO place_cb should be set in add_place?
                        fixed=not self.editable)
                    self.place.set_selected(not self.editable)

                    if self.editable:

                        if not self.check_place_pose([ps.pose.position.x,
                                                      ps.pose.position.y,
                                                      ps.pose.position.z]):
                            self.place.get_attention()

            elif self.editable:

                self.place = self.ui.add_place(self.get_name(self.block_id, it_id), self.ui.get_def_pose(
                ), object_type, object_id, place_cb=self.place_pose_changed, fixed=not self.editable)
                self.place.set_selected(not self.editable)

                self.ui.program_vis.item_edit_btn.set_enabled(False)
                self.place.get_attention()

            return

        if self.ui.ph.is_pose_set(self.block_id, it_id):
            self.ui.add_place(
                unicode(
                    self.get_name(self.block_id, it_id)) + " (" + str(it_id) + ")",
                self.ui.ph.get_pose(self.block_id, it_id)[0][0],
                object_type,
                object_id,
                fixed=True,
                dashed=True)

    def check_place_pose(self, pp):

        if not self.arms_pos:  # robot without arms (e.g. art_empty_arm)
            return True

        for arm, pos in self.arms_pos.iteritems():

            if self.selected_arm and arm != self.selected_arm:
                continue

            d = sqrt((pos.x - pp[0]) ** 2 + (pos.y - pp[1]) ** 2)

            if arm.range[0] < d < arm.range[1]:
                self.place.set_ext_color()
                self.out_of_reach = False
                return True

        self.place.set_ext_color(QtCore.Qt.red)

        if not self.out_of_reach:
            self.ui.notif(translate("PlaceToPose", "Pose out of reach."), temp=True,
                          message_type=NotifyUserRequest.WARN)
            self.out_of_reach = True

        return False

    def place_pose_changed(self, place):

        if self.ui.program_vis.editing_item:

            self.ui.program_vis.set_place_pose(place)
            self.ui.state_manager.update_program_item(self.ui.ph.get_program_id(
            ), self.ui.program_vis.block_id, self.ui.program_vis.get_current_item())

            check = self.check_place_pose(place.position)
            self.ui.program_vis.item_edit_btn.set_enabled(check)
            if check:
                self.ui.program_vis.item_edit_btn.get_attention()
            else:
                self.ui.program_vis.item_edit_btn.stop_getting_attention()

    def object_selected(self, obj, selected, msg):

        return

    def cleanup(self):

        super(PlaceToPoseLearn, self).cleanup()

        for ind in self.range_ind:
            self.ui.scene.removeItem(ind)

        self.range_ind = []

        return ()


class PlaceToPoseRun(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseRun, self).__init__(*args, **kwargs)

        try:
            obj_id = self.flags["SELECTED_OBJECT_ID"]
        except KeyError:
            self.logerr(
                "PLACE_TO_POSE: SELECTED_OBJECT_ID flag not set")
            return

        obj = self.ui.get_object(obj_id)

        if obj is not None:

            place_pose = self.ui.ph.get_pose(*self.cid)[0][0]
            self.place = self.ui.add_place(self.get_name(*self.cid), place_pose, obj.object_type, obj_id, fixed=True)
            self.ui.notif(translate("PlaceToPose", "Placing object to pose."))

        else:

            self.logerr("Selected object_id not found: " + obj_id)


class PlaceToPoseVis(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseVis, self).__init__(*args, **kwargs)

        object_type = None
        object_id = None

        if self.ui.ph.is_object_set(*self.cid):
            object_type = self.ui.art.get_object_type(self.ui.ph.get_object(*self.cid)[0][0])

        if object_type is not None:
            place_pose = self.ui.ph.get_pose(*self.cid)[0][0]

            self.ui.add_place(translate("PlaceToPose", "OBJECT PLACE POSE"),
                              place_pose, object_type, object_id, fixed=True)
