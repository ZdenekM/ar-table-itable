from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore, QtGui
from art_projected_gui.items import Item
from geometry_msgs.msg import PoseStamped
import tf
from math import sqrt
from art_msgs.srv import NotifyUserRequest

translate = QtCore.QCoreApplication.translate


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

    def get_name(self, block_id, item_id):

        name = self.ui.ph.get_name(block_id, item_id)

        if name:
            return name

        return translate("PlaceToPose", "PLACE POSE")


class PlaceToPoseLearn(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseLearn, self).__init__(*args, **kwargs)

        self.range_ind = []
        self.arms_pos = {}
        self.place = None
        self.selected_arm = None
        self.out_of_reach = False

        if not self.ui.ph.is_object_set(*self.cid):

            (obj_arr, ref_id) = self.ui.ph.get_object(*self.cid)

            self.ui.notif(translate(
                "PlaceToPose", "Select object to be picked up in instruction %1").arg(ref_id))
            self.notified = True

        else:

            for it_id in self.ui.ph.get_items_ids(self.block_id):

                if self.ui.ph.get_item_msg(self.block_id, it_id).type != "PlaceToPose":
                    continue

                object_type = None
                object_id = None

                if self.ui.ph.is_object_set(self.block_id, it_id):
                    object_type = self.ui.art.get_object_type(self.ui.ph.get_object(self.block_id, it_id)[0][0])

                if it_id == self.instruction_id:

                    pick_msg = self.ui.ph.get_ref_pick_item_msg(*self.cid)

                    if self.editable:

                        add_only_one = pick_msg.type in self.ui.ih.properties.using_pose and self.ui.ph.is_pose_set(
                            self.block_id, pick_msg.id)

                        for arm in self.ui.rh.get_robot_arms():

                            p = PoseStamped()
                            p.header.frame_id = arm.base_link

                            # TODO get world frame from ui (where it should be read from param)
                            try:
                                p = self.ui.tfl.transformPose("marker", p)
                            except tf.Exception as e:
                                self.logerr(str(e))
                                continue

                            self.arms_pos[arm] = p.pose.position

                        if add_only_one:

                            closer_arm, closer_dist = None, None
                            lpos = self.ui.ph.get_pose(self.block_id, pick_msg.id)[0][0].pose.position

                            for arm, pos in self.arms_pos.iteritems():

                                dist = sqrt((pos.x - lpos.x)**2 + (pos.y - lpos.y)**2 + (pos.z - lpos.z)**2)

                                if not closer_arm or dist < closer_dist:
                                    closer_arm, closer_dist = arm, dist

                            self.selected_arm = closer_arm
                            pos = self.arms_pos[closer_arm]
                            self.range_ind.append(RangeVisItem(self.ui.scene, pos.x, pos.y, *closer_arm.range))

                        else:

                            for arm, pos in self.arms_pos.iteritems():
                                self.range_ind.append(RangeVisItem(self.ui.scene, pos.x, pos.y, *arm.range))

                        self.ui.notif(
                            translate(
                                "PlaceToPose",
                                "Drag object outline to set place pose. Use blue point to set orientation."))

                    if self.ui.ph.is_pose_set(self.block_id, it_id):

                        if object_type is not None:

                            ps = self.ui.ph.get_pose(self.block_id, it_id)[0][0]

                            self.ui.select_object_type(object_type.name)
                            self.place = self.ui.add_place(
                                self.get_name(self.block_id, it_id),
                                ps,
                                object_type,
                                object_id,
                                place_cb=self.place_pose_changed,  # TODO place_cb should be set in add_place?
                                fixed=not self.editable)

                            if self.editable:

                                if not self.check_place_pose([ps.pose.position.x,
                                                              ps.pose.position.y,
                                                              ps.pose.position.z]):
                                    self.place.get_attention()

                    elif self.editable:

                        self.place = self.ui.add_place(self.get_name(self.block_id, it_id), self.ui.get_def_pose(
                        ), object_type, object_id, place_cb=self.place_pose_changed, fixed=not self.editable)

                        self.place.get_attention()

                    continue

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
            self.ui.notif(translate("PickFromFeeder", "Pose out of reach."), temp=True,
                          message_type=NotifyUserRequest.WARN)
            self.out_of_reach = True

        return False

    def place_pose_changed(self, place):

        if self.ui.program_vis.editing_item:

            self.ui.program_vis.set_place_pose(place)
            self.ui.state_manager.update_program_item(self.ui.ph.get_program_id(
            ), self.ui.program_vis.block_id, self.ui.program_vis.get_current_item())

            self.check_place_pose(place.position)

    def object_selected(self, obj, selected, msg):

        return

    def cleanup(self):

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
            self.ui.add_place(self.get_name(*self.cid), place_pose, obj.object_type, obj_id, fixed=True)
            self.ui.notif(translate("PlaceToPose", "Placing object to pose."))

        else:

            self.logerr("Selected object_id not found: " + obj_id)


class PlaceToPoseVis(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseVis, self).__init__(*args, **kwargs)

        object_type = None
        object_id = None

        self.ui.select_object_type(self.ui.ph.get_object(*self.cid)[0][0])

        if self.ui.ph.is_object_set(*self.cid):
            object_type = self.ui.art.get_object_type(self.ui.ph.get_object(*self.cid)[0][0])

        if object_type is not None:
            place_pose = self.ui.ph.get_pose(*self.cid)[0][0]

            self.ui.add_place(translate("PlaceToPose", "OBJECT PLACE POSE"),
                              place_pose, object_type, object_id, fixed=True)
