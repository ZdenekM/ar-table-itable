import rospy
from moveit_commander import PlanningSceneInterface
from art_utils import ObjectHelper, ArtApiHelper
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Header
from art_msgs.msg import CollisionObjects, CollisionPrimitive, ObjInstance, ObjectType
import uuid
from threading import RLock
from shape_msgs.msg import SolidPrimitive
from art_utils import array_from_param
from tf import TransformListener
import tf
from typing import List, Tuple

"""
TODO
tests!!!
grouping of artificial objects - to move whole group at once
ability to set/store/update artificial objects wrt the program block/instruction?
"""


class CollisionEnvException(Exception):
    pass


class CollisionEnv(object):

    NS = "/art/collision_env/"

    def __init__(self, setup, world_frame):

        assert setup != ""

        self.ready = False
        self.setup = setup
        self.world_frame = world_frame

        self.tf_listener = TransformListener()

        self.api = ArtApiHelper()
        rospy.loginfo("Waiting for DB API")
        self.api.wait_for_db_api()
        self.ignored_prefixes = array_from_param("~ignored_prefixes")
        rospy.loginfo("Will ignore following prefixes: " + str(self.ignored_prefixes))

        self.lock = RLock()

        self.ps = PlanningSceneInterface()

        self._paused = False
        self.oh = ObjectHelper(self.object_cb)
        self.artificial_objects = {}

        self.collision_objects_pub = rospy.Publisher(self.NS + "artificial", CollisionObjects, latch=True, queue_size=1)
        self.pub_artificial()

        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)

        self.paused_pub = rospy.Publisher(self.NS + "paused", Bool, latch=True, queue_size=1)
        self.paused = False

    def start(self):
        # type: () -> None

        self.ready = True
        rospy.loginfo("Ready")

    def load_from_db(self):
        # type: () -> None

        for prim in self.api.get_collision_primitives(self.setup):

            rospy.loginfo("Loading object: " + prim.name)
            self.add_primitive(prim)

    def save_primitive(self, name):
        # type: (str) -> None

        with self.lock:

            if name not in self.artificial_objects:
                raise CollisionEnvException("Unknown object name")

            p = self.artificial_objects[name]
            self.api.add_collision_primitive(p)

    def save_primitives(self):
        # type: () -> None

        with self.lock:

            for name in self.artificial_objects.keys():

                self.save_primitive(name)

    def set_primitive_pose(self, name, ps):
        # type: (str, PoseStamped) -> None

        with self.lock:

            p = self.artificial_objects[name]
            assert p.pose.header.frame_id == ps.header.frame_id
            p.pose.pose = ps.pose
            self.ps.add_box(p.name, p.pose, p.bbox.dimensions)
            self.pub_artificial()

    def add_primitive(self, p):
        # type: (CollisionPrimitive) -> None

        with self.lock:

            self.artificial_objects[p.name] = p
            self.ps.add_box(p.name, p.pose, p.bbox.dimensions)
            self.pub_artificial()

    def pub_artificial(self):
        # type: () -> None

        msg = CollisionObjects()

        for v in self.artificial_objects.values():
            # transform all collision primitives into world frame (= marker)
            if v.pose.header.frame_id != self.world_frame:
                try:
                    self.tf_listener.waitForTransform(
                        v.pose.header.frame_id, self.world_frame, rospy.Time(0), rospy.Duration(1.0))
                    v.pose = self.tf_listener.transformPose(self.world_frame, v.pose)
                except tf.Exception as e:
                    rospy.logwarn("Failed to transform artificial object: " + str(e))
                    continue

            msg.primitives.append(v)

        self.collision_objects_pub.publish(msg)

    @property
    def paused(self):

        return self._paused

    @paused.setter
    def paused(self, val):

        self._paused = val
        self.paused_pub.publish(val)

    def remove_name(self, name):
        # type: (str) -> bool

        with self.lock:

            if name not in self.artificial_objects:
                rospy.logwarn("Unknown artificial object: " + name)
                return False

            self.ps.remove_world_object(name)
            del self.artificial_objects[name]
            self.pub_artificial()

        if not self.api.clear_collision_primitives(self.setup, names=[name]):
            rospy.logwarn("Failed to remove from permanent storage")

        rospy.loginfo("Removed object: " + name)
        return True

    def clear_all(self, permanent=True):
        # type: (bool) -> None

        with self.lock:

            rospy.loginfo("Clearing " + str(len(self.artificial_objects)) + " artificial objects...")

            for k in self.artificial_objects.keys():

                if self.is_ignored(k):
                    continue

                self.ps.remove_world_object(k)

            self.artificial_objects = {}
            self.pub_artificial()

        if permanent and not self.api.clear_collision_primitives(self.setup):
            rospy.logwarn("Failed to remove from permanent storage")

    def reload(self):
        # type: () -> None

        self.clear_all(permanent=False)
        self.load_from_db()
        self.pub_artificial()

    def _generate_name(self):
        # type: () -> str

        # as we use only part of uuid, there might be collisions...
        while True:

            name = str(uuid.uuid4())[:8]
            if name not in self.artificial_objects.keys():
                break

        return name

    def set_det_pose(self, name, ps):
        # type: (str, PoseStamped) -> None

        object_type = self.api.get_object_type(self.oh.objects[name].object_type)

        if object_type is not None:
            self.add_detected(name, ps, object_type)

    def clear_det_on_table(self, inv=False, ignore=None):
        # type: (bool, List[str]) -> List[str]

        if ignore is None:
            ignore = []

        ret = []

        with self.lock:

            for v in self.oh.objects.values():

                if v.object_id in ignore:
                    continue

                if not inv and not v.on_table:
                    continue

                if inv and v.on_table:
                    continue

                ret.append(v.object_id)
                self.clear_detected(v.object_id)

        return ret

    def is_ignored(self, name):
        # type: (str) -> bool

        for ip in self.ignored_prefixes:
            if name.startswith(ip):
                return True

        return False

    def timer_cb(self, evt):

        if self.paused:
            return

        with self.lock:

            known_objects = self.ps.get_known_object_names()

            for name in known_objects:

                if name not in self.artificial_objects and name not in self.oh.objects and not self.is_ignored(name):

                    rospy.loginfo("Removing outdated detected object: " + name)
                    self.clear_detected(name)

            # restore artificial objects if they are lost somehow (e.g. by restart  of MoveIt!)
            for k, v in self.artificial_objects.iteritems():

                if k in known_objects:
                    continue

                rospy.loginfo("Restoring artificial object: " + v.name)
                if v.bbox.type == SolidPrimitive.BOX:
                    self.ps.add_box(v.name, v.pose, v.bbox.dimensions)
                else:
                    # TODO other types
                    pass

    def object_cb(self, evt, h, inst):
        # type: (int, Header, ObjInstance) -> None

        if self.paused or not self.ready:
            return

        with self.lock:

            attached_objects = self.ps.get_attached_objects()

            if evt in (ObjectHelper.OBJECT_ADDED, ObjectHelper.OBJECT_UPDATED):

                if inst.object_id in attached_objects:
                    return

                ps = PoseStamped()
                ps.header = h
                ps.pose = inst.pose

                object_type = self.api.get_object_type(inst.object_type)

                if object_type is not None:

                    self.add_detected(inst.object_id, ps, object_type)

            elif evt == ObjectHelper.OBJECT_LOST:

                if inst.object_id not in attached_objects:

                    self.clear_detected(inst.object_id)

    def add_detected(self, name, ps, object_type):
        # type: (str, PoseStamped, ObjectType) -> None

        with self.lock:

            self.ps.add_box(name, ps, object_type.bbox.dimensions)

    def clear_detected(self, name):
        # type: (str) -> None

        with self.lock:

            self.ps.remove_world_object(name)

    def clear_all_det(self, ignore=None):
        # type: (List[str]) -> List[str]

        if ignore is None:
            ignore = []

        ret = []

        with self.lock:

            for v in self.oh.objects.values():
                name = v.object_id
                if name in ignore:
                    continue
                self.clear_detected(name)
                ret.append(name)

        rospy.loginfo("Removed " + str(len(ret)) + " detected objects.")
        return ret

    def get_attached(self, transform_to_world=True):
        # type: (bool) -> List[Tuple[str, PoseStamped, SolidPrimitive]]
        """
        keep in mind - attached objects might not be detected
        """

        ret = []

        with self.lock:

            ao = self.ps.get_attached_objects()

        for k, v in ao.iteritems():

            if len(v.object.primitives) != 1 or len(v.object.primitive_poses) != 1:
                rospy.logwarn("Unsupported type of attached object: " + k)
                continue

            if v.object.primitives[0].type != SolidPrimitive.BOX:
                rospy.logwarn("Only box-like attached objects are supported so far.")
                continue

            ps = PoseStamped()
            ps.header = v.object.header
            ps.pose = v.object.primitive_poses[0]

            if transform_to_world and ps.header.frame_id != self.world_frame:
                try:
                    self.tf_listener.waitForTransform(
                        ps.header.frame_id, self.world_frame, ps.header.stamp, rospy.Duration(1.0))
                    ps = self.tf_listener.transformPose(self.world_frame, ps)
                except tf.Exception as e:
                    rospy.logwarn("Failed to transform attached object: " + str(e))
                    continue

            ret.append((k, ps, v.object.primitives[0]))

        return ret
