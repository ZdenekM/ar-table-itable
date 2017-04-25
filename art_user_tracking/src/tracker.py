#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from art_msgs.msg import User, UserArray
import tf
from scipy.spatial import distance


class UserPart():

    def __init__(self, idx):

        self.idx = idx
        self.pt = None
        self.updated = None

    def update(self, pt):  # TODO use score?

        self.updated = rospy.Time.now()
#        if self.pt is not None:
#            self.pt.x = 0.2*self.pt.x + 0.8*pt.x
#            self.pt.y = 0.2*self.pt.x + 0.8*pt.y
#            self.pt.z = 0.2*self.pt.x + 0.8*pt.z
#        else:
        self.pt = pt


class TrackedUser():

    def __init__(self, id):

        self.id = id
        self.updated = rospy.Time.now()

        self.parts = {}

        for i in range(18):
            self.parts[i] = UserPart(i)

    def get_pos(self, idx):

        return self.parts[idx].pt

    def update_parts(self, parts):

        rospy.logdebug("updating user id: " + str(self.id))
        self.updated = rospy.Time.now()

        for k, v in parts.iteritems():

            self.parts[k].update(v)


class UserTracker():

    def __init__(self):

        self.tfl = tf.TransformListener()
        self.users = {}
        self.world_frame = "kinect2_n2_ir_optical_frame"  # TODO change to marker
        self.dist_th = 1.5
        self.detected_users_sub = rospy.Subscriber(
            '/art/users/detections', UserArray, self.detected_users_cb)
        self.user_array_pub = rospy.Publisher(
            'tracked_users', UserArray, queue_size=10)
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.pub_timer_cb)
        rospy.loginfo('UserTracker ready.')

    def point2tuple(self, pt):

        return (pt.x, pt.y, pt.z)

    def get_dist(self, pt1, pt2):

        return distance.euclidean(self.point2tuple(pt1), self.point2tuple(pt2))

    def get_new_id(self):

        if len(self.users.keys()) == 0:
            return 1
        else:
            return max(self.users.keys()) + 1

    def add_user(self, parts):

        user = TrackedUser(self.get_new_id())
        rospy.logdebug("adding user id: " + str(user.id))
        user.update_parts(parts)
        self.users[user.id] = user

    def get_avg_dist(self, user, parts, detected_parts):

        dist_sum = 0.0
        dist_cnt = 0

        for i in range(18):

            if i not in detected_parts:
                continue

            if user.parts[i].updated is None:
                continue

            d = self.get_dist(user.parts[i].pt, parts[i])
            rospy.logdebug("user id: " + str(user.id) +
                           ", idx: " + str(i) + ", dist: " + str(d))
            dist_sum += d
            dist_cnt += 1

        return dist_sum / dist_cnt

    def pub_timer_cb(self, event):

        to_delete = []
        now = rospy.Time.now()

        for k, v in self.users.iteritems():

            if (now - v.updated) > rospy.Duration(5.0):

                to_delete.append(k)

        for d in to_delete:

            rospy.logdebug("deleting user id: " + str(d))
            del self.users[d]

        ua = UserArray()
        ua.header.frame_id = self.world_frame
        ua.header.stamp = now

        for k, v in self.users.iteritems():

            u = User()
            u.user_id = k
            u.activity = User.UNKNOWN  # TODO !

            for kk, vv in v.parts.iteritems():

                if vv.updated is not None and (
                        now - vv.updated) < rospy.Duration(5.0):

                    u.detected_parts.append(kk)
                    u.parts[kk] = vv.pt

            ua.users.append(u)
        self.user_array_pub.publish(ua)

    def detected_users_cb(self, msg):

        for user in msg.users:

            parts = {}

            for part_idx in user.detected_parts:

                ps = PointStamped()
                ps.header = msg.header
                ps.header.stamp = rospy.Time(0)  # TODO delete this!!!!!
                ps.point = user.parts[part_idx]

                try:
                    ps = self.tfl.transformPoint(self.world_frame, ps)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr(
                        "Can't get transform between " +
                        self.world_frame +
                        " and " +
                        ps.header.frame_id +
                        ".")
                    continue

                parts[part_idx] = ps.point

            if len(parts) == 0:
                continue

            if len(self.users) == 0:

                self.add_user(parts)

            else:

                min_avg_dist = None
                min_avg_dist_id = None

                for k, v in self.users.iteritems():

                    d = self.get_avg_dist(v, parts, user.detected_parts)

                    if min_avg_dist is None or d < min_avg_dist:
                        min_avg_dist = d
                        min_avg_dist_id = k

                rospy.logdebug("min_avg_dist: " + str(min_avg_dist))
                if min_avg_dist < self.dist_th:

                    self.users[min_avg_dist_id].update_parts(parts)

                else:

                    self.add_user(parts)


if __name__ == '__main__':

    try:

        rospy.init_node('art_user_tracker')
        UserTracker()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
