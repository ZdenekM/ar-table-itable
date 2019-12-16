#!/usr/bin/env python
import rospy
import jsonpickle
from art_msgs.msg import InstancesArray
from std_msgs.msg import String
from art_msgs.srv import getObjectTypeRequest, getObjectType
from art_utils.art_api_helper import ArtApiHelper


class BridgeToJsonMsg:

    def __init__(self):
        rospy.Subscriber("/art/object_detector/object_filtered", InstancesArray, self.callback)
        self.pub = rospy.Publisher("/objects_string", String, queue_size=1)
        self.api = ArtApiHelper()
        self.api.wait_for_db_api()
        rospy.spin()

    def callback(self, detected_object):
        """
        :type detected_object: InstancesArray
        :return:
        """

        objects = []
        for obj in detected_object.instances:
            '''to_json = {'name': obj.object_id, 'pose': {'position': {'x': obj.pose.position.x,
                                                                         'y': obj.pose.position.y,
                                                                         'z': obj.pose.position.z},
                                                            'orientation': {'x': obj.pose.orientation.x,
                                                                            'y': obj.pose.orientation.y,
                                                                            'z': obj.pose.orientation.z,
                                                                            'w': obj.pose.orientation.w}}}'''
            obj_type = self.api.get_object_type(obj.object_type)

            if not obj_type:
                continue

            to_json = {'name': obj.object_id,
                       'type': obj.object_type,
                       'position': {'x': obj.pose.position.x,  # in meters
                                    'y': obj.pose.position.y,
                                    'z': obj.pose.position.z},
                       'orientation': {'x': obj.pose.orientation.x,
                                       'y': obj.pose.orientation.y,
                                       'z': obj.pose.orientation.z,
                                       'w': obj.pose.orientation.w},
                       'bbox': {'x': obj_type.bbox.dimensions[0],
                                'y': obj_type.bbox.dimensions[1],
                                'z': obj_type.bbox.dimensions[2]}}
            objects.append(to_json)
        self.pub.publish(jsonpickle.encode(objects))


if __name__ == '__main__':
    rospy.init_node('bridge_to_json_msg')

    try:
        node = BridgeToJsonMsg()
    except rospy.ROSInterruptException:
        pass
