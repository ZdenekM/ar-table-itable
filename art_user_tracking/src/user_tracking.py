#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import rospy
from image_geometry import PinholeCameraModel
import tf
import rospkg
import message_filters
import numpy as np
import caffe
import time
import cv2
import pylab as plt


class UserTracking():

    def __init__(self):

        self.stride = 8
        self.padValue = 128

        self.bridge = CvBridge()

        self.subs = []
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/image_color_rect', Image))
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/camera_info', CameraInfo))
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/image_depth_rect', Image))

        self.ts = message_filters.TimeSynchronizer(self.subs, 10)
        self.ts.registerCallback(self.sync_cb)

        self.data_path = rospkg.RosPack().get_path('art_user_tracking') + '/data/'

        if rospy.get_param("use_gpu", False):
            caffe.set_mode_gpu()
            caffe.set_device(rospy.get_param('GPUdeviceNumber', 0))  # set to your device!
        else:
            caffe.set_mode_cpu()
        self.net = caffe.Net(self.data_path + 'pose_deploy.prototxt', self.data_path + 'pose_iter_440000.caffemodel', caffe.TEST)

    def padRightDownCorner(self, img, stride, padValue):
        h = img.shape[0]
        w = img.shape[1]

        pad = 4 * [None]
        pad[0] = 0  # up
        pad[1] = 0  # left
        pad[2] = 0 if (h % stride == 0) else stride - (h % stride)  # down
        pad[3] = 0 if (w % stride == 0) else stride - (w % stride)  # right

        img_padded = img
        pad_up = np.tile(img_padded[0:1, :, :] * 0 + padValue, (pad[0], 1, 1))
        img_padded = np.concatenate((pad_up, img_padded), axis=0)
        pad_left = np.tile(img_padded[:, 0:1, :] * 0 + padValue, (1, pad[1], 1))
        img_padded = np.concatenate((pad_left, img_padded), axis=1)
        pad_down = np.tile(img_padded[-2:-1, :, :] * 0 + padValue, (pad[2], 1, 1))
        img_padded = np.concatenate((img_padded, pad_down), axis=0)
        pad_right = np.tile(img_padded[:, -2:-1, :] * 0 + padValue, (1, pad[3], 1))
        img_padded = np.concatenate((img_padded, pad_right), axis=1)

        return img_padded, pad

    def sync_cb(self, image, cam_info, depth):

        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

        try:
            cv_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth)
        except CvBridgeError as e:
            print(e)
            return

        print "sync_cb"

        print ("cv_img.shape", cv_img.shape)
        scale = 368.0 / cv_img.shape[0]
        img_scaled = cv2.resize(cv_img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        print ("img_scaled.shape", img_scaled.shape)
        img_padded, pad = self.padRightDownCorner(img_scaled, self.stride, self.padValue)

        print ("img_padded.shape", img_padded.shape)

        # cv2.imshow('dst_rt', img_padded)
        # cv2.waitKey(0)

        self.net.blobs['data'].reshape(*(1, 3, img_padded.shape[0], img_padded.shape[1]))
        # self.net.forward() # dry run
        self.net.blobs['data'].data[...] = np.transpose(np.float32(img_padded[:, :, :, np.newaxis]), (3, 2, 0, 1)) / 256 - 0.5;
        start_time = time.time()
        print "forward..."
        output_blobs = self.net.forward()
        print('The CNN took %.2f ms.' % (1000 * (time.time() - start_time)))


if __name__ == '__main__':

    try:

        rospy.init_node('art_user_tracking')
        UserTracking()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
