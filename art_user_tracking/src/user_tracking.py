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
import scipy
from scipy.ndimage.filters import gaussian_filter


class UserTracking():

    def __init__(self):

        self.stride = 8
        self.padValue = 128
        self.thre1 = 0.1

        self.bridge = CvBridge()

        self.subs = []
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/image_color_rect', Image))
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/camera_info', CameraInfo))
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/image_depth_rect', Image))

        self.ts = message_filters.TimeSynchronizer(self.subs, 1)
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

        scale = 0.25 * 368.0 / cv_img.shape[0]
        img_scaled = cv2.resize(cv_img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

        img_padded, pad = self.padRightDownCorner(img_scaled, self.stride, self.padValue)

        # cv2.imshow('dst_rt', img_padded)
        # cv2.waitKey(0)

        self.net.blobs['data'].reshape(*(1, 3, img_padded.shape[0], img_padded.shape[1]))
        # self.net.forward() # dry run
        self.net.blobs['data'].data[...] = np.transpose(np.float32(img_padded[:, :, :, np.newaxis]), (3, 2, 0, 1)) / 256 - 0.5;
        start_time = time.time()
        print "forward..."
        output_blobs = self.net.forward()
        print('The CNN took %.2f ms.' % (1000 * (time.time() - start_time)))

        heatmap = np.transpose(np.squeeze(self.net.blobs[output_blobs.keys()[1]].data), (1, 2, 0))  # output 1 is heatmaps
        heatmap = cv2.resize(heatmap, (0, 0), fx=self.stride, fy=self.stride, interpolation=cv2.INTER_CUBIC)
        heatmap = heatmap[:img_padded.shape[0] - pad[2], :img_padded.shape[1] - pad[3], :]
        heatmap = cv2.resize(heatmap, (cv_img.shape[1], cv_img.shape[0]), interpolation=cv2.INTER_CUBIC)

        paf = np.transpose(np.squeeze(self.net.blobs[output_blobs.keys()[0]].data), (1, 2, 0))  # output 0 is PAFs
        paf = cv2.resize(paf, (0, 0), fx=self.stride, fy=self.stride, interpolation=cv2.INTER_CUBIC)
        paf = paf[:img_padded.shape[0] - pad[2], :img_padded.shape[1] - pad[3], :]
        paf = cv2.resize(paf, (cv_img.shape[1], cv_img.shape[0]), interpolation=cv2.INTER_CUBIC)

        all_peaks = []
        peak_counter = 0

        for part in range(19 - 1):
            x_list = []
            y_list = []
            map_ori = heatmap[:, :, part]
            map = gaussian_filter(map_ori, sigma=3)

            map_left = np.zeros(map.shape)
            map_left[1:, :] = map[:-1, :]
            map_right = np.zeros(map.shape)
            map_right[:-1, :] = map[1:, :]
            map_up = np.zeros(map.shape)
            map_up[:, 1:] = map[:, :-1]
            map_down = np.zeros(map.shape)
            map_down[:, :-1] = map[:, 1:]

            peaks_binary = np.logical_and.reduce((map >= map_left, map >= map_right, map >= map_up, map >= map_down, map > self.thre1))
            peaks = zip(np.nonzero(peaks_binary)[1], np.nonzero(peaks_binary)[0])  # note reverse
            peaks_with_score = [x + (map_ori[x[1], x[0]],) for x in peaks]
            id = range(peak_counter, peak_counter + len(peaks))
            peaks_with_score_and_id = [peaks_with_score[i] + (id[i],) for i in range(len(id))]

            all_peaks.append(peaks_with_score_and_id)
            peak_counter += len(peaks)


if __name__ == '__main__':

    try:

        rospy.init_node('art_user_tracking')
        UserTracking()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
