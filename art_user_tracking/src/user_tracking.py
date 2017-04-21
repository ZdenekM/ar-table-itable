#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import rospy
from image_geometry import PinholeCameraModel
import tf
import rospkg
import message_filters
from configobj import ConfigObj
import numpy as np
import caffe


class UserTracking():

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic", Image, self.image_callback)

        self.subs = []
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/image_color_rect', Image))
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/camera_info', CameraInfo))
        self.subs.append(message_filters.Subscriber('/kinect2_n2/sd/image_depth_rect', Image))

        self.ts = message_filters.TimeSynchronizer(self.subs, 10)
        self.ts.registerCallback(self.sync_cb)

        self.data_path = rospkg.RosPack().get_path('art_user_tracking') + '/data/'
        self.read_config()

        if param['use_gpu']:
            caffe.set_mode_gpu()
            caffe.set_device(self.param['GPUdeviceNumber'])  # set to your device!
        else:
            caffe.set_mode_cpu()
        self.net = caffe.Net(self.data_path + 'pose_deploy.prototxt', self.data_path + 'pose_iter_440000.caffemodel', caffe.TEST)

    def padRightDownCorner(img, stride, padValue):
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

    def read_config(self):

        config = ConfigObj(self.data_path + 'config')

        self.param = config['param']
        model_id = self.param['modelID']
        self.model = config['models'][model_id]
        self.model['boxsize'] = int(self.model['boxsize'])
        self.model['stride'] = int(self.model['stride'])
        self.model['padValue'] = int(self.model['padValue'])
        # self.param'starting_range'] = float(self.param'starting_range'])
        # self.param'ending_range'] = float(self.param'ending_range'])
        self.param['octave'] = int(self.param['octave'])
        self.param['use_gpu'] = int(self.param['use_gpu'])
        self.param['starting_range'] = float(self.param['starting_range'])
        self.param['ending_range'] = float(self.param['ending_range'])
        self.param['scale_search'] = map(float, self.param['scale_search'])
        self.param['thre1'] = float(self.param['thre1'])
        self.param['thre2'] = float(self.param['thre2'])
        self.param['thre3'] = float(self.param['thre3'])
        self.param['mid_num'] = int(self.param['mid_num'])
        self.param['min_num'] = int(self.param['min_num'])
        self.param['crop_ratio'] = float(self.param['crop_ratio'])
        self.param['bbox_ratio'] = float(self.param['bbox_ratio'])
        self.param['GPUdeviceNumber'] = int(self.param['GPUdeviceNumber'])

    def sync_cb(self, image, cam_info, depth):

        model = PinholeCameraModel()
        model.fromCameraInfo(info)

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

        img_padded, pad = self.padRightDownCorner(cv_img, self.model['stride'], self.model['padValue'])

        self.net.blobs['data'].reshape(*(1, 3, img_padded.shape[0], img_padded.shape[1]))
        # net.forward() # dry run
        self.net.blobs['data'].data[...] = np.transpose(np.float32(img_padded[:, :, :, np.newaxis]), (3, 2, 0, 1)) / 256 - 0.5;
        start_time = time.time()
        output_blobs = self.net.forward()
        print('The CNN took %.2f ms.' % (1000 * (time.time() - start_time)))


if __name__ == '__main__':

    try:

        rospy.init_node('art_user_tracking')
        UserTracking()

    except rospy.ROSInterruptException:
        pass
