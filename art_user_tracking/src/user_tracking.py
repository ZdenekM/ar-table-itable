#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import rospy
from image_geometry import PinholeCameraModel
import rospkg
import message_filters
import numpy as np
import caffe
import time
import cv2
import pylab as plt
import scipy
from scipy.ndimage.filters import gaussian_filter
import math
from geometry_msgs.msg import Point
from art_msgs.msg import User, UserArray
from visualization_msgs.msg import Marker


class UserDetector():

    def __init__(self):

        self.stride = 8
        self.padValue = 128
        self.thre1 = 0.1
        self.thre2 = 0.05
        self.thre3 = 0.5

        self.bridge = CvBridge()

        self.subs = []
        self.subs.append(
            message_filters.Subscriber(
                '/kinect2_n2/sd/image_color_rect',
                Image))
        self.subs.append(
            message_filters.Subscriber(
                '/kinect2_n2/sd/camera_info',
                CameraInfo))
        self.subs.append(
            message_filters.Subscriber(
                '/kinect2_n2/sd/image_depth_rect',
                Image))

        self.ts = message_filters.TimeSynchronizer(self.subs, 1)
        self.ts.registerCallback(self.sync_cb)

        self.data_path = rospkg.RosPack().get_path('art_user_tracking') + '/data/'

        if rospy.get_param("use_gpu", False):
            caffe.set_mode_gpu()
            caffe.set_device(
                rospy.get_param(
                    'GPUdeviceNumber',
                    0))  # set to your device!
        else:
            caffe.set_mode_cpu()
        self.net = caffe.Net(
            self.data_path +
            'pose_deploy.prototxt',
            self.data_path +
            'pose_iter_440000.caffemodel',
            caffe.TEST)

        self.part_str = ['nose', 'neck', 'Rsho', 'Relb', 'Rwri', 'Lsho', 'Lelb', 'Lwri', 'Rhip', 'Rkne', 'Rank', 'Lhip', 'Lkne', 'Lank', 'Leye', 'Reye', 'Lear', 'Rear', 'pt19']

        self.user_array_pub = rospy.Publisher('users', UserArray, queue_size=10)
        self.marker_pub = rospy.Publisher('marker', Marker, queue_size=10)
        rospy.loginfo('User detector ready')

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
        pad_left = np.tile(img_padded[:, 0:1, :]
                           * 0 + padValue, (1, pad[1], 1))
        img_padded = np.concatenate((pad_left, img_padded), axis=1)
        pad_down = np.tile(img_padded[-2:-1, :, :]
                           * 0 + padValue, (pad[2], 1, 1))
        img_padded = np.concatenate((img_padded, pad_down), axis=0)
        pad_right = np.tile(
            img_padded[:, -2:-1, :] * 0 + padValue, (1, pad[3], 1))
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

        scale = 0.25 * 368.0 / cv_img.shape[0]
        img_scaled = cv2.resize(
            cv_img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

        img_padded, pad = self.padRightDownCorner(
            img_scaled, self.stride, self.padValue)

        self.net.blobs['data'].reshape(
            *(1, 3, img_padded.shape[0], img_padded.shape[1]))
        # self.net.forward() # dry run
        self.net.blobs['data'].data[...] = np.transpose(np.float32(
            img_padded[:, :, :, np.newaxis]), (3, 2, 0, 1)) / 256 - 0.5;
        start_time = time.time()

        output_blobs = self.net.forward()
        rospy.logdebug('The CNN took %.2f ms.' % (1000 * (time.time() - start_time)))

        heatmap = np.transpose(np.squeeze(self.net.blobs[output_blobs.keys()[
                               1]].data), (1, 2, 0))  # output 1 is heatmaps
        heatmap = cv2.resize(
            heatmap,
            (0,
             0),
            fx=self.stride,
            fy=self.stride,
            interpolation=cv2.INTER_CUBIC)
        heatmap = heatmap[:img_padded.shape[0] -
                          pad[2], :img_padded.shape[1] - pad[3], :]
        heatmap = cv2.resize(
            heatmap,
            (cv_img.shape[1],
             cv_img.shape[0]),
            interpolation=cv2.INTER_CUBIC)

        paf = np.transpose(np.squeeze(self.net.blobs[output_blobs.keys()[
                           0]].data), (1, 2, 0))  # output 0 is PAFs
        paf = cv2.resize(paf, (0, 0), fx=self.stride,
                         fy=self.stride, interpolation=cv2.INTER_CUBIC)
        paf = paf[:img_padded.shape[0] - pad[2],
                  :img_padded.shape[1] - pad[3], :]
        paf = cv2.resize(
            paf,
            (cv_img.shape[1],
             cv_img.shape[0]),
            interpolation=cv2.INTER_CUBIC)

        all_peaks = []
        peak_counter = 0

        for part in range(19 - 1):

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

            peaks_binary = np.logical_and.reduce(
                (map >= map_left,
                 map >= map_right,
                 map >= map_up,
                 map >= map_down,
                 map > self.thre1))
            peaks = zip(
                np.nonzero(peaks_binary)[1],
                np.nonzero(peaks_binary)[0])  # note reverse
            peaks_with_score = [x + (map_ori[x[1], x[0]],) for x in peaks]
            id = range(peak_counter, peak_counter + len(peaks))
            peaks_with_score_and_id = [
                peaks_with_score[i] + (id[i],) for i in range(len(id))]

            all_peaks.append(peaks_with_score_and_id)
            peak_counter += len(peaks)

        # find connection in the specified sequence, center 29 is in the
        # position 15
        limbSeq = [[2, 3], [2, 6], [3, 4], [4, 5], [6, 7], [7, 8], [2, 9], [9, 10],
                   [10, 11], [2, 12], [12, 13], [13, 14], [2, 1], [1, 15], [15, 17],
                   [1, 16], [16, 18], [3, 17], [6, 18]]
        # the middle joints heatmap correpondence
        mapIdx = [
            [
                31, 32], [
                39, 40], [
                33, 34], [
                    35, 36], [
                        41, 42], [
                            43, 44], [
                                19, 20], [
                                    21, 22], [
                                        23, 24], [
                                            25, 26], [
                                                27, 28], [
                                                    29, 30], [
                                                        47, 48], [
                                                            49, 50], [
                                                                53, 54], [
                                                                    51, 52], [
                                                                        55, 56], [
                                                                            37, 38], [
                                                                                45, 46]]

        connection_all = []
        special_k = []
        mid_num = 10

        for k in range(len(mapIdx)):
            score_mid = paf[:, :, [x - 19 for x in mapIdx[k]]]
            candA = all_peaks[limbSeq[k][0] - 1]
            candB = all_peaks[limbSeq[k][1] - 1]
            nA = len(candA)
            nB = len(candB)
            indexA, indexB = limbSeq[k]
            if(nA != 0 and nB != 0):
                connection_candidate = []
                for i in range(nA):
                    for j in range(nB):
                        vec = np.subtract(candB[j][:2], candA[i][:2])
                        norm = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])
                        vec = np.divide(vec, norm)

                        startend = zip(
                            np.linspace(
                                candA[i][0], candB[j][0], num=mid_num), np.linspace(
                                candA[i][1], candB[j][1], num=mid_num))

                        vec_x = np.array([score_mid[int(round(startend[I][1])), int(
                            round(startend[I][0])), 0] for I in range(len(startend))])
                        vec_y = np.array([score_mid[int(round(startend[I][1])), int(
                            round(startend[I][0])), 1] for I in range(len(startend))])

                        score_midpts = np.multiply(
                            vec_x, vec[0]) + np.multiply(vec_y, vec[1])
                        score_with_dist_prior = sum(
                            score_midpts) / len(score_midpts) + min(0.5 * cv_img.shape[0] / norm - 1, 0)
                        criterion1 = len(np.nonzero(score_midpts > self.thre2)[
                                         0]) > 0.8 * len(score_midpts)
                        criterion2 = score_with_dist_prior > 0
                        if criterion1 and criterion2:
                            connection_candidate.append(
                                [i, j, score_with_dist_prior, score_with_dist_prior + candA[i][2] + candB[j][2]])

                connection_candidate = sorted(
                    connection_candidate, key=lambda x: x[2], reverse=True)
                connection = np.zeros((0, 5))
                for c in range(len(connection_candidate)):
                    i, j, s = connection_candidate[c][0:3]
                    if(i not in connection[:, 3] and j not in connection[:, 4]):
                        connection = np.vstack(
                            [connection, [candA[i][3], candB[j][3], s, i, j]])
                        if(len(connection) >= min(nA, nB)):
                            break

                connection_all.append(connection)
            else:
                special_k.append(k)
                connection_all.append([])

        # last number in each row is the total parts number of that person
        # the second last number in each row is the score of the overall
        # configuration
        subset = -1 * np.ones((0, 20))
        candidate = np.array(
            [item for sublist in all_peaks for item in sublist])

        for k in range(len(mapIdx)):
            if k not in special_k:
                partAs = connection_all[k][:, 0]
                partBs = connection_all[k][:, 1]
                indexA, indexB = np.array(limbSeq[k]) - 1

                for i in range(len(connection_all[k])):  # = 1:size(temp,1)
                    found = 0
                    subset_idx = [-1, -1]
                    for j in range(len(subset)):  # 1:size(subset,1):
                        if subset[j][indexA] == partAs[i] or subset[j][indexB] == partBs[i]:
                            subset_idx[found] = j
                            found += 1

                    if found == 1:
                        j = subset_idx[0]
                        if(subset[j][indexB] != partBs[i]):
                            subset[j][indexB] = partBs[i]
                            subset[j][-1] += 1
                        subset[j][-2] += candidate[partBs[i].astype(
                            int), 2] + connection_all[k][i][2]
                    elif found == 2:  # if found 2 and disjoint, merge them
                        j1, j2 = subset_idx
                        print "found = 2"
                        membership = ((subset[j1] >= 0).astype(
                            int) + (subset[j2] >= 0).astype(int))[:-2]
                        if len(np.nonzero(membership == 2)[0]) == 0:  # merge
                            subset[j1][:-2] += (subset[j2][:-2] + 1)
                            subset[j1][-2:] += subset[j2][-2:]
                            subset[j1][-2] += connection_all[k][i][2]
                            subset = np.delete(subset, j2, 0)
                        else:  # as like found == 1
                            subset[j1][indexB] = partBs[i]
                            subset[j1][-1] += 1
                            subset[j1][-2] += candidate[partBs[i].astype(
                                int), 2] + connection_all[k][i][2]

                    # if find no partA in the subset, create a new subset
                    elif not found and k < 17:
                        row = -1 * np.ones(20)
                        row[indexA] = partAs[i]
                        row[indexB] = partBs[i]
                        row[-1] = 2
                        row[-2] = sum(candidate[connection_all[k][i,
                                                                  :2].astype(int), 2]) + connection_all[k][i][2]
                        subset = np.vstack([subset, row])

        # delete some rows of subset which has few parts occur
        deleteIdx = []
        for i in range(len(subset)):
            if subset[i][-1] < 4 or subset[i][-2] / subset[i][-1] < 0.4:
                deleteIdx.append(i)
        subset = np.delete(subset, deleteIdx, axis=0)

        ua = UserArray()
        ua.header = image.header

        marker = Marker()
        marker.header = image.header
        marker.id = 0
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.orientation.w = 1.0
        marker.ns = "detected_parts"

        for n in range(len(subset)):

            user = User()
            ua.users.append(user)

            user.score = subset[n][-2]

            for i in range(18):

                idx = int(subset[n][i])

                if idx == -1:
                    rospy.logdebug(self.part_str[i] + " not found")
                    continue
                Y = candidate[idx, 0]
                X = candidate[idx, 1]

                cv2.circle(cv_img,(int(Y),int(X)), 5, (0,255,0), -1)
                cv2.circle(cv_depth,(int(Y),int(X)), 5, (255,255,255), -1)

                pt = list(model.projectPixelTo3dRay((X, Y)))
                pt[:] = [x / pt[2] for x in pt]

                # depth image is noisy - let's make mean of few pixels
                da = []
                depth_win = 6
                for x in range(max(int(X) - depth_win/2, 0), min(int(X) + depth_win/2+1, cv_depth.shape[1] - 1)):
                    for y in range(max(int(Y) - depth_win/2, 0), min(int(Y) + depth_win/2+1, cv_depth.shape[0] - 1)):
                        val = cv_depth[y, x]
                        if val > 0:
                            da.append(cv_depth[y, x] / 1000.0)

                if len(da) == 0:
                    rospy.logdebug("unknown depth for " + self.part_str[i])
                    continue
                d = np.median(da)
                pt[:] = [x * d for x in pt]

                p = Point()
                p.x = pt[0]
                p.y = pt[1]
                p.z = pt[2]

                user.detected_parts.append(i)
                marker.points.append(p)
                user.parts[i] = p

        # cv2.imshow('image', cv_img)
        # cv2.imshow('depth', cv_depth)
        # cv2.waitKey(25)

        self.user_array_pub.publish(ua)
        self.marker_pub.publish(marker)
        rospy.logdebug('Whole callback took %.2f ms.' % (1000 * (time.time() - start_time)))


if __name__ == '__main__':

    try:

        rospy.init_node('art_user_tracking')
        UserDetector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
