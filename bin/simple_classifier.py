#! /usr/bin/env python2

import rospy
import numpy as np

class SimpleClassifier(object):

    def classify(self, data):
        n_nodes = data.shape[0]
        labels = []
        for i in range(0, n_nodes):
            low_mean = data[i,0]
            mid_mean = data[i,1]
            high_mean = data[i,2]
            dists = np.array([low_mean, mid_mean, high_mean])
            max_dist_idx = np.argmax(dists)

            np.set_printoptions(suppress=True)
            local_labels = []
            # h floor
            # if dists[0] > 0.4:
            #     local_labels.append(1)
            # if dists[1] > 0.11: # for h_naymal_2 we had 0.2
            #     local_labels.append(2)
            # if dists[2] > 0.05:
            #     local_labels.append(3)

            # if dists[0] > 0.5:
            #     local_labels.append(1)
            # if dists[1] > 0.6: # for h_naymal_2 we had 0.2
            #     local_labels.append(2)
            # if dists[2] > 0.7:
            #     local_labels.append(3)

            # -----------------------------------------------

            # hagerbach anymal 2
            # if dists[0] > 0.31:
            #     local_labels.append(1)
            # if dists[1] > 0.21: # for h_naymal_2 we had 0.2
            #     local_labels.append(2)
            # if dists[2] > 0.037:
            #     local_labels.append(3)

            # hagerbach anymal 1
            # if dists[0] > 2.96:
            #     local_labels.append(1)
            # if dists[1] > 0.80: # for h_naymal_2 we had 0.2
            #     local_labels.append(2)
            # if dists[2] > 0.025:
            #     local_labels.append(3)

            # h floor
            # if dists[0] > 0.31:
            #     local_labels.append(1)
            # if dists[1] > 0.21: # for h_naymal_2 we had 0.2
            #     local_labels.append(2)
            # if dists[2] > 0.07:
            #     local_labels.append(3)

            # EuRoC
            if dists[0] > 0.2:
                local_labels.append(1)
            if dists[1] > 0.1:
                local_labels.append(2)
            if dists[2] > 0.5:
                local_labels.append(3)

            # if len(local_labels) > 0:
            rospy.loginfo('[WaveletEvaluator] dists are {dists}'.format(dists=dists))
            rospy.loginfo('local labels are {local_labels}'.format(local_labels=local_labels))
            labels.append(local_labels)
        return labels
