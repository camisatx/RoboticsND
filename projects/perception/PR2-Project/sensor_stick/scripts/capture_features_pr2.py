#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals',
                                          GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [
       'biscuits',
       'book',
       'eraser',
       'glue',
       'snacks',
       'soap',
       'soap2',
       'soda_can',
       'sticky_notes',
       ]
 
    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    capture_attempts = 50
    histogram_bins = 128

    for model_name in models:
        spawn_model(model_name)

        for i in range(capture_attempts):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                # Capture the point cloud using the sensor stock RGBD camera
                sample_cloud = capture_sample()
                # Convert the ros cloud to a pcl cloud
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud,
                                              nbins=histogram_bins,
                                              using_hsv=True)

            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals,
                                               nbins=histogram_bins)

            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

        delete_model()

    pickle.dump(labeled_features, open('training_set_pr2.sav', 'wb'))
