#!/usr/bin/env python

import pickle
import yaml

import numpy as np
import rospy
import sklearn
from sklearn.preprocessing import LabelEncoder
import tf

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from pr2_robot.srv import *
from rospy_message_converter import message_converter
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *


def get_normals(cloud):
    """Helper function to get surface normals"""
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals',
                                          GetNormals)
    return get_normals_prox(cloud).cluster


def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    """Helper function to create a yaml friendly dictionary from ROS messages

    :param test_scene_num: ROS message of the current scene number
    :param arm_name: ROS message holding the arm name
    :param object_name: ROS message holding the object name
    :param pick_pose: ROS message of the pick pose dictionary
    :param place_pose: ROS message of the place pose dictionary
    :return: yaml dictionary
    """

    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = \
            message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = \
            message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


def send_to_yaml(yaml_filename, dict_list):
    """Helper function to output the provided dictionary to a yaml file

    :param yaml_filename: String of the name to save the yaml file as
    :param dict_list: Dictionary to save into the yaml file
    """
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


def pcl_callback(pcl_msg):
    """Callback function for the Point Cloud Subscriber

    :param pcl_msg: ROS point cloud message
    """

    # Convert ROS msg to PCL data (XYZRGB)
    cloud = ros_to_pcl(pcl_msg)

    # Statistical outlier filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)
    # Any point with a mean distance larger than global will be considered out
    outlier_filter.set_std_dev_mul_thresh(0.5)
    cloud_filtered = outlier_filter.filter()

    # Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter to remove the areas on the side of the table
    passthrough_y = cloud_filtered.make_passthrough_filter()
    passthrough_y.set_filter_field_name('y')
    y_axis_min = -0.4
    y_axis_max = 0.4
    passthrough_y.set_filter_limits(y_axis_min, y_axis_max)
    cloud_filtered = passthrough_y.filter()

    # PassThrough Filter to isolate only the objects on the table surface
    passthrough_z = cloud_filtered.make_passthrough_filter()
    passthrough_z.set_filter_field_name('z')
    z_axis_min = 0.6
    z_axis_max = 0.9
    passthrough_z.set_filter_limits(z_axis_min, z_axis_max)
    cloud_filtered = passthrough_z.filter()

    # RANSAC Plane Segmentation to identify the table from the objects
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers (table surface) and outliers (objects)
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Convert the xyzrgb cloud to only xyz since k-d tree only needs xyz
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    # k-d tree decreases the computation of searching for neighboring points.
    #   PCL's euclidian clustering algo only supports k-d trees
    tree = white_cloud.make_kdtree()

    # Euclidean clustering used to build individual point clouds for each
    #   object. The Cluster-Mask point cloud allows each cluster to be
    #   visualized separately.
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(3000)
    ec.set_SearchMethod(tree)       # Search the k-d tree for clusters
    # Extract indices for each found cluster. This is a list of indices for
    #   each cluster (list of lists)
    cluster_indices = ec.Extract()

    # Assign a random color to each isolated object in the scene
    cluster_color = get_color_list(len(cluster_indices))
        
    # Store the detected objects and labels in these lists
    detected_objects_labels = []
    detected_objects = []
    color_cluster_point_list = []

    # Iterate through each detected object cluster for object recognition
    for index, pts_list in enumerate(cluster_indices):
        
        # Store the object's cloud in this list
        object_cluster = []

        # Create an individual cluster just for the object being processed
        for i, pts in enumerate(pts_list):
            # Retrieve cloud values for the x, y, z, rgb object
            object_cluster.append([cloud_objects[pts][0],
                                   cloud_objects[pts][1],
                                   cloud_objects[pts][2],
                                   cloud_objects[pts][3]])
            
            # Retrieve cloud values for the x, y, z object, assigning a
            #   preidentified color to all cloud values
            color_cluster_point_list.append([white_cloud[pts][0],
                                             white_cloud[pts][1],
                                             white_cloud[pts][2],
                                             rgb_to_float(cluster_color[index])])


        # Convert list of point cloud features (x,y,z,rgb) into a point cloud
        pcl_cluster = pcl.PointCloud_PointXYZRGB()
        pcl_cluster.from_list(object_cluster)

        # Convert the cluster from pcl to ROS using helper function
        ros_cloud = pcl_to_ros(pcl_cluster)
        #pcl_objects_pub.publish(ros_cloud)

        # Extract histogram features (similar to capture_features.py)
        histogram_bins = 128
        chists = compute_color_histograms(ros_cloud,
                                          nbins=histogram_bins,
                                          using_hsv=True)
        normals = get_normals(ros_cloud)
        nhists = compute_normal_histograms(normals,
                                           nbins=histogram_bins)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result and add it
        #   to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    
    # Create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_object_cluster = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # Publish ROS messages of the point clouds and detected objects
    pcl_objects_cloud_pub.publish(ros_cloud_object_cluster) # solid color objects
    pcl_objects_pub.publish(ros_cloud_objects)      # original color objects
    pcl_table_pub.publish(ros_cloud_table)          # table cloud
    detected_objects_pub.publish(detected_objects)  # detected object labels

    try:
        # Add some logic to determine whether or not your object detections
        #   are robust enough before calling pr2_mover()
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass


def pr2_mover(object_list):
    """Cycle through each object provided in the object list, moving the
    correct arm to pickup the object.
    
    Function to load parameters and request PickPlace service

    :param object_list: List of detected objects
    """

    # Initialize variables
    labels = []
    centroids = []      # List of tuples (x, y, z)

    # Load the parameters from the object list
    object_list_param = rospy.get_param('/object_list')

    # Parse parameters into individual variables
    object_name = object_list_param[i]['name']
    object_group = object_list_param[i]['group']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the object pick list
    for object_val in object_list:

        labels.append(object_val.label)

        # Get the PointCloud for the object and obtain it's centroid (the
        #   average position of all points in the object cloud). Convert the
        #   cloud to an array, then calculate the average of the array.
        points_array = ros_to_pcl(object_val.cloud).to_array()
        centroids.append(np.asscalar(np.mean(points_array, axis=0)[:3]))

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':
    
    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscriber to receive the published data coming from the
    #   pcl_callback() function that will be processing the point clouds
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',
                                           DetectedObjectsArray,
                                           queue_size=1)
    # Isolated object point cloud with the object's original colors
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    # Isolated object point cloud with random colors
    pcl_objects_cloud_pub = rospy.Publisher('/pcl_objects_cloud', PointCloud2,
                                            queue_size=1)
    # Table point cloud without the objects
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # Load model from disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
