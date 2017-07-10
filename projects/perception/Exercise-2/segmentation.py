#!/usr/bin/env python

import pcl

from pcl_helper import *


def pcl_callback(pcl_msg):
    # Callback function for your Point Cloud Subscriber

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter to isolate the table and objects
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    axis_min = 0.75
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation to identify the table
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)  # kdtree only needs XYZ
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    cluster_tolerance = 0.01
    min_cluster_size = 20
    max_cluster_size = 300
    ec.set_ClusterTolerance(cluster_tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(max_cluster_size)
    ec.set_SearchMethod(tree)       # Search the k-d tree for clusters
    # Extract indices for each found cluster. This is a list of indices for
    #   each cluster (list of lists)
    cluster_indices = ec.Extract()

    # Assign a color corresponding to each segmented object in the scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])


    # Create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_object_cluster = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # Publish ROS messages of the final object and table point clouds
    pcl_objects_cloud_pub.publish(ros_cloud_object_cluster)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)


if __name__ == '__main__':

    # Initialization ROS node called 'clustering'
    rospy.init_node('clustering', anonymous=True)

    # Create a subscriber to receive the published data coming from the
    #   pcl_callback() function that will be processing the point clouds
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # Create publishers to publish the point cloud data from the table and the
    #   objects on the table
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_objects_cloud_pub = rospy.Publisher('/pcl_objects_cloud', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
