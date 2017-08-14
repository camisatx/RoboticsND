import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    """Converert RGB list of values to HSV values

    :param rgb_list: List of 3 RGB values
    :return: HSV values
    """

    rgb_normalized = [1.0 * rgb_list[0] / 255,
                      1.0 * rgb_list[1] / 255,
                      1.0 * rgb_list[2] / 255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


def create_hist(ch1, ch2, ch3, nbins=32, bins_range=(0, 256)):
    """Create normalized histogram features from the 3 image channels provided.

    :param ch1: List of channel 1 values
    :param ch2: List of channel 2 values
    :param ch3: List of channel 3 values
    :param nbins: Integer of the number of histogram bins
    :param bins_range: Tuple of 2 integers of the lower and upper bin range,
        representing the color range
    :return: Feature vector
    """
    
    # Compute the histogram for each channel
    ch1_hist = np.histogram(ch1, bins=nbins, range=bins_range)
    ch2_hist = np.histogram(ch2, bins=nbins, range=bins_range)
    ch3_hist = np.histogram(ch3, bins=nbins, range=bins_range)

    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((ch1_hist[0], ch2_hist[0], ch3_hist[0])).astype(np.float64)

    # Normalize the historgram features
    norm_features = hist_features / np.sum(hist_features)

    return norm_features


def compute_color_histograms(cloud, nbins=32, using_hsv=False):
    """Compute the normalized color historgram features from the provided
    point cloud, with the option of converting the RGB data to HSV values.

    :param cloud: Point cloud
    :param nbins: Integer of the number of histogram bins
    :param using_hsv: Optional boolean of whether to convert RGB values to HSV
    :return: Normalized historgram features
    """

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])

    # Create the normalized histogram features based on the channels
    normed_features = create_hist(ch1=channel_1_vals,
                                  ch2=channel_2_vals,
                                  ch3=channel_3_vals,
                                  nbins=nbins)
    
    return normed_features 


def compute_normal_histograms(normal_cloud, nbins=32):
    """Compute the normalized histogram feattures from the provided point cloud

    :param normal_cloud: Point cloud
    :param nbins: Integer of the number of histogram bins
    :return: Normalized histogram features
    """

    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(
            normal_cloud, field_names = ('normal_x', 'normal_y', 'normal_z'),
            skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])
    
    # Create the normalized histogram features based on the channels
    normed_features = create_hist(ch1=norm_x_vals,
                                  ch2=norm_y_vals,
                                  ch3=norm_z_vals,
                                  nbins=nbins)
    
    return normed_features
