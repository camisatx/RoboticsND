import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

   
def color_hist(img, nbins=32, bins_range=(0, 256)):
    """Define a function to compute color histogram features

    :param img: matplotlib image object
    :param nbins: Integer of the number of histogram bins
    :param bins_range: Tuple of 2 integers of the lower and upper bin range,
        representing the color range
    :return: Feature vector
    """

    # Convert from RGB to HSV using cv2.cvtColor()
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Compute the histogram for each HSV channel
    h_hist = np.histogram(hsv_img[:,:,0], bins=nbins, range=bins_range)
    s_hist = np.histogram(hsv_img[:,:,1], bins=nbins, range=bins_range)
    v_hist = np.histogram(hsv_img[:,:,2], bins=nbins, range=bins_range)

    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)

    # Normalize the historgram features
    norm_features = hist_features / np.sum(hist_features)

    return norm_features
    

if __name__ == '__main__':

    # Read in an image
    image = mpimg.imread('Udacican.jpeg')
    # Your other options for input images are:
        # hammer.jpeg
        # beer.jpeg
        # bowl.jpeg
        # create.jpeg
        # disk_part.jpeg
 
    feature_vec = color_hist(image, nbins=32, bins_range=(0, 256))
    
    # Plot a figure with all three bar charts
    if feature_vec is not None:
        fig = plt.figure(figsize=(12,6))
        plt.plot(feature_vec)
        plt.title('HSV Feature Vector', fontsize=30)
        plt.tick_params(axis='both', which='major', labelsize=20)
        fig.tight_layout()
    else:
        print('Your function is returning None...')
