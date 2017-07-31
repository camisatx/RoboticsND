import cv2
import glob
import matplotlib.image as mpimg
#import matplotlib.pyplot as plt
import numpy as np
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
import time
try:
    from sklearn.model_selection import train_test_split    # sklearn >=0.18
except ImportError:
    from sklearn.cross_validation import train_test_split   # sklearn <=0.17


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


def extract_features(imgs, hist_bins=32, hist_range=(0, 256)):
    """Extract features from a list of images
    
    :param imgs: List of image paths
    :param hist_bins: Integer of the number of histogram bins
    :param hist_range: Tuple of 2 integers of the lower and upper bin range,
        representing the color range
    :return: List of feature vectors
    """

    # Create a list to append feature vectors to
    features = []

    # Iterate through the list of images
    for img_path in imgs:
        # Read in the image
        image = mpimg.imread(img_path)
        # Use color_hist() to get the image's feature vector
        hist_features = color_hist(image, nbins=hist_bins, bins_range=hist_range)
        # Append the image's feature vector to the features list
        features.append(hist_features)
    return features


if __name__ == '__main__':

    # Read in car and non-car images
    images = glob.glob('*.jpeg')
    cars = []
    notcars = []
    for image in images:
        if 'image' in image or 'extra' in image:
            notcars.append(image)
        else:
            cars.append(image)
    
    # TODO: play with these values to see how your classifier performs under
    #    different binning scenarios
    histbin = 32
    
    car_features = extract_features(cars, hist_bins=histbin, hist_range=(0, 256))
    notcar_features = extract_features(notcars, hist_bins=histbin, hist_range=(0, 256))
    
    # Create an array stack of feature vectors; each image will be its own row
    X = np.vstack((car_features, notcar_features)).astype(np.float64)
    
    # Perform per-column normalization by scaling to zero mean and unit variance
    X_scaler = StandardScaler().fit(X)  # Fit a per-column scaler
    scaled_X = X_scaler.transform(X)    # Apply the scaler to X
    
    # Define the labels vector; 1 for cars and 0 for non cars
    y = np.hstack((np.ones(len(car_features)), np.zeros(len(notcar_features))))
    
    # Split up data into randomized training and test sets
    X_train, X_test, y_train, y_test = train_test_split(
        scaled_X, y, test_size=0.2, random_state=20)
    
    print('Dataset includes %i cars and %i non-cars' % (len(cars), len(notcars)))
    print('Using %i histogram bins' % histbin)
    print('Feature vector length: %i' % len(X_train[0]))

    # Use a linear SVC 
    svc = SVC(kernel='linear', C=0.1)
    # Check the training time for the SVC
    t=time.time()
    svc.fit(X_train, y_train)
    t2 = time.time()
    print('%0.4f seconds to train SVC...' % (t2 - t,))

    # Check the score of the SVC
    print('Test Accuracy of SVC = %0.4f' % (svc.score(X_test, y_test),))
    print('')

    # Check the prediction time for a single sample
    t=time.time()
    n_predict = 10
    print('My SVC predicts:\t%s' % svc.predict(X_test[0:n_predict]))
    print('For these %i labels:\t%s' % (n_predict, y_test[0:n_predict]))
    t2 = time.time()
    print('%0.4f seconds to predict %i labels with SVC' % (t2-t, n_predict))
