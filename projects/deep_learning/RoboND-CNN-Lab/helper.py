import pickle
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import LabelBinarizer


def _load_label_names():
    """
    Load the label names from file
    """
    return ['t-shirt', 'trouser', 'pullover', 'dress', 'coat', 'sandal', 'shirt', 'sneaker', 'bag', 'ankle_boot']


def load_dataset(dataset_folder_path):
    """
    Load the training and test datasets
    """
    with open(dataset_folder_path, mode='rb') as file:
        pickle_data = pickle.load(file)
    
    train_features = pickle_data[0].reshape((len(pickle_data[0]), 1, 28, 28)).transpose(0, 2, 3, 1)
    train_labels = pickle_data[1]
    test_features = pickle_data[2].reshape((len(pickle_data[2]), 1, 28, 28)).transpose(0, 2, 3, 1)
    test_labels = pickle_data[3]   

    return train_features, train_labels, test_features, test_labels


def display_stats(dataset_folder_path, sample_id):
    """
    Display Stats of the dataset
    """

    train_features, train_labels, test_features, test_labels = load_dataset(dataset_folder_path)

    if not (0 <= sample_id < len(train_features)):
        print('{} samples in training set.  {} is out of range.'.format(len(train_features), sample_id))
        return None

    print('Samples: {}'.format(len(train_features)))
    print('Label Counts: {}'.format(dict(zip(*np.unique(train_labels, return_counts=True)))))
    print('First 20 Labels: {}'.format(train_labels[:20]))

    sample_image = train_features[sample_id]
    sample_label = train_labels[sample_id]
    label_names = _load_label_names()

    print('\nExample of Image {}:'.format(sample_id))
    print('Image - Min Value: {} Max Value: {}'.format(sample_image.min(), sample_image.max()))
    print('Image - Shape: {}'.format(sample_image.shape))
    print('Label - Label Id: {} Name: {}'.format(sample_label, label_names[sample_label]))
    plt.axis('off')
    plt.imshow(sample_image.squeeze(), cmap = "gray")


def _preprocess_and_save(normalize, one_hot_encode, features, labels, filename):
    """
    Preprocess data and save it to file
    """
    features = normalize(features)
    labels = one_hot_encode(labels)

    pickle.dump((features, labels), open(filename, 'wb'))


def preprocess_and_save_data(dataset_folder_path, normalize, one_hot_encode):
    """
    Preprocess Training and Validation Data
    """

    valid_features = []
    valid_labels = []

    train_features, train_labels, test_features, test_labels = load_dataset(dataset_folder_path)
    validation_count = int(len(train_features) * 0.1)

    # Preprocess and save new training data
    _preprocess_and_save(
        normalize,
        one_hot_encode,
        train_features[:-validation_count],
        train_labels[:-validation_count],
        'preprocess_train' + '.p')

    # Use a portion of training batch for validation
    valid_features.extend(train_features[-validation_count:])
    valid_labels.extend(train_labels[-validation_count:])

    # Preprocess and Save all validation data
    _preprocess_and_save(
        normalize,
        one_hot_encode,
        np.array(valid_features),
        np.array(valid_labels),
        'preprocess_validation.p')


    # Preprocess and Save all test data
    _preprocess_and_save(
        normalize,
        one_hot_encode,
        np.array(test_features),
        np.array(test_labels),
        'preprocess_test.p')


def batch_features_labels(features, labels, batch_size):
    """
    Split features and labels into batches
    """
    for start in range(0, len(features), batch_size):
        end = min(start + batch_size, len(features))
        yield features[start:end], labels[start:end]


def load_preprocess_training_batch(batch_size):
    """
    Load the Preprocessed Training data and return them in batches of <batch_size> or less
    """
    filename = 'preprocess_train' + '.p'
    features, labels = pickle.load(open(filename, mode='rb'))

    # Return the training data in batches of size <batch_size> or less
    return batch_features_labels(features, labels, batch_size)


def display_image_predictions(features, labels, predictions):
    n_classes = 10
    label_names = _load_label_names()
    label_binarizer = LabelBinarizer()
    label_binarizer.fit(range(n_classes))
    label_ids = label_binarizer.inverse_transform(np.array(labels))

    fig, axies = plt.subplots(nrows=4, ncols=2)
    fig.tight_layout()
    fig.suptitle('Softmax Predictions', fontsize=20, y=1.1)

    n_predictions = 3
    margin = 0.05
    ind = np.arange(n_predictions)
    width = (1. - 2. * margin) / n_predictions

    for image_i, (feature, label_id, pred_indicies, pred_values) in enumerate(zip(features, label_ids, predictions.indices, predictions.values)):
        pred_names = [label_names[pred_i] for pred_i in pred_indicies]
        correct_name = label_names[label_id]

        axies[image_i][0].imshow(feature.squeeze(), cmap='gray')
        axies[image_i][0].set_title(correct_name)
        axies[image_i][0].set_axis_off()

        axies[image_i][1].barh(ind + margin, pred_values[::-1], width)
        axies[image_i][1].set_yticks(ind + margin)
        axies[image_i][1].set_yticklabels(pred_names[::-1])
        axies[image_i][1].set_xticks([0, 0.5, 1.0])
