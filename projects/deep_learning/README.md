[//]: # (Image References)

# Deep Learning Project (Follow Me)
---

The deep learning project, also known as the Follow Me project, uses a fully convolutional network (FCN) to build a model for identifying a target person from a simulated drone camera feed. The purpose of the model is to have the drone able to follow the target person as they walk around, while ignoring other non-target people.

The model is built within Tensorflow and Keras, and was trained using a K80 GPU enabled AWS compute instance.

To learn more about running the training code, view the original Udacity project repo [here](https://github.com/udacity/RoboND-DeepLearning-Project).

## Contents

- [Architecture](#architecture)
  - [Fully Convolutional Networks](#fully-convolutional-networks)
  - [Model Used](#model-used)
- [Hyperparameters](#hyperparmeters)
- [Training](#training)
- [Performance](#performance)
- [Future Uses](#future-uses)

## Architecture

A fully convolutional network (FCN) was used in this project because the model required both the performance of a CNN and the preservation of spatial information throughout the network.

### Fully Convolutional Networks

CNNs and FCNs both have an encoder section compromised of regular convolutions, however, instead of a final fully connected layer, FCNs have a decoder section that include a 1x1 convolution layer and one or more bilinear upsampling layer(s).

#### 1x1 Convolution Layer

By using a 1x1 convolution layer, the network is able to retain spatial information from the encoder. When using a fully connected layer, the data is flattened, retaining only 2 dimensions of information (as opposed to 4 of the 1x1 convolution layer).

The 1x1 convolution layer is simply a regular convolution, with a kernel and stride of 1.

#### Decoder

The decoder section of the model could either be composed of transposed convolution layers or bilinear upsampling layers. The transposed convolution layers essential reverse the regular convolution layers, multiplying each pixel of the input with the kernel. Bilinear upsampling uses the weighted average of the four nearest known pixels from the given pixel, estimating a new pixel intensity value. Although bilinear upsampling loses some finer details when compared to transposed convolutions, it has much better performance, which is essential for this model.

#### Skip Connections

Skip connections are also used, allowing the network to retain information from prior layers that was lost in subsequent layers. Skip layers use the output of one layer as the input to another layer. By using information from multiple image sizes, the model is able to make more precise segmentation decisions.

### Model Used

TODO: Add image of the model

TODO: Describe each layer of the model

## Hyperparameters

TODO: Describe each hyperparameter and how each was determined

## Training

TODO: Describe what infrastructure was used to train the model

## Performance

TODO: Describe the performance of the model

## Model Uses

TODO: Explain if this model could be used to detect other objects, and what would be required for that to happen
