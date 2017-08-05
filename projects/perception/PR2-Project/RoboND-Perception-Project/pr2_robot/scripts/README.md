## Capturing Point Cloud Features

To capture the point cloud features, I used the sensor_stick model to analyze and record each object of the PR2 project. In order to do this, I copied the models folder from the PR2 project into the models folder of the sensor stick folder (the one used for Exercise 3). Once the models were stored there, I just had to alter the model names in the `capture_features.py` file in the `sensor_stick/scripts` folder to match the PR2 model names. I saved this file under the name `capture_features_pr2.py` in order to preserve the existing Exercise 3 code.

With the file prepared, the next step is to launch the Gazebo environment:

```bash
roslaunch sensor_stick training.launch
```

Then the `capture_features_pr2.py` script could be run:

```bash
rosrun sensor_stick capture_features_pr2.py
```

This script will cycle through each model, orienting it in random directions, and capturing point cloud features for it. The final features will be saved in `training_set.sav` (I saved my features in `training_set_pr2.sav`).

## Training the SVM

With the point cloud features saved, the support vector machine can now be trained. The same script used in Exercise 3 can be used in this project.

The training can occur in any file location, as long as the `train_svm.py` and `training_set_pr2.sav` are in the same directory.

To start the training, run:

```bash
python train_svm.py
```

This will train the SVM and save the final model to `model.sav`.

You can modify the classifier parameters in this file, including changing the kernel type and C value used.

I am able to achieve an accuracy of 92% (+/- 27%), which is sufficient for the purpose here. This used a linear kernel in the SVM with a C value of 0.1.

## PR2 Object Processing

The PR2 robot is able to operate with the `model.sav` file.
