# Copyright (c) 2017, Udacity
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# The views and conclusions contained in the software and documentation are those
# of the authors and should not be interpreted as representing official policies,
# either expressed or implied, of the FreeBSD Project

# Author: Devin Anzelmo

import os
import json
from tensorflow.contrib.keras.python import keras 
from scipy import misc
from . import data_iterator
import numpy as np
import glob

def make_dir_if_not_exist(path):
    if not os.path.exists(path):
        os.makedirs(path)


def save_network(your_model, your_weight_filename):
    config_name = 'config' + '_' + your_weight_filename
    weight_path = os.path.join('..', 'data', 'weights', your_weight_filename)
    config_path = os.path.join('..', 'data', 'weights', config_name)
    your_model_json = your_model.to_json()
    
    with open(config_path, 'w') as file:
        json.dump(your_model_json, file)  
        
    your_model.save_weights(weight_path) 
        
        
def load_network(your_weight_filename):
    config_name = 'config' + '_' + your_weight_filename
    weight_path = os.path.join('..', 'data', 'weights', your_weight_filename)
    config_path = os.path.join('..', 'data', 'weights', config_name)
    
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            json_string = json.load(file)  
            
        your_model = keras.models.model_from_json(json_string)
        
    else:
        raise ValueError('No config_yourmodel file found at {}'.format(config_path))
        
    if os.path.exists(weight_path):
        your_model.load_weights(weight_path)
        return your_model
    else:
        raise ValueError('No weight file found at {}'.format(weight_path))


def write_predictions_grade_set(model, run_number, validation_folder):
    
    validation_path = os.path.join('..', 'data', validation_folder)
    file_names = sorted(glob.glob(os.path.join(validation_path, 'images', '*.jpeg')))

    output_path = os.path.join('..', 'data', 'runs', run_number)
    make_dir_if_not_exist(output_path)
    image_shape = model.layers[0].output_shape[1]

    for name in file_names:
        image = misc.imread(name)
        if image.shape[0] != image_shape:
             image = misc.imresize(image, (image_shape, image_shape, 3))
        image = data_iterator.preprocess_input(image.astype(np.float32))
        pred = model.predict_on_batch(np.expand_dims(image, 0))
        base_name = os.path.basename(name).split('.')[0]
        base_name = base_name + '_prediction.png'
        misc.imsave(os.path.join(output_path, base_name), np.squeeze((pred * 255).astype(np.uint8)))

    return validation_path, output_path
