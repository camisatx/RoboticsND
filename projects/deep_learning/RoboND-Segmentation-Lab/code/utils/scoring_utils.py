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

import numpy as np 
import glob
import os
from scipy import misc

def intersection_over_union(y_true, y_pred):
    """Computes the intersection over union of to arrays containing 1's and 0's

    Assumes y_true has converted from real value to binary values. 
    """

    if np.sum(y_true == 1) + np.sum(y_true == 0) != y_true.shape[0]*y_true.shape[1]:
        raise ValueError('Groud truth mask must only contain values from the set {0,1}')

    if np.sum(y_pred == 1) + np.sum(y_pred == 0) != y_pred.shape[0]*y_pred.shape[1]:
        raise ValueError('Segmentation mask must only contain values from the set {0,1}')

    if y_true.ndim != 2:
        if y_true.shape[2] != 1 or y_true.shape[2] != 0:
            raise ValueError('Too many ground truth masks are present')

    if y_pred.ndim != 2:
        if y_pred.shape[2] != 1 or y_pred.shape[2] != 0:
            raise ValueError('too many segmentation masks are present')

    if y_pred.shape != y_true.shape:
        raise ValueError('The dimensions of y_true, and y_pred are not the same')

    intersection = np.sum(y_true * y_pred).astype(np.float)
    union = np.sum(np.clip(y_true + y_pred, 0, 1)).astype(np.float)

    # Alternatively we can return some small value epsilon
    if union == 0:
        # return 1e-10
        return 0

    else:
        return intersection/union # + 1e-10


def score_run(gt_dir, pred_dir):
    gt_files = sorted(glob.glob(os.path.join(gt_dir, 'masks', '*.png')))
    pred_files = sorted(glob.glob(os.path.join(pred_dir, '*.png')))
    ious = [0,0,0]
    n_preds = len(gt_files)

    for e, gt_file in enumerate(gt_files):
        gt_mask = misc.imread(gt_file).clip(0, 1)
        pred_mask = (misc.imread(pred_files[e]) > 127).astype(np.int)

        if gt_mask.shape[0] != pred_mask.shape[0]:
            gt_mask = misc.imresize(gt_mask, pred_mask.shape)

        for i in range(3):
            ious[i] += intersection_over_union(pred_mask[:,:,i], gt_mask[:,:,i])

    background = ious[0] / n_preds
    people = ious[1] / n_preds
    hero = ious[2] / n_preds

    average = (background + hero + people)/3
    print('number of validation samples intersection over the union evaulated on {}'.format(n_preds))
    print('average intersection over union for background is {}'.format(background))
    print('average intersection over union for other people is {}'.format(people))
    print('average intersection over union for hero is {}'.format(hero))

    print('global average intersection over union is {}'.format(average))
