#!/usr/bin/env python3
"""
Scripts to train a keras model using tensorflow.
Basic usage should feel familiar: train.py --model models/mypilot

Usage:
    train.py [--tubs=tubs] (--model=<model>)
             [--type=(linear|inferred|latent|tensorrt_linear|tflite_linear)]
             [--latent_trained=<model>]

Options:
    -h --help              Show this screen.
"""
import datetime
import os
from enum import Enum
import time
from pathlib import Path
from contextlib import AbstractContextManager

import cv2
import numpy as np
from docopt import docopt
from PIL import Image
from tensorflow.python.keras.callbacks import EarlyStopping, ModelCheckpoint
from tensorflow.python.keras.utils.data_utils import Sequence

import donkeycar
from donkeycar.parts.keras import KerasLinear, KerasInferred, \
    KerasCategorical, KerasLatent
from donkeycar.parts.tflite import keras_model_to_tflite
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.augumentations import Augumentations
from donkeycar.utils import get_model_by_type, load_image_arr, \
    train_test_split, linear_bin, normalize_image


class TrainState(Enum):
    """ Helper class to disentangle different data manipulations required in
        training """
    LINEAR = 0
    CATEGORICAL = 1
    INFERRED = 2
    LATENT_DECODER = 3
    LATENT_CONTROLLER = 4

    @staticmethod
    def create(model):
        if type(model) is KerasLinear:
            return TrainState.LINEAR
        elif type(model) is KerasCategorical:
            return TrainState.CATEGORICAL
        elif type(model) is KerasInferred:
            return TrainState.INFERRED
        elif type(model) is KerasLatent:
            if model.train_mode == 'controller':
                return TrainState.LATENT_CONTROLLER
            else:
                return TrainState.LATENT_DECODER

        else:
            raise ValueError(f'Non-trainable model chosen: {type(model)}')


class TubDataset(object):
    '''
    Loads the dataset, and creates a train/test split.
    '''

    def __init__(self, tub_paths, test_size=0.2, shuffle=True, skip=[],
                 include_only=[]):
        assert type(tub_paths) is list, \
            "TubDataset expects list in first argument"
        self.tub_paths = tub_paths
        self.test_size = test_size
        self.shuffle = shuffle
        self.tubs = []
        self.del_index = []
        for tub_path in self.tub_paths:
            print('Adding tub', tub_path, 'to dataset')
            tub = Tub(tub_path)
            tub.skip_or_include(skip, True)
            tub.skip_or_include(include_only, False)
            self.tubs.append(tub)

        self.records = list()

    def train_test_split(self):
        count = 0
        for tub in self.tubs:
            for record in tub:
                record['_image_base_path'] = tub.images_base_path
                self.records.append(record)
                count += 1
        print(f'Loading tubs from paths {self.tub_paths} with {count} records')
        return train_test_split(self.records, shuffle=self.shuffle,
                                test_size=self.test_size)

    def restore_tubs(self):
        for t in self.tubs:
            t.clear_skip_or_include(True)
            t.clear_skip_or_include(False)


class TubSequence(Sequence):
    def __init__(self, keras_model, config, records=list()):
        self.keras_model = keras_model
        self.config = config
        self.records = records
        self.batch_size = self.config.BATCH_SIZE
        self.train_state = TrainState.create(keras_model)
        print('Building TubSequence', end='')
        self.aug_in = None
        if getattr(self.config, 'IMG_AUG', False):
            self.aug_in = Augumentations.brightness(rand_lower=0.4,
                                                    rand_upper=2.5)
            print(' with input augmentation', end='')
        self.aug_out = None
        if getattr(self.config, 'IMG_BRIGHTNESS_NORM', False):
            self.aug_out = Augumentations.brightness()
            print(' with output normalisation', end='')
        print()
        if self.train_state == TrainState.LATENT_CONTROLLER and self.aug_in \
                is not None and self.aug_out is not None:
            raise ValueError('Training controller cannot have output and '
                             'input augmentation as this erases original '
                             'images.')

    def __len__(self):
        return len(self.records) // self.batch_size

    def __getitem__(self, index):
        count = 0
        records = []
        images = []
        images_aug_in = None
        images_aug_out = None
        latent_vectors = []
        angles = []
        throttles = []

        while count < self.batch_size:
            i = (index * self.batch_size) + count
            if i >= len(self.records):
                break

            record = self.records[i]
            record = self._transform_record(record)
            records.append(record)
            count += 1

        for record in records:
            image = record['cam/image_array']
            images.append(image)
            angle = record['user/angle']
            angles.append(angle)
            throttle = record['user/throttle']
            throttles.append(throttle)
            if self.train_state == TrainState.LATENT_CONTROLLER:
                latent_vector = record['img/latent']
                latent_vectors.append(latent_vector)

        if self.aug_in:
            images_aug_in = np.array(self.aug_in(images=images))

        if self.aug_out:
            images_aug_out = np.array(self.aug_out(images=images))

        # fill X
        if self.train_state == TrainState.LATENT_CONTROLLER:
            X = np.array(latent_vectors)
        elif self.aug_in:
            X = normalize_image(images_aug_in)
        else:
            X = normalize_image(np.array(images))

        # fill Y
        if self.train_state == TrainState.INFERRED:
            Y = np.array(angles)
        else:
            Y = [np.array(angles), np.array(throttles)]
            if self.train_state == TrainState.LATENT_DECODER:
                # use brightness normalisation on the output
                if self.aug_out:
                    Y.append(normalize_image(images_aug_out))
                else:
                    Y.append(normalize_image(np.array(images)))

        return X, Y

    def _transform_record(self, record):

        for key, value in record.items():
            if key == 'cam/image_array' and isinstance(value, str):
                image_path = os.path.join(record['_image_base_path'], value)
                image = load_image_arr(image_path, self.config)
                record[key] = image

            # for categorical convert to one-hot vector
            if key in ['user/angle', 'user/throttle'] and self.train_state == \
                    TrainState.CATEGORICAL and isinstance(value, float):
                if key == 'user/angle':
                    angle = linear_bin(value, N=15, offset=1, R=2.0)
                    record[key] = angle
                elif key == 'user/throttle':
                    R = self.config.MODEL_CATEGORICAL_MAX_THROTTLE_RANGE
                    throttle = linear_bin(value, N=20, offset=0.0, R=R)
                    record[key] = throttle

        # when training only the controller, pre-compute latent vectors
        if self.train_state == TrainState.LATENT_CONTROLLER and \
                'img/latent' not in record:
            img_arr = normalize_image(record['cam/image_array'])
            img_arr = img_arr.reshape((1, ) + img_arr.shape)
            record['img/latent'] = self.keras_model.encoder.predict(img_arr)[0]

        return record


class ImagePreprocessing(Sequence):
    '''
    A Sequence which wraps another Sequence with an Image Augumentation.
    '''

    def __init__(self, sequence, augmentation):
        self.sequence = sequence
        self.augumentation = augmentation

    def __len__(self):
        return len(self.sequence)

    def __getitem__(self, index):
        X, Y = self.sequence[index]
        return self.augumentation.augment_images(X), Y


def train(cfg, tub_paths, output_path, model_type):
    """
    Train the model
    """
    # convert single path into list of one element
    if type(tub_paths) is str:
        tub_paths = [tub_paths]

    train_type = 'linear' if 'linear' in model_type else model_type
    kl = get_model_by_type(train_type, cfg)

    if cfg.PRINT_MODEL_SUMMARY:
        print(kl.model.summary())

    batch_size = cfg.BATCH_SIZE
    shuffle = getattr(cfg, 'TRAIN_SHUFFLE', True)
    suppress_list = getattr(cfg, 'TRAIN_SUPPRESS', [])
    include_list = getattr(cfg, 'TRAIN_ONLY', [])
    dataset = TubDataset(tub_paths, test_size=(1. - cfg.TRAIN_TEST_SPLIT),
                         shuffle=shuffle, skip=suppress_list,
                         include_only=include_list)
    training_records, validation_records = dataset.train_test_split()
    print('Records # Training %s' % len(training_records))
    print('Records # Validation %s' % len(validation_records))

    training = TubSequence(kl, cfg, training_records)
    validation = TubSequence(kl, cfg, validation_records)
    assert len(validation) > 0, "Not enough validation data, decrease the "\
                                "batch size or add more data."

    history = kl.train(model_path=output_path,
                       train_data=training,
                       train_steps=len(training),
                       batch_size=batch_size,
                       validation_data=validation,
                       validation_steps=len(validation),
                       epochs=cfg.MAX_EPOCHS,
                       verbose=cfg.VERBOSE_TRAIN,
                       min_delta=cfg.MIN_DELTA,
                       patience=cfg.EARLY_STOP_PATIENCE)

    return history


def main():
    args = docopt(__doc__)
    cfg = donkeycar.load_config()
    tubs = args['--tubs']
    model = args['--model']
    model_type = args['--type']
    latent_trained = args['--latent_trained']
    if latent_trained:
        cfg.LATENT_TRAINED = latent_trained
        model_type = 'latent'
    model_name, model_ext = os.path.splitext(model)
    is_tflite = model_ext == '.tflite'
    if is_tflite:
        model = f'{model_name}.h5'

    if not model_type:
        model_type = cfg.DEFAULT_MODEL_TYPE

    tubs = tubs.split(',')
    data_paths = [Path(os.path.expanduser(tub)).absolute().as_posix() for tub in tubs]
    output_path = os.path.expanduser(model)
    start = time.time()
    history = train(cfg, data_paths, output_path, model_type)
    if is_tflite:
        tflite_model_path = f'{os.path.splitext(output_path)[0]}.tflite'
        keras_model_to_tflite(output_path, tflite_model_path)
    td = datetime.timedelta(seconds=time.time() - start)
    print(f'{"-" * 40}\nTraining completed in {td}')


if __name__ == "__main__":
    main()
