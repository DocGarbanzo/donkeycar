import os
import numpy as np
from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import List, Dict, Any, Tuple, Union, TypeVar, Iterable
import math

import tensorflow as tf
from tensorflow.python.keras.utils.data_utils import Sequence as TfSequence
from donkeycar.parts.tub_v2 import Tub
from donkeycar.utils import get_model_by_type, load_image_arr, \
    train_test_split, normalize_image
from donkeycar.parts.keras import KerasPilot
from donkeycar.config import Config
from donkeycar.pipeline.sequence import TubDataset, TubSequence, Pipeline
from donkeycar.pipeline.types import TubRecord

# for typing
Record = Dict[str, Any]
Records = List[Record]
X = TypeVar('X', covariant=True)
Y = TypeVar('Y', covariant=True)


class BatchSequence(TfSequence):
    def __init__(self,
                 pipeline: Iterable[Tuple[X, Y]],
                 batch_size: int) -> None:
        self.pipeline = pipeline
        self.batch_size = batch_size

    def __len__(self) -> int:
        return math.ceil(len(self.pipeline) / self.batch_size)

    def __getitem__(self, index: int) -> Tuple[Any, Any]:
        count = 0
        x = []
        y = []
        # collecting across the whole batch
        while count < self.batch_size:
            i = (index * self.batch_size) + count
            if i >= len(self.pipeline):
                break
            single_x, single_y = self.pipeline[i]
            x.append(single_x)
            y.append(single_y)
            count += 1

        # reshape X, Y
        def reshape(z):
            # each entry in z could either be a single value, or a numpy
            # array, or a tuple containing values and numpy arrays
            if type(z[0]) is tuple:
                dim = len(z[0])
                ret_z = []
                for j in range(dim):
                    z_j = np.array([zi[j] for zi in z])
                    ret_z.append(z_j)
                return ret_z
            else:
                return np.array(z)

        x_res = reshape(x)
        y_res = reshape(y)
        return x_res, y_res


def make_tf_data(pipeline, batch_size):
    dataset = tf.data.Dataset.from_generator(
        generator=lambda: pipeline,
        output_types=(tf.float64, {'n_outputs0': tf.float64,
                                   'n_outputs1': tf.float64}))

    return dataset.repeat().batch(batch_size)


def train(cfg: Config,
          tub_paths: Union[str, List[str]],
          output_path: str,
          model_type: str) -> Dict[str, Any]:
    """
    Train the model
    :param cfg:         donkey config
    :param tub_paths:   single path or list of multiple paths for tubs
    :param output_path: output model path
    :param model_type:  model type, e.g linear, categorical, etc
    :return:            history dictionary
    """
    # convert single path into list of one element
    if type(tub_paths) is str:
        tub_paths = [tub_paths]

    if 'linear' in model_type:
        train_type = 'linear'
    else:
        train_type = model_type

    kl = get_model_by_type(train_type, cfg)

    if cfg.PRINT_MODEL_SUMMARY:
        print(kl.model.summary())

    batch_size: int = cfg.BATCH_SIZE
    # loading all records into a single data set
    dataset = TubDataset(tub_paths, config=cfg)
    records = dataset.load_records()
    training_records, validation_records \
        = train_test_split(records, shuffle=True,
                           test_size=(1. - cfg.TRAIN_TEST_SPLIT))
    print('Records # Training %s' % len(training_records))
    print('Records # Validation %s' % len(validation_records))

    # step 1 of pipeline, create the sequence:
    training = TubSequence(records=training_records)
    validation = TubSequence(records=validation_records)

    # step 2 of pipeline, extract X, Y sequence from data
    # get X from tub record:
    def get_X(t: TubRecord) -> np.ndarray:
        img_arr = t.image(cached=True, normalize=True)
        return img_arr

    def get_Y(t: TubRecord) -> Tuple[float, float]:
        y1 = t.underlying['user/angle']
        y2 = t.underlying['user/throttle']
        return {'n_outputs0': y1, 'n_outputs1': y2}

    # TODO: training_pipe iterates only once and then is exhausted. That's
    #  why keras training fails after one epoch.
    # training_pipe = training.build_pipeline(get_X, get_Y)
    # validation_pipe = validation.build_pipeline(get_X, get_Y)

    # # this version is working.
    training_pipe = Pipeline(training, get_X, get_Y)
    validation_pipe = Pipeline(validation, get_X, get_Y)

    # step 3 of pipeline, transform into tf.data or tf.sequence
    # here using tf.data
    dataset_train = make_tf_data(training_pipe, cfg.BATCH_SIZE)
    dataset_validate = make_tf_data(validation_pipe, cfg.BATCH_SIZE)
    train_size = math.ceil(len(training_pipe) / cfg.BATCH_SIZE)
    val_size = math.ceil(len(validation_pipe) / cfg.BATCH_SIZE)

    # here using tf.sequence
    # dataset_train = BatchSequence(training_pipe, cfg.BATCH_SIZE)
    # dataset_validate = BatchSequence(validation_pipe, cfg.BATCH_SIZE)
    # train_size = len(dataset_train)
    # val_size = len(dataset_validate)

    assert val_size > 0, \
        "Not enough validation data, decrease the batch size or add more data."

    history = kl.train(model_path=output_path,
                       train_data=dataset_train,
                       train_steps=train_size,
                       batch_size=batch_size,
                       validation_data=dataset_validate,
                       validation_steps=val_size,
                       epochs=cfg.MAX_EPOCHS,
                       verbose=cfg.VERBOSE_TRAIN,
                       min_delta=cfg.MIN_DELTA,
                       patience=cfg.EARLY_STOP_PATIENCE)

    return history
