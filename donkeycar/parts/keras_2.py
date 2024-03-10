import copy
import time
import logging
from collections import deque

import numpy as np
from typing import Dict, Tuple, Union, List, Callable
import tensorflow as tf

from tensorflow import keras
from tensorflow.keras import regularizers
from tensorflow.keras import Model, Input
from tensorflow.keras.layers import Dense, Concatenate, Conv2D, \
    BatchNormalization, Dropout, Flatten, Reshape, UpSampling2D, \
    Conv2DTranspose, LSTM, MaxPooling2D, TimeDistributed as TD, LeakyReLU
from tensorflow.python.keras.layers import Activation

from donkeycar.parts.interpreter import Interpreter, KerasInterpreter
from donkeycar.parts.keras import KerasLinear, XY, KerasMemory
from donkeycar.pipeline.types import TubRecord
from donkeycar.utils import normalize_image


logger = logging.getLogger(__name__)


class KerasSquarePlus(KerasLinear):
    """
    improved cnn over standard keraslinear. the cnn translates into square
    matrices from layer one on and uses batch normalisation, average pooling
    & l2 regularisor instead of dropout in the perceptron layers. because of
    the square form, the first layers stride need to be 3x4 (as this is the
    input picture h x w ratio). with this, the reduction in the first layer is
    larger, hence the whole model only has 5 cnn layers.
    """
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (120, 160, 3),
                 *args, **kwargs):
        self.size = kwargs.get('size', 'S').upper()
        self.use_speed = kwargs.get('use_speed', True)
        self.max_speed = 1.0 if not self.use_speed else kwargs['max_speed']
        self.offset = kwargs.get('offset', 0)
        super().__init__(interpreter, input_shape)
        self.num_to_freeze = 2

    def __str__(self) -> str:
        return super().__str__() + \
            f'-{self.size}-speed:{int(self.use_speed)}-os:{self.offset}'

    def seq_size(self) -> int:
        return self.offset + 1 if self.offset else 0

    def create_model(self):
        return linear_square_plus(self.input_shape, size=self.size,
                                  pos_throttle=self.use_speed)

    # def compile(self):
    #     self.interpreter.compile(optimizer=self.optimizer, loss='mse',
    #                              weighted_metrics=['mse'])

    def x_transform(
            self,
            record: Union[TubRecord, List[TubRecord]],
            img_processor: Callable[[np.ndarray], np.ndarray]) \
            -> Dict[str, Union[float, np.ndarray]]:
        """ Transforms the record into x for training the model to x,y """
        if self.offset == 0:
            assert isinstance(record, TubRecord), 'TubRecord expected'
            rec = record
        else:
            assert isinstance(record, list), 'List[TubRecord] expected'
            rec = record[-(1 + self.offset)]

        img_arr = rec.image(processor=img_processor)
        return {'img_in': img_arr}

    def y_transform(self, record: Union[TubRecord, List[TubRecord]]) \
            -> Dict[str, Union[float, List[float]]]:
        if self.offset == 0:
            assert isinstance(record, TubRecord), 'TubRecord expected'
            rec = record
        else:
            assert isinstance(record, list), 'List[TubRecord] expected'
            rec = record[-1]
        angle = rec.underlying['user/angle']
        throttle = self._get_throttle(rec)
        return {'angle': angle, 'throttle': throttle}

    def output_shapes(self):
        # need to cut off None from [None, 120, 160, 3] tensor shape
        img_shape = self.get_input_shape('img_in')[1:]
        shapes = ({'img_in': tf.TensorShape(img_shape)},
                  {'angle': tf.TensorShape([]),
                   'throttle': tf.TensorShape([])})
        return shapes

    def freeze_first_layers(self, num_last_layers_to_train=None):
        assert isinstance(self.interpreter, KerasInterpreter), \
            'Can only freeze layers in Keras model but not in TfLite and others'
        # We freeze the first layer which is the CNN encoder. Note the input
        # layer is the first layer, hence skip 2.
        frozen_layers = []
        for i in range(self.num_to_freeze):
            self.interpreter.model.layers[i].trainable = False
            frozen_layers.append(self.interpreter.model.layers[i].name)
        logger.info(f'Freezing layers {frozen_layers}')
        return self.num_to_freeze

    def reset(self):
        logger.info("Pilot reset called")

    def _get_throttle(self, record):
        if self.use_speed:
            return record.underlying['car/speed'] / self.max_speed
        else:
            return record.underlying['user/throttle']


class KerasSquarePlusMemory(KerasMemory, KerasSquarePlus):
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (120, 160, 3),
                 *args, **kwargs):
        super().__init__(interpreter, input_shape, *args, **kwargs)
        # because we have 2 inputs (img, mem) we need to freeze in1, in2, cnn
        self.num_to_freeze = 3

    def create_model(self):
        return linear_square_plus_mem(self.input_shape, self.size,
                                      self.mem_length, self.mem_depth,
                                      has_lap_pct=False,
                                      pos_throttle=self.use_speed)

    def use_lap_pct(self) -> bool:
        return True

    def x_transform(
            self,
            record: Union[TubRecord, List[TubRecord]],
            img_processor: Callable[[np.ndarray], np.ndarray]) \
            -> Dict[str, Union[float, np.ndarray]]:
        """ Transforms the record into x for training the model to x,y """
        assert isinstance(record, list), 'List[TubRecord] expected'
        assert len(record) == self.mem_length + 1, \
            f"Record list of length {self.mem_length} required but " \
            f"{len(record)} was passed"
        img_arr = record[-(1 + self.offset)].image(processor=img_processor)
        mem = [[r.underlying['user/angle'], self._get_throttle(r)]
               for r in record[:-1]]
        mem_arr = np.array(mem).reshape((2 * self.mem_length,))
        return {'img_in': img_arr, 'mem_in': mem_arr}

    def y_transform(self, records: Union[TubRecord, List[TubRecord]]) \
            -> Dict[str, Union[float, List[float]]]:
        assert isinstance(records, list), 'List[TubRecord] expected'
        angle = records[-1].underlying['user/angle']
        throttle = self._get_throttle(records[-1])
        return {'angle': angle, 'throttle': throttle}

    def w_transform(self, records: Union[TubRecord, List[TubRecord]]) \
            -> Dict[str, Union[float, List[float]]]:
        assert isinstance(records, list), 'List[TubRecord] expected'
        weight = records[-1].underlying['lap_pct']
        return {'angle': weight, 'throttle': weight}

    def output_shapes(self):
        # need to cut off None from [None, 120, 160, 3] tensor shape
        img_shape = self.get_input_shape('img_in')[1:]
        shapes = ({'img_in': tf.TensorShape(img_shape),
                   'mem_in': tf.TensorShape(2 * self.mem_length)},
                  {'angle': tf.TensorShape([]),
                   'throttle': tf.TensorShape([])},
                  # {'angle': tf.TensorShape([]),
                  #  'throttle': tf.TensorShape([])}
                  )
        return shapes

    def reset(self):
        super().reset()
        logger.info(f"Resetting with start speed {self.mem_start_speed}")
        self.mem_seq = deque([[0.0, self.mem_start_speed]] * self.mem_length)


class KerasSquarePlusMemoryLap(KerasSquarePlusMemory):
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (120, 160, 3),
                 *args, **kwargs):
        self.lap_dim = kwargs.get('lap_dim', 3)
        super().__init__(interpreter, input_shape, *args, **kwargs)
        # needs to come after base class ctor otherwise gets overwritten
        self.num_to_freeze = 4

    def use_lap_pct(self) -> bool:
        return True

    def create_model(self):
        return linear_square_plus_mem(self.input_shape, self.size,
                                      self.mem_length, self.mem_depth,
                                      has_lap_pct=True,
                                      pos_throttle=self.use_speed,
                                      lap_dim=self.lap_dim)

    def x_transform(self, record: Union[TubRecord, List[TubRecord]],
                    img_processor: Callable[[np.ndarray], np.ndarray]) \
            -> Dict[str, Union[float, np.ndarray]]:
        x_dict = super().x_transform(record, img_processor)
        lap_pct = np.array([record[-1].underlying['lap_pct']])\
            .reshape((self.lap_dim,))
        x_dict['lap_pct_in'] = lap_pct
        return x_dict

    def output_shapes(self):
        # need to cut off None from [None, 120, 160, 3] tensor shape
        img_shape = self.get_input_shape('img_in')[1:]
        shapes = ({'img_in': tf.TensorShape(img_shape),
                   'mem_in': tf.TensorShape(2 * self.mem_length),
                   'lap_pct_in': tf.TensorShape(self.lap_dim)},
                  {'angle': tf.TensorShape([]),
                   'throttle': tf.TensorShape([])},
                    # this is for the weights
                  # {'angle': tf.TensorShape([]),
                  #  'throttle': tf.TensorShape([])}
                  )
        return shapes

    def run(self, img_arr: np.ndarray, *other_arr: List[float]) -> \
            Tuple[Union[float, np.ndarray], ...]:
        # Only called at start to fill the previous values
        np_mem_arr = np.array(self.mem_seq).reshape((2 * self.mem_length,))
        norm_img_arr = normalize_image(img_arr)
        lap_pct, = other_arr
        np_lap_arr = np.array(lap_pct)
        # create dictionary on the fly, we expect the order of the arguments:
        # img_arr, *other_arr to exactly match the order of the
        # self.output_shape() first dictionary keys, because that's how we
        # set up the model
        values = (norm_img_arr, np_mem_arr, np_lap_arr)
        # note output_shapes() returns a 2-tuple of dicts for input shapes
        # and output shapes(), so we need the first tuple here
        input_dict = dict(zip(self.output_shapes()[0].keys(), values))
        angle, throttle = self.inference_from_dict(input_dict)
        # fill new values into back of history list for next call
        self.mem_seq.popleft()
        self.mem_seq.append([angle, throttle])
        return angle, throttle


class KerasSquarePlusLstm(KerasSquarePlus):
    """
    LSTM version of square plus model
    """
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (120, 160, 3),
                 *args, **kwargs):
        self.seq_length = kwargs.get('seq_length', 3)
        self.img_seq = []
        super().__init__(interpreter, input_shape, *args, **kwargs)
        self.normaliser = kwargs['imu_normaliser']

    def create_model(self):
        return linear_square_plus(self.input_shape,
                                  size=self.size,
                                  seq_len=self.seq_length,
                                  pos_throttle=self.use_speed)

    def __str__(self) -> str:
        return super().__str__() + f'-{self.size}-seq:{self.seq_length}'


class KerasSquarePlusImu(KerasSquarePlus):
    """
    The model is a variation of the SquarePlus model that also uses imu data
    """
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (120, 160, 3),
                 *args, **kwargs):
        self.imu_dim = kwargs.get('imu_dim', 6)
        self.accel_norm = kwargs['accel_norm']
        self.gyro_norm = kwargs['gyro_norm']
        super().__init__(interpreter, input_shape, *args, **kwargs)

    def create_model(self):
        model = linear_square_plus_imu(self.input_shape,
                                       imu_dim=self.imu_dim,
                                       size=self.size,
                                       pos_throttle=self.use_speed)
        return model

    def normalize_imu(self, accel, gyro):
        """
        Normalises IMU values into [-1, 1]

        :param accel:   list of 3 accel values
        :param gyro:    list of 3 gyro values
        :return:        np.array of normalised values resized to a smaller
                        dimension, if the imu_dim is < 6
        """
        accel_norm = np.array(accel) / self.accel_norm
        gyro_norm = np.array(gyro) / self.gyro_norm
        imu = np.concatenate((accel_norm, gyro_norm))[:self.imu_dim]
        return imu

    def run(self, img_arr: np.ndarray, other_arr: List[float] = None) \
            -> Tuple[Union[float, np.ndarray], ...]:
        """
        Donkeycar parts interface to run the part in the loop.

        :param img_arr:     uint8 [0,255] numpy array with image data
        :param other_arr:   numpy imu array with raw data with accel / gyro
        :return:            tuple of (angle, throttle)
        """
        norm_img_arr = normalize_image(img_arr)
        np_imu_array = self.normalize_imu(other_arr[:3], other_arr[3:])

        # create dictionary on the fly, we expect the order of the arguments:
        # img_arr, *other_arr to exactly match the order of the
        # self.output_shape() first dictionary keys, because that's how we
        # set up the model
        values = (norm_img_arr, np_imu_array)
        # note output_shapes() returns a 2-tuple of dicts for input shapes
        # and output shapes(), so we need the first tuple here
        input_dict = dict(zip(self.output_shapes()[0].keys(), values))
        return self.inference_from_dict(input_dict)

    def x_transform(self,
                    record: Union[TubRecord, List[TubRecord]],
                    img_processor: Callable[[np.ndarray], np.ndarray]) \
            -> Dict[str, Union[float, np.ndarray]]:
        # this transforms the record into x for training the model to x,y
        assert isinstance(record, TubRecord), 'TubRecord expected'
        img_arr = record.image(processor=img_processor)
        imu_process = self.normalize_imu(record.underlying['car/accel'],
                                         record.underlying['car/gyro'])
        return {'img_in': img_arr, 'imu_in': imu_process}

    def output_shapes(self):
        # need to cut off None from [None, 120, 160, 3] tensor shape
        img_shape = self.get_input_shape('img_in')[1:]
        # the keys need to match the models input/output layers
        shapes = ({'img_in': tf.TensorShape(img_shape),
                   'imu_in': tf.TensorShape([self.imu_dim])},
                  {'angle': tf.TensorShape([]),
                   'throttle': tf.TensorShape([])})
        return shapes


class KerasSquarePlusImuLstm(KerasSquarePlusLstm):
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (120, 160, 3),
                 *args, **kwargs):
        self.imu_dim = kwargs.get('imu_dim', 6)
        self.imu_seq = []
        super().__init__(interpreter, input_shape, *args, **kwargs)

    def create_model(self):
        return linear_square_plus_imu(self.input_shape,
                                      imu_dim=self.imu_dim,
                                      size=self.size,
                                      seq_len=self.seq_length,
                                      pos_throttle=self.use_speed)


class KerasWorld(KerasSquarePlus):
    """
    World model for pilot. Allows pre-trained model from which it only takes
    the encoder that transforms images into latent vectors.
    """
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (144, 192, 3),
                 encoder_path=None, *args, **kwargs):
        self.encoder_path = encoder_path
        self.latent_dim = kwargs.get('latent_dim', 128)
        self.encoder = self.make_encoder(input_shape)
        self.encoder.trainable = self.encoder_path is None
        super().__init__(interpreter, input_shape, size='R', *args, **kwargs)

    def make_encoder(self, input_shape):
        if self.encoder_path is None:
            return AutoEncoder(input_shape=input_shape,
                               latent_dim=self.latent_dim).encoder
        else:
            # load full world model
            k_world = tf.keras.models.load_model(self.encoder_path)
            # return layer 1 which is the encoder
            encoder = k_world.layers[1]
            self.encoder_checks(encoder)
            # reset latent dimension from loaded model
            self.latent_dim = encoder.outputs[0].shape[1]
            return encoder

    def make_controller_inputs(self):
        latent_input = keras.Input(shape=(self.latent_dim,), name='latent_in')
        return latent_input

    def transform_controller_inputs(self, inputs):
        return inputs

    def make_controller(self):
        l2 = 0.001
        inputs = self.make_controller_inputs()
        x = self.transform_controller_inputs(inputs)
        for i in range(3):
            x = Dense(units=self.latent_dim / 2, activation='relu',
                      kernel_regularizer=regularizers.l2(l2),
                      name='dense' + str(i))(x)
        angle_out = Dense(units=1, activation='linear', name='angle')(x)
        throttle_out = Dense(units=1, activation='linear', name='throttle')(x)
        controller = Model(inputs=inputs,
                           outputs=[angle_out, throttle_out],
                           name='controller')
        return controller

    def create_model(self):
        controller = self.make_controller()
        pilot_input = keras.Input(shape=self.input_shape)
        encoded_img = self.encoder(pilot_input)
        [angle, throttle] = controller(encoded_img)
        model = Model(pilot_input, [angle, throttle], name="world_pilot")
        return model

    def __str__(self) -> str:
        return super().__str__() + f'-enc:{self.encoder_path}'

    def load_encoder(self, model_path):
        print('Loading encoder from', model_path, 'into world model...', end='')
        model = keras.models.load_model(model_path)
        encoder = model.layers[1]
        self.encoder_checks(encoder)
        self.encoder = encoder
        self.latent_dim = self.encoder.outputs[0].shape[1]
        self.encoder.trainable = False
        self.input_shape = encoder.inputs[0].shape[1:]
        self.interpreter.model = self.create_model()
        print("done - encoder is not trainable now")

    @staticmethod
    def encoder_checks(encoder):
        assert type(encoder) is tf.keras.Model, \
            'first layer of model needs to be a model'
        assert encoder.name == 'encoder', \
            'first layer model should have name "encoder"'


class KerasWorldImu(KerasWorld, KerasSquarePlusImu):
    """
    World model for pilot. Uses pre-trained encoder which breaks down images
    into latent vectors.
    """

    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (144, 192, 3),
                 encoder_path=None, *args, **kwargs):
        super().__init__(interpreter, input_shape,
                         encoder_path=encoder_path, *args, **kwargs)

    def make_controller_inputs(self):
        latent_input = keras.Input(shape=(self.latent_dim,), name='latent_in')
        imu_input = keras.Input(shape=(self.imu_dim,), name='imu_in')
        return [latent_input, imu_input]

    def transform_controller_inputs(self, inputs):
        return Concatenate()(inputs)

    def create_model(self):
        controller = self.make_controller()
        pilot_input = keras.Input(shape=self.input_shape, name='img_in')
        encoded_img = self.encoder(pilot_input)
        imu_input = keras.Input(shape=(self.imu_dim,), name='imu_in')
        [angle, throttle] = controller([encoded_img, imu_input])
        model = Model(inputs=[pilot_input, imu_input],
                      outputs=[angle, throttle],
                      name="world_pilot_imu")
        return model


class WorldMemory:
    def __init__(self, encoder_path='models/encoder.h5', *args, **kwargs):
        self.seq_length = kwargs.get('seq_length', 3)
        # self.imu_dim = kwargs.get('imu_dim', 6)
        self.layers = kwargs.get('lstm_layers', 3)
        self.units = kwargs.get('lstm_units', 128)
        self.encoder = keras.models.load_model(encoder_path).layers[1]
        self.latent_dim = self.encoder.outputs[0].shape[1]
        self.model = self.make_model()
        print('Created WorldMemory with encoder path:', encoder_path,
              'seq length', self.seq_length)

    def make_model(self):
        l2 = 0.001
        input_shape_latent = (self.seq_length, self.latent_dim)
        # input_shape_imu = (self.seq_length, self.imu_dim)
        input_shape_drive = (self.seq_length, 2)
        latent_seq_in = keras.Input(input_shape_latent, name='latent_seq_in')
        # imu_seq_in = keras.Input(input_shape_imu, name='imu_seq_in')
        drive_seq_in = keras.Input(input_shape_drive, name='drive_seq_in')
        inputs = [latent_seq_in, drive_seq_in]
        x = Concatenate(axis=-1)(inputs)
        for i in range(self.layers):
            last = (i == self.layers - 1)
            x = LSTM(units=self.units,
                     kernel_regularizer=regularizers.l2(l2),
                     recurrent_regularizer=regularizers.l2(l2),
                     name='lstm' + str(i),
                     return_sequences=not last,
                     return_state=last)(x)
        # now x[0] has return sequences and x[1] has the state
        sequences = x[0]
        state = x[1]
        latent_out = Dense(units=self.latent_dim, name='latent_out')(sequences)
        # imu_out = Dense(units=self.imu_dim, name='imu_out')(sequences)
        # drive_out = Dense(units=2, name='drive_out')(sequences)
        outputs = [latent_out, state]  # [latent_out, imu_out, drive_out, state]
        model = Model(inputs=inputs, outputs=outputs, name='Memory')
        return model

    def load(self, model_path):
        prev = str(self)
        self.model = keras.models.load_model(model_path)
        print(f"Load model: overwriting {prev} with {self}")

    def __str__(self) -> str:
        return type(self).__name__ + f'_l:{self.layers}_units:{self.units}'

    def compile(self):
        # here we set the loss for the internal state output to None so it
        # doesn't get used in training
        opt = tf.keras.optimizers.Adam(learning_rate=0.01,
                                       beta_1=0.9,
                                       beta_2=0.999)
        self.model.compile(optimizer=opt, loss=['mse', None])


class WorldPilot(KerasWorldImu):
    def __init__(self,
                 interpreter: Interpreter = KerasInterpreter(),
                 input_shape: Tuple[int, ...] = (144, 192, 3),
                 encoder_path=None, memory_path=None,
                 *args, **kwargs):
        if memory_path:
            assert encoder_path, "Need to provide encoder path too"
            self.memory = keras.models.load_model(memory_path)
            # get seq length from input shape of memory model
            self.seq_length = self.memory.inputs[0].shape[1]
            # get state vector length from last output of memory model
            self.state_dim = self.memory.outputs[1].shape[1]
            super().__init__(interpreter, input_shape,
                             encoder_path=encoder_path)
            # fill sequence to size
            self.latent_seq = [np.zeros((self.latent_dim,))] * self.seq_length
            self.drive_seq = [np.zeros((2,))] * self.seq_length
            self.encoder.trainable = False
            self.memory.trainable = False
        else:
            assert encoder_path is None, "Don't pass encoder w/o memory model"
            self.memory = self.seq_length = self.state_dim = None
            self.latent_seq = self.drive_seq = None
            print('Created empty WorldPilot - you need to load a model')

    def make_controller_inputs(self):
        latent_input = keras.Input(shape=(self.latent_dim,), name='latent_in')
        imu_input = keras.Input(shape=(self.imu_dim,), name='imu_in')
        state_input = keras.Input(shape=(self.state_dim,), name='state_in')
        return [latent_input, imu_input, state_input]

    def create_model(self):
        latent_seq_input = keras.Input(shape=(self.seq_length,
                                              self.latent_dim),
                                       name='latent_seq_in')
        drive_seq_input = keras.Input(shape=(self.seq_length, 2),
                                      name='drive_seq_in')
        [latent_out, state] = self.memory([latent_seq_input, drive_seq_input])
        img_input = keras.Input(shape=self.input_shape, name='img_in')
        latent = self.encoder(img_input)
        imu_input = keras.Input(shape=(self.imu_dim,), name='imu_in')
        controller = self.make_controller()
        [angle, throttle] = controller([latent, imu_input, state])
        model = Model(inputs=[img_input, imu_input,
                              latent_seq_input, drive_seq_input],
                      outputs=[angle, throttle, latent],
                      name='world_pilot')
        return model

    def compile(self):
        # here we set the loss for the latent vector output to None so it
        # doesn't get used in training
        self.interpreter.compile(optimizer='adam', loss=['mse', 'mse', None])

    def __str__(self) -> str:
        return super().__str__() + \
            f'_sd_{self.state_dim}_ld_{self.latent_dim,}_seql_{self.seq_length}'

    def load(self, model_path):
        self.interpreter.model = keras.models.load_model(model_path)
        model = self.interpreter.model
        model.summary(expand_nested=True)
        self.memory = model.get_layer('Memory')
        # get seq length from input shape of memory model
        self.seq_length = self.memory.inputs[0].shape[1]
        # get state vector length from last output of memory model
        self.state_dim = self.memory.outputs[1].shape[1]
        self.encoder = model.get_layer('encoder')
        self.latent_dim = self.encoder.outputs[0].shape[1]
        self.imu_dim = model.get_layer('imu_in').output_shape[0][1]
        # fill sequence to size
        self.latent_seq = [np.zeros((self.latent_dim,))] * self.seq_length
        self.drive_seq = [np.zeros((2,))] * self.seq_length
        self.encoder.trainable = False
        self.memory.trainable = False

    def run(self, img_arr, imu_arr=None):
        # convert imu python list into numpy array first, img_arr is already
        # numpy array
        if imu_arr is None:
            imu_arr = np.zeros((1, self.imu_dim))
        else:
            imu_arr = np.array([imu_arr])
        # convert image array into latent vector first
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        # now we run the model returning drive vector and last latent vector
        inputs = (img_arr, imu_arr,
                  np.array([self.latent_seq]), np.array([self.drive_seq]))
        keys = ('img_in', 'imu_in', 'latent_seq_in', 'drive_seq_in')
        input_dict = dict(zip(keys, inputs))
        [angle, throttle, latent] \
            = self.interpreter.predict_from_dict(input_dict)
        # convert drive array of ndarray into more convenient type
        drive_arr = np.array([angle[0][0], throttle[0][0]])
        # add new results for angle, throttle and latent vector into sequence
        for seq, new in zip([self.latent_seq, self.drive_seq],
                            [latent[0], drive_arr]):
            # pop oldest entry from front and append new entry at end
            seq.pop(0)
            seq.append(new)
        # return angle, throttle
        return drive_arr[0], drive_arr[1]


class AutoEncoder:
    def __init__(self, input_shape=(144, 192, 3), latent_dim=256,
                 encoder_path=None, decoder_path=None):
        self.input_shape = input_shape
        self.output_shape = None
        self.latent_dim = latent_dim
        self.filters = [16, 32, 48, 64, 80]
        self.kernel = (3, 3)
        self.strides = [(2, 2)] + [(1, 1)] * 5
        self.encoder = keras.models.load_model(encoder_path) if encoder_path \
            else self.make_encoder()
        self.decoder = keras.models.load_model(decoder_path) if decoder_path \
            else self.make_decoder()
        img_input = keras.Input(shape=(144, 192, 3))
        encoded_img = self.encoder(img_input)
        decoded_img = self.decoder(encoded_img)
        self.autoencoder = keras.Model(img_input, decoded_img,
                                       name="autoencoder")

    def make_encoder(self):
        encoder_input = keras.Input(self.input_shape, name='img_in')
        x = encoder_input
        drop = 0.02
        conv = None
        num_l = len(self.filters)
        for i, f, s in zip(range(num_l), self.filters, self.strides):
            conv = Conv2D(filters=f, kernel_size=self.kernel, strides=s,
                          padding='same', activation='relu',
                          name='conv' + str(i))
            x = conv(x)
            x = BatchNormalization(name='batch_norm' + str(i))(x)
            x = Dropout(rate=drop, name='drop' + str(i))(x)
            if i < num_l - 1:
                x = MaxPooling2D(pool_size=(3, 3) if i == num_l - 2 else (2, 2),
                                 padding='same',
                                 name='pool' + str(i))(x)

        # remove first entry from (, a, b, c)
        self.output_shape = tuple(conv.output_shape[1:])
        x = Flatten()(x)
        latent = Dense(self.latent_dim, name="dense", activation='sigmoid')(x)
        encoder = keras.Model(encoder_input, latent, name="encoder")
        return encoder

    def make_decoder(self):
        latent_input = keras.Input(shape=(self.latent_dim,), name='latent_in')
        dim = np.prod(self.output_shape)
        x = Dense(dim, activation="relu")(latent_input)
        x = Reshape(self.output_shape)(x)
        for i, f, s in zip(reversed(range(len(self.filters))),
                           reversed(self.filters), reversed(self.strides)):
            if i < 4:
                x = UpSampling2D((3, 3) if i == 3 else (2, 2))(x)
            x = Conv2DTranspose(f, self.kernel, activation="relu",
                                strides=s, padding="same",
                                name='deconv' + str(i))(x)
        decoder_output = Conv2DTranspose(3, self.kernel,
                                         strides=self.strides[0],
                                         activation="sigmoid",
                                         padding="same",
                                         name='deconv_convert')(x)
        decoder = keras.Model(latent_input, decoder_output, name="decoder")
        return decoder


class ModelLoader:
    """ This donkey part is meant to be continuously looking for an updated
    pilot to overwrite the existing one"""

    def __init__(self, keras_pilot, model_path):
        self.remote_model = keras_pilot
        self.model_path = model_path
        self.update_trigger = False
        self.is_updated = False
        # make own copy of the model
        self.model = copy.copy(keras_pilot)

    def run_threaded(self, update):
        """
        Donkey part interface for threaded parts
        :param bool update:
            Should be true if the model file changed otherwise false. The
            input here is expected to come from the output of the FileWatcher.
        :return bool:
            Indicator if the remote model was updated
        """
        if update:
            # if FileWatcher recognises a new version of the file set the flag
            self.update_trigger = True
            return False
        else:
            # check pilot is loaded, and we shove it into the remote object
            if self.is_updated:
                self.remote_model.update(self.model)
                # reset update state to false so we don't update until
                # the flag gets set to true by the file watcher again
                self.is_updated = False
                print('ModelLoader updated model.')
                return True
            # otherwise, no updated model available - do nothing
            else:
                return False

    def update(self):
        """
        Donkey parts interface
        """
        while True:
            # self.update has been set by the FileWatcher in the loop
            if self.update_trigger:
                self.model.load(model_path=self.model_path)
                self.update_trigger = False
                self.is_updated = True
            else:
                # no update wait 1s
                time.sleep(1.0)


class ModelResetter:
    """ Part to reset a pilot back to start. Used in gym, where we restart
        when receiving a game-over signal from the sim. In that case we want
        to make sure the models memory or other static values get reset,
        so it won't drive backward or similar.
    """
    def __init__(self, pilot):
        self.pilot = pilot

    def run(self, reset: bool = False):
        if reset:
            self.pilot.reset()


def linear_square_plus_cnn(img_in, size='R', l2=0.01, is_seq=False):
    drop = 0.1
    x = img_in
    # This makes the picture square in 1 steps (assuming 3x4 input) in all
    # following layers
    if size in ['XS', 'S', 'M']:
        filters = [16, 32, 64, 96]
        kernels = [(7, 7), (5, 5), (3, 3), (2, 2)]
        if size == 'M':
            filters += [128]
            kernels += [(2, 2)]
    elif size == 'L':  #
        filters = [20, 40, 80, 120, 160]
        kernels = [(9, 9), (7, 7), (5, 5), (3, 3), (2, 2)]
    else:  # size is R
        filters = [16, 32, 48, 64, 80]
        kernels = [(3, 3)] * 5
    if size in ['XS', 'S']:
        strides = [(3, 4), (2, 2)] + [(1, 1)] * 2
    elif size in ['M', 'L']:  # M or L
        strides = [(3, 4)] + [(1, 1)] * 4
    elif size == 'R':  # size is R
        strides = [(2, 2)] + [(1, 1)] * 4
    else:
        raise ValueError('Size must be in XS, X, M, L, R not ' + str(size))

    # build CNN layers with data as above and batch norm, pooling & dropout
    for i, f, k, s in zip(range(len(filters)), filters, kernels, strides):
        conv = Conv2D(filters=f, kernel_size=k, strides=s, padding='same',
                      use_bias=False, name='conv' + str(i))
        norm = BatchNormalization(name='batch_norm' + str(i))
        act = Activation('relu', name='act' + str(i))
        pool = MaxPooling2D(pool_size=(2, 2), padding='same',
                            name='pool' + str(i))
        dropout = Dropout(rate=drop, name='drop' + str(i))
        if is_seq:
            x = TD(conv, name='td_conv' + str(i))(x)
            x = TD(norm, name='td_norm' + str(i))(x)
            x = TD(act, name='td_act' + str(i))(x)
            x = TD(pool, name='td_pool' + str(i))(x)
            x = TD(dropout, name='td_drop' + str(i))(x)
        else:
            x = conv(x)
            x = norm(x)
            x = act(x)
            # if size != 'R':
            #     x = pool(x)
            # else:
            #     if i < 3:
            #         x = pool(x)
            #     elif i == 3:
            #         x = MaxPooling2D(pool_size=(3, 3), padding='same',
            #                          name='pool' + str(i))(x)
            x = pool(x)
            x = dropout(x)

    # flatten and pack into latent vector:
    flat = Flatten(name='flattened')
    units = square_plus_dense(size)[0]
    if is_seq:
        x = TD(flat, name='td_flat')(x)
        x = LSTM(units=units,
                 kernel_regularizer=regularizers.l2(l2),
                 recurrent_regularizer=regularizers.l2(l2),
                 name='lstm_latent',
                 return_sequences=True)(x)
    else:
        x = flat(x)
        x = Dense(units=units, activation='relu',
                  kernel_regularizer=regularizers.l2(l2),
                  name='dense_latent')(x)
    output = x
    return Model(inputs=[img_in], outputs=[output], name='CNN')


def square_plus_dense(size='R'):
    d = dict(XS=[72] * 2 + [48],
             S=[96] * 3 + [48],
             M=[128] * 4 + [64],
             L=[144] * 7,
             R=[128] * 4 + [64])
    if size not in d:
        raise ValueError('size must be in', d.keys(), 'but was', size)
    return d[size]


def linear_square_plus(input_shape=(120, 160, 3), size='S', seq_len=None,
                       pos_throttle=True):
    # L2 regularisation
    l2 = 0.001
    if seq_len:
        input_shape = (seq_len,) + input_shape

    img_in = keras.Input(shape=input_shape, name='img_in')
    cnn = linear_square_plus_cnn(img_in, size, l2, seq_len is not None)

    latent = cnn(img_in)
    # outputs = square_plus_output_layers(latent, size, l2,
    #                                     seq_len=seq_len,
    #                                     pos_throttle=pos_throttle)
    controller = square_plus_controller(latent, size=size, l2=l2,
                                        seq_len=seq_len,
                                        pos_throttle=pos_throttle)
    angle_throttle = controller(latent)

    name = create_name(False, None, 0, False, seq_len, size)
    model = Model(inputs=[img_in], outputs=angle_throttle, name=name)
    model.output_names = controller.output_names
    return model


def linear_square_plus_imu(input_shape=(120, 160, 3),
                           imu_dim=6, size='S', seq_len=None,
                           pos_throttle=True):
    assert 0 < imu_dim <= 6, 'imu_dim must be number in [1,..,6]'
    l2 = 0.001
    imu_shape = (imu_dim,)
    if seq_len:
        input_shape = (seq_len,) + input_shape
        imu_shape = (seq_len,) + imu_shape
    img_in = Input(shape=input_shape, name='img_in')
    cnn = linear_square_plus_cnn(img_in, size, l2, seq_len is not None)
    x = cnn(img_in)

    imu_in = Input(shape=imu_shape, name="imu_in")
    y = imu_in
    imu_dense_size = dict(XS=20, S=24, M=36, L=48, R=36)
    imu_dense = Dense(units=imu_dense_size[size], activation='relu',
                      kernel_regularizer=regularizers.l2(l2),
                      name='dense_imu')
    y = TD(imu_dense, name='td_imu_dense')(y) if seq_len else imu_dense(y)
    z = Concatenate(name='concat_latent_imu')([x, y])
    outputs = square_plus_output_layers(z, size, l2, seq_len,
                                        pos_throttle=pos_throttle)
    name = create_name(False, None, 0, False, seq_len, size)
    inputs = [img_in, imu_in]
    model = Model(inputs=inputs, outputs=outputs, name=name)
    return model


def linear_square_plus_mem(input_shape=(120, 160, 3),
                           size='R', mem_length=3, mem_depth=0,
                           has_lap_pct=False, pos_throttle=True, lap_dim=3):
    l2 = 0.001
    drop2 = 0.1 #0.02 #0.1
    img_in = Input(shape=input_shape, name='img_in')
    cnn = linear_square_plus_cnn(img_in, size, l2)
    latent = cnn(img_in)
    mem_in = Input(shape=(2 * mem_length,), name='mem_in')
    memory = memory_model(mem_in, mem_length, mem_depth, drop2)
    mem_out = memory(mem_in)
    concat_layers = [latent, mem_out]
    concat_name = 'concat_latent_mem'
    inputs = [img_in, mem_in]
    if has_lap_pct:
        # using leaky relu here with negative branch, so we get some
        # extrapolation if we put smaller values than the minimum/maximum
        # percentile we used in training
        lap_in = Input(shape=(lap_dim,), name='lap_pct_in')
        xl = lap_in
        for i in range(3):
            xl = Dense(16, name=f'lap_{i}')(xl)
            xl = LeakyReLU(alpha=0.5)(xl)
        lap_influencer = Model(inputs=lap_in, outputs=xl,
                               name='Lap_Influencer')
        lap_out = lap_influencer(lap_in)
        concat_layers.append(lap_out)
        concat_name += '_lap'
        inputs.append(lap_in)

    x = Concatenate(name=concat_name)(concat_layers)
    # outputs = square_plus_output_layers(x, size, l2, None,
    #                                     pos_throttle=pos_throttle)
    controller = square_plus_controller(x, size=size, l2=l2,
                                        pos_throttle=pos_throttle)
    angle_throttle = controller(x)
    name = create_name(has_lap_pct, None, mem_length, True, None, size)
    model = Model(inputs=inputs, outputs=angle_throttle, name=name)
    model.output_names = controller.output_names
    return model


def memory_model(in_tensor, mem_length, mem_depth, drop):
    y = in_tensor
    for i in range(mem_depth):
        y = Dense(4 * mem_length, activation='relu', name=f'mem_{i}')(y)
        y = Dropout(drop)(y)
    for i in range(0, mem_length - 1):  # memlength # memlength-1
        y = Dense(2 * (mem_length - i), activation='relu', name=f'mem_c_{i}')(y)
        y = Dropout(drop)(y)
    model = Model(inputs=[in_tensor], outputs=[y], name='Memory')
    return model


def square_plus_output_layers(in_tensor, size, l2,
                              seq_len=None, pos_throttle=True):
    layers = square_plus_dense(size)
    z = in_tensor
    for i, l in zip(range(len(layers)), layers):
        if seq_len:
            z = LSTM(units=l,
                     kernel_regularizer=regularizers.l2(l2),
                     recurrent_regularizer=regularizers.l2(l2),
                     name='lstm' + str(i),
                     return_sequences=(i != len(layers) - 1))(z)
        else:
            z = Dense(units=l, activation='relu',
                      kernel_regularizer=regularizers.l2(l2),
                      name='dense' + str(i))(z)

    angle_out = Dense(units=1, activation='tanh', name='angle')(z)
    activation = 'sigmoid' if pos_throttle else 'tanh'
    throttle_out = Dense(units=1, activation=activation, name='throttle')(z)
    return [angle_out, throttle_out]


def square_plus_controller(in_tensor, size, l2, seq_len=None,
                           pos_throttle=True):
    out_layers = square_plus_output_layers(in_tensor, size, l2, seq_len=seq_len,
                                           pos_throttle=pos_throttle)
    model = Model(inputs=[in_tensor], outputs=out_layers, name='controller')
    return model


def create_name(has_lap_pct, imu_dim, mem_len, multi_input, seq_len, size):
    name = None

    if multi_input:
        if imu_dim:
            name = f'SquarePlusImu_{size}_{imu_dim}'
        elif has_lap_pct:
            name = f'SquarePlusMemLap_{size}_{mem_len}'
        elif mem_len:
            assert seq_len is None, "SquarePlusMem doesn't work with LSTM"
            name = f'SquarePlusMem_{size}_{mem_len}'
    else:
        name = 'SquarePlus_' + size

    if seq_len:
        name += '_lstm_' + str(seq_len)

    return name


