import tensorflow as tf
import numpy as np


def keras_model_to_tflite(in_filename, out_filename, data_gen=None):
    verStr = tf.__version__
    if verStr.find('1.1')  == 0: # found MAJOR.MINOR match for version 1.1x.x
        converter = tf.lite.TFLiteConverter.from_keras_model_file(in_filename)
    if verStr.find('2.')  == 0: # found MAJOR.MINOR match for version 2.x.x
        new_model= tf.keras.models.load_model(in_filename) #filepath="keras_model.h5")
        converter = tf.lite.TFLiteConverter.from_keras_model(new_model)
    if data_gen is not None:
        #when we have a data_gen that is the trigger to use it to 
        #create integer weights and calibrate them. Warning: this model will
        #no longer run with the standard tflite engine. That uses only float.
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.representative_dataset = data_gen
        try:
            converter.target_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        except:
            pass
        try:
            converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        except:
            pass
        converter.inference_input_type = tf.uint8
        converter.inference_output_type = tf.uint8
        print("----- using data generator to create int optimized weights for Coral TPU -----")
    tflite_model = converter.convert()
    open(out_filename, "wb").write(tflite_model)


def keras_session_to_tflite(model, out_filename):
    inputs = model.inputs
    outputs = model.outputs
    with tf.keras.backend.get_session() as sess:        
        converter = tf.lite.TFLiteConverter.from_session(sess, inputs, outputs)
        tflite_model = converter.convert()
        open(out_filename, "wb").write(tflite_model)


class TFLitePilot(object):
    '''
    Base class for TFlite models that will provide steering and throttle to guide a car.
    '''
    def __init__(self):
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        self.input_shape_0 = None
        self.input_shape_1 = None

    def load(self, model_path):
        # Load TFLite model and allocate tensors.
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        # Get input and output tensors.
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Get Input shape
        self.input_shape_0 = self.input_details[0]['shape']
        self.input_shape_1 = None
        if len(self.input_details) > 1:
            self.input_shape_1 = self.input_details[1]['shape']
        print('Load model with tflite input tensor details:')
        for l in self.input_details:
            print(l)

    def run(self, image, imu_in=None):
        input_data = image.reshape(self.input_shape_0).astype('float32')
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        if imu_in is not None:
            imu_arr = np.array(imu_in)
            imu_data = imu_arr.reshape(self.input_shape_1).astype('float32')
            self.interpreter.set_tensor(self.input_details[1]['index'],
                                        imu_data)
        self.interpreter.invoke()

        steering = 0.0
        throttle = 0.0
        outputs = []

        for tensor in self.output_details:
            output_data = self.interpreter.get_tensor(tensor['index'])
            outputs.append(output_data[0][0])

        if len(outputs) > 1:
            steering = outputs[0]
            throttle = outputs[1]

        elif len(outputs) > 0:
            steering = outputs[0]

        return steering, throttle

    def get_input_shape(self):
        assert self.input_shape_0 is not None, "Need to load model first"
        return tuple(self.input_shape_0)

    def compile(self):
        pass

    def update(self, tflite_pilot=None):
        if tflite_pilot is None:
            return
        assert isinstance(tflite_pilot, TFLitePilot), \
            'Can only update TFLitePilot from TFLitePilot but not from ' \
            + type(tflite_pilot).__name__
        self.interpreter = tflite_pilot.interpreter
        self.input_details = tflite_pilot.input_details
        self.output_details = tflite_pilot.output_details
        self.input_shape_0 = tflite_pilot.input_shape_0
        self.input_shape_1 = tflite_pilot.input_shape_1
