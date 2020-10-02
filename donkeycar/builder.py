import yaml
from donkeycar.parts.part import PartFactory
from donkeycar import Vehicle
# -- !!! THIS import cannot be removed as otherwise the metaclass
# initialisation does not run for all parts !!!
from donkeycar.parts.camera import *


class Builder:
    """
    Class that builds donkey parts from a yaml file
    """
    def __init__(self, cfg, car_file='car.yml'):
        """
        :param cfg:         car config file
        :param car_file:    car construction recipe
        """
        self.cfg = cfg
        self.car_file = car_file
        self.verbose = False

    def insert_config(self, parameters):
        """
        Function to replace parameters called cfg.xyz or cfg.XYZ with XYZ
        from the car config file
        :param dict parameters:  input parameters
        :return:                updated parameters
        :rtype:                 dict
        """
        if parameters is None:
            return
        for k, v in parameters.items():
            if type(v) is str and v[:4].lower() == 'cfg.':
                parameters[k] = getattr(self.cfg, v[4:].upper())

    def build_vehicle(self):
        with open(self.car_file) as f:
            obj_file = yaml.load(f, Loader=yaml.FullLoader)

        self.verbose = obj_file.get('verbose', False)
        parts = obj_file.get('parts')
        car = Vehicle()

        for part in parts:
            for part_name, part_params in part.items():
                # we are using .get on part parameters here as the part might
                # might not require any
                part_args = part_params.get('parameters')
                # updated part parameters with config values
                self.insert_config(part_args)
                # this creates the part
                vehicle_part = PartFactory.make(part_name, part_args)
                inputs = part_params.get('inputs', [])
                outputs = part_params.get('outputs', [])
                threaded = part_params.get('threaded', False)
                run_condition = None
                # adding part to vehicle
                car.add(vehicle_part, inputs=inputs, outputs=outputs,
                        threaded=threaded, run_condition=run_condition)

        return car, self.vehicle_hz, self.max_loop_count, self.verbose

