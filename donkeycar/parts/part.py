from donkeycar.config import Config
import logging

logger = logging.getLogger(__name__)


class CreatableFactory(type):
    """
    Metaclass to hold the registration dictionary of the part creation function
    """
    register = {}

    def __init__(cls, name, bases, dct):
        super().__init__(name, bases, dct)
        l_name = name.lower()
        # don't register base class constructor
        if l_name != 'creatable':
            cls.register[l_name] = cls

    @classmethod
    def make(mcs, concrete, cfg, kwargs):
        return mcs.register[concrete.lower()].create(cfg, kwargs)

    @classmethod
    def get_all_classes(cls):
        return list(cls.register.keys())

    @classmethod
    def get_docstring_of_class(cls, creatable):
        return cls.register[creatable].__doc__

    @classmethod
    def get_docstring_of_init_class(cls, creatable):
        return cls.register[creatable].__init__.__doc__

    @classmethod
    def get_init_args_of_class(cls, creatable):
        return list(cls.register[creatable].__init__.__code__.co_varnames)[1:]

    @classmethod
    def get_docstring_of_run(cls, creatable):
        try:
            return cls.register[creatable].run.__doc__
        except AttributeError:
            logger.warning(f'Part {creatable} has not run method')
            return ''

    @classmethod
    def get_docstring_of_run_threaded(cls, creatable):
        try:
            return cls.register[creatable].run_threaded.__doc__
        except AttributeError:
            logger.warning(f'Part {creatable} has not run_threaded method')
            return ''

    @classmethod
    def pretty_print_of_class(cls, creatable):
        s = '-' * 80 + '\n'
        s += cls.get_docstring_of_class(creatable) or '' + '\n'
        s += '-' * 80 + '\n'
        s += 'Construction of the class\n'
        s += cls.get_docstring_of_init_class(creatable) or '' + '\n'
        s += '-' * 80 + '\n'
        s += 'For non threaded parts the run method is described here:' + '\n'
        run = cls.get_docstring_of_run(creatable) or ''
        s += run or 'non threaded run is not supported' + '\n'
        s += 'For threaded parts the run method is described here:' + '\n'
        run_t = cls.get_docstring_of_run_threaded(creatable) or ''
        s += run_t or 'threaded run is not supported' + '\n'
        return s


class Creatable(object, metaclass=CreatableFactory):
    """
    Base class for factory creatable parts, implementing create() by calling
    constructor without any config
    """
    @classmethod
    def create(cls, cfg, kwargs):
        return cls(**kwargs)

    def __init__(self, **kwargs):
        self.kwargs = kwargs


class TestCreatable(Creatable):
    """
    Test Part that shows the creation of new parts
    """
    def __init__(self, value=2):
        self.value = value
        print('Created TestCreatable with value', self.value)


if __name__ == '__main__':
    # we allow any case for the part in the dictionary, as python classes are
    # expected to be camel case
    data = [{'testcreatable': None},
            {'TestCreatable': {'arguments': {'value': 4}}}]
    cfg = Config()

    for d in data:
        for k, v in d.items():
            args = {}
            if type(v) is dict and 'arguments' in v:
                args = v['arguments']
            part = CreatableFactory.make(k, cfg, args)
