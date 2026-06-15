# -*- coding: utf-8 -*-
from copy import copy
from tempfile import gettempdir

import pytest

from donkeycar.templates import complete
import donkeycar as dk
import os

from .setup import default_template, d2_path, custom_template


def test_config():
    path = default_template(d2_path(gettempdir()))
    cfg = dk.load_config(os.path.join(path, 'config.py'))
    assert (cfg is not None)


def test_drive():
    path = custom_template(d2_path(gettempdir()), template="complete")
    myconfig = open(os.path.join(path, 'myconfig.py'), "wt")
    myconfig.write("CAMERA_TYPE = 'MOCK'\n")
    myconfig.write("USE_SSD1306_128_32 = False \n")
    myconfig.write("DRIVE_TRAIN_TYPE = 'None'")
    myconfig.close()
    cfg = dk.load_config(os.path.join(path, 'config.py'))
    cfg.MAX_LOOPS = 10
    complete.drive(cfg=cfg)


def test_custom_templates():
    template_names = ["complete", "basic", "square"]
    for template in template_names:
        path = custom_template(d2_path(gettempdir()), template=template)
        cfg = dk.load_config(os.path.join(path, 'config.py'))
        assert (cfg is not None)
        mcfg = dk.load_config(os.path.join(path, 'myconfig.py'))
        assert (mcfg is not None)


@pytest.fixture
def overwrite():
    return dict(ROI_CROP_TOP=88, TRANSFORMATIONS=['CROP'], SEQUENCE_LENGTH=8)


sub_set = [[], [0], [2], [0, 1]]


@pytest.mark.parametrize('sub_set', sub_set)
def test_config_overwrite(overwrite, sub_set):
    path = default_template(d2_path(gettempdir()))
    cfg = dk.load_config(os.path.join(path, 'config.py'))
    cfg_orig = copy(cfg)

    # check overwrite values are different
    for k, v in overwrite.items():
        assert getattr(cfg, k) != v, \
            "Config and overwrite values should be different"

    # select keys that should be used in overwrite
    use = [list(overwrite.keys())[i] for i in sub_set]

    # now overwrite
    cfg.from_dict(overwrite, use)
    # note, if use is empty all keys of overwrite will be used hence update
    # the list for checking if [] was passed
    use_effective = use or overwrite.keys()
    # check all values present in overwrite have been overwritten
    for k, v in cfg.__dict__.items():
        if k in use_effective:
            assert v == overwrite[k], \
                "Config and overwrite values should be same"
        else:
            assert v == cfg_orig.__dict__[k], \
                "Config and original config values should be same"
