#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import unittest
import numpy as np
from donkeycar.memory import Memory


class TestMemory(unittest.TestCase):

    def test_setitem_single_item(self):
        mem = Memory()
        mem['myitem'] = 999
        assert mem['myitem'] == 999

    def test_setitem_multi_items(self):
        mem = Memory()
        mem[('myitem1', 'myitem2')] = [888, '999']
        assert mem[('myitem1', 'myitem2')] == [888, '999']

    def test_put_single_item(self):
        mem = Memory()
        mem.put(['myitem'], 999)
        assert mem['myitem'] == 999

    def test_put_single_item_as_tuple(self):
        mem = Memory()
        mem.put(('myitem',), 999)
        assert mem['myitem'] == 999

    def test_put_multi_item(self):
        mem = Memory()
        mem.put(['my1stitem','my2nditem'], [777, '999'])
        assert mem['my1stitem'] == 777
        assert mem['my2nditem'] == '999'

    def test_put_multi_item_as_tuple(self):
        mem = Memory()
        mem.put(('my1stitem','my2nditem'), (777, '999'))
        assert mem['my1stitem'] == 777
        assert mem['my2nditem'] == '999'

    def test_get_multi_item(self):
        mem = Memory()
        mem.put(['my1stitem','my2nditem'], [777, '999'])
        assert mem.get(['my1stitem','my2nditem']) == [777, '999']

    def test_update_item(self):
        mem = Memory()
        mem.put(['myitem'], 888)
        assert mem['myitem'] == 888

        mem.update({'myitem': '111'})
        assert mem['myitem'] == '111'

    def test_get_keys(self):
        mem = Memory()
        mem.put(['myitem'], 888)
        assert list(mem.keys()) == ['myitem']

    def test_get_values(self):
        mem = Memory()
        mem.put(['myitem'], 888)
        assert list(mem.values()) == [888]

    def test_get_iter(self):
        mem = Memory()
        mem.put(['myitem'], 888)
        assert dict(mem.items()) == {'myitem': 888}

    def test_np_array(self):
        mem = Memory()
        arr = np.full((120, 160, 3), fill_value=124).astype(np.uint8)
        mem.put(['cam/image_array'], arr)
