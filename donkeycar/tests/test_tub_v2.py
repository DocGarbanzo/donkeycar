import shutil
import tempfile
import unittest
from datetime import datetime
from random import randint

from donkeycar.parts.tub_v2 import Tub, TubWriter


class TestTub(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """ Here we create the tub already with a session id field as if it was
            created from a TubWriter. """
        cls._path = tempfile.mkdtemp()
        cls.inputs = ['input', 'session_id']
        cls.types = ['int', 'str']
        cls.tub = Tub(cls._path, cls.inputs, cls.types)

    def test_basic_tub_operations(self):
        entries = list(self.tub)
        self.assertEqual(len(entries), 0)
        write_count = 10
        delete_indexes = [0, 8]
        date = datetime.now().strftime('%y-%m-%d')
        session_number = randint(0, 20)
        session_id = date + '_' + str(session_number)
        records = [{'input': i, 'session_id': session_id}
                   for i in range(write_count)]

        for record in records:
            self.tub.write_record(record)

        for index in delete_indexes:
            self.tub.delete_record(index)

        count = 0
        for record in self.tub:
            print(f'Record: {record}')
            count += 1

        self.assertEqual(count, (write_count - len(delete_indexes)))
        self.assertEqual(len(self.tub), (write_count - len(delete_indexes)))

    def test_basic_tubwriter_operations(self):
        path = self.tub.base_path
        tub_writer = TubWriter(path, inputs=['input'], types=['int'])
        start_length = len(tub_writer.tub)
        write_count = 10
        for i in range(write_count):
            tub_writer.run(i)

        # Check we have good session id for all new records:
        count = 0
        for record in tub_writer.tub:
            print(f'Record: {record}')
            session_number = int(record['session_id'].split('_')[1])
            if count >= start_length:
                self.assertEqual(session_number, last_session_number + 1,
                                 'Session id not correctly generated')
            else:
                last_session_number = session_number
            count += 1

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls._path)


if __name__ == '__main__':
    unittest.main()
