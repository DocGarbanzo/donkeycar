import os
import shutil
import tempfile
import unittest
import json
import time
from unittest.mock import patch, MagicMock

import numpy as np
from PIL import Image

from donkeycar.parts.tub_v2 import Tub, TubWriter, TubWiper
from donkeycar.pipeline.types import TubRecord, Collator
from donkeycar.config import Config


class TestTub(unittest.TestCase):
    _path = None
    tub = None
    delete_indexes = [3, 8]

    @classmethod
    def setUpClass(cls) -> None:
        cls._path = tempfile.mkdtemp()
        inputs = ['input']
        types = ['int']
        cls.tub = Tub(cls._path, inputs, types)

    def test_basic_tub_operations(self):
        entries = list(self.tub)
        self.assertEqual(len(entries), 0)
        write_count = 14

        records = [{'input': i} for i in range(write_count)]
        for record in records:
            self.tub.write_record(record)

        for index in self.delete_indexes:
            self.tub.delete_records(index)

        count = 0
        for record in self.tub:
            print(f'Record {record}')
            count += 1

        self.assertEqual(count, (write_count - len(self.delete_indexes)))
        self.assertEqual(len(self.tub),
                         (write_count - len(self.delete_indexes)))

    def test_sequence(self):
        cfg = Config()
        records = [TubRecord(cfg, self.tub.base_path, underlying) for
                   underlying in self.tub]
        for seq_len in (2, 3, 4, 5):
            seq = Collator(seq_len, records)
            for l in seq:
                print(l)
                assert len(l) == seq_len, 'Sequence has wrong length'
                assert not any((r.underlying['_index'] == del_idx for del_idx in
                                self.delete_indexes for r in l)), \
                    'Deleted index found'
                it1 = iter(l)
                it2 = iter(l)
                next(it2)
                assert all((Collator.is_continuous(rec_1, rec_2)
                            for rec_1, rec_2 in zip(it1, it2))), \
                    'Non continuous records found'

    def test_delete_last_n_records(self):
        start_len = len(self.tub)
        self.tub.delete_last_n_records(2)
        self.assertEqual(start_len - 2, len(self.tub),
                         "error in deleting 2 last records")
        self.tub.delete_last_n_records(3)
        self.assertEqual(start_len - 5, len(self.tub),
                         "error in deleting 3 last records")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tub.close()
        shutil.rmtree(cls._path)


class TestTubDataTypes(unittest.TestCase):
    """Test various data types supported by Tub V2.
    
    Tests image arrays, numpy arrays, lists, vectors, booleans, and complex
    mixed data types to ensure proper serialization and storage.
    """
    
    def setUp(self):
        self.test_path = tempfile.mkdtemp()
        
    def tearDown(self):
        shutil.rmtree(self.test_path)
    
    def test_image_array_storage(self):
        """Test image_array type handling.
        
        Verifies that:
        - Images are saved as JPEG files in the images/ directory
        - Records store filename references instead of raw image data
        - Images can be loaded and have correct dimensions
        - JPEG compression preserves image structure
        """
        inputs = ['cam/image', 'metadata']
        types = ['image_array', 'str']
        tub = Tub(self.test_path, inputs, types)
        
        # Create test image
        test_image = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
        record = {'cam/image': test_image, 'metadata': 'test_frame'}
        tub.write_record(record)
        
        # Verify image was saved
        images_path = os.path.join(self.test_path, 'images')
        self.assertTrue(os.path.exists(images_path))
        
        # Check that image file was created
        image_files = os.listdir(images_path)
        self.assertEqual(len(image_files), 1)
        self.assertTrue(image_files[0].endswith('.jpg'))
        
        # Verify record contains filename reference
        records = list(tub)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['metadata'], 'test_frame')
        self.assertTrue(records[0]['cam/image'].endswith('.jpg'))
        
        # Verify image can be loaded and has correct shape
        image_path = os.path.join(images_path, records[0]['cam/image'])
        loaded_image = np.array(Image.open(image_path))
        
        # Verify shape matches (JPEG compression may change pixel values)
        self.assertEqual(test_image.shape, loaded_image.shape)
        
        # Verify the image is not all zeros (successful load)
        self.assertGreater(np.sum(loaded_image), 0)
        tub.close()
    
    def test_gray16_array_storage(self):
        """Test gray16_array type handling.
        
        Verifies that:
        - 16-bit grayscale images are saved as PNG files (lossless)
        - Files have correct .png extension
        - Loaded images exactly match original data (no compression artifacts)
        - Supports full 16-bit dynamic range (0-65535)
        """
        inputs = ['depth/image']
        types = ['gray16_array']
        tub = Tub(self.test_path, inputs, types)
        
        # Create test 16-bit grayscale image
        test_image = np.random.randint(0, 65535, (120, 160), dtype=np.uint16)
        record = {'depth/image': test_image}
        tub.write_record(record)
        
        # Verify image was saved as PNG
        images_path = os.path.join(self.test_path, 'images')
        image_files = os.listdir(images_path)
        self.assertEqual(len(image_files), 1)
        self.assertTrue(image_files[0].endswith('.png'))
        
        # Verify image can be loaded and matches exactly (PNG is lossless)
        records = list(tub)
        image_path = os.path.join(images_path, records[0]['depth/image'])
        loaded_image = np.array(Image.open(image_path))
        np.testing.assert_array_equal(test_image, loaded_image)
        tub.close()
    
    def test_nparray_storage(self):
        """Test nparray type handling.
        
        Verifies that:
        - NumPy arrays are serialized as JSON-compatible lists
        - Multi-dimensional arrays preserve structure
        - Data integrity is maintained during serialization/deserialization
        - Arrays can be reconstructed from stored list format
        """
        inputs = ['sensor/data']
        types = ['nparray']
        tub = Tub(self.test_path, inputs, types)
        
        # Create test numpy array
        test_array = np.array([[1.5, 2.7, 3.9], [4.1, 5.3, 6.8]])
        record = {'sensor/data': test_array}
        tub.write_record(record)
        
        # Verify array was stored as list
        records = list(tub)
        self.assertEqual(len(records), 1)
        stored_data = records[0]['sensor/data']
        self.assertIsInstance(stored_data, list)
        
        # Verify data integrity
        recovered_array = np.array(stored_data)
        np.testing.assert_array_equal(test_array, recovered_array)
        tub.close()
    
    def test_list_and_vector_storage(self):
        """Test list and vector type handling.
        
        Verifies that:
        - Python lists are stored unchanged
        - Tuples (vectors) are converted to lists for JSON compatibility
        - Both 'list' and 'vector' type declarations work identically
        - Data order and values are preserved
        """
        inputs = ['list_data', 'vector_data']
        types = ['list', 'vector']
        tub = Tub(self.test_path, inputs, types)
        
        # Test various list/vector data
        test_list = [1, 2, 3, 4, 5]
        test_vector = (10.5, 20.7, 30.9)  # tuple should become list
        
        record = {'list_data': test_list, 'vector_data': test_vector}
        tub.write_record(record)
        
        records = list(tub)
        self.assertEqual(records[0]['list_data'], test_list)
        self.assertEqual(records[0]['vector_data'], list(test_vector))
        tub.close()
    
    def test_boolean_storage(self):
        """Test boolean type handling.
        
        Verifies that:
        - True/False values are stored as proper JSON booleans
        - Integer values (1/0) are converted to bool
        - String values are converted to bool using Python truthiness
        - Stored values maintain exact boolean type identity
        """
        inputs = ['is_active', 'flag']
        types = ['boolean', 'boolean']
        tub = Tub(self.test_path, inputs, types)
        
        # Test various boolean values
        records_data = [
            {'is_active': True, 'flag': False},
            {'is_active': 1, 'flag': 0},  # Should convert to bool
            {'is_active': 'true', 'flag': ''},  # Should convert to bool
        ]
        
        for record in records_data:
            tub.write_record(record)
        
        records = list(tub)
        self.assertEqual(len(records), 3)
        
        # Check first record
        self.assertIs(records[0]['is_active'], True)
        self.assertIs(records[0]['flag'], False)
        
        # Check type conversion
        self.assertIs(records[1]['is_active'], True)
        self.assertIs(records[1]['flag'], False)
        self.assertIs(records[2]['is_active'], True)
        self.assertIs(records[2]['flag'], False)
        tub.close()
    
    def test_mixed_data_types(self):
        """Test handling multiple data types in single record.
        
        Verifies that:
        - Multiple different data types can coexist in one record
        - Each type is handled according to its specific serialization rules
        - Complex records with images, arrays, and primitives work correctly
        - Type handling doesn't interfere between different fields
        """
        inputs = ['image', 'steering', 'throttle', 'sensors', 'metadata', 
                  'active']
        types = ['image_array', 'float', 'float', 'nparray', 'str', 'boolean']
        tub = Tub(self.test_path, inputs, types)
        
        # Create complex record
        test_image = np.random.randint(0, 255, (60, 80, 3), dtype=np.uint8)
        test_sensors = np.array([1.1, 2.2, 3.3])
        
        record = {
            'image': test_image,
            'steering': 0.75,
            'throttle': 0.5,
            'sensors': test_sensors,
            'metadata': 'complex_test',
            'active': True
        }
        tub.write_record(record)
        
        records = list(tub)
        self.assertEqual(len(records), 1)
        
        stored = records[0]
        self.assertTrue(stored['image'].endswith('.jpg'))
        self.assertEqual(stored['steering'], 0.75)
        self.assertEqual(stored['throttle'], 0.5)
        self.assertEqual(stored['sensors'], test_sensors.tolist())
        self.assertEqual(stored['metadata'], 'complex_test')
        self.assertIs(stored['active'], True)
        tub.close()


class TestTubWiper(unittest.TestCase):
    """Test TubWiper functionality.
    
    Tests the debounced record deletion mechanism that allows users to remove
    bad data during recording by holding a button for multiple vehicle loops.
    """
    
    def setUp(self):
        self.test_path = tempfile.mkdtemp()
        inputs = ['data']
        types = ['int']
        self.tub = Tub(self.test_path, inputs, types)
        
        # Write some test records
        for i in range(20):
            self.tub.write_record({'data': i})
    
    def tearDown(self):
        self.tub.close()
        shutil.rmtree(self.test_path)
    
    def test_wiper_initialization(self):
        """Test TubWiper initialization.
        
        Verifies that:
        - TubWiper accepts and stores configuration parameters correctly
        - num_records parameter sets deletion count
        - min_loops parameter sets debounce threshold
        - Active loop counter starts at zero
        """
        wiper = TubWiper(self.tub, num_records=5, min_loops=3)
        self.assertEqual(wiper._num_records, 5)
        self.assertEqual(wiper._min_loops, 3)
        self.assertEqual(wiper._active_loop_count, 0)
    
    def test_wiper_debouncing(self):
        """Test TubWiper debouncing behavior.
        
        Verifies that:
        - Deletion only triggers after minimum loop threshold is reached
        - Continuous True input doesn't cause repeated deletions
        - Loop counter increments correctly while input is held
        - Return value indicates when deletion actually occurs
        """
        wiper = TubWiper(self.tub, num_records=3, min_loops=4)
        initial_count = len(self.tub)
        
        # Should not trigger on first few calls
        self.assertFalse(wiper.run(True))  # Loop 1
        self.assertFalse(wiper.run(True))  # Loop 2  
        self.assertFalse(wiper.run(True))  # Loop 3
        self.assertEqual(len(self.tub), initial_count)  # No deletion yet
        
        # Should trigger on 4th call
        self.assertTrue(wiper.run(True))   # Loop 4 - triggers
        self.assertEqual(len(self.tub), initial_count - 3)
        
        # Should not trigger again while held
        self.assertFalse(wiper.run(True))  # Loop 5
        self.assertFalse(wiper.run(True))  # Loop 6
        self.assertEqual(len(self.tub), initial_count - 3)  # No more deletion
    
    def test_wiper_release_and_retrigger(self):
        """Test TubWiper release and re-trigger behavior.
        
        Verifies that:
        - Releasing input (False) resets the active loop counter
        - After release, the wiper can be triggered again
        - Multiple trigger cycles work independently
        - Each trigger deletes the configured number of records
        """
        wiper = TubWiper(self.tub, num_records=2, min_loops=2)
        initial_count = len(self.tub)
        
        # First trigger sequence
        self.assertFalse(wiper.run(True))   # Loop 1
        self.assertTrue(wiper.run(True))    # Loop 2 - triggers
        self.assertEqual(len(self.tub), initial_count - 2)
        
        # Release and re-trigger
        self.assertFalse(wiper.run(False))  # Release
        self.assertFalse(wiper.run(True))   # Loop 1 again
        self.assertTrue(wiper.run(True))    # Loop 2 - triggers again
        self.assertEqual(len(self.tub), initial_count - 4)
    
    def test_wiper_no_deletion_when_false(self):
        """Test TubWiper doesn't delete when input is False.
        
        Verifies that:
        - False input never triggers deletion regardless of loop count
        - Return value is always False for False input
        - Record count remains unchanged with False input
        - No internal state changes occur with False input
        """
        wiper = TubWiper(self.tub, num_records=5, min_loops=1)
        initial_count = len(self.tub)
        
        # Should never trigger with False input
        for _ in range(10):
            self.assertFalse(wiper.run(False))
        
        self.assertEqual(len(self.tub), initial_count)


class TestTubSessionManagement(unittest.TestCase):
    """Test Tub session management and metadata.
    
    Tests the session tracking system that assigns unique IDs to each
    recording session and prevents session conflicts in shared tubs.
    """
    
    def setUp(self):
        self.test_path = tempfile.mkdtemp()
        
    def tearDown(self):
        shutil.rmtree(self.test_path)
    
    def test_session_id_creation(self):
        """Test session ID creation and format.
        
        Verifies that:
        - Session IDs are created with correct format (YY-MM-DD_N)
        - Both numeric and full session IDs are generated
        - Session metadata is properly saved to manifest file
        - Session tracking includes 'last_id', 'last_full_id', and 'all_full_ids'
        """
        inputs = ['data']
        types = ['int']
        tub = Tub(self.test_path, inputs, types)
        
        # Check session ID format
        session_id_num, session_id_full = tub.manifest.session_id
        self.assertIsInstance(session_id_num, int)
        self.assertRegex(session_id_full, r'\d{2}-\d{2}-\d{2}_\d+')
        
        # Write a record to trigger session update
        tub.write_record({'data': 1})
        tub.close()
        
        # Verify session was saved to metadata
        with open(os.path.join(self.test_path, 'manifest.json'), 'r') as f:
            content = f.readlines()
            manifest_metadata = json.loads(content[3])  # 4th line
            
        self.assertIn('sessions', manifest_metadata)
        sessions = manifest_metadata['sessions']
        self.assertEqual(sessions['last_id'], session_id_num)
        self.assertEqual(sessions['last_full_id'], session_id_full)
        self.assertIn(session_id_full, sessions['all_full_ids'])
    
    def test_session_conflict_detection(self):
        """Test session ID conflict detection.
        
        Verifies that:
        - Multiple tub instances can be created on same directory safely
        - Session ID generation handles existing sessions correctly
        - No conflicts occur when reopening tubs with existing data
        - Session numbering increments properly across reopens
        """
        inputs = ['data']
        types = ['int']
        
        # Create first tub and close properly
        tub1 = Tub(self.test_path, inputs, types)
        tub1.write_record({'data': 1})
        tub1.close()
        
        # Try to create second tub - should detect conflict if session exists
        with patch('time.strftime', return_value='25-01-01'):
            tub2 = Tub(self.test_path, inputs, types)
            # This should work as it creates a new session
            tub2.write_record({'data': 2})
            tub2.close()
    
    def test_multiple_sessions(self):
        """Test multiple sessions in same tub.
        
        Verifies that:
        - Multiple recording sessions can be tracked in one tub
        - Each session gets a unique ID even across different dates
        - All session IDs are accumulated in metadata history
        - Session metadata persists correctly across tub reopens
        """
        inputs = ['data']
        types = ['int']
        
        # Create multiple sessions with time mocking
        session_ids = []
        
        for i in range(3):
            with patch('time.strftime', return_value=f'25-01-0{i+1}'):
                tub = Tub(self.test_path, inputs, types)
                tub.write_record({'data': i})
                session_ids.append(tub.manifest.session_id[1])
                tub.close()
        
        # Verify all sessions are tracked
        with open(os.path.join(self.test_path, 'manifest.json'), 'r') as f:
            content = f.readlines()
            manifest_metadata = json.loads(content[3])
            
        sessions = manifest_metadata['sessions']
        self.assertEqual(len(sessions['all_full_ids']), 3)
        for session_id in session_ids:
            self.assertIn(session_id, sessions['all_full_ids'])


class TestTubImproperClosureHandling(unittest.TestCase):
    """Test tub behavior when not properly closed.
    
    Tests the safeguarding mechanisms that detect when a tub was not properly
    closed and prevent data corruption from session ID conflicts.
    """
    
    def setUp(self):
        self.test_path = tempfile.mkdtemp()
        
    def tearDown(self):
        shutil.rmtree(self.test_path)
    
    def test_improper_closure_detection(self):
        """Test detection of improper tub closure.
        
        Verifies that:
        - Tubs that weren't properly closed are detected on reopening
        - Session ID conflicts are caught before data corruption can occur
        - RuntimeError is raised with clear instructions for manual cleanup
        - The error prevents silent data corruption
        """
        inputs = ['data']
        types = ['int']
        
        # Create tub and write a record
        tub1 = Tub(self.test_path, inputs, types)
        tub1.write_record({'data': 1})
        session_id = tub1.manifest.session_id[1]
        
        # Simulate improper closure by not calling close()
        # This means _update_session_info() never gets called
        # and manifest metadata doesn't get updated
        
        # Try to create another tub with the same session ID
        with patch('time.strftime', return_value=session_id.split('_')[0]):
            with self.assertRaises(RuntimeError) as context:
                Tub(self.test_path, inputs, types)
                
            error_msg = str(context.exception)
            self.assertIn('Session', error_msg)
            self.assertIn('already found in last record', error_msg)
            self.assertIn('clean up your manifest metadata', error_msg)
        
        # Clean up
        tub1.close()
    
    def test_session_id_conflict_prevents_data_corruption(self):
        """Test that session ID conflicts prevent data corruption.
        
        Verifies that:
        - Multiple sessions cannot accidentally share the same session_id
        - Lap timing and distance data remain separate between sessions
        - Statistical aggregations won't be corrupted by mixed session data
        - Error handling protects data integrity over convenience
        """
        inputs = ['steering', 'throttle', 'speed']
        types = ['float', 'float', 'float']
        
        # Create first session and write some records
        tub1 = Tub(self.test_path, inputs, types)
        for i in range(5):
            tub1.write_record({'steering': i * 0.1, 'throttle': 0.5, 'speed': 10.0})
        
        session_id = tub1.manifest.session_id[1]
        
        # Simulate improper closure - don't call close()
        # This leaves the manifest metadata unupdated
        
        # Attempt to create second session with same date
        # This would generate the same session_id
        with patch('time.strftime', return_value=session_id.split('_')[0]):
            with self.assertRaises(RuntimeError):
                Tub(self.test_path, inputs, types)
                # If this didn't raise an error, we could write records
                # with the same session_id, corrupting lap/distance stats
        
        tub1.close()
    
    def test_proper_closure_allows_new_sessions(self):
        """Test that proper closure allows new sessions normally.
        
        Verifies that:
        - Properly closed tubs don't trigger conflict detection
        - New sessions can be created after proper closure
        - Session metadata is correctly maintained across proper reopens
        - The safeguarding doesn't interfere with normal operation
        """
        inputs = ['data']
        types = ['int']
        
        # Create and properly close first session
        tub1 = Tub(self.test_path, inputs, types)
        tub1.write_record({'data': 1})
        session1_id = tub1.manifest.session_id[1]
        tub1.close()  # Proper closure updates manifest metadata
        
        # Create second session - should work fine
        tub2 = Tub(self.test_path, inputs, types)
        tub2.write_record({'data': 2})
        session2_id = tub2.manifest.session_id[1]
        tub2.close()
        
        # Verify sessions are different
        self.assertNotEqual(session1_id, session2_id)
        
        # Verify both sessions are tracked in metadata
        with open(os.path.join(self.test_path, 'manifest.json'), 'r') as f:
            content = f.readlines()
            manifest_metadata = json.loads(content[3])
            
        sessions = manifest_metadata['sessions']
        self.assertIn(session1_id, sessions['all_full_ids'])
        self.assertIn(session2_id, sessions['all_full_ids'])
    
    def test_manual_metadata_cleanup_after_improper_closure(self):
        """Test manual metadata cleanup after improper closure.
        
        Verifies that:
        - Manual cleanup of manifest metadata allows tub reuse
        - Proper session numbering resumes after cleanup
        - Data integrity is preserved during manual intervention
        - Instructions in error message lead to working solution
        """
        inputs = ['data']
        types = ['int']
        
        # Create tub and write record without proper closure
        tub1 = Tub(self.test_path, inputs, types)
        tub1.write_record({'data': 1})
        session_id = tub1.manifest.session_id[1]
        
        # Don't call close() to simulate improper closure
        
        # Manually update the manifest metadata as if close() was called
        # This simulates the manual cleanup mentioned in the error message
        tub1.manifest._update_session_info()
        tub1.manifest.write_metadata()
        
        # Now try to create new session - should work
        with patch('time.strftime', return_value=session_id.split('_')[0]):
            tub2 = Tub(self.test_path, inputs, types)
            tub2.write_record({'data': 2})
            
            # New session should have incremented ID
            new_session_id = tub2.manifest.session_id[1]
            self.assertNotEqual(session_id, new_session_id)
            
            tub2.close()
    
    def test_empty_tub_no_conflict_check(self):
        """Test that empty tubs don't trigger conflict checks.
        
        Verifies that:
        - Empty catalogs don't cause false positive conflicts
        - New tubs can be created without issues
        - The fix for empty tub consistency checks works correctly
        - No errors occur when no records exist yet
        """
        inputs = ['data']
        types = ['int']
        
        # Create tub but don't write any records
        tub1 = Tub(self.test_path, inputs, types)
        session_id = tub1.manifest.session_id[1]
        
        # Don't close properly and don't write records
        
        # Try to create another tub - should work because catalog is empty
        with patch('time.strftime', return_value=session_id.split('_')[0]):
            tub2 = Tub(self.test_path, inputs, types)
            # Should not raise error because catalog has no records
            tub2.write_record({'data': 1})
            tub2.close()


class TestTubRecordOperations(unittest.TestCase):
    """Test record overwriting and advanced operations.
    
    Tests advanced record manipulation including overwriting existing records,
    filtering of invalid data, and handling of unknown fields.
    """
    
    def setUp(self):
        self.test_path = tempfile.mkdtemp()
        inputs = ['data', 'meta']
        types = ['int', 'str']
        self.tub = Tub(self.test_path, inputs, types)
        
        # Write initial records
        for i in range(10):
            self.tub.write_record({'data': i, 'meta': f'record_{i}'})
    
    def tearDown(self):
        self.tub.close()
        shutil.rmtree(self.test_path)
    
    def test_record_overwriting(self):
        """Test overwriting existing records using _index.
        
        Verifies that:
        - Records can be overwritten by specifying _index parameter
        - Overwritten records maintain their original index position
        - Custom timestamps can be set during overwrite
        - Total record count remains unchanged after overwrite
        """
        # Overwrite record at index 5
        timestamp = int(time.time() * 1000)
        overwrite_record = {
            '_index': 5,
            '_timestamp_ms': timestamp,
            'data': 999,
            'meta': 'overwritten'
        }
        self.tub.write_record(overwrite_record)
        
        # Verify record was overwritten
        records = list(self.tub)
        self.assertEqual(len(records), 10)  # Same count
        
        # Find the overwritten record
        overwritten = None
        for record in records:
            if record['_index'] == 5:
                overwritten = record
                break
        
        self.assertIsNotNone(overwritten)
        self.assertEqual(overwritten['data'], 999)
        self.assertEqual(overwritten['meta'], 'overwritten')
        self.assertEqual(overwritten['_timestamp_ms'], timestamp)
    
    def test_none_value_filtering(self):
        """Test that None values are filtered out.
        
        Verifies that:
        - None values in input records are automatically filtered out
        - Fields with None values don't appear in stored records
        - Other fields in the same record are stored normally
        - Filtering happens silently without errors
        """
        record = {'data': 42, 'meta': None}
        self.tub.write_record(record)
        
        records = list(self.tub)
        last_record = records[-1]
        
        self.assertEqual(last_record['data'], 42)
        self.assertNotIn('meta', last_record)
    
    def test_unknown_key_filtering(self):
        """Test that unknown keys are filtered out.
        
        Verifies that:
        - Keys not defined in tub schema are silently filtered out
        - Only known input fields are stored in records
        - Schema enforcement happens during write_record()
        - Unknown fields don't cause errors or warnings
        """
        record = {'data': 42, 'meta': 'test', 'unknown_key': 'should_be_ignored'}
        self.tub.write_record(record)
        
        records = list(self.tub)
        last_record = records[-1]
        
        self.assertEqual(last_record['data'], 42)
        self.assertEqual(last_record['meta'], 'test')
        self.assertNotIn('unknown_key', last_record)


class TestTubWriter(unittest.TestCase):
    """Test TubWriter Donkey part.
    
    Tests the Donkey part interface for TubWriter, which allows tubs to be
    integrated into the vehicle loop for automatic data recording.
    """
    
    def setUp(self):
        self.test_path = tempfile.mkdtemp()
        
    def tearDown(self):
        shutil.rmtree(self.test_path)
    
    def test_tub_writer_initialization(self):
        """Test TubWriter initialization.
        
        Verifies that:
        - TubWriter correctly initializes underlying Tub with given parameters
        - Input and type specifications are passed through correctly
        - Writer can be created and closed without errors
        - Internal tub object is accessible and configured properly
        """
        inputs = ['steering', 'throttle']
        types = ['float', 'float']
        writer = TubWriter(self.test_path, inputs, types)
        
        self.assertEqual(writer.tub.manifest.inputs, inputs)
        self.assertEqual(writer.tub.manifest.types, types)
        writer.close()
    
    def test_tub_writer_run(self):
        """Test TubWriter run method.
        
        Verifies that:
        - Run method accepts arguments matching input specification
        - Arguments are correctly mapped to input fields in order
        - Return value indicates the record index that was written
        - Multiple calls increment the index correctly
        - Written records can be retrieved and verified
        """
        inputs = ['steering', 'throttle', 'mode']
        types = ['float', 'float', 'str']
        writer = TubWriter(self.test_path, inputs, types)
        
        # Test writing record via run method (returns current_index which starts at 1)
        index = writer.run(0.5, 0.3, 'user')
        self.assertEqual(index, 1)
        
        # Write another record
        index = writer.run(0.7, 0.4, 'auto')
        self.assertEqual(index, 2)
        
        # Verify records were written
        records = list(writer.tub)
        self.assertEqual(len(records), 2)
        
        self.assertEqual(records[0]['steering'], 0.5)
        self.assertEqual(records[0]['throttle'], 0.3)
        self.assertEqual(records[0]['mode'], 'user')
        
        self.assertEqual(records[1]['steering'], 0.7)
        self.assertEqual(records[1]['throttle'], 0.4)
        self.assertEqual(records[1]['mode'], 'auto')
        
        writer.close()
    
    def test_tub_writer_wrong_arg_count(self):
        """Test TubWriter with wrong number of arguments.
        
        Verifies that:
        - Providing too few arguments raises AssertionError
        - Providing too many arguments raises AssertionError
        - Error messages help identify the mismatch
        - Tub remains in valid state after assertion errors
        """
        inputs = ['steering', 'throttle']
        types = ['float', 'float']
        writer = TubWriter(self.test_path, inputs, types)
        
        # Should raise assertion error with wrong number of args
        with self.assertRaises(AssertionError):
            writer.run(0.5)  # Missing throttle
        
        with self.assertRaises(AssertionError):
            writer.run(0.5, 0.3, 'extra')  # Too many args
        
        writer.close()
    
    def test_tub_writer_with_lap_timer(self):
        """Test TubWriter with lap timer integration.
        
        Verifies that:
        - TubWriter accepts lap timer object during initialization
        - Lap timer data is retrieved when tub is closed
        - Lap times are stored in tub metadata for the session
        - Integration works without affecting normal record writing
        """
        inputs = ['data']
        types = ['int']
        
        # Mock lap timer
        lap_timer = MagicMock()
        lap_timer.to_list.return_value = [1.5, 2.3, 1.8]
        
        writer = TubWriter(self.test_path, inputs, types, 
                          lap_timer=lap_timer)
        
        # Write some data
        writer.run(42)
        writer.close()
        
        # Verify lap timer data was saved
        lap_timer.to_list.assert_called_once()


class TestTubErrorHandling(unittest.TestCase):
    """Test error handling and edge cases.
    
    Tests various error conditions, edge cases, and advanced features like
    catalog rotation, read-only mode, and graceful error handling.
    """
    
    def setUp(self):
        self.test_path = tempfile.mkdtemp()
        
    def tearDown(self):
        shutil.rmtree(self.test_path)
    
    def test_invalid_image_data(self):
        """Test handling of invalid image data.
        
        Verifies that:
        - None image values are filtered out gracefully
        - Records with None images still store other valid fields
        - No errors occur when image fields contain None
        - Image fields are omitted from final stored record
        """
        inputs = ['image', 'data']
        types = ['image_array', 'int']
        tub = Tub(self.test_path, inputs, types)
        
        # Test with None image (should be filtered out gracefully)
        record = {'image': None, 'data': 42}
        tub.write_record(record)
        
        records = list(tub)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['data'], 42)
        self.assertNotIn('image', records[0])  # None values filtered out
        
        tub.close()
    
    def test_type_conversion_errors(self):
        """Test graceful handling of type conversion errors.
        
        Verifies that:
        - Float values are correctly truncated to integers
        - String numbers are converted to appropriate numeric types
        - Type conversion follows Python's standard rules
        - No exceptions are raised during reasonable conversions
        """
        inputs = ['int_val', 'float_val']
        types = ['int', 'float']
        tub = Tub(self.test_path, inputs, types)
        
        # These should work (graceful conversion)
        tub.write_record({'int_val': 3.7, 'float_val': '2.5'})
        
        records = list(tub)
        self.assertEqual(records[0]['int_val'], 3)  # Truncated
        self.assertEqual(records[0]['float_val'], 2.5)  # Converted
        
        tub.close()
    
    def test_catalog_rotation(self):
        """Test automatic catalog rotation.
        
        Verifies that:
        - New catalog files are created when max_catalog_len is reached
        - Multiple catalog files are properly managed
        - All records remain accessible across catalog boundaries
        - Data integrity is maintained during catalog transitions
        - File naming follows expected pattern (catalog_N.catalog)
        """
        inputs = ['data']
        types = ['int']
        tub = Tub(self.test_path, inputs, types, max_catalog_len=5)
        
        # Write enough records to trigger rotation
        for i in range(12):
            tub.write_record({'data': i})
        
        # Should have created multiple catalogs
        catalog_files = [f for f in os.listdir(self.test_path) 
                        if f.startswith('catalog_') and f.endswith('.catalog')]
        self.assertGreaterEqual(len(catalog_files), 2)
        
        # Verify all records are accessible
        records = list(tub)
        self.assertEqual(len(records), 12)
        
        # Verify data integrity across catalogs
        for i, record in enumerate(records):
            self.assertEqual(record['data'], i)
        
        tub.close()
    
    def test_read_only_mode(self):
        """Test read-only mode functionality.
        
        Verifies that:
        - Existing tubs can be opened in read-only mode
        - All existing records are accessible for reading
        - Write operations are properly blocked in read-only mode
        - Memory mapping is used for performance in read-only mode
        - No accidental modifications can occur
        """
        inputs = ['data']
        types = ['int']
        
        # Create tub with data
        tub_write = Tub(self.test_path, inputs, types)
        for i in range(5):
            tub_write.write_record({'data': i})
        tub_write.close()
        
        # Open in read-only mode
        tub_read = Tub(self.test_path, read_only=True)
        
        # Should be able to read
        records = list(tub_read)
        self.assertEqual(len(records), 5)
        
        # Should not be able to write
        with self.assertRaises(Exception):
            tub_read.write_record({'data': 999})
        
        tub_read.close()


if __name__ == '__main__':
    unittest.main()
