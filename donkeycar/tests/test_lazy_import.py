import subprocess
import sys
from pathlib import Path

import pytest

MODULE_AND_FORBIDDEN_PACKAGE = [
    ('donkeycar.parts.image_transformations', 'cv2'),
    ('donkeycar.management.ui.pilot_screen', 'cv2'),
    ('donkeycar.management.ui.ui', 'cv2'),
    ('donkeycar.management.ui.train_screen', 'tensorflow'),
]


class TestLazyImport:
    @pytest.mark.parametrize(
        'module_name,forbidden_package', MODULE_AND_FORBIDDEN_PACKAGE)
    def test_import_does_not_load_forbidden_package(
            self, module_name, forbidden_package):
        root = Path(__file__).resolve().parents[2]
        code = f"""
import importlib
import sys

importlib.import_module('{module_name}')
print('{forbidden_package}' in sys.modules)
"""
        result = subprocess.run(
            [sys.executable, '-c', code],
            cwd=root,
            capture_output=True,
            text=True,
            check=True,
        )

        assert result.stdout.strip().endswith('False')
