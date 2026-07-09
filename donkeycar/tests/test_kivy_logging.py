import os
import logging

from kivy.logger import ConsoleHandler, FileHandler

from donkeycar.management.ui.kivy_logging import configure_kivy_logging


class TestKivyLogging:
    def test_defaults_to_kivy_mode(self, monkeypatch):
        monkeypatch.delenv('KIVY_LOG_MODE', raising=False)

        configure_kivy_logging()

        assert 'KIVY' == os.environ['KIVY_LOG_MODE']

    def test_keeps_explicit_mode(self, monkeypatch):
        monkeypatch.setenv('KIVY_LOG_MODE', 'MIXED')

        configure_kivy_logging()

        assert 'MIXED' == os.environ['KIVY_LOG_MODE']

    def test_removes_plain_root_stream_handler(self, monkeypatch):
        monkeypatch.delenv('KIVY_LOG_MODE', raising=False)
        root = logging.getLogger()
        old_handlers = list(root.handlers)
        handler = logging.StreamHandler()
        root.handlers = [handler]

        try:
            configure_kivy_logging()
            assert handler not in root.handlers
        finally:
            root.handlers = old_handlers

    def test_keeps_root_stream_handler_in_python_mode(self, monkeypatch):
        monkeypatch.setenv('KIVY_LOG_MODE', 'PYTHON')
        root = logging.getLogger()
        old_handlers = list(root.handlers)
        handler = logging.StreamHandler()
        root.handlers = [handler]

        try:
            configure_kivy_logging()
            assert handler in root.handlers
        finally:
            root.handlers = old_handlers

    def test_removes_kivy_console_handler_subclass(self, monkeypatch):
        monkeypatch.delenv('KIVY_LOG_MODE', raising=False)
        root = logging.getLogger()
        old_handlers = list(root.handlers)
        handler = ConsoleHandler()
        root.handlers = [handler]

        try:
            configure_kivy_logging()
            assert handler not in root.handlers
        finally:
            root.handlers = old_handlers

    def test_keeps_non_stream_handler(self, monkeypatch):
        monkeypatch.delenv('KIVY_LOG_MODE', raising=False)
        root = logging.getLogger()
        old_handlers = list(root.handlers)
        handler = FileHandler()
        root.handlers = [handler]

        try:
            configure_kivy_logging()
            assert handler in root.handlers
        finally:
            root.handlers = old_handlers

    def test_disables_kivy_consolelog_before_kivy_import(self, monkeypatch):
        monkeypatch.delenv('KIVY_LOG_MODE', raising=False)
        monkeypatch.delenv('KIVY_NO_CONSOLELOG', raising=False)

        configure_kivy_logging()

        assert '1' == os.environ['KIVY_NO_CONSOLELOG']

    def test_keeps_explicit_consolelog_setting(self, monkeypatch):
        monkeypatch.delenv('KIVY_LOG_MODE', raising=False)
        monkeypatch.setenv('KIVY_NO_CONSOLELOG', '0')

        configure_kivy_logging()

        assert '0' == os.environ['KIVY_NO_CONSOLELOG']
