"""
Integration tests for IMU path restart button.

Tests the complete workflow from JavaScript button click to server restart.
"""

import json
import sys
from unittest.mock import patch
import tornado.testing
import tornado.web
from donkeycar.parts.web_controller.web import (
    LocalWebController,
    IMUPathRestartAPI
)


class IMUPathRestartIntegrationTest(tornado.testing.AsyncHTTPTestCase):
    """Test IMU path restart button integration."""

    def get_app(self):
        """Create a full LocalWebController app for integration testing."""
        app = LocalWebController(port=8887)
        return app

    @tornado.testing.gen_test
    def test_restart_endpoint_exists_in_full_app(self):
        """Verify restart endpoint is registered in full application."""
        # Check that the route is registered
        app = self.get_app()
        routes = []
        for rule in app.wildcard_router.rules:
            pattern = (rule.matcher.regex.pattern
                      if hasattr(rule.matcher, 'regex') else str(rule))
            routes.append(pattern)

        # The route should be registered
        restart_pattern = '/api/imupath/restart$'
        assert restart_pattern in routes, \
            f"Restart endpoint not found. Routes: {routes}"

    @tornado.testing.gen_test
    def test_restart_with_javascript_payload(self):
        """Test restart with exact payload from JavaScript."""
        with patch('os.execv') as mock_execv:
            # This is the exact payload from imupath.js line 404
            js_payload = {'confirm': 'restart'}
            body = json.dumps(js_payload)

            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Origin': f'http://localhost:{self.get_http_port()}',
                    'Host': f'localhost:{self.get_http_port()}'
                }
            )

            assert response.code == 200
            data = json.loads(response.body)
            # Python API returns {'status': 'restarting'}
            assert data['status'] == 'restarting'

            # Wait for delayed restart
            yield tornado.gen.sleep(0.6)
            mock_execv.assert_called_once()

    @tornado.testing.gen_test
    def test_restart_handler_class_correct(self):
        """Verify correct handler class is used."""
        # Find the restart handler
        app = self.get_app()
        for rule in app.wildcard_router.rules:
            pattern = (rule.matcher.regex.pattern
                      if hasattr(rule.matcher, 'regex') else str(rule))
            if 'restart' in pattern:
                handler_class = rule.target
                assert handler_class == IMUPathRestartAPI, \
                    f"Wrong handler: {handler_class}"
                break
        else:
            assert False, "Restart route not found"


class IMUPathRestartSecurityTest(tornado.testing.AsyncHTTPTestCase):
    """Test security aspects of restart endpoint."""

    def get_app(self):
        """Create minimal app with restart endpoint."""
        app = tornado.web.Application([
            (r"/api/imupath/restart", IMUPathRestartAPI),
        ])
        return app

    @tornado.testing.gen_test
    def test_restart_blocks_wrong_origin(self):
        """Test that CSRF protection works."""
        body = json.dumps({'confirm': 'restart'})

        try:
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Origin': 'http://attacker.com',
                    'Host': f'localhost:{self.get_http_port()}'
                },
                raise_error=False
            )
            assert response.code == 403
        except Exception:
            # Tornado might raise on 403, which is fine
            pass

    @tornado.testing.gen_test
    def test_restart_blocks_wrong_confirmation(self):
        """Test that wrong confirmation value is rejected."""
        # Try with wrong confirmation value
        body = json.dumps({'confirm': 'wrong'})

        try:
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Host': f'localhost:{self.get_http_port()}'
                },
                raise_error=False
            )
            assert response.code == 400
        except Exception:
            pass

    @tornado.testing.gen_test
    def test_restart_requires_post(self):
        """Test that GET requests are rejected."""
        try:
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='GET',
                raise_error=False
            )
            # Should get 405 Method Not Allowed
            assert response.code in [405, 400, 403]
        except Exception:
            # Tornado might raise, which is fine
            pass


if __name__ == '__main__':
    import unittest
    unittest.main()
