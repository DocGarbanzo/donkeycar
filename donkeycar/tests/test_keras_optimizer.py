from types import SimpleNamespace

import donkeycar.parts.keras as dk_keras


class FakeInterpreter:
    def set_model(self, model):
        self.model = model

    def set_optimizer(self, optimizer):
        self.optimizer = optimizer


class DummyPilot(dk_keras.KerasPilot):
    def create_model(self):
        return None

    def interpreter_to_output(self, interpreter_out):
        return interpreter_out


def _make_optimizer_class():
    class FakeOptimizer:
        called = False

        def __init__(self, lr, decay):
            # Keep legacy-style args to mirror donkeycar optimizer calls.
            self.lr = lr
            self.decay = decay
            FakeOptimizer.called = True

    return FakeOptimizer


def test_set_optimizer_uses_legacy_adam_when_metal_installed(monkeypatch):
    FakeAdam = _make_optimizer_class()
    FakeLegacyAdam = _make_optimizer_class()

    fake_keras = SimpleNamespace(
        optimizers=SimpleNamespace(
            Adam=FakeAdam,
            legacy=SimpleNamespace(Adam=FakeLegacyAdam),
        )
    )

    monkeypatch.setattr(dk_keras, "keras", fake_keras)
    monkeypatch.setattr(dk_keras, "_is_metal_installed", lambda: True)

    pilot = DummyPilot(interpreter=FakeInterpreter())
    pilot.set_optimizer("adam", rate=0.01, decay=0.001)

    assert isinstance(pilot.interpreter.optimizer, FakeLegacyAdam)
    assert FakeLegacyAdam.called
    assert not FakeAdam.called


def test_set_optimizer_falls_back_when_legacy_adam_unavailable(
        monkeypatch, caplog):
    FakeAdam = _make_optimizer_class()

    fake_keras = SimpleNamespace(
        optimizers=SimpleNamespace(
            Adam=FakeAdam,
        )
    )

    monkeypatch.setattr(dk_keras, "keras", fake_keras)
    monkeypatch.setattr(dk_keras, "_is_metal_installed", lambda: True)

    with caplog.at_level("WARNING"):
        pilot = DummyPilot(interpreter=FakeInterpreter())
        pilot.set_optimizer("adam", rate=0.01, decay=0.001)

    assert isinstance(pilot.interpreter.optimizer, FakeAdam)
    assert FakeAdam.called
    assert "legacy Adam is unavailable" in caplog.text


def test_set_optimizer_uses_standard_adam_without_metal(monkeypatch):
    FakeAdam = _make_optimizer_class()
    FakeLegacyAdam = _make_optimizer_class()

    fake_keras = SimpleNamespace(
        optimizers=SimpleNamespace(
            Adam=FakeAdam,
            legacy=SimpleNamespace(Adam=FakeLegacyAdam),
        )
    )

    monkeypatch.setattr(dk_keras, "keras", fake_keras)
    monkeypatch.setattr(dk_keras, "_is_metal_installed", lambda: False)

    pilot = DummyPilot(interpreter=FakeInterpreter())
    pilot.set_optimizer("adam", rate=0.01, decay=0.001)

    assert isinstance(pilot.interpreter.optimizer, FakeAdam)
    assert FakeAdam.called
    assert not FakeLegacyAdam.called
