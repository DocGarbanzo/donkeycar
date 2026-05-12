import numpy as np
import pytest

tf = pytest.importorskip(
    "tensorflow",
    reason="TensorFlow not installed, skipping keras optimizer tests",
)

import donkeycar.parts.keras as dk_keras

CONVERGENCE_THRESHOLD = 0.2


def _linear_data():
    x = np.linspace(-1.0, 1.0, 128, dtype=np.float32).reshape(-1, 1)
    y = (2.5 * x - 0.3).astype(np.float32)
    return x, y


def _train_loss_history(optimizer):
    tf.keras.backend.clear_session()
    tf.keras.utils.set_random_seed(123)
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=(1,)),
        tf.keras.layers.Dense(8, activation="tanh"),
        tf.keras.layers.Dense(1),
    ])
    model.compile(optimizer=optimizer, loss="mse")
    x, y = _linear_data()
    history = model.fit(x, y, epochs=25, batch_size=16, verbose=0)
    return history.history["loss"]


def test_adam_optimizer_converges_without_metal(monkeypatch):
    monkeypatch.setattr(dk_keras, "_is_metal_installed", lambda: False)
    optimizer = dk_keras._adam_optimizer(rate=0.01, decay=0.0)
    legacy = getattr(dk_keras.keras.optimizers, "legacy", None)
    assert optimizer.__class__.__name__ == "Adam"
    if legacy is not None and hasattr(legacy, "Adam"):
        assert not isinstance(optimizer, legacy.Adam)
    loss = _train_loss_history(optimizer)
    assert loss[-1] < loss[0] * CONVERGENCE_THRESHOLD


def test_legacy_adam_converges_with_metal(monkeypatch):
    legacy = getattr(dk_keras.keras.optimizers, "legacy", None)
    if legacy is None or not hasattr(legacy, "Adam"):
        pytest.skip("legacy Adam is unavailable in this TensorFlow build")

    monkeypatch.setattr(dk_keras, "_is_metal_installed", lambda: True)
    optimizer = dk_keras._adam_optimizer(rate=0.01, decay=0.0)

    assert isinstance(optimizer, legacy.Adam)
    loss = _train_loss_history(optimizer)
    assert loss[-1] < loss[0] * CONVERGENCE_THRESHOLD
