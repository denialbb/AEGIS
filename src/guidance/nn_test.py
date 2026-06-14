import numpy as np
import pytest
from src.guidance.nn import NNFeedforward, generate_training_data


def test_nn_feedforward_initialization():
    """Test NNFeedforward initialization with default weights."""
    nn = NNFeedforward()
    assert nn.W1.shape == (9, 20)
    assert nn.b1.shape == (20,)
    assert nn.W2.shape == (20, 20)
    assert nn.b2.shape == (20,)
    assert nn.W3.shape == (20, 3)
    assert nn.b3.shape == (3,)
    assert nn.clamp == 5.0
    assert not nn.is_trained


def test_nn_feedforward_predict_zero_input():
    """Test predict returns zero for zero input."""
    nn = NNFeedforward()
    state = np.zeros(9)
    output = nn.predict(state)
    assert output.shape == (3,)
    assert np.allclose(output, np.zeros(3), atol=1e-6)


def test_nn_feedforward_predict_clamping():
    """Test output is clamped to [-clamp, +clamp]."""
    nn = NNFeedforward(clamp=2.0)
    # Make sure weights produce large output
    nn.W1 = np.ones((9, 20)) * 1000
    nn.b1 = np.ones(20) * 1000
    nn.W2 = np.ones((20, 20)) * 1000
    nn.b2 = np.ones(20) * 1000
    nn.W3 = np.ones((20, 3)) * 1000
    nn.b3 = np.ones(3) * 1000
    state = np.ones(9)
    output = nn.predict(state)
    assert np.all(output <= 2.0)
    assert np.all(output >= -2.0)


def test_nn_feedforward_predict_invalid_shape():
    """Test predict raises ValueError for invalid state shape."""
    nn = NNFeedforward()
    with pytest.raises(ValueError):
        nn.predict(np.zeros(8))
    with pytest.raises(ValueError):
        nn.predict(np.zeros(10))


def test_nn_feedforward_train_invalid_shapes():
    """Test train raises ValueError for invalid X/y shapes."""
    nn = NNFeedforward()
    X = np.random.random((10, 8))
    y = np.random.random((10, 3))
    with pytest.raises(ValueError):
        nn.train(X, y)
    X = np.random.random((10, 9))
    y = np.random.random((10, 2))
    with pytest.raises(ValueError):
        nn.train(X, y)


def test_nn_feedforward_train_simple():
    """Test train can converge on a simple linear problem."""
    # Create a simple problem: output = input[0] + input[1] + input[2]
    # So we want W3[0, :] = [1, 1, 1, 0, 0, 0, 0, 0, 0] and rest 0
    nn = NNFeedforward()
    X = np.random.random((100, 9))
    y = X[:, 0] + X[:, 1] + X[:, 2]  # shape (100,)
    y = np.column_stack([y, np.zeros(100), np.zeros(100)])  # shape (100, 3)

    result = nn.train(X, y, max_nfev=100, verbose=0)
    assert result['success']
    assert result['cost'] < 1e-3
    assert nn.is_trained


def test_generate_training_data():
    """Test generate_training_data returns valid X, y."""
    inertia = np.diag([500.0, 800.0, 600.0])
    X, y, info = generate_training_data(inertia, n_trajectories=1, n_steps=10)
    assert X.shape == (10, 9)
    assert y.shape == (10, 3)
    assert info['n_trajectories'] == 1
    assert info['n_steps'] == 10
    assert info['dt'] == 0.02
    assert info['inertia_tensor'].shape == (3, 3)
    assert len(info['profiles']) == 1


def test_generate_training_data_stab_gains():
    """Test generate_training_data uses stable gains when requested."""
    inertia = np.diag([500.0, 800.0, 600.0])
    X, y, info = generate_training_data(inertia, n_trajectories=1, n_steps=10, stab_gains=True)
    assert info['stab_gains'] is True


def test_generate_training_data_no_adrc():
    """Test generate_training_data works without ADRC (no side effect)."""
    # This test ensures the function doesn't require ADRC to be active
    # and that it uses its internal ADRCController correctly.
    inertia = np.diag([500.0, 800.0, 600.0])
    X, y, info = generate_training_data(inertia, n_trajectories=1, n_steps=10)
    assert X.shape == (10, 9)
    assert y.shape == (10, 3)


def test_nn_feedforward_predict_after_train():
    """Test predict works after training."""
    inertia = np.diag([500.0, 800.0, 600.0])
    X, y, info = generate_training_data(inertia, n_trajectories=1, n_steps=10)
    nn = NNFeedforward()
    nn.train(X, y, max_nfev=50, verbose=0)
    assert nn.is_trained
    pred = nn.predict(X[0])
    assert pred.shape == (3,)


def test_nn_feedforward_predict_untrained():
    """Test predict returns zero for untrained network."""
    nn = NNFeedforward()
    state = np.zeros(9)
    output = nn.predict(state)
    assert np.allclose(output, np.zeros(3), atol=1e-6)
    nn.is_trained = False
    output = nn.predict(state)
    assert np.allclose(output, np.zeros(3), atol=1e-6)