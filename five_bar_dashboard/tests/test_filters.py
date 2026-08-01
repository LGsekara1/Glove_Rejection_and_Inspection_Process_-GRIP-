from five_bar_dashboard.filters import MovingAverageFilter, MedianVelocityFilter, SinglePoleLowPassFilter


def test_moving_average():
    filt = MovingAverageFilter(3)
    assert filt.update(1.0, 0.1) == 1.0
    assert filt.update(2.0, 0.1) == 1.5
    assert filt.update(3.0, 0.1) == 2.0
    assert filt.update(4.0, 0.1) == 3.0


def test_median_rejects_spike():
    filt = MedianVelocityFilter(5)
    values = [1.0, 1.0, 100.0, 1.0, 1.0]
    result = 0.0
    for value in values:
        result = filt.update(value, 0.1)
    assert result == 1.0


def test_low_pass_moves_toward_input():
    filt = SinglePoleLowPassFilter(2.0)
    assert filt.update(0.0, 0.01) == 0.0
    output = filt.update(10.0, 0.01)
    assert 0.0 < output < 10.0
