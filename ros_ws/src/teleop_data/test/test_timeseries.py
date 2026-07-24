import numpy as np

from teleop_data.timeseries import TimeSeries, fixed_rate_times


def test_linear_and_hold_sampling():
    series = TimeSeries([0.0, 1.0], [[0.0], [2.0]])
    np.testing.assert_allclose(series.linear([0.5]), [[1.0]])
    np.testing.assert_allclose(series.hold([0.5]), [[0.0]])


def test_fixed_rate_timeline():
    np.testing.assert_allclose(fixed_rate_times(1.0, 1.1, 20), [1.0, 1.05, 1.1])
