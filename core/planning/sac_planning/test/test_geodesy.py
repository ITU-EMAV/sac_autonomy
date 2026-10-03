import math

import numpy as np
from sac_planning.geodesy import MapFrame
from sac_planning.route import headings, resample, smooth


def test_round_trip():
    frame = MapFrame(38.1628083, -122.4579944, 0.0, 0.83)
    for x, y in [(0, 0), (277.88, -135.2), (-500, 400), (1000, 1000)]:
        lat, lon, alt = frame.to_geodetic(x, y)
        assert np.allclose(frame.to_map(lat, lon, alt), [x, y, 0.0], atol=1e-3)


def test_heading_turns_the_map_counter_clockwise():
    # 100 m east of the origin is at -heading in the map frame
    frame = MapFrame(38.0, -122.0, 0.0, 30.0)
    lat, lon, _ = MapFrame(38.0, -122.0).to_geodetic(100.0, 0.0)
    x, y, _ = frame.to_map(lat, lon)
    assert math.isclose(math.degrees(math.atan2(y, x)), -30.0, abs_tol=0.01)


def test_loop_resample_and_headings():
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    p = resample(smooth(square, True, 2), True, 0.5)
    assert np.all(np.hypot(*np.diff(p, axis=0).T) < 0.6)
    assert np.hypot(*(p[-1] - p[0])) < 0.6  # the loop closes without repeating a point
    h = headings(p, True)
    assert abs(h[0]) < 0.3  # counter-clockwise square starts eastwards
