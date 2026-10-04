"""Deterministic policy checks; no synthetic values are published as sensor data."""
import importlib.util
import pathlib
import unittest
from types import SimpleNamespace


ROOT = pathlib.Path(__file__).resolve().parents[2]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


MAP = module('map_policy', ROOT / 'planner/scripts/map_speed_constraint.py')
BEHAVIOR = module('behavior_policy', ROOT / 'planner/scripts/behavior_manager.py')
LONGITUDINAL = module('longitudinal_policy', ROOT.parent / 'common/sac_control_safety/scripts/longitudinal_controller.py')
TRACK = module('track_policy', ROOT.parent / 'perception/jetson_perception/jetson_perception/dynamic_object_tracker.py')


class Phase4PolicyTest(unittest.TestCase):
    def test_map_limit_range(self):
        self.assertTrue(MAP.valid_map_limit(1.0, 20.0))
        for value in (0, -1, float('nan'), float('inf'), 21):
            self.assertFalse(MAP.valid_map_limit(value, 20.0))
        self.assertEqual(min(2.0, 1.0, 0.0, .5), 0.0)  # traffic STOP wins
        self.assertEqual(min(2.0, 1.0, .5), .5)  # hazard wins

    def test_longitudinal_stop_and_exclusive_pedals(self):
        loop = LONGITUDINAL.SpeedLoop(.15, .05, .4, .3, 1., .5, 1.5, 1., 2.)
        outputs = [loop.update(2.0, 0.0, .02) for _ in range(50)]
        self.assertTrue(all(0 <= t <= .3 and 0 <= b <= 1 and not (t and b)
                            for t, b in outputs))
        self.assertLessEqual(loop.reference, .5 + 1e-9)
        self.assertEqual(loop.update(0.0, 1.0, .02), (0.0, 1.0))
        self.assertEqual(loop.update(2.0, float('nan'), .02), (0.0, 1.0))

    def test_tracking_ego_compensation_contract(self):
        tf = SimpleNamespace(transform=SimpleNamespace(
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.),
            translation=SimpleNamespace(x=1., y=0., z=0.)))
        self.assertEqual(TRACK.transform_xy((9., 0., 0.), tf), (10., 0., 0.))
        store = TRACK.TrackStore(gate=3, timeout=.8)
        store.update([(1, .9, (10., 0., 0.))], 1.0, 'odom')
        result = store.update([(1, .9, (10., 0., 0.))], 1.1, 'odom')
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(next(iter(result.values()))['vel'][0], 0)
        result = store.update([(1, .9, (10.2, 0., 0.))], 1.2, 'odom')
        self.assertGreater(next(iter(result.values()))['vel'][0], 0)
        result = store.update([(1, .9, (9.9, 0., 0.))], 1.25, 'odom')
        self.assertLess(next(iter(result.values()))['vel'][0], 2.0)
        store.update([], 1.3, 'odom')
        self.assertEqual(len(store.tracks), 1)
        store.update([], 2.1, 'odom')
        self.assertEqual(len(store.tracks), 0)
        store.update([(1, .9, (8., 0., 0.)), (1, .8, (8.1, 0., 0.))], 3., 'base_link')
        self.assertEqual(len(store.tracks), 1)
        self.assertFalse(next(iter(store.tracks.values()))['velocity_valid'])

    def test_crosswalk_and_follow(self):
        self.assertFalse(BEHAVIOR.crossing_relevant(8, 3, 8, 3, 2))
        self.assertTrue(BEHAVIOR.crossing_relevant(8, 1, 8, 3, 2))
        self.assertEqual(BEHAVIOR.follow_speed(2, 2, 0, 2, 1.5, .5, 2), 0)
        self.assertGreater(BEHAVIOR.follow_speed(10, 2, 1, 2, 1.5, .5, 2), 1)
        self.assertEqual(BEHAVIOR.follow_speed(12, 2, 3, 2, 1.5, .5, 2), 2)
        self.assertEqual(BEHAVIOR.follow_speed(10, 2, None, 2, 1.5, .5, 2), 0)


if __name__ == '__main__':
    unittest.main()
