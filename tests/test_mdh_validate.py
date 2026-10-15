import math
import json
from pathlib import Path
import unittest

from mdh_validate import fk_mdh, identity, load_config, rotation_error_deg, validate_sample

ROOT = Path(__file__).resolve().parents[1]


class MdhForwardKinematicsTests(unittest.TestCase):
    def test_zero_angle_single_translation_chain(self):
        rows = [[0.0, 0.0, 0.1, 0.0] for _ in range(6)]
        t = fk_mdh(rows, [0.0] * 6)
        self.assertAlmostEqual(t[2][3], 0.6)
        self.assertAlmostEqual(t[0][3], 0.0)
        self.assertAlmostEqual(t[1][3], 0.0)

    def test_joint_rotation_moves_following_offset(self):
        rows = [[0.0, 0.0, 0.0, 0.0] for _ in range(6)]
        rows[0][2] = 0.2
        rows[1] = [0.3, 0.0, 0.0, 0.0]
        t = fk_mdh(rows, [math.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(t[0][3], 0.0, places=12)
        self.assertAlmostEqual(t[1][3], 0.3, places=12)
        self.assertAlmostEqual(t[2][3], 0.2, places=12)

    def test_identical_rotations_have_zero_error(self):
        r = [row[:3] for row in identity()[:3]]
        self.assertAlmostEqual(rotation_error_deg(r, r), 0.0)

    def test_rejects_wrong_dof_count(self):
        with self.assertRaises(ValueError):
            fk_mdh([[0, 0, 0, 0]] * 6, [0.0] * 5)

    def test_right_snapshot_regression(self):
        config = load_config(ROOT / "config/right_main_arm.json")
        result = validate_sample(ROOT / "tests/fixtures/right_snapshot.json", config, "right_main_arm")
        self.assertLess(result["position_error_mm"], 0.001)
        self.assertLess(result["orientation_error_deg"], 0.002)

    def test_left_snapshot_regression(self):
        config = load_config(ROOT / "config/left_slave_arm.json")
        result = validate_sample(ROOT / "tests/fixtures/left_snapshot.json", config, "left_slave_arm")
        self.assertLess(result["position_error_mm"], 0.001)
        self.assertLess(result["orientation_error_deg"], 0.003)


if __name__ == "__main__":
    unittest.main()
