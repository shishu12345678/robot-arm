import math
import json
import tempfile
from pathlib import Path
import unittest

from mdh_validate import fk_mdh, identity, rotation_error_deg
from extract_mdh_config import extract


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

    def test_extractor_drops_controller_identifiers_and_joint_telemetry(self):
        snapshot = {
            "target": "right_main_arm",
            "note": "read_only sample",
            "controller_ip": "192.0.2.1",
            "get_robot_type": ["private-serial"],
            "get_actual_joints_position": [9.0] * 6,
            "get_calib_DH": [[0.0, 0.0, 0.1, 0.0, 0.0] for _ in range(6)],
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output = root / "snapshot.json", root / "model.json"
            source.write_text(json.dumps(snapshot), encoding="utf-8")
            extract(source, "right_main_arm", output)
            result = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(result["arm"], "right_main_arm")
        self.assertEqual(len(result["get_calib_DH"]), 6)
        self.assertNotIn("controller_ip", result)
        self.assertNotIn("get_robot_type", result)
        self.assertNotIn("get_actual_joints_position", result)

if __name__ == "__main__":
    unittest.main()
