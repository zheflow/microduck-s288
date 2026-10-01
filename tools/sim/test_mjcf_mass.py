"""Mass accounting regressions; no CAD build, output regeneration or hardware."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import trimesh
import yaml

import make_mjcf as model


class MassAccountingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "components.yaml"
        self.data = {"components": [
            {"id": "sbc_radxa_zero3w", "mass_g": {"v": 14.93}},
            {"id": "camera_csi", "mass_g": {"v": 7.32}},
            {"id": "imu_icm42688", "mass_g": {"v": 1.7}},
            {"id": "connector_xt30", "mass_g": {"v": 1.7}},
            {"id": "bearing_6700zz", "mass_g": {"v": 1.9}},
            {"id": "mic_inmp441", "mass_g": {"v": None}},
        ]}
        self.path.write_text(yaml.safe_dump(self.data))
        replacement = patch.object(model, "COMPONENTS_YAML", self.path)
        replacement.start()
        self.addCleanup(replacement.stop)
        self.envelope = trimesh.creation.box(extents=[20, 30, 40])

    def test_purchased_mass_does_not_depend_on_envelope_volume(self):
        for stem, expected in (("zz_sbc", 14.93), ("zz_camera", 7.32),
                               ("zz_imu", 1.7), ("zz_xt30", 1.7), ("bearing_jaw", 1.9)):
            with self.subTest(stem=stem):
                mass, src = model.part_mass_g(stem, self.envelope)
                self.assertEqual(mass, expected)
                self.assertTrue(src.startswith("components.yaml:"))

    def test_mesh_and_point_mass_are_mutually_exclusive(self):
        points = model.point_masses_for_parts(["zz_sbc", "zz_xt30"])
        self.assertEqual(points, [])
        points = model.point_masses_for_parts(["zz_sbc"])
        self.assertEqual([p[0] for p in points], ["connector_xt30"])

    def test_unweighed_envelope_is_explicitly_a_proxy(self):
        mass, src = model.part_mass_g("zz_mic", self.envelope)
        self.assertGreater(mass, 0)
        self.assertTrue(src.startswith("unmeasured_proxy:mic_inmp441;"))

    def test_invalid_measured_mass_is_not_silently_replaced_by_a_proxy(self):
        self.data["components"][0]["mass_g"]["v"] = -1
        self.path.write_text(yaml.safe_dump(self.data))
        with self.assertRaises(ValueError):
            model.part_mass_g("zz_sbc", self.envelope)

    def test_unmapped_or_retired_electronics_cannot_be_printed_plastic(self):
        with self.assertRaises(ValueError):
            model.part_mass_g("zz_switch", self.envelope)


if __name__ == "__main__":
    unittest.main()
