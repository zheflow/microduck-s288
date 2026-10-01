"""Geometry-kernel regression; tiny solids only, no full CAD rebuild."""
import unittest
import numpy as np
import trimesh
import manifold3d as M
from assembly_audit import solid, intersection


class KernelTests(unittest.TestCase):
    def test_native_readonly_mesh_buffers_are_accepted(self):
        raw = M.Manifold.cube((2, 3, 4)).to_mesh64()
        mesh = trimesh.Trimesh(vertices=np.asarray(raw.vert_properties)[:, :3],
                               faces=np.asarray(raw.tri_verts), process=False)
        self.assertFalse(mesh.vertices.flags.writeable)
        self.assertAlmostEqual(solid(mesh).volume(), 24.)

    def test_measured_intersection_and_empty_case(self):
        a = M.Manifold.cube((1, 1, 1))
        self.assertAlmostEqual(intersection(a, a.translate((.5, .5, .5))), .125)
        self.assertEqual(intersection(a, a.translate((2, 0, 0))), 0.)

    def test_open_mesh_does_not_pass_as_no_collision(self):
        mesh = trimesh.Trimesh(vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                               faces=[[0, 1, 2]], process=False)
        with self.assertRaises(ValueError):
            solid(mesh)


if __name__ == '__main__':
    unittest.main()
