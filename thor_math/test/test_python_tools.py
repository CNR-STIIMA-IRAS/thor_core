import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from tools.add_intermediate_joints import divide_xyz, transform_urdf


class AddIntermediateJointsTest(unittest.TestCase):
    def test_divide_xyz_validates_and_divides_three_components(self):
        self.assertEqual(divide_xyz("3 6 9", 3), (1.0, 2.0, 3.0))
        with self.assertRaises(ValueError):
            divide_xyz("1 2", 3)

    def test_transform_inserts_frames_and_preserves_total_translation(self):
        source = """\
<robot name="fixture">
  <link name="base"/>
  <link name="tip"/>
  <joint name="axis" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 0 0.9" rpy="0.1 0.2 0.3"/>
  </joint>
</robot>
"""
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "input.urdf"
            output_path = Path(directory) / "output.urdf"
            input_path.write_text(source, encoding="utf-8")

            transform_urdf(input_path, output_path, frames=2)
            root = ET.parse(output_path).getroot()

        self.assertIsNotNone(root.find("link[@name='axis_intermediate_1']"))
        self.assertIsNotNone(root.find("link[@name='axis_intermediate_2']"))
        self.assertEqual(len(root.findall("joint[@type='fixed']")), 2)

        movable = root.find("joint[@name='axis']")
        self.assertIsNotNone(movable)
        self.assertEqual(
            movable.find("parent").get("link"), "axis_intermediate_2"
        )
        self.assertEqual(movable.find("origin").get("xyz"), "0 0 0.3")
        self.assertEqual(movable.find("origin").get("rpy"), "0.1 0.2 0.3")

    def test_transform_rejects_zero_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "input.urdf"
            input_path.write_text("<robot name='fixture'/>", encoding="utf-8")
            with self.assertRaises(ValueError):
                transform_urdf(input_path, Path(directory) / "out.urdf", 0)


if __name__ == "__main__":
    unittest.main()
