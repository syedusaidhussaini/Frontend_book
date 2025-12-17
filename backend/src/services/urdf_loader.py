"""
URDF Loader Service for Digital Twin Module

Handles loading and parsing URDF files for robot models in Gazebo simulation.
"""

import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple
import logging
from pathlib import Path

from src.models.robot_model import JointInfo, LinkInfo, JointType, JointLimits, JointDynamics, Vector3, Quaternion

logger = logging.getLogger(__name__)


class URDFLoader:
    """
    Service for loading and parsing URDF files into robot model representations
    """

    @staticmethod
    def load_urdf_file(urdf_path: str) -> Optional[Tuple[List[LinkInfo], List[JointInfo]]]:
        """
        Load and parse a URDF file, returning links and joints

        Args:
            urdf_path: Path to the URDF file

        Returns:
            Tuple of (links, joints) or None if failed
        """
        try:
            # Verify file exists
            path = Path(urdf_path)
            if not path.exists():
                logger.error(f"URDF file does not exist: {urdf_path}")
                return None

            # Parse XML
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            if root.tag != 'robot':
                logger.error(f"Invalid URDF file: root element is not 'robot', got '{root.tag}'")
                return None

            robot_name = root.get('name', 'unknown_robot')
            logger.info(f"Loading URDF for robot: {robot_name}")

            links = URDFLoader._parse_links(root)
            joints = URDFLoader._parse_joints(root)

            logger.info(f"Successfully parsed URDF: {len(links)} links, {len(joints)} joints")
            return links, joints

        except ET.ParseError as e:
            logger.error(f"Failed to parse URDF XML: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error loading URDF: {e}")
            return None

    @staticmethod
    def _parse_links(root) -> List[LinkInfo]:
        """Parse link elements from URDF"""
        links = []

        for link_elem in root.findall('link'):
            link_name = link_elem.get('name')
            if not link_name:
                logger.warning("Link without name found, skipping")
                continue

            # Extract mass if available
            mass = 1.0  # default
            mass_elem = link_elem.find('inertial/mass')
            if mass_elem is not None and 'value' in mass_elem.attrib:
                try:
                    mass = float(mass_elem.get('value'))
                except ValueError:
                    logger.warning(f"Invalid mass value for link {link_name}, using default")

            # Extract visual and collision mesh info
            visual_mesh = None
            collision_mesh = None

            visual_elem = link_elem.find('visual/geometry/mesh')
            if visual_elem is not None and 'filename' in visual_elem.attrib:
                visual_mesh = visual_elem.get('filename')

            collision_elem = link_elem.find('collision/geometry/mesh')
            if collision_elem is not None and 'filename' in collision_elem.attrib:
                collision_mesh = collision_elem.get('filename')

            link_info = LinkInfo(
                name=link_name,
                mass=mass,
                position=Vector3(),  # Default position
                orientation=Quaternion(),  # Default orientation
                visual_mesh=visual_mesh,
                collision_mesh=collision_mesh
            )
            links.append(link_info)

        return links

    @staticmethod
    def _parse_joints(root) -> List[JointInfo]:
        """Parse joint elements from URDF"""
        joints = []

        for joint_elem in root.findall('joint'):
            joint_name = joint_elem.get('name')
            joint_type_str = joint_elem.get('type')

            if not joint_name or not joint_type_str:
                logger.warning(f"Joint missing name or type, skipping: {joint_elem.attrib}")
                continue

            # Validate joint type
            try:
                joint_type = JointType(joint_type_str)
            except ValueError:
                logger.warning(f"Unknown joint type '{joint_type_str}', using revolute")
                joint_type = JointType.REVOLUTE

            # Extract parent and child links
            parent_elem = joint_elem.find('parent')
            child_elem = joint_elem.find('child')

            if parent_elem is None or child_elem is None:
                logger.warning(f"Joint {joint_name} missing parent or child, skipping")
                continue

            parent_link = parent_elem.get('link')
            child_link = child_elem.get('link')

            if not parent_link or not child_link:
                logger.warning(f"Joint {joint_name} has invalid parent/child links, skipping")
                continue

            # Extract axis (default to Z-axis if not specified)
            axis = Vector3(x=0.0, y=0.0, z=1.0)
            axis_elem = joint_elem.find('axis')
            if axis_elem is not None and 'xyz' in axis_elem.attrib:
                try:
                    xyz_str = axis_elem.get('xyz')
                    xyz_vals = [float(x) for x in xyz_str.split()]
                    if len(xyz_vals) == 3:
                        axis = Vector3(x=xyz_vals[0], y=xyz_vals[1], z=xyz_vals[2])
                except (ValueError, AttributeError):
                    logger.warning(f"Invalid axis for joint {joint_name}, using default Z-axis")

            # Extract limits
            limits = JointLimits()
            limit_elem = joint_elem.find('limit')
            if limit_elem is not None:
                if 'lower' in limit_elem.attrib:
                    try:
                        limits.position_min = float(limit_elem.get('lower'))
                    except ValueError:
                        pass
                if 'upper' in limit_elem.attrib:
                    try:
                        limits.position_max = float(limit_elem.get('upper'))
                    except ValueError:
                        pass
                if 'velocity' in limit_elem.attrib:
                    try:
                        limits.velocity_max = float(limit_elem.get('velocity'))
                    except ValueError:
                        pass
                if 'effort' in limit_elem.attrib:
                    try:
                        limits.effort_max = float(limit_elem.get('effort'))
                    except ValueError:
                        pass

            # Extract dynamics
            dynamics = JointDynamics()
            dynamics_elem = joint_elem.find('dynamics')
            if dynamics_elem is not None:
                if 'damping' in dynamics_elem.attrib:
                    try:
                        dynamics.damping = float(dynamics_elem.get('damping'))
                    except ValueError:
                        pass
                if 'friction' in dynamics_elem.attrib:
                    try:
                        dynamics.friction = float(dynamics_elem.get('friction'))
                    except ValueError:
                        pass

            joint_info = JointInfo(
                name=joint_name,
                joint_type=joint_type,
                parent_link=parent_link,
                child_link=child_link,
                axis=axis,
                limits=limits,
                dynamics=dynamics
            )
            joints.append(joint_info)

        return joints

    @staticmethod
    def create_sample_humanoid_urdf(output_path: str):
        """
        Create a sample humanoid URDF file for testing purposes
        """
        sample_urdf_content = '''<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Left leg -->
  <link name="left_leg">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Right leg -->
  <link name="right_leg">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Left hip joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0.1 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <!-- Right hip joint -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="-0.1 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>
</robot>'''

        with open(output_path, 'w') as f:
            f.write(sample_urdf_content)

        logger.info(f"Sample humanoid URDF created at: {output_path}")


# Example usage and testing
if __name__ == "__main__":
    # Create a sample URDF for testing
    URDFLoader.create_sample_humanoid_urdf("sample_humanoid.urdf")

    # Load and parse it
    result = URDFLoader.load_urdf_file("sample_humanoid.urdf")
    if result:
        links, joints = result
        print(f"Loaded {len(links)} links and {len(joints)} joints")
        for link in links:
            print(f"  Link: {link.name}, mass: {link.mass}")
        for joint in joints:
            print(f"  Joint: {joint.name}, type: {joint.joint_type}, parent: {joint.parent_link}, child: {joint.child_link}")