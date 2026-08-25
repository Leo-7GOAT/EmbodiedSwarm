from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = (
    Path(__file__).resolve().parents[1]
    / "ros2"
    / "embodied_swarm_hardware"
)


def test_ros2_skeleton_has_valid_package_and_plugin_xml():
    package = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    plugin = ET.parse(PACKAGE_ROOT / "embodied_swarm_hardware.xml").getroot()
    assert package.findtext("name") == "embodied_swarm_hardware"
    assert plugin.tag == "library"
    assert plugin.find("class").attrib["base_class_type"] == (
        "hardware_interface::SystemInterface"
    )


def test_ros2_skeleton_exposes_required_mapping_and_truthful_gate():
    source = (PACKAGE_ROOT / "src" / "embodied_swarm_system.cpp").read_text()
    readme = (PACKAGE_ROOT / "README.md").read_text()
    assert "state.base_twist[0]" in source
    assert "state.base_twist[2]" in source
    assert "command.base_twist = {base_vx_command_, 0.0, base_wz_command_}" in source
    assert "joint_position_targets" in source
    assert "return false" in source
    assert "skeleton only" in readme.lower()
    assert "not built" in readme.lower()
