from pathlib import Path
import xml.etree.ElementTree as ET
import math


PACKAGE_ROOT = (
    Path(__file__).resolve().parents[1]
    / "ros2"
    / "embodied_swarm_hardware"
)


def test_ros2_has_valid_package_and_plugin_xml():
    package = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    plugin = ET.parse(PACKAGE_ROOT / "embodied_swarm_hardware.xml").getroot()
    assert package.findtext("name") == "embodied_swarm_hardware"
    assert plugin.tag == "library"
    assert plugin.find("class").attrib["base_class_type"] == (
        "hardware_interface::SystemInterface"
    )


def test_ros2_seven_joint_model_and_truthful_backend_boundary():
    model = ET.parse(PACKAGE_ROOT / "config" / "panda.urdf.xacro").getroot()
    control = model.find("ros2_control")
    assert control.findtext("hardware/plugin") == "embodied_swarm_hardware/EmbodiedSwarmSystem"
    assert control.find("hardware/param").attrib["name"] == "backend"
    ns = "{http://www.ros.org/wiki/xacro}"
    joints = control.findall(ns + "control_joint")
    assert {joint.attrib["index"] for joint in joints} == set("1234567")
    for joint in joints:
        lower, initial, upper = [float(joint.attrib[name]) for name in ("lower", "initial", "upper")]
        assert all(map(math.isfinite, (lower, initial, upper)))
        assert lower <= initial <= upper
        assert lower < upper
    macro = control.find(ns + "macro").find("joint")
    assert [item.attrib["name"] for item in macro.findall("command_interface")] == ["position"]
    assert {item.attrib["name"] for item in macro.findall("state_interface")} == {"position", "velocity"}
    readme = (PACKAGE_ROOT / "README.md").read_text()
    assert "PENDING_HARDWARE" in readme
    assert "mock hand" in readme.lower()
    assert "ROS2_ACCEPTANCE.md" in readme
