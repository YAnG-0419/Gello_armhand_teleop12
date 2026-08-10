from pathlib import Path
from xml.etree import ElementTree

import yaml


ROOT = Path(__file__).resolve().parent


def test_hand2_configs_resolve_twenty_joint_models():
    for side, prefix in (("left", "l_"), ("right", "r_")):
        config_path = ROOT / "config" / f"retarget_manus_wuji_hand_2_{side}.yaml"
        optimizer = yaml.safe_load(config_path.read_text())["optimizer"]
        urdf_path = (config_path.parent / optimizer["urdf_path"]).resolve()
        mjcf_path = (config_path.parent / optimizer["mjcf_path"]).resolve()
        assert urdf_path.is_file()
        assert mjcf_path.is_file()

        urdf = ElementTree.parse(urdf_path).getroot()
        urdf_joints = {
            joint.attrib["name"]
            for joint in urdf.findall("joint")
            if joint.attrib.get("type") in {"revolute", "continuous"}
        }
        mjcf_joints = {
            joint.attrib["name"]
            for joint in ElementTree.parse(mjcf_path).getroot().findall(".//joint")
            if joint.attrib.get("name")
        }
        assert len(urdf_joints) == 20
        assert urdf_joints == mjcf_joints
        assert all(name.startswith(prefix) for name in urdf_joints)
