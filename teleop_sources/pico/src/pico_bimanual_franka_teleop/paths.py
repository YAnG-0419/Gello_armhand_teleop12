from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = REPO_ROOT / "assets" / "dual_fr3"
URDF_PATH = ASSET_ROOT / "dual_fr3_kinematics.urdf"
MJCF_PATH = ASSET_ROOT / "scene.xml"
