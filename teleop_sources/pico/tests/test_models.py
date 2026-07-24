import mujoco
import numpy as np

from pico_bimanual_franka_teleop.ik import BimanualPinkIK
from pico_bimanual_franka_teleop.paths import MJCF_PATH
from teleop_core.contract import COMMAND_JOINT_NAMES


def test_pinocchio_and_mujoco_have_the_same_joint_order():
    ik = BimanualPinkIK()
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    mujoco_names = tuple(model.joint(index).name for index in range(model.njnt))
    assert ik.joint_names == COMMAND_JOINT_NAMES
    assert mujoco_names == COMMAND_JOINT_NAMES
    assert model.nq == model.nu == 14


def test_home_keyframe_is_inside_both_models_limits():
    ik = BimanualPinkIK()
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    assert np.all(data.qpos >= ik.model.lowerPositionLimit)
    assert np.all(data.qpos <= ik.model.upperPositionLimit)
    np.testing.assert_allclose(data.qpos, model.key("home").qpos)


def test_mujoco_home_pose_has_no_collision_and_holds_under_gravity():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    arm_geoms = [
        index
        for index in range(model.ngeom)
        if model.geom(index).name.startswith(("left_link", "right_link"))
    ]
    assert arm_geoms
    assert all(
        model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH for index in arm_geoms
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    home = data.qpos.copy()
    data.ctrl[:] = home
    mujoco.mj_forward(model, data)
    assert data.ncon == 0

    for _ in range(500):
        mujoco.mj_step(model, data)

    assert data.ncon == 0
    assert np.max(np.abs(data.qpos - home)) < 0.005
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


def test_ik_holds_without_active_targets():
    ik = BimanualPinkIK()
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    np.testing.assert_allclose(ik.step(data.qpos, {}), data.qpos)


def test_ik_clamps_slightly_out_of_limit_seed():
    ik = BimanualPinkIK()
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    q = data.qpos.copy()
    q[6] = ik.model.lowerPositionLimit[6] - 1e-5
    held = ik.step(q, {})
    assert held[6] >= ik.model.lowerPositionLimit[6]



def test_left_target_does_not_move_disconnected_right_chain():
    ik = BimanualPinkIK(dt=0.01)
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    target = ik.frame_pose(data.qpos, "left")
    target = type(target)(target.position + np.array([0.01, 0.0, 0.0]), target.rotation)
    result = ik.step(data.qpos, {"left": target})
    assert np.linalg.norm(result[:7] - data.qpos[:7]) > 0.0
    np.testing.assert_allclose(result[7:], data.qpos[7:], atol=1e-9)
