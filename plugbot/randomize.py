"""Sim-to-Real용 모델 바꾸기: 물리(서보 강성, 마찰)와 화면(조명, 색, 카메라 자리) 무작위화.

- perturb(scene, kp_scale=, friction=): 강건성 시험(sim2real_robustness.py)에서 한 가지씩 바꾼다.
- randomize(scene, rng): 학습 데이터(make_sim_dataset.py --randomize)에서 에피소드마다 한꺼번에 바꾼다.
- restore(scene): 처음 값으로 되돌린다.
모델 값만 바꾸므로 다시 컴파일할 필요가 없다.
"""

import numpy as np


def _snapshot(scene):
    m = scene.model
    if not hasattr(scene, "_orig"):
        scene._orig = {k: getattr(m, k).copy() for k in ("actuator_gainprm", "actuator_biasprm", "geom_friction",
                                                         "geom_rgba", "light_diffuse", "light_dir", "light_pos",
                                                         "cam_pos", "cam_quat", "mat_rgba")}
    return scene._orig


def restore(scene):
    for k, v in _snapshot(scene).items():
        getattr(scene.model, k)[:] = v


def perturb(scene, kp_scale=1.0, friction=None):
    """팔 서보 위치 강성을 kp_scale배로, 소켓 마찰을 friction으로."""
    o, m = _snapshot(scene), scene.model
    a = scene.act
    m.actuator_gainprm[a, 0] = o["actuator_gainprm"][a, 0] * kp_scale
    m.actuator_biasprm[a, 1] = o["actuator_biasprm"][a, 1] * kp_scale
    if friction is not None:
        for g in scene.socket_geoms:
            m.geom_friction[g, 0] = friction


def randomize(scene, rng, physics=True, visual=True):
    """에피소드마다 한꺼번에 바꾼다. 바꾼 값을 dict로 돌려준다(meta.json에 남긴다)."""
    restore(scene)
    m, info = scene.model, {}
    if physics:
        info["kp_scale"] = float(rng.uniform(0.6, 1.4))
        info["socket_friction"] = float(rng.uniform(0.15, 0.6))
        perturb(scene, info["kp_scale"], info["socket_friction"])
    if visual:
        o = scene._orig
        s = rng.uniform(0.5, 1.4)
        m.light_diffuse[:] = np.clip(o["light_diffuse"] * s * rng.uniform(0.85, 1.15, (m.nlight, 3)), 0, 1)
        m.light_dir[:] = o["light_dir"] + rng.normal(0, 0.15, o["light_dir"].shape)
        info["light_scale"] = float(s)
        for name in ("quay", "water", "hull", "side_panel", "cabin"):
            try:
                g = m.geom(name).id
            except KeyError:  # 배치나 장면에 없는 것
                continue
            m.geom_rgba[g, :3] = np.clip(o["geom_rgba"][g, :3] + rng.normal(0, 0.08, 3), 0, 1)
        m.mat_rgba[:, :3] = np.clip(o["mat_rgba"][:, :3] * rng.uniform(0.85, 1.15, (m.nmat, 1)), 0, 1)
        # 앞 카메라 자리: 실물 셀카봉을 다시 세울 때의 차이(±1.5 cm, ±2°)
        c = m.camera("front").id
        m.cam_pos[c] = o["cam_pos"][c] + rng.uniform(-0.015, 0.015, 3)
        q = o["cam_quat"][c] + rng.normal(0, 0.017, 4)
        m.cam_quat[c] = q / np.linalg.norm(q)
        info["front_cam_shift_mm"] = ((m.cam_pos[c] - o["cam_pos"][c]) * 1000).round(1).tolist()
    return info
