"""URDF 순기구학(numpy). MuJoCo 모델과 Isaac에서 쓸 URDF가 같은 관절각에서 같은 자세인지 확인하는 데 쓴다."""

import xml.etree.ElementTree as ET

import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(a, t):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


class URDF:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([0, 0, 1.0])
            self.joints[j.find("child").get("link")] = (j.get("name"), j.get("type"), j.find("parent").get("link"),
                                                        xyz, _rpy(*rpy), axis)

    def link_pose(self, link, q):
        """q: {관절 이름: rad} → 링크 좌표계의 (R, p) (베이스 기준)."""
        chain = []
        while link in self.joints:
            chain.append(self.joints[link])
            link = self.joints[link][2]
        R, p = np.eye(3), np.zeros(3)
        for name, typ, _, xyz, Ro, axis in reversed(chain):
            p = p + R @ xyz
            R = R @ Ro
            if typ in ("revolute", "continuous"):
                R = R @ _axis_angle(axis, q.get(name, 0.0))
        return R, p
