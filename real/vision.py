"""웹캠 영상에서 소켓 마커를 찾아 깔때기 입구 위치와 소켓 축을 낸다(카메라 좌표계, m).

- Camera: 별도 스레드로 계속 읽어 (촬영 시각, 영상)을 최신 한 장만 들고 있다. 제어 루프가 영상 읽기에 막히지 않게.
- Intrinsics: 카메라 내부 파라미터. ChArUco 보정 결과(json)가 없으면 화각으로 근사한다.
- SocketTracker: ArUco 검출 → IPPE_SQUARE PnP → 마커 자세로 입구 위치(geometry.ENTRANCE_IN_MARKER) 계산.

카메라 보정(한 번): .venv/bin/python -m real.vision calib --cam 0   (보정판 03을 여러 각도로 15장쯤 찍는다)
추적 확인:          .venv/bin/python -m real.vision show --cam 0
"""

import argparse
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from . import geometry as G

CONFIG = Path(__file__).resolve().parent / "config"


@dataclass
class Intrinsics:
    K: np.ndarray
    dist: np.ndarray
    size: tuple  # (폭, 높이)

    @classmethod
    def from_fov(cls, width, height, hfov_deg=70.0):
        """보정 전 근사값. 웹캠 수평 화각은 보통 60~78°다."""
        f = width / 2 / np.tan(np.deg2rad(hfov_deg) / 2)
        K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]])
        return cls(K, np.zeros(5), (width, height))

    @classmethod
    def load(cls, path=CONFIG / "camera.json"):
        d = json.loads(Path(path).read_text())
        return cls(np.array(d["K"]), np.array(d["dist"]), tuple(d["size"]))

    def save(self, path=CONFIG / "camera.json"):
        Path(path).parent.mkdir(exist_ok=True)
        Path(path).write_text(json.dumps({"K": self.K.tolist(), "dist": self.dist.tolist(), "size": list(self.size)},
                                         indent=1))


class Camera:
    """OpenCV 웹캠을 백그라운드에서 읽는다. latest()는 (촬영 시각 time.monotonic, BGR 영상)."""

    def __init__(self, index=0, width=640, height=480, fps=30):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 오래된 영상이 쌓여 지연이 커지지 않게
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {index}를 열 수 없다(맥: 터미널에 카메라 권한 필요)")
        self._frame, self._lock, self._run = None, threading.Lock(), True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()
        while self._frame is None:
            time.sleep(0.01)

    def _loop(self):
        while self._run:
            ok, img = self.cap.read()
            t = time.monotonic()
            if ok:
                with self._lock:
                    self._frame = (t, img)

    def latest(self):
        with self._lock:
            return self._frame

    def close(self):
        self._run = False
        self._th.join(timeout=1)
        self.cap.release()


def _detector(dict_name=G.DICT):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(d, p)


def marker_pose(corners, size, intr: Intrinsics):
    """마커 네 꼭짓점(영상 좌표) → (R, t): 마커 좌표계를 카메라 좌표계로."""
    h = size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, corners.reshape(4, 2).astype(np.float64), intr.K, intr.dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)


class SocketTracker:
    """영상 한 장 → 소켓 입구 위치와 축(카메라 좌표계). 마커가 없으면 None."""

    def __init__(self, intr: Intrinsics):
        self.intr = intr
        self.det = _detector()

    def detect(self, img_bgr):
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.det.detectMarkers(gray)
        return {} if ids is None else {int(i): c for i, c in zip(ids.flatten(), corners)}

    def socket_pose(self, img_bgr):
        """소켓 좌표계 → 카메라 좌표계 (R, t). 탭 마커와 갑판 마커 중 보이는 것의 꼭짓점을 모두 쓴다."""
        found = self.detect(img_bgr)
        board = G.board_points()
        ids = [i for i in found if i in board]
        if not ids:
            return None
        obj = np.concatenate([board[i] for i in ids])
        img = np.concatenate([found[i].reshape(4, 2) for i in ids]).astype(np.float64)
        if len(ids) == 1:  # 마커 하나: 평면 정사각형 전용 해법으로 시작
            size = G.SOCKET_MARKER_SIZE if ids[0] == G.SOCKET_MARKER_ID else G.DECK_MARKER_SIZE
            pose = marker_pose(found[ids[0]], size, self.intr)
            if pose is None:
                return None
            Rm, tm = pose
            center = board[ids[0]].mean(0)  # 마커 축은 소켓 축과 같다
            return Rm, tm - Rm @ center
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.intr.K, self.intr.dist, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.intr.K, self.intr.dist, rvec, tvec)
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.reshape(3)

    def socket(self, img_bgr):
        """깔때기 입구 위치와 소켓 축(카메라 좌표계). 마커가 하나도 없으면 None."""
        pose = self.socket_pose(img_bgr)
        if pose is None:
            return None
        R, t = pose
        return t + R @ G.ENTRANCE_IN_SOCKET, R[:, 2]

    def calib_points(self, img_bgr):
        """보정 시트 마커 중심(카메라 좌표계) {id: 위치}."""
        out = {}
        for mid, c in self.detect(img_bgr).items():
            if mid in G.CALIB_MARKER_IDS:
                pose = marker_pose(c, G.CALIB_MARKER_SIZE, self.intr)
                if pose is not None:
                    out[mid] = pose[1]
        return out


# ---------------- 명령 ----------------

def _calibrate_camera(cam_index, n_target=15):
    """ChArUco 보정판으로 내부 파라미터를 구한다. 스페이스: 저장, q: 끝내고 계산."""
    board = cv2.aruco.CharucoBoard(G.CHARUCO_SQUARES, G.CHARUCO_SQUARE, G.CHARUCO_MARKER,
                                   cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, G.CHARUCO_DICT)))
    det = cv2.aruco.CharucoDetector(board)
    cam = Camera(cam_index)
    objs, imgs, size = [], [], None
    try:
        while True:
            _, img = cam.latest()
            size = (img.shape[1], img.shape[0])
            cc, ci, _, _ = det.detectBoard(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            view = img.copy()
            if ci is not None and len(ci) >= 6:
                cv2.aruco.drawDetectedCornersCharuco(view, cc, ci)
            cv2.putText(view, f"{len(objs)}/{n_target}  space=save q=done", (10, 25), 0, 0.7, (0, 0, 255), 2)
            cv2.imshow("charuco", view)
            k = cv2.waitKey(30) & 0xFF
            if k == ord(" ") and ci is not None and len(ci) >= 6:
                o, i = board.matchImagePoints(cc, ci)
                objs.append(o)
                imgs.append(i)
            if k == ord("q") or len(objs) >= n_target:
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()
    if len(objs) < 5:
        raise SystemExit("보정판 영상이 5장 미만이다")
    rms, K, dist, _, _ = cv2.calibrateCamera(objs, imgs, size, None, None)
    Intrinsics(K, dist.reshape(-1), size).save()
    print(f"카메라 보정 RMS {rms:.2f} px → {CONFIG / 'camera.json'}")


def _show(cam_index):
    cam = Camera(cam_index)
    intr = Intrinsics.load() if (CONFIG / "camera.json").exists() else Intrinsics.from_fov(640, 480)
    tr = SocketTracker(intr)
    try:
        while True:
            t, img = cam.latest()
            s = tr.socket(img)
            view = img.copy()
            msg = "socket: -" if s is None else "socket(cam) mm: " + " ".join(f"{v * 1000:6.1f}" for v in s[0])
            cv2.putText(view, msg, (10, 25), 0, 0.6, (0, 0, 255), 2)
            cv2.imshow("tracker", view)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["calib", "show"])
    ap.add_argument("--cam", type=int, default=0)
    a = ap.parse_args()
    _calibrate_camera(a.cam) if a.cmd == "calib" else _show(a.cam)
