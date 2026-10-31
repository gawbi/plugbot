"""실물 시연 키트의 마커와 소켓 기하(cad/make_parts.py와 같은 치수). 단위 m.

배치(LAYOUT, 환경 변수 PLUGBOT_LAYOUT로 바꿈):
- "side"(기본, 본선): 소켓을 세운 판(cad side_mount)에 붙이고 L자 플러그로 옆에서 꽂는다. 마커 4개는 판 앞면 소켓 좌우.
- "top": 갑판 위 소켓에 일자 플러그로 위에서 꽂는다(예선). 마커 4개는 소켓 아래 갑판 시트.

소켓 좌표계: 원점은 플랜지 바닥 중심, +z는 깔때기 입구 쪽(소켓 축), +x는 마커 탭 쪽.
세운 판에서는 +x(탭)가 위, +y가 로봇에서 볼 때 왼쪽이다.
마커 좌표계(OpenCV ArUco): 원점은 마커 중심, x는 인쇄된 마커의 오른쪽, y는 위쪽, z는 종이 밖.
마커를 탭 홈에 붙일 때 마커의 오른쪽(x)이 탭 바깥쪽(소켓에서 먼 쪽)을 향하게 하면 두 좌표계의 축이 같다.
"""

import os

import numpy as np

MM = 0.001
LAYOUT = os.environ.get("PLUGBOT_LAYOUT", "side")
assert LAYOUT in ("side", "top"), LAYOUT

# 소켓 마커: 탭 홈(30 × 30 mm, 깊이 0.6 mm)에 25 mm 마커를 붙인다
DICT = "DICT_4X4_50"
SOCKET_MARKER_ID = 0
SOCKET_MARKER_SIZE = 25 * MM  # 검은 테두리 바깥 한 변
MARKER_IN_SOCKET = np.array([55.0, 0.0, 3.5]) * MM  # 마커 중심(홈 바닥 3.4 mm + 종이)
ENTRANCE_IN_SOCKET = np.array([0.0, 0.0, 46.0]) * MM  # 깔때기 입구 중심(cad SOCKET_H)
ENTRANCE_IN_MARKER = ENTRANCE_IN_SOCKET - MARKER_IN_SOCKET  # 마커 좌표계로 본 입구(축이 같으므로 그대로)
BORE_BOTTOM_IN_SOCKET = np.array([0.0, 0.0, 4.0]) * MM  # 끝까지 꽂았을 때(자석이 붙을 때) 핀 끝이 닿는 보어 바닥
BORE_BOTTOM_IN_MARKER = BORE_BOTTOM_IN_SOCKET - MARKER_IN_SOCKET

# 갑판 마커 시트: 소켓 아래에 깔고 소켓 볼트 구멍 4개로 위치를 맞춘다. 마커 한 개(25 mm)로 잰 거리는 수 mm~18 mm까지
# 틀어지므로(합성 영상 시험), 소켓 둘레 마커의 꼭짓점을 모두 써서 소켓 자세를 한 번에 푼다.
DECK_MARKER_SIZE = 30 * MM
DECK_MARKERS = {5: (-60.0, 25.0), 6: (-60.0, -55.0), 7: (60.0, -55.0), 8: (10.0, -55.0)}  # 소켓 좌표 (x, y) mm
DECK_SHEET_Z = 0.1 * MM  # 종이 두께
BOLT_PITCH = 54 * MM
PLATE_IN_SOCKET = (0.0, -15.0)  # 갑판판(170 × 120 mm) 중심, 소켓 좌표 mm. 소켓 -y가 로봇(힌지) 쪽

# 세운 판 마커(side): 판 앞면(소켓 플랜지가 붙는 면)에 붙인 30 mm 마커. 소켓 좌표 (x, y) mm, x가 위.
# 소켓 중심 높이는 판 바닥에서 85 mm라 x = -23, +23은 판 높이 62, 108 mm다(cad/make_parts.py side_mount)
PANEL_MARKERS = {5: (-23.0, 57.0), 6: (23.0, 57.0), 7: (-23.0, -57.0), 8: (23.0, -57.0)}
BOARD_MARKERS = PANEL_MARKERS if LAYOUT == "side" else DECK_MARKERS

# 소켓이 둘(선박, 자동차)이고 앞 카메라에 함께 보이므로 마커 번호로 구분한다. 자동차 쪽은 모든 번호에 10을 더한다
# (탭 10, 판 15~18). 선박 쪽은 위 번호 그대로
TARGET_ID_OFFSET = {"ship": 0, "car": 10}

# side 배치 책상 위 놓는 자리(로봇 베이스 기준, m). 선박 갑판: 베이스 중심을 정면 331 mm(힌지가 로봇 쪽),
# 세운 판 앞면이 정면 346 mm, 소켓 입구가 300 mm. 자동차 충전구(car_port_side)는 선박 소켓 자리를 어깨 회전축
# (베이스 중심에서 정면 38.8 mm) 둘레로 왼쪽 50° 돌린 자리라, 시작 자세에 어깨 회전만 더하면 같은 동작이 된다
SIDE_DECK_ORIGIN = np.array([0.331, 0.0, -0.10])
CAR_PORT_ANGLE = np.deg2rad(50.0)
PAN_AXIS_XY = np.array([0.0388, 0.0, 0.0])  # SO-101 shoulder_pan 축(MuJoCo 모델)


def board_points(target="ship"):
    """{마커 id: 소켓 좌표계 꼭짓점 4개(ArUco 순서: 왼위, 오위, 오아래, 왼아래)}. target은 "ship" 또는 "car"."""
    def corners(c, size):
        h = size / 2
        return np.array([c + [-h, h, 0], c + [h, h, 0], c + [h, -h, 0], c + [-h, -h, 0]])
    k = TARGET_ID_OFFSET[target]
    pts = {SOCKET_MARKER_ID + k: corners(MARKER_IN_SOCKET, SOCKET_MARKER_SIZE)}
    for mid, (x, y) in BOARD_MARKERS.items():
        pts[mid + k] = corners(np.array([x * MM, y * MM, DECK_SHEET_Z]), DECK_MARKER_SIZE)
    return pts


# 카메라-로봇 보정: 플러그 핀 끝을 마커 중심에 대는 점 4개
CALIB_MARKER_IDS = (1, 2, 3, 4)
CALIB_MARKER_SIZE = 30 * MM
CALIB_SPACING = 60 * MM  # 시트 위 마커 중심 간격(2 × 2 배치)

# 카메라 보정판(ChArUco): A4 한 장
CHARUCO_SQUARES = (7, 5)
CHARUCO_SQUARE = 30 * MM
CHARUCO_MARKER = 22 * MM
CHARUCO_DICT = "DICT_5X5_100"
