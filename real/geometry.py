"""실물 시연 키트의 마커와 소켓 기하(cad/make_parts.py와 같은 치수). 단위 m.

소켓 좌표계: 원점은 플랜지 바닥 중심, +z는 깔때기 입구 쪽(소켓 축), +x는 마커 탭 쪽.
마커 좌표계(OpenCV ArUco): 원점은 마커 중심, x는 인쇄된 마커의 오른쪽, y는 위쪽, z는 종이 밖.
마커를 탭 홈에 붙일 때 마커의 오른쪽(x)이 탭 바깥쪽(소켓에서 먼 쪽)을 향하게 하면 두 좌표계의 축이 같다.
"""

import numpy as np

MM = 0.001

# 소켓 마커: 탭 홈(30 × 30 mm, 깊이 0.6 mm)에 25 mm 마커를 붙인다
DICT = "DICT_4X4_50"
SOCKET_MARKER_ID = 0
SOCKET_MARKER_SIZE = 25 * MM  # 검은 테두리 바깥 한 변
MARKER_IN_SOCKET = np.array([55.0, 0.0, 3.5]) * MM  # 마커 중심(홈 바닥 3.4 mm + 종이)
ENTRANCE_IN_SOCKET = np.array([0.0, 0.0, 39.5]) * MM  # 깔때기 입구 중심
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


def board_points():
    """{마커 id: 소켓 좌표계 꼭짓점 4개(ArUco 순서: 왼위, 오위, 오아래, 왼아래)}."""
    def corners(c, size):
        h = size / 2
        return np.array([c + [-h, h, 0], c + [h, h, 0], c + [h, -h, 0], c + [-h, -h, 0]])
    pts = {SOCKET_MARKER_ID: corners(MARKER_IN_SOCKET, SOCKET_MARKER_SIZE)}
    for mid, (x, y) in DECK_MARKERS.items():
        pts[mid] = corners(np.array([x * MM, y * MM, DECK_SHEET_Z]), DECK_MARKER_SIZE)
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
