# PlugBot 사실표 / Fact Sheet

포트폴리오(README, 웹, PDF, 이력서)의 숫자와 주장은 모두 이 표에서만 가져온다. 표에 없는 숫자는 쓰지 않는다.
Every number in the portfolio (README, web page, PDF, résumé) comes from this sheet only.

상태 / Status: **실물 확인** = real robot, judged by a person · **시뮬레이션** = simulation · **시각화** = kinematic visualization (no contact physics)

## 1. 결과 / Results

| 주장 / Claim | 숫자 / Number | 조건 / Conditions | 상태 | 출처 / Source |
|---|---|---|---|---|
| Real-to-Real, 정지 소켓 / static socket | 20/20 (95% CI 84~100%) | ACT `act_real_v4` 80k steps, 사람 원격조종 시범 134개 / 134 human teleop demos | 실물 확인 | `outputs/real_results.json`, `outputs/real_results_chart.png` |
| Real-to-Real, 흔들림 소켓 / moving socket | 20/20 (84~100%) | 같은 모델, 흔들림 갑판 / same model, servo-driven shaking deck | 실물 확인 | 같음 |
| Sim-to-Real, 정지 소켓 | 20/20 (84~100%) | ACT `act_twin_v4`: 트윈 렌더로만 학습한 `act_twin_v2`(2만 스텝, 보정 전 렌더)에서 보정 후 렌더로 1천 스텝 추가 학습. 실물 사진 0장 / fine-tuned 1k steps on calibrated twin renders from a 20k-step twin-only model; zero real images | 실물 확인 | 같음 |
| Sim-to-Real, 흔들림 소켓 | 19/20 (76~99%) | 같은 모델(act_twin_v4 1k, 사용자 확인 2026-11-14) / same model | 실물 확인 | 같음 |
| 합계 / Totals | R2R 40/40 (91~100%), S2R 39/40 (87~100%) | 두 노트북 합산, 2026-11-09~10 선상 / two laptops combined, aboard ship | 실물 확인 | 같음 |
| 더 학습한 트윈 모델은 실패 / longer sim training failed | 2k, 3k, 4k steps 각 0/1 | 같은 데이터로 학습만 더 함 / same data, more steps | 실물 확인(소수) | `data/rollouts/*/result.json` (tally_rollouts) |
| 예측 대 따라가기 / predict vs follow, 파고 6 cm | 예측 88% (80~93), 따라가기 46% (37~56), 정지 25% (18~34) | MuJoCo, 같은 시드 1000~1099, 100회 / paired seeds, 100 trials | 시뮬레이션 | `outputs/compare_predict_follow.json` |
| 짝 비교 / paired outcome, 6 cm | 예측만 성공 45, 따라가기만 성공 3 | 같은 파도 / same wave | 시뮬레이션 | 같음 |
| 다른 파고 / other wave heights | 4 cm: 99 / 87 / 76%, 8 cm: 60 / 24 / 12% (예측/따라가기/정지) | 같음 | 시뮬레이션 | 같음 |
| 소켓을 치는 힘 / contact force p95, 6 cm | 따라가기 약 311 N, 예측 약 37 N | 강체 접촉 모델, 비교로만 / rigid contact model, relative only | 시뮬레이션 | 같음 |
| 첫 실험(30회) 재현 / earlier 30-trial run | 6 cm: 예측 87%, 따라가기 50%, 정지 20% | 시드 0~29 | 시뮬레이션 | `outputs/experiment_side.json` |
| 물리 엔진 갭 / physics-engine gap | Isaac 6 cm: 따라가기 100%, 예측 97%, 정지 63% (충돌을 실패로 셈) | Isaac Sim 5.1 PhysX, 같은 제어 코드, 30회 | 시뮬레이션 | `outputs/ISAAC_RESULTS.md` |
| 갭 원인 / gap cause | 서보 모델은 원인 아님, 원인 미확인 / not the servo model; root cause not identified | Isaac 드라이브를 MuJoCo 값으로 바꿔도 따라가기 100% | 시뮬레이션 | `outputs/ISAAC_RESULTS.md` |
| 실물 조건 흔들기 / robustness sweep | 예측은 지연 0.05~0.2 s, 잡음, 보정 오차, 강성 변화에서 대체로 90% 이상 | MuJoCo, 파고 4 cm, 30회 | 시뮬레이션 | `outputs/sim2real_robustness.json` |
| Isaac 시범으로 학습한 ACT / ACT trained on Isaac demos | Isaac 안에서 20/20 (0~6 cm) | 분포 안 평가, 실물 근거 아님 / in-distribution only | 시뮬레이션 | `outputs/ISAAC_RESULTS.md` |
| 이종 원격조종 / heterogeneous teleop | SO-101 리더 팔 → Isaac 안의 UR10e(실제 크기) 실시간 조종, 부두 소켓 꽂기 | 5배 확대 + Lula IK, 팀원 노트북(Isaac 6.2) | 시각화(시뮬레이션) | `isaac/scaleup/README.md`, branch `teleop-laptop2` |
| 대회 결과 / award | 본선 공동 2위, (재)부산테크노파크 원장상 / joint 2nd place, Busan Techno Park President's Award | 2026 동남권 멀티도메인 모빌리티 비즈니스 해커톤 개인부문, 107팀 참가 / 107 teams | 사실 | README(2026-11-11), 사용자 보고 |

## 2. 쓰면 안 되는 말 / Do not claim
- 실물 산업용 로봇(UR10e)으로 했다 / ran on a real industrial robot → **Isaac 시뮬레이션만**
- UR10e가 학습했다 / UR10e learned → 조종과 재생만 / teleop and replay only
- 실물 로봇이 0.3초 앞을 예측해 꽂는다 → 예측기는 **시뮬레이션 제어기**, 실물 ACT에는 없다 / predictor exists only in simulation
- 두 엔진이 예측의 우위를 확인했다 → Isaac에서는 **재현되지 않았다** / not reproduced in Isaac
- 흔들려도 95% 등 표에 없는 숫자 / any number not in this sheet
- 실제 배에서 꽂았다 → 배 **위에서** 흔들림 갑판(모형)으로 시험했다 / tested on a model shaking deck aboard the ship

## 3. 타임라인 / Timeline
| 날짜 / Date | 일 / Event |
|---|---|
| 10/31 | 첫 커밋, 현측 옆 꽂기 배치 결정 / first commit, side-insertion layout |
| 11/1 | Isaac Sim 5.1 첫 실행, 30회 실험 / first Isaac runs |
| 11/2 | Isaac 시범 ACT(시뮬 안 20/20), UR10e 확대 시각화, 실물 키트 시작 |
| 11/3~11/5 | 카메라, 그리퍼 덮개 문제 해결, 원격조종 녹화, 녹화 화면 / camera + cover fixes, recording GUI |
| 11/6 | **실물 첫 자율 꽂기**(정지 소켓), 하루 56커밋 / first autonomous insertion |
| 11/8 | 승선, 기준선 태그 `boarding-20261108` / boarding baseline |
| 11/9 | 선상 실물 측정, 포토박스 Isaac 트윈 구축 / onboard tests, photobox twin |
| 11/10 | 트윈 카메라 보정 → Sim-to-Real 성공, 최종 결과표, 제출 / camera calibration, final results |
| 결과 | 본선 공동 2위, 부산테크노파크 원장상(107팀) / joint 2nd of 107 teams |

## 4. 팀과 역할 / Team and roles (팀 Land2Sea, 4명)
- **김광빈(팀장 / team lead)**: 시스템 아키텍처, 실물 로봇 소프트웨어(녹화 화면, 원격조종, 안전 장치), ACT 학습과 평가, MuJoCo 실험, Isaac 포트와 디지털 트윈, 하드웨어 통합 설계(덮개 CAD, 카메라 배치), 측정과 문제 기록. 저장소 커밋 전부(main 113, 전체 193).
- 조선기: 하드웨어 제작(부스, 흔들림 갑판, 카메라 고정, 소켓 정렬) / hardware build
- 나다원: 발표 구성, 대본, 질의응답 / pitch and presentation
- 이주은: 시장, 가격, 사업 모델, 데이터 출처 조사 / business and market research
- 개발 방식 / how it was built: AI 코딩 에이전트(Claude Code)를 지휘해 22일 안에 하위 시스템 4개를 통합. 방향, 실험 설계, 판정, 하드웨어 디버깅, 주장 검증은 직접 했다.

## 5. 규모 / Scale
- Python 77개 파일, 11,332줄 / 77 files, 11,332 lines; main 113 commits (193 all branches), 2026-10-31 ~ 11-10 (11일 / 11 days). 발표의 '22일'은 예선부터 센 기간
- 실물 시범 134개(정지 94, 흔들림 40), 트윈 렌더 에피소드 69개(30,920장면), 실물 실행 기록 106회
- 문제 기록 19건(원인과 해결) / 19 logged hardware-software incidents: `docs/troubleshooting.md`
