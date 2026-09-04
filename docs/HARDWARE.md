# 하드웨어 사양

시각장애인 안내 로봇의 하드웨어 구성과 Hello Robot Stretch 3 플랫폼 사양.
설계·파라미터 튜닝 시 이 문서의 물리 제약을 기준으로 한다.

---

## 하드웨어
- 로봇: Hello Robot Stretch SE3
- OS: Ubuntu 22.04 / ROS2 Humble
- LiDAR: RPLidar (자율주행 장애물 감지)
- 카메라: Intel RealSense D435i (RGB + Depth, 시각 보조 및 장애물 감지)
- 마이크: ReSpeaker 4 Mic Array (index 5)
- 스피커: 내장 HDA Intel PCH ALC256 Analog (index 0)
- 버튼/센서: Arduino FTDI (`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0`)

## 하드웨어 성능 (Hello Robot Stretch 3)

### 컴퓨팅
| 항목 | 사양 |
|------|------|
| CPU | Intel Core i5-8259U |
| RAM | 16 GB |
| 저장장치 | 480 GB SSD |

### 외형 및 배터리
| 항목 | 사양 |
|------|------|
| 무게 | 24.5 kg |
| 크기 (W × D × H) | 330 × 340 × 1410 mm |
| 배터리 | 12V SLA × 2개 (합계 18 AH) |
| 동작 시간 | 약 2~5시간 (부하에 따라 다름) |

### 이동 (Base)
| 항목 | 사양 |
|------|------|
| 구동 방식 | 차동 구동 (Differential Drive, 스테퍼 모터 × 2) |
| 최대 속도 | 0.3 m/s |
| 극복 가능 턱 높이 | 1 cm |
| 절벽 감지 센서 | Sharp GP2Y0A51SK0F × 4 (감지 범위 2~15 cm) |
| IMU | Bosch BNO085 9-DOF |

### 리프트 (Lift)
| 항목 | 사양 |
|------|------|
| 수직 이동 범위 | 110 cm |
| 최대 페이로드 | 5 kg |

### 암 (Arm)
| 항목 | 사양 |
|------|------|
| 수평 연장 범위 | 51 cm |
| 최대 페이로드 | 3 kg |
| 구조 | 알루미늄 텔레스코핑 5단 링크 |

### 손목 및 그리퍼
| 항목 | 사양 |
|------|------|
| Yaw 범위 | 330° |
| Pitch 범위 | 150° |
| Roll 범위 | 345° |
| 그리퍼 최대 페이로드 | 2 kg |
| 그리퍼 최대 개구 | 15 cm |

### 헤드 (Pan-Tilt)
| 항목 | 사양 |
|------|------|
| Pan 범위 | 346° (-234° ~ +112°) |
| Tilt 범위 | 115° (-25° ~ +90°) |

### 내장 센서 및 I/O
| 항목 | 사양 |
|------|------|
| 헤드 카메라 | Intel RealSense D435if (최대 인식 거리 10 m) |
| 그리퍼 카메라 | Intel RealSense D405 (최적 범위 7~50 cm) |
| 보조 RGB 카메라 | Arducam OV9782 광각 |
| LiDAR | Slamtec RPLIDAR A1 (범위 0.15~12 m, 각도 해상도 1°) |
| 마이크 어레이 | ReSpeaker v2.0 (4-MEMS 마이크, 음성 인식 최대 5 m) |
| 손목 가속도계 | ADXL343 3축 |
| 개발자 전원 (트렁크) | 12V @ 5A |
| USB 허브 | USB 3.0 4포트 |

> **참고:** 이 프로젝트는 내장 D435if 대신 외장 **Intel RealSense D435i**를 시각 보조용으로 추가 사용하며, RPLIDAR를 자율주행 장애물 감지에 활용한다. 최대 주행 속도 0.3 m/s와 턱 높이 1 cm 제약을 고려해 경로 계획 및 속도 파라미터를 설정해야 한다.

---

## 카메라 시리얼 고정 (필수)

카메라 2대는 반드시 `serial_no`로 고정한다. 고정하지 않으면 부팅 순서에 따라
body/gripper가 뒤바뀐다.

| 카메라 | 위치 | 시리얼 | 비고 |
|---|---|---|---|
| RealSense D435if | body | `250222073610` | USB3, `/camera/camera/` |
| RealSense D405 | gripper | `130322271074` | USB2 딥허브, `image_rect_raw`만 발행 |

**D405 제약 (실측 확정)**
- USB2 연결이므로 color `1280x720x6fps`가 상한
- depth 파라미터 이름은 `depth_module.depth_profile`
- 지원 프로파일: Depth `480x270 {60,30,15,5}` / Color `1280x720 {15,10,5}` — **6은 없음**

## 알려진 하드웨어 이슈

| 증상 | 원인 / 대응 |
|---|---|
| 카메라가 USB에서 사라짐 | 웜 리부트로 복구 안 됨 → **종료 후 메인 전원 30초 차단(콜드 리셋)** |
| 런치 잦은 재시작 시 리얼센스 얼음 | 런치는 유지하고 **앱만** 재시작 |
| 배터리 `pct`가 NaN | **voltage 기준으로 판단.** `round(NaN)`이 spin 스레드를 죽인 이력 있음 |
| 주행 중 멈춤 | 뎁스 포인트클라우드가 UDP로 무거워 transform 캐시 드롭 유발 |
| 유령 참가자 | FastDDS SHM 비활성 → UDP 강제 (`~/.ros/fastdds_no_shm.xml`) |

## 아두이노 포트 고정 경로

```
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
```
재부팅 후에도 포트 번호가 변경되지 않는다.
