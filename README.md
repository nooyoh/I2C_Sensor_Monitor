# 📡 I2C Sensor Monitor (Basys3 × 2)

> Basys3 보드 2개를 I2C로 연결해 온습도·거리 센서 값을 읽고 FND · LED · UART · OLED · Qt GUI로 표시하는 **Verilog RTL 설계 프로젝트**

![Verilog](https://img.shields.io/badge/Verilog-RTL-blue) ![Vivado](https://img.shields.io/badge/Vivado-Xilinx-red) ![FPGA](https://img.shields.io/badge/Board-Basys3%20×2-green) ![Qt](https://img.shields.io/badge/Qt-SerialPort-41CD52)

---

## 📌 개요

- **I2C Master / Slave, 센서, OLED 드라이버를 모두 RTL로 직접 설계** (IP 미사용)
- **Slave(0x42)** 가 DHT11 · HC-SR04를 측정하고, **Master** 가 0.2초마다 6 byte를 읽어 표시
- Master의 I2C 버스 하나를 **센서 읽기와 OLED(0x3C) 출력이 공유** → 자체 Arbiter로 중재
- 센서 하나가 실패해도 통신은 유지하고, **실패한 값만 `--` 로 표시**하는 예외 처리

## 🎬 동작 영상



## 🏗️ 시스템 아키텍처

<img width="2470" height="1164" alt="zz" src="https://github.com/user-attachments/assets/16ef89de-5ef5-4433-9e2d-4788a51844d5" />


## 🛠️ 기술 스택

| 분류 | 사용 기술 |
|---|---|
| HDL | Verilog HDL |
| 통신 | I2C (Master / Slave 직접 설계), UART 9600 bps |
| GUI | Qt (C++17, Qt Widgets, QSerialPort) |
| 검증 | Vivado Simulator(Testbench), 로직 분석기 |

## 🔌 사용 하드웨어

| 부품 | 용도 |
|---|---|
| Digilent Basys3 × 2 | Master / Slave (Artix-7 xc7a35tcpg236-1) |
| DHT11 | 온도 · 습도 |
| HC-SR04 | 초음파 거리 |
| SSD1306 OLED 0.91" (128×32) | 센서 값 표시 (I2C 0x3C) |

> I2C 핀 : `JA1 = SDA`, `JA2 = SCL` (내부 PULLUP, Open-drain 구동)

## ✨ 주요 기능

- **I2C 읽기** : `S 0x84 0x00 Sr 0x85 [6 byte, 마지막 NACK] P` (Repeated START로 방향 전환)
- **버스 중재** : 비어 있을 때만 grant, 동시 요청 시 센서 우선 / OLED는 512 byte를 끊김 없이 연속 전송
- **OLED 출력** : 5×7 폰트를 세로 8 pixel column byte로 변환해 PAGE0~3, 512 byte 전송
- **상태 표시** : LED로 PASS / FAIL / NACK / 동작 중 / 센서 값 표시, 버튼으로 FND 표시 항목 전환
- **예외 처리**
  - 센서 실패(STATUS `0xA2`, `0xA1`) → 해당 값만 `--`, 나머지는 정상 갱신
  - 서명 불일치 · Slave NACK → FAIL/NACK LED + 이전 값 유지
  - OLED NACK → 초기화 명령부터 재전송
- **PC 모니터링** : Qt GUI가 Slave UART 프레임을 받아 온도·습도·거리와 센서 오류를 표시

### 📦 I2C 데이터 (Slave 레지스터)

| 주소 | 내용 |
|---|---|
| `0x00` | STATUS — `[7:4]`=`0xA` 서명, `[1]` DIST_OK, `[0]` DHT_OK (모두 정상 `0xA3`) |
| `0x01` / `0x02` | 온도 정수 / 소수 |
| `0x03` / `0x04` | 습도 정수 / 소수 |
| `0x05` | 거리 (cm) |

## 🧪 테스트 및 검증

- **Master TB** : 정상 → 버튼 → DHT 실패 → 거리 실패 → 서명 불일치 → NACK → 복구, 7개 시나리오 검증
- **Slave TB** : trigger/echo, DHT11 40bit 수신, I2C 6 byte 송신, UART 출력이 기대값 `a3 19 04 3d 07 0a`와 일치
- **실측** : 로직 분석기에서 `A3 19 08 30 00 09` 수신 = OLED `25.8℃ / 48.0% / 9cm` 일치 (프레임 약 0.85ms)

## 🔧 트러블슈팅

**문제** : 첫 센서 읽기 후 I2C 버스가 멈추고 이후 통신이 계속 NACK

- **원인** : Master가 마지막(6번째) byte에도 ACK를 보내 Slave가 다음 byte를 송신 → SDA가 LOW로 고정되어 STOP 미성립
- **해결** : 마지막 byte만 NACK 전송 → Slave가 SDA를 놓아 STOP 정상 성립
  ```verilog
  read_ack <= (r_rx_cnt == RX_LAST);  // 마지막 byte만 NACK
  ```
- **배운 점** : I2C 읽기에서 NACK는 오류가 아니라 **'읽기 끝' 신호**이며, SDA를 누가 언제 놓는지까지 설계해야 한다

## 📁 파일 구조

```
I2C_Sensor_Monitor/
├── master/
│   ├── src/        # top, i2c_master, i2c_arbiter, sensor_reader, oled_controller,
│   │               # oled_text, font_5x7, fnd_*, uart_*, btn_debounce10ms, tick_generator
│   ├── constrs/    # Basys3_master.xdc
│   └── sim/        # tb_master.v, tb_oled.v
├── slave/
│   ├── src/        # top, i2c, dht11, hcsr04, data_sender, fnd_controller, uart_tx, tick_generator
│   ├── constrs/    # basys3_slave.xdc
│   └── sim/        # tb_top.v
└── qt/             # PC 모니터링 GUI (basys3_sensor_monitor.pro)
```

## 👥 팀 협업

| 이름 | 담당 |
|---|---|
| **이호윤** | Master 보드 — I2C Master, Bus Arbiter, Sensor Reader, OLED Controller, FND/UART 출력 |
| **곽혜민** | Slave 보드 — I2C Slave, DHT11 · HC-SR04 드라이버, UART 전송, Qt 모니터링 GUI |

- 6 byte 데이터 포맷(STATUS 서명 + 센서 값)을 먼저 합의하고 보드별로 분업
- 로직 분석기 실측으로 Master ↔ Slave 간 데이터 해석 일치 여부를 함께 검증

## 💻 개발 환경

| 항목 | 내용 |
|---|---|
| FPGA 툴 | Xilinx Vivado |
| Target | Basys3 (xc7a35tcpg236-1), 100 MHz |
| GUI | Qt Creator, Qt Widgets + SerialPort, C++17 |
| UART | 9600 bps, 8N1 |

### ▶️ 빌드 및 실행

1. Vivado에서 Basys3 프로젝트 생성 → 각 보드의 `src/*.v`, `constrs/*.xdc` 추가 → `top`을 최상위로 합성 후 각 보드에 다운로드
2. 두 보드의 `JA1(SDA)`, `JA2(SCL)`, GND를 연결하고 OLED를 같은 버스에 연결
3. `qt/basys3_sensor_monitor.pro`를 Qt Creator로 열어 빌드 → Slave 보드 COM 포트 연결

## 💭 회고

- **이호윤** : 센서 읽기와 화면 출력이 같은 버스를 동시에 쓰려다 충돌을 겪으며, 장치 연결만큼 **공유 자원을 어떻게 분배하는지**가 중요하다는 것을 배웠다.
- **곽혜민** : 통신은 양쪽이 **데이터를 같은 의미로 해석하도록 규칙을 정하는 것**이 핵심이며, FPGA에서는 계산 결과뿐 아니라 **계산에 걸리는 시간**도 중요하다는 것을 배웠다.
