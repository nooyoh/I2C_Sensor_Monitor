# I2C Sensor Monitor (Basys3 × 2)

Basys3 보드 2개를 I2C로 연결해 센서 값을 읽고 표시하는 프로젝트입니다.

- **Master** : Slave 센서 값 읽기 → FND / LED / UART / SSD1306 OLED(0x3C) 표시
- **Slave (0x42)** : DHT11(온습도) + HC-SR04(거리) 측정 → I2C 레지스터로 제공

```
[Slave Basys3] ── SDA/SCL ──┬── [Master Basys3] ── UART → PC
  DHT11, HC-SR04            └── [SSD1306 OLED 0x3C]
```

## 폴더 구조

```
master/
  src/       Verilog 소스 (top.v 가 최상위)
  constrs/   Basys3 XDC (I2C : JA1 = SDA, JA2 = SCL, 내부 PULLUP)
  sim/       testbench (tb_master.v, tb_oled.v)
  docs/      블록 다이어그램 (draw.io)
slave/
  src/       Verilog 소스 (top.v 가 최상위)
```

## Master 모듈

| 파일 | 역할 |
|---|---|
| `top.v` | 최상위 연결 |
| `i2c_master.v` | I2C bit-level 엔진 (100 kHz, 1 bit = 4 phase, 명령 START/WRITE/READ/STOP) |
| `i2c_arbiter.v` | sensor_reader / oled_controller 버스 중재 (센서 우선) |
| `sensor_reader.v` | Slave 레지스터 0x00~0x05 읽기 (200 ms 주기, 마지막 byte NACK) |
| `oled_controller.v` | SSD1306 초기화 + GDDRAM 512 byte 전송 |
| `oled_text.v`, `font_5x7.v` | OLED 화면 글자 / 막대그래프 생성 |
| `fnd_sensor_display.v`, `fnd_controller.v` | 7-segment 표시 |
| `uart_sensor_sender.v`, `uart_tx.v` | PC로 센서 값 전송 |
| `btn_debounce10ms.v`, `tick_generator.v` | 공통 유틸 |

## Slave 레지스터 맵

| 주소 | 내용 |
|---|---|
| 0x00 | STATUS (`[7:4]`=0xA 서명, `[1]` DIST_OK, `[0]` DHT_OK) |
| 0x01 / 0x02 | 온도 정수 / 소수 |
| 0x03 / 0x04 | 습도 정수 / 소수 |
| 0x05 | 거리 (cm) |

읽기 순서 : `S 0x84 0x00 Sr 0x85 [6 byte, 마지막 NACK] P`

## 빌드

Vivado에서 새 프로젝트(Basys3, xc7a35tcpg236-1)를 만들고 각 보드의 `src/*.v`와 XDC를 추가한 뒤, `top`을 최상위로 지정해 합성합니다.
