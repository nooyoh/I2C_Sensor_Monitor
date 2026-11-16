`timescale 1ns / 1ps

// Master basys3 top : I2C bus(JA1:SDA, JA2:SCL)로 Slave basys3의 센서 값을 읽어 OLED / FND / UART에 출력
//   I2C bus 공유 : sensor_reader(Slave 읽기) 와 oled_controller(OLED 쓰기)가 i2c_arbiter 를 통해 i2c_master 사용
//   Slave data : [STATUS][온도 정수][온도 소수][습도 정수][습도 소수][거리(0~255cm)]
//     STATUS 0xA3 정상 / 0xA2 온습도 실패 / 0xA1 거리 실패 / 0xA0 둘 다 실패 / 그 외 통신 이상
//     센서 실패 : 해당 값 "--" 표시,  통신 이상 / NACK : 이전 값 유지 + FAIL
//   FND / led[7:0] : btnU, btnD 로 표시 상태 전환 (0:온도 1:습도 2:거리 3:3초마다 자동 전환)
//     온도 t00.0, 습도 h00.0, 거리 d000
//   OLED : 항상 모든 값 표시
//   UART : 1초마다 모든 값 전송 (9600bps, "TEMP:25.3C  HUMI:40.7%  DIST: 61cm\r\n")
//   led[7:0]: Slave에서 읽어 FND에 표시 중인 값
//   led[12]: I2C 동작 중, led[13]: Slave NACK, led[14]: FAIL, led[15]: PASS (나머지 OFF)
module top(
    input clk,
    input reset,
    input btnU,
    input btnD,
    output RsTx,       // UART tx 
    inout i2c_sda,     // JA1 (Slave basys3, OLED 공유)
    output i2c_scl,    // JA2
    output [7:0] seg,
    output [3:0] an,
    output [15:0] led
    );

    // 버튼 / UART / FND
    wire [1:0] w_clean_btn;     // 디바운스된 버튼 [0] btnU, [1] btnD
    wire w_tx_busy;
    wire w_tx_start;
    wire [7:0] w_tx_data;
    wire [7:0] w_led_value;     // FND에 표시 중인 값 -> led[7:0]

    // 센서 값 (sensor_reader)
    wire [7:0] w_temp_int;
    wire [3:0] w_temp_dec;
    wire [7:0] w_humi_int;
    wire [3:0] w_humi_dec;
    wire [7:0] w_dist;
    wire w_dht_ok;              // 0이면 온습도 "--" 표시
    wire w_dist_ok;             // 0이면 거리 "--" 표시
    wire w_link_pass;           // 마지막 읽기 정상
    wire w_link_fail;           // 마지막 읽기 실패 (서명 불일치 / NACK)
    wire w_slave_nack;          // Slave 무응답

    // OLED 화면
    wire [1:0] w_char_line;
    wire [3:0] w_char_pos;
    wire [7:0] w_char_code;
    wire [6:0] w_pixel_col;
    wire w_pixel_mode;
    wire [7:0] w_pixel_data;
    wire w_frame_start;         // 화면 전송 시작 : 표시 값 고정

    // i2c bus : sensor_reader(A) / oled_controller(B) -> i2c_arbiter -> i2c_master
    wire w_sr_bus_req, w_sr_bus_grant, w_sr_i2c_start, w_sr_read_ack, w_sr_i2c_done;
    wire [1:0] w_sr_i2c_cmd;
    wire [7:0] w_sr_i2c_tx_data;
    wire w_oled_bus_req, w_oled_bus_grant, w_oled_i2c_start, w_oled_i2c_done;
    wire [1:0] w_oled_i2c_cmd;
    wire [7:0] w_oled_i2c_tx_data;
    wire w_i2c_start;
    wire [1:0] w_i2c_cmd;
    wire [7:0] w_i2c_tx_data;
    wire w_i2c_read_ack;
    wire [7:0] w_i2c_rx_data;
    wire w_i2c_busy;
    wire w_i2c_done;
    wire w_i2c_ack_error;

    // 버튼 디바운스 (10ms) : [0] btnU, [1] btnD
    btn_debouncer u_btn_debouncer (
        .clk(clk),
        .reset(reset),
        .noisy_btn({btnD, btnU}),
        .clean_btn(w_clean_btn)
    );

    // Slave basys3 센서 값 읽기 : 200ms 마다 6 byte, STATUS 해석
    sensor_reader # (
        .CLK_FREQ(100_000_000),
        .READ_Hz(5),
        .SLAVE_ADDR(7'h42)          // Slave와 맞출 것
    ) u_sensor_reader (
        .clk(clk),
        .reset(reset),
        .bus_req(w_sr_bus_req),
        .bus_grant(w_sr_bus_grant),
        .i2c_start(w_sr_i2c_start),
        .i2c_cmd(w_sr_i2c_cmd),
        .i2c_tx_data(w_sr_i2c_tx_data),
        .read_ack(w_sr_read_ack),
        .i2c_done(w_sr_i2c_done),
        .i2c_ack_error(w_i2c_ack_error),
        .i2c_rx_data(w_i2c_rx_data),
        .temp_int(w_temp_int),
        .temp_dec(w_temp_dec),
        .humi_int(w_humi_int),
        .humi_dec(w_humi_dec),
        .dist(w_dist),
        .dht_ok(w_dht_ok),
        .dist_ok(w_dist_ok),
        .link_pass(w_link_pass),
        .link_fail(w_link_fail),
        .slave_nack(w_slave_nack)
    );

    // FND / led[7:0] 표시 : 버튼으로 온도 / 습도 / 거리 / 자동 전환(3초)
    fnd_sensor_display # (
        .CLK_FREQ(100_000_000),
        .SHOW_MS(3000)
    ) u_fnd_sensor_display (
        .clk(clk),
        .reset(reset),
        .btn_up(w_clean_btn[0]),
        .btn_down(w_clean_btn[1]),
        .temp_int(w_temp_int),
        .temp_dec(w_temp_dec),
        .humi_int(w_humi_int),
        .humi_dec(w_humi_dec),
        .dist(w_dist),
        .dht_ok(w_dht_ok),
        .dist_ok(w_dist_ok),
        .led_value(w_led_value),
        .an(an),
        .seg(seg)
    );

    // UART 문자열 생성 : 1초마다 모든 센서 값 한 줄
    uart_sensor_sender # (
        .CLK_FREQ(100_000_000),
        .SEND_Hz(1)
    ) u_uart_sensor_sender (
        .clk(clk),
        .reset(reset),
        .temp_int(w_temp_int),
        .temp_dec(w_temp_dec),
        .humi_int(w_humi_int),
        .humi_dec(w_humi_dec),
        .dist(w_dist),
        .dht_ok(w_dht_ok),
        .dist_ok(w_dist_ok),
        .tx_busy(w_tx_busy),
        .tx_start(w_tx_start),
        .tx_data(w_tx_data)
    );

    // UART 송신 : 1 byte 씩 9600bps
    uart_tx # (
        .BPS(9600)
    ) u_uart_tx (
        .clk(clk),
        .reset(reset),
        .tx_data(w_tx_data),
        .tx_start(w_tx_start),
        .tx(RsTx),
        .tx_busy(w_tx_busy)
    );

    // OLED 화면 구성 : 요청한 위치의 글자 / 막대그래프 pixel 제공
    oled_text u_oled_text (
        .clk(clk),
        .reset(reset),
        .frame_start(w_frame_start),
        .temp(w_temp_int),
        .temp_dec(w_temp_dec),
        .humi(w_humi_int),
        .humi_dec(w_humi_dec),
        .dist(w_dist),
        .dht_ok(w_dht_ok),
        .dist_ok(w_dist_ok),
        .char_line(w_char_line),
        .char_pos(w_char_pos),
        .pixel_col(w_pixel_col),
        .char_code(w_char_code),
        .pixel_mode(w_pixel_mode),
        .pixel_data(w_pixel_data)
    );

    // SSD1306 제어 : 초기화 후 100ms 마다 화면 512 byte 전송
    oled_controller # (
        .CLK_FREQ(100_000_000),
        .POWER_ON_WAIT_MS(100),
        .REFRESH_Hz(10),
        .OLED_ADDR(7'h3C)
    ) u_oled_controller (
        .clk(clk),
        .reset(reset),
        .char_line(w_char_line),
        .char_pos(w_char_pos),
        .char_code(w_char_code),
        .pixel_col(w_pixel_col),
        .pixel_mode(w_pixel_mode),
        .pixel_data(w_pixel_data),
        .frame_start(w_frame_start),
        .bus_req(w_oled_bus_req),
        .bus_grant(w_oled_bus_grant),
        .i2c_start(w_oled_i2c_start),
        .i2c_cmd(w_oled_i2c_cmd),
        .i2c_tx_data(w_oled_i2c_tx_data),
        .i2c_done(w_oled_i2c_done),
        .i2c_ack_error(w_i2c_ack_error)
    );

    // I2C bus 중재 : A(센서) / B(OLED) 중 grant 받은 쪽만 i2c_master 사용 (동시 요청시 A 우선)
    i2c_arbiter u_i2c_arbiter (
        .clk(clk),
        .reset(reset),
        .a_bus_req(w_sr_bus_req),
        .a_bus_grant(w_sr_bus_grant),
        .a_i2c_start(w_sr_i2c_start),
        .a_i2c_cmd(w_sr_i2c_cmd),
        .a_i2c_tx_data(w_sr_i2c_tx_data),
        .a_read_ack(w_sr_read_ack),
        .a_i2c_done(w_sr_i2c_done),
        .b_bus_req(w_oled_bus_req),
        .b_bus_grant(w_oled_bus_grant),
        .b_i2c_start(w_oled_i2c_start),
        .b_i2c_cmd(w_oled_i2c_cmd),
        .b_i2c_tx_data(w_oled_i2c_tx_data),
        .b_i2c_done(w_oled_i2c_done),
        .i2c_start(w_i2c_start),
        .i2c_cmd(w_i2c_cmd),
        .i2c_tx_data(w_i2c_tx_data),
        .read_ack(w_i2c_read_ack),
        .i2c_done(w_i2c_done)
    );

    // I2C master : START / WRITE / READ / STOP 실행, SDA / SCL open-drain 구동 (100KHz)
    i2c_master # (
        .CLK_FREQ(100_000_000),
        .I2C_FREQ(100_000)
    ) u_i2c_master (
        .clk(clk),
        .reset(reset),
        .i2c_start(w_i2c_start),
        .cmd(w_i2c_cmd),
        .tx_data(w_i2c_tx_data),
        .read_ack(w_i2c_read_ack),
        .rx_data(w_i2c_rx_data),
        .busy(w_i2c_busy),
        .done(w_i2c_done),
        .ack_error(w_i2c_ack_error),
        .sda(i2c_sda),
        .scl(i2c_scl)
    );

    // led[15] PASS, [14] FAIL, [13] NACK, [12] I2C 동작 중, [11:8] OFF, [7:0] Slave에서 읽은 값
    assign led = {w_link_pass, w_link_fail, w_slave_nack, w_i2c_busy, 4'b0, w_led_value};
endmodule
