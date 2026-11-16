`timescale 1ns / 1ps

// UART 센서 값 전송 : SEND_Hz 마다 모든 센서 값을 한 줄 문자열로 전송
//   "TEMP:25.3C  HUMI:40.7%  DIST: 61cm\r\n"  (36 byte, 9600bps 에서 약 37.5ms)
//   센서 실패(dht_ok / dist_ok = 0) : "TEMP:--.-C  HUMI:--.-%" / "DIST:---cm"
//   전송 시작 순간의 값을 문자열로 저장한 뒤 1 byte씩 uart_tx로 보냄 (13.uart 의 data_sender 구조)
module uart_sensor_sender # (
    parameter CLK_FREQ = 100_000_000,
    parameter SEND_Hz = 1
) (
    input clk,
    input reset,
    input [7:0] temp_int,
    input [3:0] temp_dec,
    input [7:0] humi_int,
    input [3:0] humi_dec,
    input [7:0] dist,         // 거리 (0~255 cm)
    input dht_ok,
    input dist_ok,
    input tx_busy,
    output reg tx_start,
    output reg [7:0] tx_data
    );

    localparam [5:0] MSG_LEN = 6'd36;   // 전송할 글자 수
    localparam [7:0] SPACE = 8'h20;     // ' '
    localparam [7:0] ZERO = 8'h30;      // '0'
    localparam [7:0] DASH = 8'h2D;      // '-'

    wire w_send_tick;
    wire [3:0] w_temp_d1, w_temp_d10;
    wire [3:0] w_humi_d1, w_humi_d10;
    wire [3:0] w_dist_d1, w_dist_d10, w_dist_d100;

    reg [3:0] r_temp_d1, r_temp_d10;               // 자리수 (나눗셈 결과 register)
    reg [3:0] r_humi_d1, r_humi_d10;
    reg [3:0] r_dist_d1, r_dist_d10, r_dist_d100;
    reg [7:0] r_temp_c10, r_temp_c1, r_temp_c01;   // 숫자 -> ASCII (register로 timing 여유 확보)
    reg [7:0] r_humi_c10, r_humi_c1, r_humi_c01;
    reg [7:0] r_dist_c100, r_dist_c10, r_dist_c1;
    reg [MSG_LEN*8-1:0] r_msg;   // 전송할 문자열 (왼쪽 글자가 상위 byte)
    reg [5:0] r_send_cnt;
    reg r_sending;               // 문자열 전송 중

    tick_generator # (
        .INPUT_FREQ(CLK_FREQ),
        .TICK_Hz(SEND_Hz)
    ) u_send_tick (
        .clk(clk),
        .reset(reset),
        .tick(w_send_tick)
    );

    bin2bcd4digit u_temp_bcd (
        .in_data({6'b0, temp_int}),
        .d1(w_temp_d1),
        .d10(w_temp_d10),
        .d100()
    );

    bin2bcd4digit u_humi_bcd (
        .in_data({6'b0, humi_int}),
        .d1(w_humi_d1),
        .d10(w_humi_d10),
        .d100()
    );

    bin2bcd4digit u_dist_bcd (
        .in_data({6'b0, dist}),
        .d1(w_dist_d1),
        .d10(w_dist_d10),
        .d100(w_dist_d100)
    );

    // 1단계 : 자리수 분리(/, %) 결과 저장
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_temp_d1 <= 0;
            r_temp_d10 <= 0;
            r_humi_d1 <= 0;
            r_humi_d10 <= 0;
            r_dist_d1 <= 0;
            r_dist_d10 <= 0;
            r_dist_d100 <= 0;
        end else begin
            r_temp_d1 <= w_temp_d1;
            r_temp_d10 <= w_temp_d10;
            r_humi_d1 <= w_humi_d1;
            r_humi_d10 <= w_humi_d10;
            r_dist_d1 <= w_dist_d1;
            r_dist_d10 <= w_dist_d10;
            r_dist_d100 <= w_dist_d100;
        end
    end

    // 2단계 : 숫자 -> ASCII, 앞자리 0은 공백 (오른쪽 정렬), 센서 실패시 '-'
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_temp_c10 <= SPACE;
            r_temp_c1 <= ZERO;
            r_temp_c01 <= ZERO;
            r_humi_c10 <= SPACE;
            r_humi_c1 <= ZERO;
            r_humi_c01 <= ZERO;
            r_dist_c100 <= SPACE;
            r_dist_c10 <= SPACE;
            r_dist_c1 <= ZERO;
        end else begin
            if (dht_ok) begin
                r_temp_c10 <= (r_temp_d10 == 0) ? SPACE : ZERO + r_temp_d10;
                r_temp_c1 <= ZERO + r_temp_d1;
                r_temp_c01 <= ZERO + temp_dec;
                r_humi_c10 <= (r_humi_d10 == 0) ? SPACE : ZERO + r_humi_d10;
                r_humi_c1 <= ZERO + r_humi_d1;
                r_humi_c01 <= ZERO + humi_dec;
            end else begin
                r_temp_c10 <= DASH;
                r_temp_c1 <= DASH;
                r_temp_c01 <= DASH;
                r_humi_c10 <= DASH;
                r_humi_c1 <= DASH;
                r_humi_c01 <= DASH;
            end
            if (dist_ok) begin
                r_dist_c100 <= (r_dist_d100 == 0) ? SPACE : ZERO + r_dist_d100;
                r_dist_c10 <= (r_dist_d100 == 0 && r_dist_d10 == 0) ? SPACE : ZERO + r_dist_d10;
                r_dist_c1 <= ZERO + r_dist_d1;
            end else begin
                r_dist_c100 <= DASH;
                r_dist_c10 <= DASH;
                r_dist_c1 <= DASH;
            end
        end
    end

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            tx_start <= 1'b0;
            tx_data <= 0;
            r_msg <= 0;
            r_send_cnt <= 0;
            r_sending <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            if (!r_sending) begin
                if (w_send_tick) begin
                    r_msg <= {"TEMP:", r_temp_c10, r_temp_c1, ".", r_temp_c01,
                              "C  HUMI:", r_humi_c10, r_humi_c1, ".", r_humi_c01,
                              "%  DIST:", r_dist_c100, r_dist_c10, r_dist_c1, "cm", 8'h0D, 8'h0A};
                    r_sending <= 1'b1;
                    r_send_cnt <= 0;
                end
            end else if (!tx_busy && !tx_start) begin
                if (r_send_cnt == MSG_LEN)
                    r_sending <= 1'b0;   // 36 byte 전송 완료
                else begin
                    tx_data <= r_msg[(MSG_LEN - 1 - r_send_cnt) * 8 +: 8];
                    tx_start <= 1'b1;
                    r_send_cnt <= r_send_cnt + 1;
                end
            end
        end
    end
endmodule
