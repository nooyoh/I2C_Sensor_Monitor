`timescale 1ns / 1ps

// FND / led[7:0] 센서 값 표시 : btnU / btnD 로 표시 상태 전환
//   S_TEMP(0) : 온도만 표시    t00.0
//   S_HUMI(1) : 습도만 표시    h00.0
//   S_DIST(2) : 거리만 표시    d000
//   S_AUTO(3) : SHOW_MS 마다 온도 -> 습도 -> 거리 순서로 자동 전환
//   btn_up : 다음 상태 (3 -> 0), btn_down : 이전 상태 (0 -> 3)
//   led_value : 현재 표시 중인 항목의 정수 값
//   dht_ok / dist_ok = 0 (센서 실패) : t--.- / h--.- / d--- 표시, led_value = 0
module fnd_sensor_display # (
    parameter CLK_FREQ = 100_000_000,
    parameter SHOW_MS = 3000          // S_AUTO 에서 한 항목 표시 시간
) (
    input clk,
    input reset,
    input btn_up,             // 디바운스된 level 신호
    input btn_down,
    input [7:0] temp_int,     // 온도 정수 (0~99)
    input [3:0] temp_dec,     // 온도 소수 첫째 자리 (0~9)
    input [7:0] humi_int,     // 습도 정수 (0~99)
    input [3:0] humi_dec,     // 습도 소수 첫째 자리 (0~9)
    input [7:0] dist,         // 거리 (0~255 cm)
    input dht_ok,             // 1: 온습도 값 정상
    input dist_ok,            // 1: 거리 값 정상
    output reg [7:0] led_value,
    output [3:0] an,
    output [7:0] seg
    );

    localparam S_TEMP = 2'd0;
    localparam S_AUTO = 2'd3;   // 0~2 는 표시 항목(ITEM_*)과 같은 값

    localparam ITEM_TEMP = 2'd0;   // 지금 FND에 표시 중인 항목
    localparam ITEM_HUMI = 2'd1;
    localparam ITEM_DIST = 2'd2;

    localparam [3:0] CHAR_T = 4'hA;   // fnd_controller 문자 코드
    localparam [3:0] CHAR_H = 4'hB;
    localparam [3:0] CHAR_D = 4'hC;
    localparam [3:0] CHAR_DASH = 4'hD;   // '-'

    wire w_tick_1ms;
    wire [3:0] w_temp_d1, w_temp_d10;
    wire [3:0] w_humi_d1, w_humi_d10;
    wire [3:0] w_dist_d1, w_dist_d10, w_dist_d100;

    reg [1:0] r_disp_state;   // 표시 상태 (S_TEMP ~ S_AUTO)
    reg r_prev_up, r_prev_down;
    reg [1:0] r_item;
    reg [$clog2(SHOW_MS)-1:0] r_ms_cnt;
    reg [15:0] r_temp_bcd, r_humi_bcd, r_dist_bcd;   // 항목별 표시 data (register로 timing 여유 확보)
    reg [15:0] r_bcd;
    reg [3:0] r_dp_on;

    wire w_up_press = btn_up & ~r_prev_up;        // 버튼을 누른 순간 (rising edge)
    wire w_down_press = btn_down & ~r_prev_down;

    tick_generator # (
        .INPUT_FREQ(CLK_FREQ),
        .TICK_Hz(1000)
    ) u_tick_1ms (
        .clk(clk),
        .reset(reset),
        .tick(w_tick_1ms)
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

    // 표시 상태 전환 : btnU 다음, btnD 이전
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_disp_state <= S_TEMP;
            r_prev_up <= 1'b0;
            r_prev_down <= 1'b0;
        end else begin
            r_prev_up <= btn_up;
            r_prev_down <= btn_down;
            if (w_up_press)
                r_disp_state <= r_disp_state + 1;   // 3 다음은 0 (2bit overflow)
            else if (w_down_press)
                r_disp_state <= r_disp_state - 1;   // 0 이전은 3
        end
    end

    // 표시 항목 : 고정 상태는 해당 항목, S_AUTO는 SHOW_MS 마다 전환
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_item <= ITEM_TEMP;
            r_ms_cnt <= 0;
        end else if (r_disp_state != S_AUTO) begin
            r_item <= r_disp_state;
            r_ms_cnt <= 0;
        end else if (w_tick_1ms) begin
            if (r_ms_cnt == SHOW_MS - 1) begin
                r_ms_cnt <= 0;
                r_item <= (r_item == ITEM_DIST) ? ITEM_TEMP : r_item + 1;
            end else begin
                r_ms_cnt <= r_ms_cnt + 1;
            end
        end
    end

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_temp_bcd <= 0;
            r_humi_bcd <= 0;
            r_dist_bcd <= 0;
            r_bcd <= 0;
            r_dp_on <= 0;
            led_value <= 0;
        end else begin
            r_temp_bcd <= dht_ok ? {CHAR_T, w_temp_d10, w_temp_d1, temp_dec} : {CHAR_T, CHAR_DASH, CHAR_DASH, CHAR_DASH};
            r_humi_bcd <= dht_ok ? {CHAR_H, w_humi_d10, w_humi_d1, humi_dec} : {CHAR_H, CHAR_DASH, CHAR_DASH, CHAR_DASH};
            r_dist_bcd <= dist_ok ? {CHAR_D, w_dist_d100, w_dist_d10, w_dist_d1} : {CHAR_D, CHAR_DASH, CHAR_DASH, CHAR_DASH};
            case (r_item)
                ITEM_TEMP: begin   // t00.0
                    r_bcd <= r_temp_bcd;
                    r_dp_on <= 4'b0010;
                    led_value <= dht_ok ? temp_int : 8'd0;
                end
                ITEM_HUMI: begin   // h00.0
                    r_bcd <= r_humi_bcd;
                    r_dp_on <= 4'b0010;
                    led_value <= dht_ok ? humi_int : 8'd0;
                end
                default: begin     // d000
                    r_bcd <= r_dist_bcd;
                    r_dp_on <= 4'b0000;
                    led_value <= dist_ok ? dist : 8'd0;
                end
            endcase
        end
    end

    fnd_controller u_fnd_controller (
        .clk(clk),
        .reset(reset),
        .tick_1ms(w_tick_1ms),
        .bcd(r_bcd),
        .dp_on(r_dp_on),
        .an(an),
        .seg(seg)
    );
endmodule
