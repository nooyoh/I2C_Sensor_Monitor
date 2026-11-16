`timescale 1ns / 1ps

// OLED 화면 구성 (16자 x 4줄)
//   line 0 : "T:25.3°C H:40.7%"    온도 / 습도 (소수 첫째 자리)
//   line 1 : "DIST:  123 cm   "    거리
//   line 2 : 거리 막대그래프 (0~50cm -> 0~128 pixel, pixel_mode, 50cm 이상은 가득 참)
//   line 3 : "0      25     50"    막대 눈금
//   센서 실패(dht_ok / dist_ok = 0) : "T:--.-°C H:--.-%" / "DIST: --- cm", 막대 비움
//   센서 값은 frame_start 에서 latch 하여 한 화면 안에서 숫자와 막대가 어긋나지 않게 한다.
//   숫자 변환(/, %) 결과는 register에 저장 : 변환 -> font -> I2C 까지 한 clk에 지나가면 timing 위반
module oled_text (
    input clk,
    input reset,
    input frame_start,
    input [7:0] temp,           // 온도 정수 (0~99 'C)
    input [3:0] temp_dec,       // 온도 소수 첫째 자리 (0~9)
    input [7:0] humi,           // 습도 정수 (0~99 %)
    input [3:0] humi_dec,       // 습도 소수 첫째 자리 (0~9)
    input [7:0] dist,           // 거리 (0~255 cm)
    input dht_ok,               // 1: 온습도 값 정상
    input dist_ok,              // 1: 거리 값 정상
    input [1:0] char_line,
    input [3:0] char_pos,
    input [6:0] pixel_col,
    output [7:0] char_code,
    output pixel_mode,
    output [7:0] pixel_data
    );

    localparam [7:0] BAR_DIST_MAX = 8'd50;    // 막대그래프 최대 거리
    localparam [7:0] DEGREE = 8'h80;    // font_5x7 의 도(°) 기호
    localparam [7:0] SPACE = 8'h20;     // ' '
    localparam [7:0] ZERO = 8'h30;      // '0'
    localparam [7:0] DASH = 8'h2D;      // '-'

    reg r_dht_ok;
    reg r_dist_ok;
    reg [7:0] r_temp;
    reg [3:0] r_temp_dec;
    reg [7:0] r_humi;
    reg [3:0] r_humi_dec;
    reg [7:0] r_dist;
    reg [127:0] r_line;
    reg [7:0] r_temp_c10, r_temp_c1, r_temp_c01;   // 화면에 찍을 ASCII
    reg [7:0] r_humi_c10, r_humi_c1, r_humi_c01;
    reg [7:0] r_dist_c100, r_dist_c10, r_dist_c1;
    reg [7:0] r_bar_len;                           // 0~128 pixel
    reg [3:0] r_temp_d1, r_temp_d10;               // 자리수 (나눗셈 결과 register)
    reg [3:0] r_humi_d1, r_humi_d10;
    reg [3:0] r_dist_d1, r_dist_d10, r_dist_d100;

    wire [3:0] w_temp_d1, w_temp_d10;
    wire [3:0] w_humi_d1, w_humi_d10;
    wire [3:0] w_dist_d1, w_dist_d10, w_dist_d100;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_dht_ok <= 1'b0;
            r_dist_ok <= 1'b0;
            r_temp <= 0;
            r_temp_dec <= 0;
            r_humi <= 0;
            r_humi_dec <= 0;
            r_dist <= 0;
        end else if (frame_start) begin
            r_dht_ok <= dht_ok;
            r_dist_ok <= dist_ok;
            r_temp <= temp;
            r_temp_dec <= temp_dec;
            r_humi <= humi;
            r_humi_dec <= humi_dec;
            r_dist <= dist;
        end
    end

    bin2bcd4digit u_temp_bcd (
        .in_data({6'b0, r_temp}),
        .d1(w_temp_d1),
        .d10(w_temp_d10),
        .d100()
    );

    bin2bcd4digit u_humi_bcd (
        .in_data({6'b0, r_humi}),
        .d1(w_humi_d1),
        .d10(w_humi_d10),
        .d100()
    );

    bin2bcd4digit u_dist_bcd (
        .in_data({6'b0, r_dist}),
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
    // 막대그래프 길이 : 거리 x 128 / 50 = 거리 x 64 / 25 (50cm 이상은 128, 거리 실패시 0)
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
            r_bar_len <= 0;
        end else begin
            if (r_dht_ok) begin
                r_temp_c10 <= (r_temp_d10 == 0) ? SPACE : ZERO + r_temp_d10;
                r_temp_c1 <= ZERO + r_temp_d1;
                r_temp_c01 <= ZERO + r_temp_dec;
                r_humi_c10 <= (r_humi_d10 == 0) ? SPACE : ZERO + r_humi_d10;
                r_humi_c1 <= ZERO + r_humi_d1;
                r_humi_c01 <= ZERO + r_humi_dec;
            end else begin
                r_temp_c10 <= DASH;
                r_temp_c1 <= DASH;
                r_temp_c01 <= DASH;
                r_humi_c10 <= DASH;
                r_humi_c1 <= DASH;
                r_humi_c01 <= DASH;
            end
            if (r_dist_ok) begin
                r_dist_c100 <= (r_dist_d100 == 0) ? SPACE : ZERO + r_dist_d100;
                r_dist_c10 <= (r_dist_d100 == 0 && r_dist_d10 == 0) ? SPACE : ZERO + r_dist_d10;
                r_dist_c1 <= ZERO + r_dist_d1;
                r_bar_len <= (r_dist >= BAR_DIST_MAX) ? 8'd128 : ({4'b0, r_dist} * 12'd64) / 12'd25;
            end else begin
                r_dist_c100 <= DASH;
                r_dist_c10 <= DASH;
                r_dist_c1 <= DASH;
                r_bar_len <= 0;
            end
        end
    end

    always @(*) begin
        case (char_line)
            2'd0: r_line = {"T:", r_temp_c10, r_temp_c1, ".", r_temp_c01, DEGREE, "C H:",
                            r_humi_c10, r_humi_c1, ".", r_humi_c01, "%"};
            2'd1: r_line = {"DIST: ", r_dist_c100, r_dist_c10, r_dist_c1, " cm    "};
            2'd2: r_line = "                ";   // pixel_mode (막대그래프)
            2'd3: r_line = "0      25     50";
            default: r_line = "                ";
        endcase
    end

    // 문자열은 왼쪽 글자가 상위 byte
    assign char_code = r_line[(15 - char_pos) * 8 +: 8];

    // 막대그래프 : 채운 부분 / 양 끝 = 세로 6 pixel(bit1~6), 빈 부분 = 위/아래 테두리(bit1, bit6)만
    assign pixel_mode = (char_line == 2'd2);
    assign pixel_data = ((pixel_col < r_bar_len) || (pixel_col == 0) || (pixel_col == 127)) ? 8'h7E : 8'h42;
endmodule
