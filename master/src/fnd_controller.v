`timescale 1ns / 1ps

// FND 컨트롤러 : tick_1ms 마다 한 자리씩 순서대로 켜는 멀티플렉싱 (4자리 한 바퀴 4ms)
//   (PROJECT.my_microwave 의 fnd_controller 에 문자 t / h / d / - 추가, 미사용 pattern 기능 제거)
// bcd        : {d3, d2, d1, d0} 각 4bit BCD (d3 가 가장 왼쪽, an[3])
//              0~9 : 숫자, A : 't', B : 'h', C : 'd', D : '-', 그 외 : 꺼짐
// dp_on      : 자리별 소수점 점등 (1 = 점등), 인덱스는 자리 번호와 동일
// an, seg 는 active low, seg = {dp, g, f, e, d, c, b, a}
module fnd_controller(
    input clk,
    input reset,
    input tick_1ms,
    input [15:0] bcd,
    input [3:0] dp_on,
    output reg [3:0] an,
    output reg [7:0] seg
    );

    reg [1:0] r_sel = 0;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_sel <= 0;
        end else if (tick_1ms) begin
            r_sel <= r_sel + 1;
        end
    end

    reg [3:0] bcd_data;
    reg [6:0] seg_bcd;

    always @(*) begin
        case (r_sel)
            2'b00: begin bcd_data = bcd[3:0];   an = 4'b1110; end
            2'b01: begin bcd_data = bcd[7:4];   an = 4'b1101; end
            2'b10: begin bcd_data = bcd[11:8];  an = 4'b1011; end
            2'b11: begin bcd_data = bcd[15:12]; an = 4'b0111; end
            default: begin bcd_data = 4'b0000;  an = 4'b1111; end
        endcase
    end

    always @(*) begin
        case (bcd_data)
            4'd0: seg_bcd = 7'b1000000;   // 0
            4'd1: seg_bcd = 7'b1111001;   // 1
            4'd2: seg_bcd = 7'b0100100;   // 2
            4'd3: seg_bcd = 7'b0110000;   // 3
            4'd4: seg_bcd = 7'b0011001;   // 4
            4'd5: seg_bcd = 7'b0010010;   // 5
            4'd6: seg_bcd = 7'b0000010;   // 6
            4'd7: seg_bcd = 7'b1111000;   // 7
            4'd8: seg_bcd = 7'b0000000;   // 8
            4'd9: seg_bcd = 7'b0010000;   // 9
            4'hA: seg_bcd = 7'b0000111;   // t (d, e, f, g)
            4'hB: seg_bcd = 7'b0001011;   // h (c, e, f, g)
            4'hC: seg_bcd = 7'b0100001;   // d (b, c, d, e, g)
            4'hD: seg_bcd = 7'b0111111;   // - (g)
            default: seg_bcd = 7'b1111111;  // all off
        endcase
    end

    always @(*) begin
        seg = {~dp_on[r_sel], seg_bcd};
    end
endmodule

// in_data: 14bit fnd에 9999까지 표현 하기 위한 bin size
// 백/십/일 자리숫자 0~9 BCD 4bit표현 (센서 값은 최대 3자리라 천의 자리 출력 제거)
//   (13.uart 의 fnd_controller.v 에서 가져옴)
module bin2bcd4digit(
    input [13:0] in_data,
    output [3:0] d1,
    output [3:0] d10,
    output [3:0] d100
);

    assign d1 = in_data % 10;
    assign d10 = (in_data / 10) % 10;
    assign d100 = (in_data / 100) % 10;
endmodule
