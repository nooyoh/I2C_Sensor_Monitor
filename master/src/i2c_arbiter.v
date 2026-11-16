`timescale 1ns / 1ps

// i2c_master 1개를 sensor_reader(A)와 oled_controller(B)가 나눠 쓰기 위한 중재기
//   bus가 비어 있으면 요청한 쪽에 grant (동시 요청이면 A 우선)
//   grant 받은 쪽이 bus_req 를 내릴 때(STOP 완료)까지 계속 점유
//   grant 받은 쪽의 명령만 i2c_master로 전달하고, done은 grant 받은 쪽에만 전달
module i2c_arbiter (
    input clk,
    input reset,
    // A : sensor_reader
    input a_bus_req,
    output a_bus_grant,
    input a_i2c_start,
    input [1:0] a_i2c_cmd,
    input [7:0] a_i2c_tx_data,
    input a_read_ack,
    output a_i2c_done,
    // B : oled_controller
    input b_bus_req,
    output b_bus_grant,
    input b_i2c_start,
    input [1:0] b_i2c_cmd,
    input [7:0] b_i2c_tx_data,
    output b_i2c_done,
    // i2c_master 연결
    output i2c_start,
    output [1:0] i2c_cmd,
    output [7:0] i2c_tx_data,
    output read_ack,
    input i2c_done
    );

    localparam OWNER_A = 1'b0;
    localparam OWNER_B = 1'b1;

    reg r_owned;   // 1: 누군가 bus 점유 중
    reg r_owner;

    assign a_bus_grant = r_owned && (r_owner == OWNER_A);
    assign b_bus_grant = r_owned && (r_owner == OWNER_B);

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_owned <= 1'b0;
            r_owner <= OWNER_A;
        end else if (!r_owned) begin
            if (a_bus_req) begin
                r_owned <= 1'b1;
                r_owner <= OWNER_A;
            end else if (b_bus_req) begin
                r_owned <= 1'b1;
                r_owner <= OWNER_B;
            end
        end else if ((r_owner == OWNER_A && !a_bus_req) || (r_owner == OWNER_B && !b_bus_req)) begin
            r_owned <= 1'b0;   // 사용 끝 : 반납
        end
    end

    assign i2c_start   = a_bus_grant ? a_i2c_start   : b_bus_grant ? b_i2c_start : 1'b0;
    assign i2c_cmd     = a_bus_grant ? a_i2c_cmd     : b_i2c_cmd;
    assign i2c_tx_data = a_bus_grant ? a_i2c_tx_data : b_i2c_tx_data;
    assign read_ack    = a_bus_grant ? a_read_ack    : 1'b1;   // OLED는 READ 없음
    assign a_i2c_done  = a_bus_grant & i2c_done;
    assign b_i2c_done  = b_bus_grant & i2c_done;
endmodule
