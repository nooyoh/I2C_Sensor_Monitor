`timescale 1ns / 1ps

module data_sender (
    input       clk,
    input       reset,
    input       start_trigger,

    input [7:0] temperature,
    input [7:0] temperature_frac,
    input [7:0] humidity,
    input [7:0] humidity_frac,
    input [7:0] distance_cm,
    input       distance_error,
    input       dht_error,

    input       tx_busy,
    output reg       tx_start,
    output reg [7:0] tx_data
);

    localparam [3:0] FRAME_LEN = 4'd8;

    reg [63:0] frame;
    reg [3:0]  send_count;
    reg        sending;

    // UART 프레임
    // AA 55 + 상태 헤더 + 온도 정수 + 온도 소수
    //       + 습도 정수 + 습도 소수 + 거리
    //
    // 상태 헤더 [7:2] = 101000
    //           [1]   = 초음파 정상 여부
    //           [0]   = 온습도 정상 여부
    // 정상 1, 오류 0

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            frame      <= 0;
            send_count <= 0;
            sending    <= 0;
            tx_start   <= 0;
            tx_data    <= 0;
        end else begin
            tx_start <= 1'b0;

            if (!sending) begin
                if (start_trigger) begin
                    // 전송 중 센서값이 바뀌어도 한 프레임은 같은 값 유지
                    frame <= {
                        8'hAA,
                        8'h55,
                        6'b101000, ~distance_error, ~dht_error,
                        temperature,
                        temperature_frac,
                        humidity,
                        humidity_frac,
                        distance_cm
                    };

                    send_count <= 0;
                    sending    <= 1'b1;
                end
            end else if (!tx_busy && !tx_start) begin
                if (send_count == FRAME_LEN) begin
                    sending <= 1'b0;
                end else begin
                    tx_data    <= frame[63:56];
                    tx_start   <= 1'b1;
                    frame      <= {frame[55:0], 8'h00};
                    send_count <= send_count + 1'b1;
                end
            end
        end
    end

endmodule