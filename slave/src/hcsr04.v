`timescale 1ns / 1ps

module hcsr04 (
    input clk,
    input reset,
    input echo,         // 센서 → FPGA
    output trig,         // FPGA → 센서
    output reg  [7:0] distance_cm,  // 마지막 정상 거리 (2~255 cm)
    output reg        data_valid,   // 새 거리 저장 시 1클럭 HIGH
    output reg        error         // 측정 실패 시 1, 다음 성공 시 0
);

    localparam IDLE      = 2'd0,
               TRIGGER   = 2'd1,
               WAIT_ECHO = 2'd2,
               MEASURE   = 2'd3;

    reg [1:0]  state;
    reg [6:0]  divider;
    reg [16:0] count_us;
    reg        echo_meta, echo_sync;

    assign trig = (state == TRIGGER);

    // 외부 ECHO 신호를 FPGA 클럭에 동기화
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            echo_meta <= 0;
            echo_sync <= 0;
        end else begin
            echo_meta <= echo;
            echo_sync <= echo_meta;
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            divider     <= 0;
            state       <= IDLE;
            count_us    <= 0;
            distance_cm <= 0;
            data_valid  <= 0;
            error       <= 1;
        end else begin
            data_valid <= 0;

            // 100 MHz에서 100클럭 = 1 us
            if (divider == 99) begin
                divider  <= 0;
                count_us <= count_us + 1'b1;

                case (state)
                    IDLE: if (count_us == 99_999) begin
                        // 측정 사이 100 ms 대기
                        count_us <= 0;
                        if (echo_sync)
                            error <= 1;       // 시작 전부터 ECHO가 HIGH
                        else
                            state <= TRIGGER;
                    end

                    TRIGGER: if (count_us == 9) begin
                        // TRIG를 10 us 유지한 뒤 내림
                        count_us <= 0;
                        state    <= WAIT_ECHO;
                    end

                    WAIT_ECHO: if (echo_sync) begin
                        count_us <= 1;        // 첫 HIGH 샘플
                        state    <= MEASURE;
                    end else if (count_us == 29_999) begin
                        error    <= 1;        // 30 ms 동안 응답 없음
                        count_us <= 0;
                        state    <= IDLE;
                    end

                    MEASURE: if (!echo_sync) begin
                        // ECHO가 LOW로 내려온 순간 펄스 길이 확정
                        if (count_us >= 116 && count_us <= (255 * 58)) begin
                            distance_cm <= (count_us + 29) / 58; // cm로 반올림
                            data_valid  <= 1;
                            error       <= 0;
                        end else begin
                            error <= 1;       // 측정 범위 밖
                        end
                        count_us <= 0;
                        state    <= IDLE;
                    end else if (count_us == 30_000) begin
                        error    <= 1;        // ECHO가 계속 HIGH
                        count_us <= 0;
                        state    <= IDLE;
                    end
                endcase
            end else begin
                divider <= divider + 1'b1;
            end
        end
    end
endmodule
