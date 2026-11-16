`timescale 1ns / 1ps

module dht11(
    input clk,
    input reset,
    inout dht11_data,

    output reg  [7:0] temperature,  // 온도 정수부
    output reg  [7:0] humidity,     // 습도 정수부
    output reg [7:0] temperature_frac, humidity_frac,
    output reg error,
    output reg        data_valid    // 새 측정값을 저장할 때 1클럭 HIGH
);

    // Basys3: 100 MHz이므로 1 us = 100클럭
    localparam CLKS_PER_US = 100;
    localparam WAIT_CLKS   = 2_000_000 * CLKS_PER_US; // 측정 사이 2초 대기
    localparam START_CLKS  = 20_000 * CLKS_PER_US; // 시작 신호 20 ms
    localparam TIMEOUT     = 200 * CLKS_PER_US; // 수신 단계별 제한
    localparam BIT_ONE_MIN = 50 * CLKS_PER_US;

    localparam [3:0]
        IDLE          = 4'd0,
        START_LOW     = 4'd1,
        RELEASE_LINE  = 4'd2,
        WAIT_RESPONSE = 4'd3,
        RESPONSE_LOW  = 4'd4,
        RESPONSE_HIGH = 4'd5,
        BIT_LOW       = 4'd6,
        BIT_HIGH      = 4'd7,
        CHECK_DATA    = 4'd8;

    reg [3:0]  state, next_state;
    reg [27:0] timer;       // 현재 상태에 머문 클럭 수
    reg [5:0]  bit_index;   // 0~39
    reg [39:0] frame;

    // 시작 신호를 보낼 때만 LOW. 나머지 시간에는 센서가 핀을 사용함.
    assign dht11_data = (state == START_LOW) ? 1'b0 : 1'bz;

    // 외부 센서 신호를 FPGA 클럭에 동기화
    reg data_meta, data_sync;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            data_meta <= 1'b1;
            data_sync <= 1'b1;
        end else begin
            data_meta <= dht11_data;
            data_sync <= data_meta;
        end
    end

    // 40비트 순서: 습도 정수, 습도 소수, 온도 정수, 온도 소수, 체크섬
    wire [7:0] checksum;
    assign checksum = frame[39:32] + frame[31:24]
                    + frame[23:16] + frame[15:8];

    always @(*) begin
        next_state = state;

        case (state)
            IDLE:
                if (timer == WAIT_CLKS - 1)
                    next_state = START_LOW;

            START_LOW:
                if (timer == START_CLKS - 1)
                    next_state = RELEASE_LINE;

            RELEASE_LINE:        // FPGA가 놓은 뒤 핀이 HIGH로 올라옴
                if (data_sync)
                    next_state = WAIT_RESPONSE;

            WAIT_RESPONSE:       // 센서가 LOW로 응답 시작
                if (!data_sync)
                    next_state = RESPONSE_LOW;

            RESPONSE_LOW:
                if (data_sync)
                    next_state = RESPONSE_HIGH;

            RESPONSE_HIGH:
                if (!data_sync)
                    next_state = BIT_LOW;

            BIT_LOW:             // 상승 에지부터 HIGH 길이 측정
                if (data_sync)
                    next_state = BIT_HIGH;

            BIT_HIGH:            // 하강 에지에서 한 비트 확정
                if (!data_sync)
                    next_state = (bit_index == 6'd39) ? CHECK_DATA : BIT_LOW;

            CHECK_DATA:
                next_state = IDLE;

            default:
                next_state = IDLE;
        endcase

        // 센서가 없거나 전송이 끊기면 대기 후 다시 측정
        if (state >= RELEASE_LINE && state <= BIT_HIGH && timer >= TIMEOUT)
            next_state = IDLE;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state       <= IDLE;
            timer       <= 0;
            bit_index   <= 0;
            frame       <= 0;
            temperature_frac <= 0;
            humidity_frac <= 0;
            error <= 1'b1;
            temperature <= 0;
            humidity    <= 0;
            data_valid  <= 0;
        end else begin
            state      <= next_state;
            data_valid <= 1'b0;

            if (state != next_state)
                timer <= 0;
            else
                timer <= timer + 1'b1;

            if (state == START_LOW) begin
                bit_index <= 0;
                frame     <= 0;
            end else if (state == BIT_HIGH && !data_sync) begin
                // DHT11은 상위 비트부터 전송
                frame     <= {frame[38:0], (timer >= BIT_ONE_MIN)};
                bit_index <= bit_index + 1'b1;
            end

            // 마지막 비트까지 frame에 저장된 다음 클럭에서 검사
            if (state >= RELEASE_LINE && state <= BIT_HIGH && next_state == IDLE)
                error <= 1'b1;
            if (state == CHECK_DATA)
                error <= (checksum != frame[7:0]);
            if (state == CHECK_DATA && checksum == frame[7:0]) begin
                humidity_frac <= frame[31:24];
                temperature_frac <= frame[15:8];
                humidity    <= frame[39:32];
                temperature <= frame[23:16];
                data_valid  <= 1'b1;
            end
        end
    end

endmodule
