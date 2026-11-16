`timescale 1ns / 1ps

// Slave basys3 센서 값 읽기 (I2C register read, READ_Hz 마다)
//   START + (SLAVE_ADDR<<1 | W) + 레지스터 주소 0x00     : 읽기 시작 위치(pointer)를 0번으로 설정
//   repeated START + (SLAVE_ADDR<<1 | R) + 6 byte READ (마지막 byte NACK) + STOP
//   수신 data (Slave 레지스터 0x00~0x05) : [STATUS][온도 정수][온도 소수][습도 정수][습도 소수][거리]
//     거리 : 1 byte, 0~255cm
//   STATUS : [7:4] 서명 4'hA (통신 확인), [3:2] 예약, [1] DIST_OK, [0] DHT_OK
//     0xA3 정상 / 0xA2 온습도 실패 / 0xA1 거리 실패 / 0xA0 둘 다 실패 / 그 외 통신 이상
//   - 서명 일치 : link_pass, OK인 센서 값만 갱신 (실패한 센서는 dht_ok/dist_ok = 0)
//   - 서명 불일치 또는 NACK : link_fail, 이전 값 유지
//   i2c_master 는 OLED와 공유하므로 bus_req -> bus_grant 받은 뒤 START
module sensor_reader # (
    parameter CLK_FREQ = 100_000_000,
    parameter READ_Hz = 5,                // 200ms 마다 읽기
    parameter [6:0] SLAVE_ADDR = 7'h42    // Slave basys3 I2C 주소 (Slave와 맞출 것)
) (
    input clk,
    input reset,
    // i2c bus 사용 요청
    output bus_req,
    input bus_grant,
    // i2c_master 연결
    output reg i2c_start,
    output reg [1:0] i2c_cmd,
    output reg [7:0] i2c_tx_data,
    output reg read_ack,
    input i2c_done,
    input i2c_ack_error,
    input [7:0] i2c_rx_data,
    // 센서 값
    output reg [7:0] temp_int,
    output reg [3:0] temp_dec,
    output reg [7:0] humi_int,
    output reg [3:0] humi_dec,
    output reg [7:0] dist,
    output reg dht_ok,
    output reg dist_ok,
    // 통신 상태
    output reg link_pass,     // 마지막 읽기 정상 (서명 일치)
    output reg link_fail,     // 마지막 읽기 실패 (서명 불일치 또는 NACK)
    output reg slave_nack     // 마지막 읽기에서 Slave NACK (배선 또는 주소 오류)
    );

    localparam CMD_START = 2'b00;
    localparam CMD_WRITE = 2'b01;
    localparam CMD_READ  = 2'b10;
    localparam CMD_STOP  = 2'b11;

    localparam S_IDLE    = 4'd0;
    localparam S_START   = 4'd1;   // START
    localparam S_ADDR_W  = 4'd2;   // Slave 주소 + W
    localparam S_POINTER = 4'd3;   // 레지스터 주소 0x00 (읽기 시작 위치)
    localparam S_RESTART = 4'd4;   // repeated START (STOP 없이 다시 START)
    localparam S_ADDR_R  = 4'd5;   // Slave 주소 + R
    localparam S_READ    = 4'd6;   // 6 byte 읽기
    localparam S_STOP    = 4'd7;   // STOP
    localparam S_UPDATE  = 4'd8;   // 받은 data 해석

    localparam [7:0] REG_STATUS = 8'h00;   // 읽기 시작 레지스터 (STATUS)
    localparam [2:0] RX_LAST = 3'd5;       // 6 byte 수신 (0~5)
    localparam [3:0] STATUS_SIG = 4'hA;

    wire w_read_tick;

    reg [3:0] r_state;
    reg r_req_sent;       // i2c_master에 명령을 보냈고 done을 기다리는 중
    reg r_error;          // 현재 transaction에서 NACK 발생
    reg [2:0] r_rx_cnt;
    reg [47:0] r_rx_buf;  // 수신 6 byte (먼저 받은 byte가 상위)

    wire [7:0] w_status   = r_rx_buf[47:40];
    wire [7:0] w_temp_int = r_rx_buf[39:32];
    wire [7:0] w_temp_dec = r_rx_buf[31:24];
    wire [7:0] w_humi_int = r_rx_buf[23:16];
    wire [7:0] w_humi_dec = r_rx_buf[15:8];
    wire [7:0] w_dist     = r_rx_buf[7:0];

    // START 부터 STOP 완료까지 bus 점유
    assign bus_req = (r_state >= S_START) && (r_state <= S_STOP);

    tick_generator # (
        .INPUT_FREQ(CLK_FREQ),
        .TICK_Hz(READ_Hz)
    ) u_read_tick (
        .clk(clk),
        .reset(reset),
        .tick(w_read_tick)
    );

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_state <= S_IDLE;
            r_req_sent <= 1'b0;
            r_error <= 1'b0;
            r_rx_cnt <= 0;
            r_rx_buf <= 0;
            i2c_start <= 1'b0;
            i2c_cmd <= CMD_START;
            i2c_tx_data <= 0;
            read_ack <= 1'b1;
            temp_int <= 0;
            temp_dec <= 0;
            humi_int <= 0;
            humi_dec <= 0;
            dist <= 0;
            dht_ok <= 1'b0;      // 첫 읽기 전에는 "--" 표시
            dist_ok <= 1'b0;
            link_pass <= 1'b0;
            link_fail <= 1'b0;
            slave_nack <= 1'b0;
        end else begin
            i2c_start <= 1'b0;
            case (r_state)
            S_IDLE: begin
                if (w_read_tick) begin
                    r_error <= 1'b0;
                    r_state <= S_START;
                end
            end
            // START / repeated START
            S_START, S_RESTART: begin
                if (!r_req_sent) begin
                    if (bus_grant) begin   // OLED 전송 중이면 끝날 때까지 대기
                        i2c_start <= 1'b1;
                        i2c_cmd <= CMD_START;
                        r_req_sent <= 1'b1;
                    end
                end else if (i2c_done) begin
                    r_req_sent <= 1'b0;
                    r_state <= (r_state == S_START) ? S_ADDR_W : S_ADDR_R;
                end
            end
            // 1 byte 쓰기 : 주소+W / 레지스터 주소 / 주소+R
            S_ADDR_W, S_POINTER, S_ADDR_R: begin
                if (!r_req_sent) begin
                    i2c_start <= 1'b1;
                    i2c_cmd <= CMD_WRITE;
                    case (r_state)
                        S_ADDR_W:  i2c_tx_data <= {SLAVE_ADDR, 1'b0};   // R/W = 0 (write)
                        S_POINTER: i2c_tx_data <= REG_STATUS;
                        default:   i2c_tx_data <= {SLAVE_ADDR, 1'b1};   // R/W = 1 (read)
                    endcase
                    r_req_sent <= 1'b1;
                end else if (i2c_done) begin
                    r_req_sent <= 1'b0;
                    if (i2c_ack_error) begin   // NACK : 읽기 중단
                        r_error <= 1'b1;
                        r_state <= S_STOP;
                    end else if (r_state == S_ADDR_W) begin
                        r_state <= S_POINTER;
                    end else if (r_state == S_POINTER) begin
                        r_state <= S_RESTART;
                    end else begin
                        r_rx_cnt <= 0;
                        r_state <= S_READ;
                    end
                end
            end
            S_READ: begin
                if (!r_req_sent) begin
                    i2c_start <= 1'b1;
                    i2c_cmd <= CMD_READ;
                    read_ack <= (r_rx_cnt == RX_LAST);   // 마지막 byte는 NACK (읽기 끝)
                    r_req_sent <= 1'b1;
                end else if (i2c_done) begin
                    r_req_sent <= 1'b0;
                    r_rx_buf <= {r_rx_buf[39:0], i2c_rx_data};
                    if (r_rx_cnt == RX_LAST)
                        r_state <= S_STOP;
                    else
                        r_rx_cnt <= r_rx_cnt + 1;
                end
            end
            S_STOP: begin
                if (!r_req_sent) begin
                    i2c_start <= 1'b1;
                    i2c_cmd <= CMD_STOP;
                    r_req_sent <= 1'b1;
                end else if (i2c_done) begin
                    r_req_sent <= 1'b0;
                    r_state <= S_UPDATE;
                end
            end
            S_UPDATE: begin
                if (r_error) begin                         // NACK : 이전 값 유지
                    slave_nack <= 1'b1;
                    link_pass <= 1'b0;
                    link_fail <= 1'b1;
                end else if (w_status[7:4] == STATUS_SIG) begin   // 정상 수신
                    slave_nack <= 1'b0;
                    link_pass <= 1'b1;
                    link_fail <= 1'b0;
                    dht_ok <= w_status[0];
                    dist_ok <= w_status[1];
                    if (w_status[0]) begin
                        temp_int <= w_temp_int;
                        temp_dec <= (w_temp_dec > 8'd9) ? 4'd9 : w_temp_dec[3:0];
                        humi_int <= w_humi_int;
                        humi_dec <= (w_humi_dec > 8'd9) ? 4'd9 : w_humi_dec[3:0];
                    end
                    if (w_status[1])
                        dist <= w_dist;
                end else begin                             // 서명 불일치 : 통신 이상, 이전 값 유지
                    slave_nack <= 1'b0;
                    link_pass <= 1'b0;
                    link_fail <= 1'b1;
                end
                r_state <= S_IDLE;
            end
            default: r_state <= S_IDLE;
            endcase
        end
    end
endmodule
