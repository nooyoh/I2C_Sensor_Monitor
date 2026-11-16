`timescale 1ns / 1ps

// SSD1306 (0.91" 128x32) OLED controller
//   전원 안정 대기 -> 초기화 command 전송 -> GDDRAM(512 byte) 전송 -> REFRESH_Hz 마다 화면 다시 전송
//   I2C 1 transaction : START + slave addr(0x3C<<1 | W) + control byte + payload... + STOP
//     control byte 0x00 : 이후 byte는 모두 command
//     control byte 0x40 : 이후 byte는 모두 GDDRAM data
//   화면은 8x8 cell 글자 16자 x 4줄 (page 0~3). 글자는 char_line/char_pos로 요청하고 char_code로 받는다.
//   pixel_mode=1 인 줄은 글자 대신 pixel_data(column 1 byte)를 그대로 출력한다. (막대그래프 등)
module oled_controller # (
    parameter CLK_FREQ = 100_000_000,
    parameter POWER_ON_WAIT_MS = 100,   // VDD 안정 대기
    parameter REFRESH_Hz = 10,          // 화면 갱신 주기 (센서 값 표시용)
    parameter [6:0] OLED_ADDR = 7'h3C
) (
    input clk,
    input reset,
    // 표시할 글자 요청
    output [1:0] char_line,     // 0~3 줄 (page)
    output [3:0] char_pos,      // 0~15 번째 글자
    input [7:0] char_code,      // ASCII
    output [6:0] pixel_col,     // 0~127 column
    input pixel_mode,           // 1: 현재 줄은 pixel_data 출력
    input [7:0] pixel_data,     // bit0 = 위쪽 pixel
    output reg frame_start,     // 화면 data 전송 시작 (1 clk pulse) : 표시할 값 latch 용
    // i2c bus 사용 요청 (sensor_reader 와 공유)
    output bus_req,
    input bus_grant,
    // i2c_master 연결
    output reg i2c_start,
    output reg [1:0] i2c_cmd,
    output reg [7:0] i2c_tx_data,
    input i2c_done,
    input i2c_ack_error
    );

    localparam CMD_START = 2'b00;
    localparam CMD_WRITE = 2'b01;
    localparam CMD_STOP  = 2'b11;

    localparam S_POWER_WAIT   = 3'd0;
    localparam S_START        = 3'd1;
    localparam S_SLAVE_ADDR   = 3'd2;
    localparam S_CTRL_BYTE    = 3'd3;
    localparam S_PAYLOAD      = 3'd4;
    localparam S_STOP         = 3'd5;
    localparam S_REFRESH_WAIT = 3'd6;

    localparam POWER_WAIT_CNT = (CLK_FREQ / 1000) * POWER_ON_WAIT_MS;
    localparam [4:0] CMD_LAST = 5'd31;          // command table 마지막 index
    localparam [4:0] ADDR_CMD_START = 5'd26;    // 화면 갱신시에는 주소 설정 command만 다시 보냄
    localparam [8:0] DATA_LAST = 9'd511;        // 128 x 4 page - 1

    wire w_refresh_tick;
    wire [7:0] w_cmd_byte;
    wire [39:0] w_glyph;
    reg [7:0] r_font_byte;

    reg [2:0] r_state;
    reg [$clog2(POWER_WAIT_CNT)-1:0] r_power_cnt;
    reg r_req_sent;        // i2c_master에 명령을 보냈고 done을 기다리는 중
    reg r_is_data;         // 0: command transaction, 1: GDDRAM data transaction
    reg r_error;           // 현재 transaction에서 NACK 발생
    reg r_init_done;       // 초기화 + 첫 화면 전송 완료 (이후 갱신은 주소 command만)
    reg [4:0] r_cmd_idx;
    reg [8:0] r_data_cnt;  // [8:7] page, [6:0] column

    wire [7:0] w_gdram_byte = pixel_mode ? pixel_data : r_font_byte;
    wire [7:0] w_payload = r_is_data ? w_gdram_byte : w_cmd_byte;
    wire w_payload_last = r_is_data ? (r_data_cnt == DATA_LAST) : (r_cmd_idx == CMD_LAST);

    // START 부터 STOP 완료까지 bus 점유 (command -> data transaction 사이에도 유지)
    assign bus_req = (r_state >= S_START) && (r_state <= S_STOP);

    assign char_line = r_data_cnt[8:7];
    assign char_pos = r_data_cnt[6:3];
    assign pixel_col = r_data_cnt[6:0];

    tick_generator # (
        .INPUT_FREQ(CLK_FREQ),
        .TICK_Hz(REFRESH_Hz)
    ) u_refresh_tick (
        .clk(clk),
        .reset(reset),
        .tick(w_refresh_tick)
    );

    oled_init_table u_oled_init_table (
        .index(r_cmd_idx),
        .data(w_cmd_byte)
    );

    font_5x7 u_font_5x7 (
        .char_code(char_code),
        .glyph(w_glyph)
    );

    // 8x8 cell 중 왼쪽 5 column은 글자, 오른쪽 3 column은 글자 간격
    always @(*) begin
        case (r_data_cnt[2:0])
            3'd0: r_font_byte = w_glyph[39:32];
            3'd1: r_font_byte = w_glyph[31:24];
            3'd2: r_font_byte = w_glyph[23:16];
            3'd3: r_font_byte = w_glyph[15:8];
            3'd4: r_font_byte = w_glyph[7:0];
            default: r_font_byte = 8'h00;
        endcase
    end

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_state <= S_POWER_WAIT;
            r_power_cnt <= 0;
            r_req_sent <= 1'b0;
            r_is_data <= 1'b0;
            r_error <= 1'b0;
            r_cmd_idx <= 0;
            r_data_cnt <= 0;
            i2c_start <= 1'b0;
            i2c_cmd <= CMD_START;
            i2c_tx_data <= 0;
            r_init_done <= 1'b0;
            frame_start <= 1'b0;
        end else begin
            i2c_start <= 1'b0;
            frame_start <= 1'b0;
            case (r_state)
            S_POWER_WAIT: begin
                if (r_power_cnt == POWER_WAIT_CNT - 1) begin
                    r_is_data <= 1'b0;
                    r_cmd_idx <= 0;
                    r_state <= S_START;
                end else begin
                    r_power_cnt <= r_power_cnt + 1;
                end
            end
            S_START: begin
                if (!r_req_sent) begin
                    if (bus_grant) begin   // 센서 읽기 중이면 끝날 때까지 대기
                        i2c_start <= 1'b1;
                        i2c_cmd <= CMD_START;
                        r_req_sent <= 1'b1;
                    end
                end else if (i2c_done) begin
                    r_req_sent <= 1'b0;
                    r_state <= S_SLAVE_ADDR;
                end
            end
            S_SLAVE_ADDR, S_CTRL_BYTE, S_PAYLOAD: begin
                if (!r_req_sent) begin
                    i2c_start <= 1'b1;
                    i2c_cmd <= CMD_WRITE;
                    case (r_state)
                        S_SLAVE_ADDR: i2c_tx_data <= {OLED_ADDR, 1'b0};          // R/W = 0 (write)
                        S_CTRL_BYTE:  i2c_tx_data <= r_is_data ? 8'h40 : 8'h00;
                        default:      i2c_tx_data <= w_payload;
                    endcase
                    r_req_sent <= 1'b1;
                end else if (i2c_done) begin
                    r_req_sent <= 1'b0;
                    if (i2c_ack_error) begin   // NACK : transaction 중단
                        r_error <= 1'b1;
                        r_state <= S_STOP;
                    end else if (r_state == S_SLAVE_ADDR) begin
                        r_state <= S_CTRL_BYTE;
                    end else if (r_state == S_CTRL_BYTE) begin
                        r_state <= S_PAYLOAD;
                    end else if (w_payload_last) begin
                        r_state <= S_STOP;
                    end else if (r_is_data) begin
                        r_data_cnt <= r_data_cnt + 1;
                    end else begin
                        r_cmd_idx <= r_cmd_idx + 1;
                    end
                end
            end
            S_STOP: begin
                if (!r_req_sent) begin
                    i2c_start <= 1'b1;
                    i2c_cmd <= CMD_STOP;
                    r_req_sent <= 1'b1;
                end else if (i2c_done) begin
                    r_req_sent <= 1'b0;
                    if (r_error) begin          // 다음 갱신 주기에 초기화부터 다시 시도
                        r_error <= 1'b0;
                        r_init_done <= 1'b0;
                        r_state <= S_REFRESH_WAIT;
                    end else if (!r_is_data) begin   // command 전송 끝 -> 화면 data 전송
                        r_is_data <= 1'b1;
                        r_data_cnt <= 0;
                        frame_start <= 1'b1;
                        r_state <= S_START;
                    end else begin                   // 화면 전송 끝
                        r_init_done <= 1'b1;
                        r_state <= S_REFRESH_WAIT;
                    end
                end
            end
            S_REFRESH_WAIT: begin
                if (w_refresh_tick) begin
                    r_is_data <= 1'b0;
                    r_cmd_idx <= r_init_done ? ADDR_CMD_START : 5'd0;
                    r_state <= S_START;
                end
            end
            default: r_state <= S_POWER_WAIT;
            endcase
        end
    end
endmodule

// SSD1306 128x32 초기화 command (datasheet 10. Command Descriptions)
module oled_init_table (
    input [4:0] index,
    output reg [7:0] data
);
    always @(*) begin
        case (index)
        5'd0:  data = 8'hAE;   // display OFF
        5'd1:  data = 8'hD5;   // display clock divide ratio / oscillator freq
        5'd2:  data = 8'h80;
        5'd3:  data = 8'hA8;   // multiplex ratio
        5'd4:  data = 8'h1F;   //   32 MUX (128x32)
        5'd5:  data = 8'hD3;   // display offset
        5'd6:  data = 8'h00;
        5'd7:  data = 8'h40;   // display start line = 0
        5'd8:  data = 8'h8D;   // charge pump
        5'd9:  data = 8'h14;   //   enable (내부 DC-DC)
        5'd10: data = 8'h20;   // memory addressing mode
        5'd11: data = 8'h00;   //   horizontal addressing
        5'd12: data = 8'hA1;   // segment re-map (column 127 -> SEG0)
        5'd13: data = 8'hC8;   // COM output scan direction : remapped
        5'd14: data = 8'hDA;   // COM pins hardware config
        5'd15: data = 8'h02;   //   sequential (128x32)
        5'd16: data = 8'h81;   // contrast
        5'd17: data = 8'h8F;
        5'd18: data = 8'hD9;   // pre-charge period
        5'd19: data = 8'hF1;
        5'd20: data = 8'hDB;   // VCOMH deselect level
        5'd21: data = 8'h40;
        5'd22: data = 8'hA4;   // display = GDDRAM 내용 (A5: 전체 픽셀 ON 테스트용)
        5'd23: data = 8'hA6;   // normal (1: 픽셀 ON)
        5'd24: data = 8'h2E;   // scroll 비활성화
        5'd25: data = 8'hAF;   // display ON
        // ---- 여기부터 화면 갱신시마다 보내는 주소 설정 (ADDR_CMD_START = 26) ----
        5'd26: data = 8'h21;   // column address
        5'd27: data = 8'h00;   //   start 0
        5'd28: data = 8'h7F;   //   end 127
        5'd29: data = 8'h22;   // page address
        5'd30: data = 8'h00;   //   start 0
        5'd31: data = 8'h03;   //   end 3
        default: data = 8'hE3; // NOP
        endcase
    end
endmodule
