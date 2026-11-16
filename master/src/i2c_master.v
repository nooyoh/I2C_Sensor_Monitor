`timescale 1ns / 1ps

// I2C master : 명령 1개(START / WRITE / READ / STOP)씩 받아서 SDA, SCL 핀으로 실행
//   사용법 : cmd, tx_data 를 정하고 i2c_start 를 1 clk 주면 실행 -> 끝나면 done 이 1 clk 나옴
//     START : 통신 시작 신호
//     WRITE : tx_data 1 byte 보내고 slave 응답(ACK) 확인 -> 응답 없으면 ack_error = 1
//     READ  : slave 에게 1 byte 받아 rx_data 로 출력, 받은 뒤 read_ack 를 응답으로 보냄
//     STOP  : 통신 끝 신호
//   SCL 1주기(10us)를 4 phase(2.5us)로 나누어 동작
//     phase 0 : SCL LOW  -> SDA 값 바꿈
//     phase 1 : SCL HIGH 로 올림
//     phase 2 : SCL HIGH 중앙 -> SDA 값 읽음
//     phase 3 : SCL LOW  로 내림
//   SDA / SCL 은 open-drain : 0 만 직접 출력, 1 은 선을 놓아서(1'bz) pull-up 저항이 HIGH 로 만듦
module i2c_master # (
    parameter CLK_FREQ = 100_000_000,
    parameter I2C_FREQ = 100_000      // SCL 속도 100KHz
) (
    input clk,
    input reset,
    input i2c_start,            // 명령 시작 (1 clk pulse)
    input [1:0] cmd,            // 00:START 01:WRITE 10:READ 11:STOP
    input [7:0] tx_data,        // WRITE 로 보낼 1 byte
    input read_ack,             // READ 후 master가 보낼 응답 (0:ACK 더 읽음, 1:NACK 마지막 byte)
    output reg [7:0] rx_data,   // READ 로 받은 1 byte
    output reg busy,            // 명령 실행 중 (led[12] 동작중 표시용)
    output reg done,            // 명령 완료 (1 clk pulse)
    output reg ack_error,       // WRITE 후 slave가 NACK 응답 (배선 또는 주소 오류)
    inout sda,                  // I2C data 
    output scl                  // I2C clock
    );

    // 명령 종류
    localparam CMD_START = 2'b00;
    localparam CMD_WRITE = 2'b01;
    localparam CMD_READ  = 2'b10;
    localparam CMD_STOP  = 2'b11;

    // FSM 상태
    localparam S_IDLE  = 3'd0;   // 명령 대기
    localparam S_START = 3'd1;   // START 신호
    localparam S_DATA  = 3'd2;   // 8 bit 보내기 / 받기
    localparam S_ACK   = 3'd3;   // 9번째 bit : ACK 확인 / 보내기
    localparam S_STOP  = 3'd4;   // STOP 신호

    wire w_phase_tick;   // SCL 1/4 주기(2.5us)마다 1 tick : 이 tick 마다 phase 1칸 진행

    reg [2:0] r_state;
    reg [1:0] r_phase;      // 현재 phase 0~3
    reg [2:0] r_bit_cnt;    // 현재 bit 위치 7 -> 0 (MSB 부터 보냄)
    reg [7:0] r_data_reg;   // 보내는 중 / 받는 중인 1 byte
    reg r_is_read;          // 1: READ 명령, 0: WRITE 명령
    reg r_read_ack;         // READ 후 보낼 응답 (명령 시작 때 저장)
    reg r_sda_out;          // SDA 출력 : 1 = 선 놓음(HIGH), 0 = LOW
    reg r_scl_out;          // SCL 출력 : 1 = 선 놓음(HIGH), 0 = LOW
    reg r_sda_sync1, r_sda_sync2;   // 외부 sda 입력 2단 동기화 (metastability 방지)

    // open-drain 출력 : 1 이면 high-impedance, 0 이면 LOW
    assign sda = r_sda_out ? 1'bz : 1'b0;
    assign scl = r_scl_out ? 1'bz : 1'b0;

    tick_generator # (
        .INPUT_FREQ(CLK_FREQ),
        .TICK_Hz(I2C_FREQ * 4)   // 100KHz x 4 phase = 400KHz (2.5us)
    ) u_phase_tick (
        .clk(clk),
        .reset(reset),
        .tick(w_phase_tick)
    );

    // sda 입력 동기화 : slave가 보낸 값은 clk와 무관하게 바뀌므로 2번 거쳐서 읽음
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_sda_sync1 <= 1'b1;
            r_sda_sync2 <= 1'b1;
        end else begin
            r_sda_sync1 <= sda;
            r_sda_sync2 <= r_sda_sync1;
        end
    end

    // I2C FSM
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_state <= S_IDLE;
            r_phase <= 0;
            r_bit_cnt <= 0;
            r_data_reg <= 0;
            r_is_read <= 1'b0;
            r_read_ack <= 1'b1;
            r_sda_out <= 1'b1;   // bus IDLE : SDA, SCL 모두 HIGH
            r_scl_out <= 1'b1;
            rx_data <= 0;
            busy <= 1'b0;
            done <= 1'b0;
            ack_error <= 1'b0;
        end else begin
            done <= 1'b0;   // done 은 1 clk 만 1
            case (r_state)
            // 명령 대기 : i2c_start 가 오면 명령 종류에 따라 상태 이동
            S_IDLE: begin
                if (i2c_start) begin
                    busy <= 1'b1;
                    r_phase <= 0;
                    r_bit_cnt <= 3'd7;          // MSB 부터
                    r_data_reg <= tx_data;      // 보낼 data 저장
                    r_read_ack <= read_ack;
                    case (cmd)
                        CMD_START: r_state <= S_START;
                        CMD_WRITE: begin
                            r_is_read <= 1'b0;
                            ack_error <= 1'b0;  // 이전 결과 지움
                            r_state <= S_DATA;
                        end
                        CMD_READ: begin
                            r_is_read <= 1'b1;
                            r_state <= S_DATA;
                        end
                        CMD_STOP: r_state <= S_STOP;
                    endcase
                end
            end
            // START : SCL 이 HIGH 인 동안 SDA 를 HIGH -> LOW 로 내림
            S_START: begin
                if (w_phase_tick) begin
                    case (r_phase)
                        2'd0: r_sda_out <= 1'b1;   // SDA 먼저 HIGH (repeated START 대비)
                        2'd1: r_scl_out <= 1'b1;   // SCL HIGH
                        2'd2: r_sda_out <= 1'b0;   // SCL HIGH 중 SDA 하강 = START
                        2'd3: r_scl_out <= 1'b0;   // SCL LOW : 다음 bit 준비
                    endcase
                    r_phase <= r_phase + 1;
                    if (r_phase == 2'd3) begin     // 4 phase 끝 -> 완료
                        r_state <= S_IDLE;
                        busy <= 1'b0;
                        done <= 1'b1;
                    end
                end
            end
            // DATA : 8 bit 를 1 bit 씩 (bit 1개 = phase 4개)
            //   WRITE : SCL LOW 일 때 SDA 에 bit 출력, slave 가 SCL HIGH 일 때 읽음
            //   READ  : SDA 를 놓고(slave 가 출력), SCL HIGH 중앙에서 master 가 읽음
            S_DATA: begin
                if (w_phase_tick) begin
                    case (r_phase)
                        2'd0: r_sda_out <= r_is_read ? 1'b1 : r_data_reg[r_bit_cnt];   // SCL LOW 구간에서 SDA 변경
                        2'd1: r_scl_out <= 1'b1;
                        2'd2: if (r_is_read) r_data_reg[r_bit_cnt] <= r_sda_sync2;      // SCL HIGH 중앙에서 샘플링
                        2'd3: r_scl_out <= 1'b0;
                    endcase
                    r_phase <= r_phase + 1;
                    if (r_phase == 2'd3) begin     // bit 1개 끝
                        if (r_bit_cnt == 3'd0)     // 8 bit 모두 끝 -> ACK
                            r_state <= S_ACK;
                        else
                            r_bit_cnt <= r_bit_cnt - 1;
                    end
                end
            end
            // ACK (9번째 bit) : 받는 쪽이 SDA 를 LOW 로 내리면 ACK, HIGH 로 두면 NACK
            //   WRITE : master 가 SDA 를 놓고 slave 의 응답을 읽음
            //   READ  : master 가 read_ack 를 보냄 (0:ACK 더 읽음, 1:NACK 그만 읽음)
            S_ACK: begin
                if (w_phase_tick) begin
                    case (r_phase)
                        2'd0: r_sda_out <= r_is_read ? r_read_ack : 1'b1;   // WRITE : slave가 ACK 하도록 SDA release
                        2'd1: r_scl_out <= 1'b1;
                        2'd2: if (!r_is_read) ack_error <= r_sda_sync2;      // HIGH 면 NACK
                        2'd3: r_scl_out <= 1'b0;
                    endcase
                    r_phase <= r_phase + 1;
                    if (r_phase == 2'd3) begin     // 1 byte 완료
                        if (r_is_read)
                            rx_data <= r_data_reg; // 받은 byte 출력
                        r_state <= S_IDLE;
                        busy <= 1'b0;
                        done <= 1'b1;
                    end
                end
            end
            // STOP : SCL 이 HIGH 인 동안 SDA 를 LOW -> HIGH 로 올림
            S_STOP: begin
                if (w_phase_tick) begin
                    case (r_phase)
                        2'd0: r_sda_out <= 1'b0;   // SDA 먼저 LOW
                        2'd1: r_scl_out <= 1'b1;   // SCL HIGH
                        2'd2: r_sda_out <= 1'b1;   // SCL HIGH 중 SDA 상승 = STOP
                        2'd3: ;                    // 다음 START 전 bus 휴식 시간
                    endcase
                    r_phase <= r_phase + 1;
                    if (r_phase == 2'd3) begin
                        r_state <= S_IDLE;         // bus IDLE : SDA, SCL 모두 HIGH
                        busy <= 1'b0;
                        done <= 1'b1;
                    end
                end
            end
            default: r_state <= S_IDLE;
            endcase
        end
    end
endmodule
