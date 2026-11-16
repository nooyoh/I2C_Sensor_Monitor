`timescale 1ns / 1ps

// Master top testbench : 파형 한 화면(약 10.6ms)에 모든 테스트 동작이 보이도록 시간 압축
//   step (radix : ASCII) 순서
//     1 NORMAL   : STATUS 0xA3 정상            -> led[15] PASS, led[7:0] = 25
//     2 BUTTON   : btnU x2, btnD x2            -> led[7:0] 25 -> 40 -> 61 -> 40 -> 25
//     3 DHT FAIL : STATUS 0xA2 온습도 실패      -> dht_ok = 0
//     4 DISTFAIL : STATUS 0xA1 거리 실패        -> dist_ok = 0
//     5 BAD SIG  : STATUS 0xFF 통신 이상        -> led[14] FAIL, 이전 값 유지
//     6 NACK     : Slave 무응답                 -> led[13] NACK, led[14] FAIL
//     7 RECOVER  : STATUS 0xA3 새 값            -> led[15] PASS, 값 갱신
//   OLED 초기화 / 화면 전송, UART 전송은 위 동작 중 주기적으로 함께 진행
//   I2C slave / UART 수신 model 은 아래 별도 module 로 분리 (파형에 내부 변수 안 보이게)
module tb_master;

    reg clk = 0;
    reg reset = 1;
    reg btnU = 0;
    reg btnD = 0;
    wire RsTx;
    wire i2c_sda;
    wire i2c_scl;
    wire [15:0] led;                 // [15]PASS [14]FAIL [13]NACK [12]I2C 동작 [7:0]FND 표시 값

    reg [8*10-1:0] step = "RESET";   // 현재 테스트 단계 (radix : ASCII)
    wire [7:0] uart_char;            // UART 로 받은 마지막 글자 (radix : ASCII)
    wire sr_grant   = dut.w_sr_bus_grant;     // I2C bus 사용 : 센서 읽기
    wire oled_grant = dut.w_oled_bus_grant;   // I2C bus 사용 : OLED 전송
    wire [7:0] temp = dut.w_temp_int;         // 읽어온 센서 값
    wire [7:0] humi = dut.w_humi_int;
    wire [7:0] dist = dut.w_dist;
    wire dht_ok     = dut.w_dht_ok;           // 0 : 온습도 실패
    wire dist_ok    = dut.w_dist_ok;          // 0 : 거리 실패

    pullup(i2c_sda);   // 보드 pull-up 저항 역할
    pullup(i2c_scl);

    top dut (
        .clk(clk),
        .reset(reset),
        .btnU(btnU),
        .btnD(btnD),
        .RsTx(RsTx),
        .i2c_sda(i2c_sda),
        .i2c_scl(i2c_scl),
        .seg(),                      // FND 표시 값은 led[7:0] 으로 확인
        .an(),
        .led(led)
    );

    i2c_slave_model u_slave (
        .sda(i2c_sda),
        .scl(i2c_scl)
    );

    uart_rx_model u_uart_rx (
        .rx(RsTx),
        .rx_char(uart_char)
    );

    // 시뮬레이션 시간 단축 (나머지 파라미터는 top 기본값 그대로)
    defparam dut.u_i2c_master.I2C_FREQ = 2_500_000;              // 100KHz -> 2.5MHz
    defparam dut.u_sensor_reader.READ_Hz = 2000;                 // 200ms -> 0.5ms 마다 읽기
    defparam dut.u_oled_controller.POWER_ON_WAIT_MS = 1;         // 100ms -> 1ms
    defparam dut.u_oled_controller.REFRESH_Hz = 250;             // 100ms -> 4ms 마다 화면 갱신
    defparam dut.u_uart_tx.BPS = 1_000_000;                      // 9600bps -> 1Mbps
    defparam dut.u_uart_sensor_sender.SEND_Hz = 500;             // 1초 -> 2ms 마다 한 줄
    defparam dut.u_btn_debouncer.u_btnU_debouncer.DEBOUNCE_LIMIT = 100;   // 10ms -> 1us
    defparam dut.u_btn_debouncer.u_btnD_debouncer.DEBOUNCE_LIMIT = 100;

    always #5 clk = ~clk;   // 100MHz

    // 센서 읽기 n번 끝날 때까지 (sensor_reader S_UPDATE 통과)
    task wait_read(input integer n);
        repeat (n) begin
            wait (dut.u_sensor_reader.r_state == 4'd8);
            wait (dut.u_sensor_reader.r_state != 4'd8);
        end
    endtask

    // Slave 가 보낼 6 byte {STATUS, T정수, T소수, H정수, H소수, 거리}
    task set_sensor(input [47:0] data);
        u_slave.sensor_data = data;
    endtask

    // 1 : btnU, 0 : btnD  -> 5us 누름 후 150us 대기
    task press(input up);
        begin
            if (up) btnU = 1; else btnD = 1;
            #5_000;
            btnU = 0; btnD = 0;
            #150_000;
        end
    endtask

    initial begin
        set_sensor({8'hA3, 8'd25, 8'd3, 8'd40, 8'd7, 8'd61});
        #1_000 reset = 0;

        step = "1 NORMAL";   wait_read(2);

        step = "2 BUTTON";
        press(1);   // btnU : 습도 40
        press(1);   // btnU : 거리 61
        press(0);   // btnD : 습도 40
        press(0);   // btnD : 온도 25

        step = "3 DHT FAIL"; set_sensor({8'hA2, 8'd0, 8'd0, 8'd0, 8'd0, 8'd75});   wait_read(2);
        step = "4 DISTFAIL"; set_sensor({8'hA1, 8'd26, 8'd1, 8'd41, 8'd2, 8'd0});  wait_read(2);
        step = "5 BAD SIG";  set_sensor({6{8'hFF}});                                wait_read(2);
        step = "6 NACK";     u_slave.sensor_nack = 1;                               wait_read(2);
        step = "7 RECOVER";  u_slave.sensor_nack = 0;
                             set_sensor({8'hA3, 8'd27, 8'd5, 8'd42, 8'd0, 8'd88});  wait_read(2);

        step = "END";
        #100_000;
        $finish;
    end

endmodule


// I2C slave 동작 model : SSD1306(0x3C, ACK 만) + Slave basys3(0x42, pointer write 후 repeated START read)
module i2c_slave_model (
    inout sda,
    input scl
    );

    reg [47:0] sensor_data = 0;   // 레지스터 0x00~0x05 (tb 에서 설정)
    reg sensor_nack = 0;          // 1 : 0x42 주소에 NACK (tb 에서 설정)
    reg [7:0] r_shift;
    reg [7:0] r_tx;
    reg r_in_frame = 0;
    reg r_oled_sel = 0;
    reg r_sensor_wr = 0;
    reg r_rd_mode = 0;
    reg r_rd_active = 0;
    reg r_sda_low = 0;
    integer bit_cnt = 0;
    integer byte_cnt = 0;
    integer rd_bit = 0;
    integer rd_idx = 0;
    integer ptr = 0;

    assign sda = r_sda_low ? 1'b0 : 1'bz;

    function [7:0] reg_byte(input integer idx);   // 레지스터 idx 의 값
        reg_byte = sensor_data[47 - idx * 8 -: 8];
    endfunction

    // SCL HIGH 중 SDA 하강 = START, 상승 = STOP
    always @(negedge sda) if (scl === 1'b1) begin
        r_in_frame = 1;
        r_oled_sel = 0;
        r_sensor_wr = 0;
        r_rd_mode = 0;
        r_rd_active = 0;
        bit_cnt = 0;
        byte_cnt = 0;
    end
    always @(posedge sda) if (scl === 1'b1) r_in_frame = 0;

    // SCL 상승 : bit 수신 / 송신 중이면 master ACK 확인
    always @(posedge scl) if (r_in_frame) begin
        if (r_rd_active) begin
            if (rd_bit == 9) begin
                if (sda == 1'b0) begin   // master ACK : 다음 byte
                    rd_idx = rd_idx + 1;
                    r_tx = reg_byte(rd_idx);
                    rd_bit = 0;
                end else begin           // master NACK : 송신 끝
                    r_rd_active = 0;
                    r_rd_mode = 0;
                end
            end
        end else if (bit_cnt < 8) begin
            r_shift = {r_shift[6:0], sda};
            bit_cnt = bit_cnt + 1;
        end
    end

    // SCL 하강 : ACK 출력 / 송신 bit 출력
    always @(negedge scl) if (r_in_frame) begin
        if (r_rd_active) begin
            if (rd_bit < 8) begin
                r_sda_low = ~r_tx[7 - rd_bit];
                rd_bit = rd_bit + 1;
            end else if (rd_bit == 8) begin
                r_sda_low = 0;
                rd_bit = 9;
            end
        end else if (bit_cnt == 8) begin
            if (byte_cnt == 0) begin
                if (r_shift == {7'h3C, 1'b0}) begin
                    r_oled_sel = 1;
                    r_sda_low = 1;
                end else if (r_shift == {7'h42, 1'b0} && !sensor_nack) begin
                    r_sensor_wr = 1;
                    r_sda_low = 1;
                end else if (r_shift == {7'h42, 1'b1} && !sensor_nack) begin
                    r_rd_mode = 1;
                    r_sda_low = 1;
                end else begin
                    r_sda_low = 0;        // NACK
                end
            end else if (r_sensor_wr && byte_cnt == 1) begin   // 레지스터 pointer
                ptr = r_shift;
                r_sda_low = 1;
            end else if (r_oled_sel) begin
                r_sda_low = 1;
            end
            byte_cnt = byte_cnt + 1;
            bit_cnt = 9;
        end else if (bit_cnt == 9) begin
            bit_cnt = 0;
            if (r_rd_mode) begin
                r_rd_active = 1;
                rd_idx = ptr;
                r_tx = reg_byte(rd_idx);
                r_sda_low = ~r_tx[7];
                rd_bit = 1;
            end else begin
                r_sda_low = 0;
            end
        end
    end
endmodule


// UART 수신 model : 1Mbps (bit 1us), 받은 byte 를 rx_char 로 출력
module uart_rx_model (
    input rx,
    output reg [7:0] rx_char = 0
    );

    reg [7:0] r_data;
    integer i;

    initial forever begin
        @(negedge rx);
        #500;                    // start bit 중앙
        for (i = 0; i < 8; i = i + 1) begin
            #1000;
            r_data[i] = rx;
        end
        #1000;                   // stop bit
        rx_char = r_data;
    end
endmodule
