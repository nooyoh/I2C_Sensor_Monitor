`timescale 1ns / 1ps

// Master top testbench
//   I2C bus에 두 slave 동작 model 연결
//     - SSD1306 (0x3C, write) : ACK 응답 + GDDRAM 저장 -> 화면을 '#'/'.' 그림으로 출력
//     - Slave basys3 (0x42) : 레지스터 주소(pointer) write 후 repeated START read 로
//         레지스터 0x00~0x05 [STATUS][T정수][T소수][H정수][H소수][거리] 응답
//   STATUS 경우별로 OLED / FND / UART / LED 반응 확인
//     1) 0xA3 정상            2) 0xFF 통신 이상 (이전 값 유지)
//     3) 0xA2 온습도 실패      4) 0xA1 거리 실패      5) 0xA0 둘 다 실패
//     6) 0xA3 복구            7) Slave NACK (이전 값 유지)
module tb_oled;

    localparam [6:0] SENSOR_ADDR = 7'h42;

    reg clk = 0;
    reg reset = 1;
    reg btnU = 0;
    reg btnD = 0;
    wire RsTx;
    wire i2c_sda;
    wire i2c_scl;
    wire [7:0] seg;
    wire [3:0] an;
    wire [15:0] led;

    pullup(i2c_sda);   // 보드의 pull-up 저항 역할
    pullup(i2c_scl);

    top dut (
        .clk(clk),
        .reset(reset),
        .btnU(btnU),
        .btnD(btnD),
        .RsTx(RsTx),
        .i2c_sda(i2c_sda),
        .i2c_scl(i2c_scl),
        .seg(seg),
        .an(an),
        .led(led)
    );

    // simulation 시간 단축
    defparam dut.u_oled_controller.POWER_ON_WAIT_MS = 1;    // 100ms -> 1ms
    defparam dut.u_oled_controller.REFRESH_Hz = 100;        // 10Hz -> 100Hz
    defparam dut.u_i2c_master.I2C_FREQ = 400_000;           // 100KHz -> 400KHz
    defparam dut.u_sensor_reader.READ_Hz = 200;             // 200ms -> 5ms 마다 읽기
    defparam dut.u_fnd_sensor_display.SHOW_MS = 12;         // 3초 -> 12ms
    defparam dut.u_uart_sensor_sender.SEND_Hz = 25;         // 1초 -> 40ms (한 줄 약 37.5ms)

    always #5 clk = ~clk;   // 100MHz

    //---------------- I2C slave 동작 model (SSD1306 + Slave basys3) ----------------
    reg [7:0] gdram [0:511];      // SSD1306 GDDRAM : 128 column x 4 page
    reg [7:0] sensor_buf [0:5];   // Slave basys3 가 보낼 6 byte
    reg [7:0] r_shift;
    reg [7:0] r_tx;               // Slave basys3 가 송신 중인 byte
    reg r_in_frame = 0;
    reg r_oled_sel = 0;           // 1: 이번 transaction은 SSD1306
    reg r_data_mode = 0;          // SSD1306 control byte 0x40 이후 GDDRAM data
    reg r_sensor_wr = 0;          // 1: Slave basys3 write 주소 ACK 함 (다음 byte = pointer)
    reg r_rd_mode = 0;            // 1: Slave basys3 read 주소 ACK 함 (송신 대기)
    reg r_rd_active = 0;          // 1: Slave basys3 송신 중
    reg r_sda_low = 0;            // 1: slave가 SDA를 LOW로 당김
    reg r_sensor_nack = 0;        // 1: Slave basys3 주소에 NACK (미연결 상황)
    integer bit_cnt = 0;
    integer byte_cnt = 0;
    integer rd_bit = 0;
    integer rd_idx = 0;
    integer sensor_ptr = 0;       // Slave basys3 레지스터 pointer
    integer gd_ptr = 0;
    integer i, row, col;

    assign i2c_sda = r_sda_low ? 1'b0 : 1'bz;

    // SCL HIGH 중 SDA 하강 = START, 상승 = STOP
    always @(negedge i2c_sda) if (i2c_scl === 1'b1) begin
        r_in_frame = 1;
        r_oled_sel = 0;
        r_sensor_wr = 0;
        r_rd_mode = 0;
        r_rd_active = 0;
        bit_cnt = 0;
        byte_cnt = 0;
    end
    always @(posedge i2c_sda) if (i2c_scl === 1'b1) r_in_frame = 0;

    // SCL 상승 : 수신 bit 샘플링 / Slave 송신 중이면 master ACK 확인
    always @(posedge i2c_scl) if (r_in_frame) begin
        if (r_rd_active) begin
            if (rd_bit == 9) begin
                if (i2c_sda == 1'b0) begin   // master ACK : 다음 byte
                    rd_idx = rd_idx + 1;
                    r_tx = sensor_buf[rd_idx];
                    rd_bit = 0;
                end else begin               // master NACK : 송신 끝
                    r_rd_active = 0;
                    r_rd_mode = 0;
                end
            end
        end else if (bit_cnt < 8) begin
            r_shift = {r_shift[6:0], i2c_sda};
            bit_cnt = bit_cnt + 1;
        end
    end

    // SCL 하강 : ACK 출력 / Slave 송신 bit 출력
    always @(negedge i2c_scl) if (r_in_frame) begin
        if (r_rd_active) begin
            if (rd_bit < 8) begin             // MSB first
                r_sda_low = ~r_tx[7 - rd_bit];
                rd_bit = rd_bit + 1;
            end else if (rd_bit == 8) begin   // master ACK 받도록 SDA release
                r_sda_low = 0;
                rd_bit = 9;
            end
        end else if (bit_cnt == 8) begin
            if (byte_cnt == 0) begin          // slave 주소
                if (r_shift == {7'h3C, 1'b0}) begin
                    r_oled_sel = 1;
                    r_sda_low = 1;
                end else if (r_shift == {SENSOR_ADDR, 1'b0} && !r_sensor_nack) begin
                    r_sensor_wr = 1;          // 다음 byte = 레지스터 pointer
                    r_sda_low = 1;
                end else if (r_shift == {SENSOR_ADDR, 1'b1} && !r_sensor_nack) begin
                    r_rd_mode = 1;
                    r_sda_low = 1;
                end else begin
                    r_sda_low = 0;            // NACK
                end
            end else if (r_sensor_wr && byte_cnt == 1) begin  // Slave basys3 레지스터 pointer
                sensor_ptr = r_shift;
                r_sda_low = 1;
            end else if (r_oled_sel && byte_cnt == 1) begin   // SSD1306 control byte
                r_data_mode = (r_shift == 8'h40);
                r_sda_low = 1;
            end else if (r_oled_sel) begin
                if (r_data_mode) begin
                    gdram[gd_ptr] = r_shift;
                    gd_ptr = (gd_ptr + 1) % 512;
                end else if (r_shift == 8'h21) begin   // column 주소 설정시 처음부터
                    gd_ptr = 0;
                end
                r_sda_low = 1;
            end
            byte_cnt = byte_cnt + 1;
            bit_cnt = 9;
        end else if (bit_cnt == 9) begin      // ACK clock 끝
            bit_cnt = 0;
            if (r_rd_mode) begin              // 주소 ACK 후 바로 pointer 위치 byte 송신 시작
                r_rd_active = 1;
                rd_idx = sensor_ptr;
                r_tx = sensor_buf[rd_idx];
                r_sda_low = ~r_tx[7];
                rd_bit = 1;
            end else begin
                r_sda_low = 0;
            end
        end
    end

    // Slave basys3 응답 data 설정
    task set_sensor(input [7:0] status, input [7:0] t_int, input [7:0] t_dec,
                    input [7:0] h_int, input [7:0] h_dec, input [7:0] d);
        begin
            sensor_buf[0] = status;
            sensor_buf[1] = t_int;
            sensor_buf[2] = t_dec;
            sensor_buf[3] = h_int;
            sensor_buf[4] = h_dec;
            sensor_buf[5] = d;
        end
    endtask

    //---------------- OLED 화면 출력 ----------------
    task print_screen;
        begin
            for (row = 0; row < 32; row = row + 1) begin
                $write("    ");
                for (col = 0; col < 128; col = col + 1)
                    $write("%s", gdram[(row / 8) * 128 + col][row % 8] ? "#" : ".");
                $write("\n");
            end
        end
    endtask

    //---------------- FND 출력 확인 ----------------
    // seg = {dp, g, f, e, d, c, b, a} (active low)
    function [7:0] seg2char(input [6:0] s);
        case (s)
            7'b1000000: seg2char = "0";
            7'b1111001: seg2char = "1";
            7'b0100100: seg2char = "2";
            7'b0110000: seg2char = "3";
            7'b0011001: seg2char = "4";
            7'b0010010: seg2char = "5";
            7'b0000010: seg2char = "6";
            7'b1111000: seg2char = "7";
            7'b0000000: seg2char = "8";
            7'b0010000: seg2char = "9";
            7'b0000111: seg2char = "t";
            7'b0001011: seg2char = "h";
            7'b0100001: seg2char = "d";
            7'b0111111: seg2char = "-";
            7'b1111111: seg2char = " ";
            default:    seg2char = "?";
        endcase
    endfunction

    // 스캔 순서(an[0] -> an[3])대로 1ms 마다 한 자리씩 seg를 읽어 "t25.3" 형태로 만듦
    task read_fnd(output [39:0] r_str, output [7:0] r_led);
        integer k;
        reg [3:0] an_on;
        reg [31:0] r_char;  // 자리별 문자 {d3, d2, d1, d0}
        reg [3:0] r_dp;
        begin
            for (k = 0; k < 4; k = k + 1) begin
                an_on = ~(4'b0001 << k);
                if (an == an_on) @(an);   // 이미 켜진 구간의 끝일 수 있으므로 다음 차례를 기다림
                wait (an == an_on);
                #10;
                r_char[k * 8 +: 8] = seg2char(seg[6:0]);
                r_dp[k] = !seg[7];
            end
            r_led = led[7:0];
            r_str = "";
            for (k = 3; k >= 0; k = k - 1) begin
                r_str = {r_str[31:0], r_char[k * 8 +: 8]};
                if (r_dp[k]) r_str = {r_str[31:0], "."};
            end
        end
    endtask

    // 자동 전환(state 3)에서 온도 / 습도 / 거리 3 항목을 차례로 읽기
    task print_fnd_all;
        reg [39:0] s0, s1, s2;
        reg [7:0] l0, l1, l2;
        begin
            @(dut.u_fnd_sensor_display.r_item); #2_000_000; read_fnd(s0, l0);
            @(dut.u_fnd_sensor_display.r_item); #2_000_000; read_fnd(s1, l1);
            @(dut.u_fnd_sensor_display.r_item); #2_000_000; read_fnd(s2, l2);
            $display("  FND  : %s (led=%0d)  %s (led=%0d)  %s (led=%0d)", s0, l0, s1, l1, s2, l2);
        end
    endtask

    //---------------- UART 수신 확인 ----------------
    localparam BIT_TIME = 104_167;   // 9600bps 1 bit (ns)
    reg [34*8-1:0] uart_line = 0;    // '\r\n' 제외 34글자
    reg [34*8-1:0] uart_last = 0;    // 마지막으로 받은 한 줄
    reg [7:0] uart_byte;
    integer uart_cnt = 0;            // 받은 줄 수
    integer ui;

    // start bit 하강 -> 각 bit 중앙에서 샘플링 (LSB first)
    initial begin
        forever begin
            @(negedge RsTx);
            #(BIT_TIME / 2);
            for (ui = 0; ui < 8; ui = ui + 1) begin
                #BIT_TIME;
                uart_byte[ui] = RsTx;
            end
            #BIT_TIME;   // stop bit
            if (uart_byte == 8'h0A) begin
                uart_last = uart_line;
                uart_line = 0;
                uart_cnt = uart_cnt + 1;
            end else if (uart_byte != 8'h0D) begin
                uart_line = {uart_line, uart_byte};
            end
        end
    end

    //---------------- 경우별 확인 ----------------
    // Slave 응답을 바꾼 뒤 2번 읽기 -> LED / OLED / UART / FND 출력
    task run_case(input [8*24-1:0] name);
        integer n;
        begin
            $display("");
            $display("==== %s ====", name);
            repeat (2) begin
                wait (dut.u_sensor_reader.r_state == 3'd5);   // S_UPDATE
                wait (dut.u_sensor_reader.r_state != 3'd5);
            end
            $display("  LED  : PASS(15)=%b FAIL(14)=%b NACK(13)=%b   dht_ok=%b dist_ok=%b",
                     led[15], led[14], led[13], dut.w_dht_ok, dut.w_dist_ok);
            // 값 갱신 이후 시작한 화면 / UART 줄 기다리기
            @(posedge dut.w_frame_start);
            wait (dut.u_oled_controller.r_state == 3'd6);   // S_REFRESH_WAIT
            n = uart_cnt;
            wait (uart_cnt == n + 2);
            $display("  UART : %s", uart_last);
            print_fnd_all;
            $display("  OLED :");
            print_screen;
        end
    endtask

    initial begin
        for (i = 0; i < 512; i = i + 1) gdram[i] = 8'h00;
        set_sensor(8'hA3, 8'd25, 8'd3, 8'd40, 8'd7, 8'd61);
        #100 reset = 0;

        // FND 자동 전환 상태로 (btnD : 0 -> 3)
        btnD = 1; #12_000_000; btnD = 0;

        run_case("1) 0xA3 normal");

        set_sensor(8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF, 8'hFF);
        run_case("2) 0xFF bad signature");

        set_sensor(8'hA2, 8'd99, 8'd9, 8'd99, 8'd9, 8'd75);
        run_case("3) 0xA2 DHT fail");

        set_sensor(8'hA1, 8'd26, 8'd1, 8'd41, 8'd2, 8'd200);
        run_case("4) 0xA1 DIST fail");

        set_sensor(8'hA0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0);
        run_case("5) 0xA0 both fail");

        set_sensor(8'hA3, 8'd27, 8'd5, 8'd42, 8'd0, 8'd88);
        run_case("6) 0xA3 recover");

        r_sensor_nack = 1;
        run_case("7) Slave NACK");

        $finish;
    end
endmodule
