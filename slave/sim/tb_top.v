`timescale 1ns / 1ps

module tb_top;

    reg clk, reset, echo, scl;
    reg dht_low, sda_low;
    tri1 dht11_data, sda;
    wire trig, tx;

    reg [7:0] expected [0:5];
    integer n;

    localparam [39:0] DHT_FRAME = {8'd61, 8'd7, 8'd25, 8'd4, 8'd97};
    localparam BIT_NS = 104167;  // UART 9600 baud

    assign dht11_data = dht_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;

    always #5 clk = ~clk;  // 100 MHz

    top dut (
        .clk(clk), 
        .reset(reset),
        .dht11_data(dht11_data),
        .echo(echo), 
        .trig(trig),
        .scl(scl), 
        .sda(sda), 
        .tx(tx),
        .led(), 
        .seg(), 
        .an()
    );

    // DHT11: 습도 61.7%, 온도 25.4도
    task send_dht;
        integer i;
        begin
            @(negedge clk);
            dut.u_dht11.timer = dut.u_dht11.WAIT_CLKS - 1; // dht 대기 카운터 줄이기

            @(negedge dht11_data);
            @(posedge dht11_data); // pin을 당겼다 놓기
            #30000;

            dht_low = 1; #80000;
            dht_low = 0; #80000;

            for (i = 39; i >= 0; i = i - 1) begin
                dht_low = 1; #50000;
                dht_low = 0;
                if (DHT_FRAME[i]) #70000;
                else              #26000;
            end

            dht_low = 1; #50000;
            dht_low = 0;
        end
    endtask

    // HC-SR04: 580us 펄스 → 10cm
    task send_echo;
        begin
            @(negedge clk);
            dut.u_hcsr04.count_us = 99999;
            dut.u_hcsr04.divider = 99;

            @(posedge trig);
            @(negedge trig);
            #20000;
            echo = 1; #580000; // 거리 만큼 기다리기
            echo = 0;
        end
    endtask

    task i2c_start;
        begin
            scl = 0; sda_low = 0; #5000;
            scl = 1; #5000;
            sda_low = 1; #5000;
            scl = 0;
        end
    endtask

    task i2c_write;
        input [7:0] data;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                sda_low = !data[i]; #5000;
                scl = 1; #5000;
                scl = 0;
            end

            sda_low = 0; #5000;
            scl = 1; #2500;
            if (sda !== 1'b0) begin
                $display("FAIL: I2C NACK for %h", data);
                $finish;
            end
            #2500; scl = 0;
        end
    endtask

    task i2c_read;
        input [7:0] wanted;
        input last_byte;
        reg [7:0] data;
        integer i;
        begin
            sda_low = 0;
            for (i = 7; i >= 0; i = i - 1) begin
                #5000; scl = 1; #2500;
                data[i] = sda;
                #2500; scl = 0;
            end

            if (data !== wanted) begin
                $display("FAIL: I2C got %h, expected %h", data, wanted);
                $finish;
            end

            sda_low = !last_byte;  // 마지막 바이트만 NACK
            #5000; scl = 1; #5000; scl = 0;
        end
    endtask

    task uart_read;
        input [7:0] wanted;
        reg [7:0] data;
        integer i;
        begin
            @(negedge tx);
            #(BIT_NS / 2);

            for (i = 0; i < 8; i = i + 1) begin
                #(BIT_NS);
                data[i] = tx;
            end

            #(BIT_NS);
            if (data !== wanted || tx !== 1'b1) begin
                $display("FAIL: UART got %h, expected %h, stop=%b",
                         data, wanted, tx);
                $finish;
            end
        end
    endtask

    initial begin
        clk = 0;
        reset = 1;
        echo = 0;
        scl = 1;
        dht_low = 0;
        sda_low = 0;

        expected[0] = 8'hA3;  // 두 센서 정상
        expected[1] = 25;     // 온도 정수
        expected[2] = 4;      // 온도 소수
        expected[3] = 61;     // 습도 정수
        expected[4] = 7;      // 습도 소수
        expected[5] = 10;     // 거리 cm

        repeat (5) @(negedge clk);
        reset = 0;

        fork
            send_dht;
            send_echo;
        join

        // I2C: 0번 레지스터부터 6바이트 읽기
        i2c_start;
        i2c_write(8'h84);
        i2c_write(8'h00);
        i2c_start;
        i2c_write(8'h85);

        for (n = 0; n < 6; n = n + 1)
            i2c_read(expected[n], n == 5);

        sda_low = 1; #5000;
        scl = 1; #5000;
        sda_low = 0; #5000;  // STOP

        // UART: AA 55 + 센서 데이터 6바이트 읽기
        @(negedge clk);
        dut.tick_count = 999;

        uart_read(8'hAA);
        uart_read(8'h55);

        for (n = 0; n < 6; n = n + 1)
            uart_read(expected[n]);

        $display("PASS: I2C and UART");
        $finish;
    end

    initial begin
        #60000000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule