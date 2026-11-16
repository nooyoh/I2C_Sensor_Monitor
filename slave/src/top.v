`timescale 1ns / 1ps

module top(
    input clk,
    input reset,
    inout dht11_data,
    input echo,
    input  wire scl,
    inout  wire sda,
    output wire [15:0] led,
    output tx,
    output trig, 
    output [7:0] seg,
    output [3:0] an
    );

    wire w_tick;
    wire [7:0] w_temperature;
    wire [7:0] w_humidity;
    wire [7:0] w_distance_cm;
    wire [7:0] w_temperature_frac, w_humidity_frac;
    wire dht_error, distance_error;

    dht11 u_dht11(
        .clk(clk),
        .reset(reset),
        .dht11_data(dht11_data),
        .temperature(w_temperature),
        .humidity(w_humidity),
        .temperature_frac(w_temperature_frac),
        .humidity_frac(w_humidity_frac),
        .error(dht_error),
        .data_valid()
    );

    tick_generator u_tick_generator(
        .clk(clk),
        .reset(reset),
        .tick(w_tick)
    );

    fnd_controller u_fnd_controller(
        .clk(clk),
        .reset(reset),
        .tick(w_tick),
        .in_data({6'b0, w_distance_cm}),
        .seg(seg),
        .an(an)
    );

    hcsr04 u_hcsr04 (
        .clk         (clk),
        .reset       (reset),
        .echo        (echo),
        .trig        (trig),
        .distance_cm (w_distance_cm),
        .data_valid  (),
        .error       (distance_error)
    );

    i2c #(
        .ADDRESS(7'h42)
    ) u_i2c (
        .clk         (clk),
        .reset       (reset),
        .scl         (scl),
        .sda         (sda),
        .distance_error(distance_error),
        .dht_error(dht_error),
        .temperature_frac(w_temperature_frac),
        .humidity_frac(w_humidity_frac),
        .temperature (w_temperature),
        .humidity    (w_humidity),
        .distance_cm (w_distance_cm),
        .led         (led)
    );


    wire w_tx_busy;
    wire w_tx_start;
    wire [7:0] w_tx_data;
    reg [9:0] tick_count;
    reg send_trigger;

    // Existing 1 ms FND tick: 1000 ticks produce one UART request.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            tick_count <= 0;
            send_trigger <= 0;
        end else begin
            send_trigger <= 0;
            if (w_tick) begin
                if (tick_count == 999) begin
                    tick_count <= 0;
                    send_trigger <= 1;
                end else begin
                    tick_count <= tick_count + 1'b1;
                end
            end
        end
    end

    data_sender u_data_sender (
        .clk              (clk),
        .reset            (reset),
        .start_trigger    (send_trigger),

        .temperature      (w_temperature),
        .temperature_frac (w_temperature_frac),
        .humidity         (w_humidity),
        .humidity_frac    (w_humidity_frac),
        .distance_cm      (w_distance_cm),
        .distance_error   (distance_error),
        .dht_error        (dht_error),

        .tx_busy          (w_tx_busy),
        .tx_start         (w_tx_start),
        .tx_data          (w_tx_data)
    );

    uart_tx #(
        .BPS(9600)
    ) u_uart_tx (
        .clk(clk),
        .reset(reset),
        .tx_data(w_tx_data),
        .tx_start(w_tx_start),
        .tx(tx),
        .tx_busy(w_tx_busy)
    );

endmodule
