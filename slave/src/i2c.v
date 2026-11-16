`timescale 1ns / 1ps

module i2c #(
    parameter [6:0] ADDRESS = 7'h42
)(
    input clk, reset,
    input scl,
    inout  sda,
    input [7:0] temperature, humidity,
    input [7:0] temperature_frac, humidity_frac, distance_cm,
    input distance_error, dht_error,
    output [15:0] led
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] scl_sync, sda_sync;
    reg scl_prev, sda_prev;
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
            scl_prev <= 1;
            sda_prev <= 1;
        end else begin
            scl_sync <= {scl_sync[0], scl};
            sda_sync <= {sda_sync[0], sda};
            scl_prev <= scl_sync[1];
            sda_prev <= sda_sync[1];
        end
    end

    wire scl_up   = !scl_prev && scl_sync[1];
    wire scl_down = scl_prev && !scl_sync[1];
    wire sda_in   = sda_sync[1];
    wire scl_high = scl_prev && scl_sync[1];
    wire start_seen = scl_high && sda_prev && !sda_in;
    wire stop_seen  = scl_high && !sda_prev && sda_in;

    reg drive_low;
    assign sda = drive_low ? 1'b0 : 1'bz;

    localparam IDLE = 0, RX = 1, ACK_BEGIN = 2, ACK_END = 3,
               TX = 4, TX_RELEASE = 5, MASTER_ACK = 6, NEXT_TX = 7;
    reg [2:0] state, bit_index;
    reg address_phase, pointer_phase, read_request;
    reg [6:0] rx_shift;
    wire [7:0] received = {rx_shift, sda_in};
    reg [7:0] pointer, scratch, tx_byte;
    reg [7:0] saved_temp, saved_humidity;
    reg [7:0] saved_temp_frac, saved_humidity_frac, saved_distance;
    reg saved_distance_error, saved_dht_error;
    reg [7:0] read_byte;
    always @(*) begin
        case (pointer)
            8'h00: read_byte = {6'b101000, ~saved_distance_error, ~saved_dht_error};
            8'h01: read_byte = saved_temp;
            8'h02: read_byte = saved_temp_frac;
            8'h03: read_byte = saved_humidity;
            8'h04: read_byte = saved_humidity_frac;
            8'h05: read_byte = saved_distance;
            8'h10: read_byte = scratch;
            default: read_byte = 8'h00;
        endcase
    end

    reg active, write_toggle, read_toggle;
    reg [7:0] last_write;
    assign led = {active, 5'd0, read_toggle, write_toggle, last_write};

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            drive_low <= 0;
            address_phase <= 0;
            pointer_phase <= 0;
            read_request <= 0;
            bit_index <= 0;
            rx_shift <= 0;
            pointer <= 0;
            scratch <= 0;
            tx_byte <= 0;
            saved_temp_frac <= 0;
            saved_humidity_frac <= 0;
            saved_distance_error <= 1;
            saved_dht_error <= 1;
            saved_temp <= 0;
            saved_humidity <= 0;
            saved_distance <= 0;
            active <= 0;
            write_toggle <= 0;
            read_toggle <= 0;
            last_write <= 0;
        end else if (start_seen) begin
            active <= 1;
            drive_low <= 0;
            address_phase <= 1;
            pointer_phase <= 1;
            read_request <= 0;
            bit_index <= 0;
            state <= RX;
        end else if (stop_seen) begin
            active <= 0;
            drive_low <= 0;
            state <= IDLE;
        end else begin
            case (state)
                RX: if (scl_up) begin
                    rx_shift <= received[6:0];
                    bit_index <= bit_index + 1'b1;
                    if (bit_index == 7) begin
                        if (address_phase) begin
                            address_phase <= 0;
                            read_request <= received[0];
                            if (received[7:1] == ADDRESS) begin
                                state <= ACK_BEGIN;
                                // One snapshot for the entire read transaction.
                                saved_distance_error <= distance_error;
                                saved_dht_error <= dht_error;
                                saved_temp_frac <= temperature_frac;
                                saved_humidity_frac <= humidity_frac;
                                saved_temp <= temperature;
                                saved_humidity <= humidity;
                                saved_distance <= distance_cm;
                            end else begin
                                state <= IDLE; // Other address: leave SDA released.
                            end
                        end else if (pointer_phase || pointer == 8'h10) begin
                            last_write <= received;
                            write_toggle <= !write_toggle;
                            if (pointer_phase) begin
                                pointer <= received;
                                pointer_phase <= 0;
                            end else begin
                                scratch <= received;
                                pointer <= pointer + 1'b1;
                            end
                            state <= ACK_BEGIN;
                        end else begin
                            state <= IDLE; // Data write to read-only register: NACK.
                        end
                    end
                end

                ACK_BEGIN: if (scl_down) begin
                    drive_low <= 1;
                    state <= ACK_END;
                end
                ACK_END: if (scl_down) begin
                    if (read_request) begin
                        tx_byte <= read_byte;
                        bit_index <= 7;
                        drive_low <= !read_byte[7];
                        state <= TX;
                    end else begin
                        drive_low <= 0;
                        bit_index <= 0;
                        state <= RX;
                    end
                end

                TX: begin
                    if (scl_down && bit_index != 0) begin
                        bit_index <= bit_index - 1'b1;
                        drive_low <= !tx_byte[bit_index - 1'b1];
                    end
                    if (scl_up && bit_index == 0) begin
                        read_toggle <= !read_toggle;
                        pointer <= pointer + 1'b1;
                        state <= TX_RELEASE;
                    end
                end
                TX_RELEASE: if (scl_down) begin
                    drive_low <= 0;
                    state <= MASTER_ACK;
                end
                MASTER_ACK: if (scl_up)
                    state <= sda_in ? IDLE : NEXT_TX;
                NEXT_TX: if (scl_down) begin
                    tx_byte <= read_byte;
                    bit_index <= 7;
                    drive_low <= !read_byte[7];
                    state <= TX;
                end
                default: drive_low <= 0;
            endcase
        end
    end
endmodule
