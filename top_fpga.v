module top_fpga (
    input  wire        CLOCK_50,    // 50 MHz, not used now but kept for future
    input  wire [7:0]  bus_data,    // 8-bit data from ESP32
    input  wire        bus_clk,     // strobe per byte from ESP32
    input  wire        bus_latch,   // currently unused in this core
    output reg         valid_out,   // HIGH for one bus_clk when new token seen
    output reg         replay_out   // HIGH for one bus_clk when token repeats
);

    reg [63:0] current_token = 64'd0;
    reg [63:0] last_token    = 64'd0;
    reg [2:0]  byte_count    = 3'd0;   // 0..7

    always @(posedge bus_clk) begin
        // default outputs low each byte
        valid_out  <= 1'b0;
        replay_out <= 1'b0;

        // Shift MSB-first: new byte becomes lowest 8 bits
        current_token <= {current_token[55:0], bus_data};

        if (byte_count == 3'd7) begin
            // After 8th byte, we have a full 64-bit token
            byte_count <= 3'd0;

            if ({current_token[55:0], bus_data} == last_token) begin
                // Same token as before → replay
                replay_out <= 1'b1;
            end else begin
                // New token → accept
                valid_out  <= 1'b1;
                last_token <= {current_token[55:0], bus_data};
            end
        end else begin
            byte_count <= byte_count + 3'd1;
        end
    end

endmodule
