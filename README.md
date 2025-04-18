# CORDIC_ALGORITHM



`timescale 1 ns/100 ps

module cordic_vectoring #(
    parameter WIDTH = 16,
    parameter ITER = 16
)(
    input wire                      clk,
    input wire signed [WIDTH-1:0]  Xin,
    input wire signed [WIDTH-1:0]  Yin,
    output reg signed [WIDTH:0]    Xout,
    output reg signed [WIDTH:0]    Yout,
    output reg signed [31:0]       Zout
);

    // Arctan lookup table (precomputed for angles atan(2^-i))
    wire signed [31:0] atan_table [0:30];
    assign atan_table[00] = 32'b00100000000000000000000000000000; // 45.000°
    assign atan_table[01] = 32'b00010010111001000000010100011101; // 26.565°
    assign atan_table[02] = 32'b00001001111110110011100001011011; // 14.036°
    assign atan_table[03] = 32'b00000101000100010001000111010100;
    assign atan_table[04] = 32'b00000010100010110000110101000011;
    assign atan_table[05] = 32'b00000001010001011101011111100001;
    assign atan_table[06] = 32'b00000000101000101111011000011110;
    assign atan_table[07] = 32'b00000000010100010111110001010101;
    assign atan_table[08] = 32'b00000000001010001011111001010011;
    assign atan_table[09] = 32'b00000000000101000101111100101110;
    assign atan_table[10] = 32'b00000000000010100010111110011000;
    assign atan_table[11] = 32'b00000000000001010001011111001100;
    assign atan_table[12] = 32'b00000000000000101000101111100110;
    assign atan_table[13] = 32'b00000000000000010100010111110011;
    assign atan_table[14] = 32'b00000000000000001010001011111001;
    assign atan_table[15] = 32'b00000000000000000101000101111101;
    assign atan_table[16] = 32'b00000000000000000010100010111110;
    assign atan_table[17] = 32'b00000000000000000001010001011111;
    assign atan_table[18] = 32'b00000000000000000000101000101111;
    assign atan_table[19] = 32'b00000000000000000000010100011000;
    assign atan_table[20] = 32'b00000000000000000000001010001100;
    assign atan_table[21] = 32'b00000000000000000000000101000110;
    assign atan_table[22] = 32'b00000000000000000000000010100011;
    assign atan_table[23] = 32'b00000000000000000000000001010001;
    assign atan_table[24] = 32'b00000000000000000000000000101000;
    assign atan_table[25] = 32'b00000000000000000000000000010100;
    assign atan_table[26] = 32'b00000000000000000000000000001010;
    assign atan_table[27] = 32'b00000000000000000000000000000101;
    assign atan_table[28] = 32'b00000000000000000000000000000010;
    assign atan_table[29] = 32'b00000000000000000000000000000001;
    assign atan_table[30] = 32'b00000000000000000000000000000000;

    // Pipeline registers for iterative rotations
    reg signed [WIDTH:0] X [0:ITER];
    reg signed [WIDTH:0] Y [0:ITER];
    reg signed [31:0]    Z [0:ITER];

    integer i;
    always @(posedge clk) begin
        // Initialize pipeline
        X[0] <= Xin;
        Y[0] <= Yin;
        Z[0] <= 0;

        // Iterative rotations
        for (i = 0; i < ITER; i = i + 1) begin
            if (Y[i][WIDTH]) begin // Y[i] < 0: rotate clockwise
                X[i+1] <= X[i] - (Y[i] >>> i);
                Y[i+1] <= Y[i] + (X[i] >>> i);
                Z[i+1] <= Z[i] - atan_table[i];
            end else begin         // Y[i] >= 0: rotate counter-clockwise
                X[i+1] <= X[i] + (Y[i] >>> i);
                Y[i+1] <= Y[i] - (X[i] >>> i);
                Z[i+1] <= Z[i] + atan_table[i];
            end
        end

        // Final outputs
        Xout <= X[ITER];
        Yout <= Y[ITER];
        Zout <= Z[ITER];
    end

endmodule


module sqrt_lut (
    input  signed [15:0] x_in,  // Q1.15 format (x ∈ [-1, 1))
    output signed [15:0] y_out  // Q1.15 format (y = sqrt(1 - x²))
);
    reg [15:0] lut [0:32767]; // LUT for x ∈ [0, 1)
    initial begin
        for (integer i = 0; i < 32768; i = i + 1) begin
            real x = i / 32768.0;
            real y = $sqrt(1.0 - x * x);
            lut[i] = y * 32768.0; // Convert to Q1.15
        end
    end
    // Handle negative x using symmetry
    wire [14:0] x_abs = x_in[15] ? (-x_in[14:0]) : x_in[14:0];
    assign y_out = lut[x_abs];
endmodule



module arccos (
    input  wire clk,
    input  wire signed [15:0] x_in,    // Q1.15 input (x ∈ [-1, 1))
    output wire signed [31:0] angle    // arccos(x) in CORDIC's angle format
);
    // Compute Y = sqrt(1 - x²)
    wire signed [15:0] y_sqrt;
    sqrt_lut sqrt_inst (
        .x_in(x_in),
        .y_out(y_sqrt)
    );

    // CORDIC in vectoring mode (computes arctan(Y/X))
    wire signed [31:0] z_raw;
    cordic_vectoring #(
        .WIDTH(16),
        .ITER(16)
    ) cordic (
        .clk(clk),
        .Xin(x_in),
        .Yin(y_sqrt),
        .Xout(), // Unused
        .Yout(), // Unused
        .Zout(z_raw)
    );

    // Adjust angle for x < 0 (add π in fixed-point)
    assign angle = x_in[15] ? (32'h80000000 + z_raw) : z_raw;
endmodule



TestBench 

module tb_arccos();
    reg clk;
    reg signed [15:0] x_in;
    wire signed [31:0] angle;

    arccos uut (
        .clk(clk),
        .x_in(x_in),
        .angle(angle)
    );

    // Clock generation (100 MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        // Test case 1: x = 0.5 (arccos(0.5) = 60° or π/3 radians)
        x_in = 16'sb0010000000000000; // 0.5 in Q1.15
        #100;
        $display("arccos(0.5) = %h (expected ~0x2AAAAAAA)", angle);

        // Test case 2: x = -0.5 (arccos(-0.5) = 120° or 2π/3 radians)
        x_in = 16'sb1110000000000000; // -0.5 in Q1.15
        #100;
        $display("arccos(-0.5) = %h (expected ~0x55555555)", angle);

        $finish;
    end
endmodule


