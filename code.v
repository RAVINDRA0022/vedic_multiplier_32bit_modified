module vedic_multiplier(
    input [31:0] A,
    input [31:0] B,
    output [63:0] S
);
    wire [31:0] m0, m1, m2, m3;
    wire [31:0] sum_part1, sum_part2, carry_part1, carry_part2;
    wire [15:0] or_result;
    wire [15:0] adder_out;
    
    // Call 16-bit Vedic multipliers for the partial products
    vedic_multiplier_16bit V0 (A[15:0], B[15:0], m0);
    vedic_multiplier_16bit V1 (A[31:16], B[15:0], m1);
    vedic_multiplier_16bit V2 (A[15:0], B[31:16], m2);
    vedic_multiplier_16bit V3 (A[31:16], B[31:16], m3);

    // Perform carry-save additions on partial products
    csa_32_bit CSA1 (m2, m1, m0[31:16], sum_part1, carry_part1);
    csa_32_bit CSA2 (m3, sum_part1, carry_part1 << 1, sum_part2, carry_part2);

    // OR operation on the lower 16 bits and the higher 16 bits addition
    assign or_result = m0[15:0] | sum_part2[15:0];

    // Final addition of the result
    adder_16bit ADDER (or_result, sum_part2[31:16], adder_out);

    // Assign final output
    assign S[15:0] = m0[15:0];
    assign S[47:16] = sum_part2[31:0];
    assign S[63:48] = adder_out;
endmodule

module vedic_multiplier_16bit(
    input [15:0] A,
    input [15:0] B,
    output [31:0] P
);
    wire [15:0] p1, p2, p3, p4;
    wire [31:0] sum_part1, sum_part2;
    wire [31:0] carry_part1, carry_part2;
    
    // Split inputs into lower and upper 8 bits
    wire [7:0] A0 = A[7:0];
    wire [7:0] A1 = A[15:8];
    wire [7:0] B0 = B[7:0];
    wire [7:0] B1 = B[15:8];

    // Perform 8-bit Vedic multiplications for partial products
    vedic_8bit v0 (A0, B0, p1);
    vedic_8bit v1 (A1, B0, p2);
    vedic_8bit v2 (A0, B1, p3);
    vedic_8bit v3 (A1, B1, p4);

    // Carry-save additions
    csa_32_bit csa1 ({16'b0, p3}, {16'b0, p2}, {8'b0, p1[15:8], 8'b0}, sum_part1, carry_part1);
    csa_32_bit csa2 ({p4, 16'b0}, sum_part1, carry_part1 << 1, sum_part2, carry_part2);

    // Assign the final product
    assign P[15:0] = p1[15:0];
    assign P[31:16] = sum_part2[31:16];
endmodule

module vedic_8bit(
    input [7:0] A, B,
    output [15:0] P
);
    wire [7:0] p1, p2, p3, p4;
    wire [15:0] sum_part1, sum_part2;
    wire [15:0] carry_part1, carry_part2;

    // Split the 8-bit inputs into 4-bit parts
    wire [3:0] A0 = A[3:0];
    wire [3:0] A1 = A[7:4];
    wire [3:0] B0 = B[3:0];
    wire [3:0] B1 = B[7:4];

    // Perform 4-bit Vedic multiplications
    vedic_4bit v0 (A0, B0, p1);
    vedic_4bit v1 (A1, B0, p2);
    vedic_4bit v2 (A0, B1, p3);
    vedic_4bit v3 (A1, B1, p4);

    // Assign the final product
    assign P[7:0] = p1[7:0];
    assign P[15:8] = sum_part2[15:8];
endmodule

module vedic_4bit(
    input [3:0] A, B,
    output [7:0] P
);
    // 4-bit multiplication directly using bitwise operations
    assign P = A * B;
endmodule

module csa_32_bit (
    input [31:0] X, Y, Z, 
    input CIN,              // Carry-in bit
    output [31:0] S,       // Sum
    output COUT            // Carry-out
);
    wire [31:0] sum_part, carry_part;
    wire [31:0] mux_sum, mux_carry;

    // Full adder to calculate sum and carry
    full_adder_32bit FA32 (
        .A(X),
        .B(Y),
        .Cin(Z),
        .Sum(sum_part),
        .Cout(carry_part)
    );

    mux2x1 MUX_SUM (
        .d0(sum_part),  
        .d1(sum_part),  
        .sel(CIN),    
        .out(mux_sum)
    );

    mux2x1 MUX_CARRY (
        .d0(carry_part), 
        .d1(carry_part),  
        .sel(CIN),     
        .out(mux_carry) 
    );

    assign S = mux_sum;     
    assign COUT = mux_carry[31];
endmodule

module full_adder_32bit (
    input [31:0] A, B, Cin,  
    output [31:0] Sum,     
    output [31:0] Cout      
);
    wire [31:0] sum_part, carry_part; 

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : full_adder_32_block
            full_adder FA (
                .a(A[i]), 
                .b(B[i]), 
                .cin(Cin), 
                .sum(sum_part[i]), 
                .cout(carry_part[i])
            );
        end
    endgenerate

    // Assign sum and carry for the 32-bit full adder
    assign Sum = sum_part;  
    assign Cout = carry_part; 
endmodule

module full_adder (
    input a, b, cin,
    output sum, cout
);
    assign sum = a ^ b ^ cin;
    assign cout = (a & b) | (b & cin) | (cin & a);
endmodule

module mux2x1 (
    input [31:0] d0, d1,   
    input sel,           
    output [31:0] out       
);
    assign out = sel ? d1 : d0; 
endmodule

module adder_16bit (
    input [15:0] A,
    input [15:0] B,
    output [15:0] Sum
);
    assign Sum = A + B;
endmodule
