// kf_cpu.v - the Kalman filter coprocessor: a small fixed-point processor.
//
// The filter itself is the microprogram in kf_rom.v (generated from model/kf_isa.py, which
// also holds a bit-exact Python model of this file). The processor has:
//   * a 1024 x W register file, two read ports (two RAM copies written together)
//   * one W x W multiplier feeding a 2W+8 bit accumulator (MAC / MACN / STA)
//   * a restoring divider (2 quotient bits per clock) and a bit-serial square root
//   * compare-and-branch and one link register (CALL / RET)
// Numbers are signed Q(W-F).F. Every operation saturates instead of wrapping.
//
// An instruction takes 3 clocks (read operands, execute, write back); branches, CLR and
// NOP take 2; DIV and SQRT add (W+F)/2 clocks.
//
// Host port: while `busy` is low the host may write any register (host_we) and read any
// register (host_raddr -> host_rdata, valid two clocks later). To run a command put its
// index on `cmd` and pulse `start`; `busy` stays high until the program reaches HALT, then
// `done` pulses for one clock. The host writes inputs and constants, starts a command and
// reads the outputs. Register addresses: `python model/kf_isa.py --map`.
module kf_cpu #(
    parameter W = 56,
    parameter F = 40
) (
    input  wire         clk,
    input  wire         rst,            // synchronous, active high

    input  wire         host_we,
    input  wire [9:0]   host_waddr,
    input  wire [W-1:0] host_wdata,
    input  wire [9:0]   host_raddr,
    output wire [W-1:0] host_rdata,

    input  wire         start,
    input  wire [2:0]   cmd,
    output reg          busy,
    output reg          done
);
    `include "kf_isa.vh"

    localparam AW = 2 * W + 8;           // accumulator width
    localparam NW = W + F;               // dividend / radicand width

    localparam signed [W-1:0]  MAXV = {1'b0, {(W-1){1'b1}}};
    localparam signed [W-1:0]  MINV = {1'b1, {(W-1){1'b0}}};
    localparam signed [W:0]    MAXE = MAXV;       // sign-extended copies for comparisons
    localparam signed [W:0]    MINE = MINV;
    localparam signed [AW-1:0] MAXA = MAXV;
    localparam signed [AW-1:0] MINA = MINV;
    localparam signed [2*W:0]  MAXP = MAXV;
    localparam signed [2*W:0]  MINP = MINV;
    localparam [AW-1:0]        HALF_A = {{(AW-1){1'b0}}, 1'b1} << (F - 1);
    localparam [2*W:0]         HALF_P = {{(2*W){1'b0}}, 1'b1} << (F - 1);

    localparam [2:0] S_IDLE = 0, S_RD = 1, S_EX = 2, S_WB = 3, S_DIV = 4, S_SQRT = 5, S_FIN = 6;
    reg [2:0] state;

    // ---- program memory (combinational ROM) ----------------------------------
    reg  [9:0]  pc, lr;
    wire [35:0] instr;
    kf_rom u_rom (.addr(pc), .data(instr));
    wire [5:0] op = instr[35:30];
    wire [9:0] fd = instr[29:20], fa = instr[19:10], fb = instr[9:0];

    // ---- register file: two copies so both operands are read in one clock -------
    reg  [W-1:0] rfa [0:1023];
    reg  [W-1:0] rfb [0:1023];
    reg  signed [W-1:0] rda, rdb;
    // synthesis translate_off
    integer ri;
    initial for (ri = 0; ri < 1024; ri = ri + 1) begin rfa[ri] = 0; rfb[ri] = 0; end
    // synthesis translate_on

    reg  [5:0]           op_q;
    reg  [9:0]           fd_q;
    reg signed [W:0]     alu;                    // ADD / SUB / AVG / MAX / MOV (one extra bit)
    reg signed [2*W-1:0] prod;
    reg signed [AW-1:0]  acc;
    reg [W-1:0]          divres;

    wire signed [W:0] ea = {rda[W-1], rda};
    wire signed [W:0] eb = {rdb[W-1], rdb};

    function [W-1:0] sat_alu(input signed [W:0] v);
        sat_alu = (v > MAXE) ? MAXV : (v < MINE) ? MINV : v[W-1:0];
    endfunction

    // round Q.2F back to Q.F, then saturate
    wire signed [AW-1:0]  acc_s   = (acc + $signed(HALF_A)) >>> F;
    wire [W-1:0]          sta_val = (acc_s > MAXA) ? MAXV : (acc_s < MINA) ? MINV : acc_s[W-1:0];
    wire signed [2*W:0]   prod_e  = {prod[2*W-1], prod};           // sign-extended
    wire signed [2*W:0]   prod_s  = (prod_e + $signed(HALF_P)) >>> F;
    wire [W-1:0]          mul_val = (prod_s > MAXP) ? MAXV : (prod_s < MINP) ? MINV : prod_s[W-1:0];

    // register-file write port: the host while idle, the datapath in S_WB
    reg                   wb_en;
    reg  [W-1:0]          wb_data;
    always @* begin
        wb_en = 1'b0; wb_data = {W{1'b0}};
        case (op_q)
        OP_ADD, OP_SUB, OP_AVG, OP_MAX, OP_MOV: begin wb_en = 1'b1; wb_data = sat_alu(alu); end
        OP_MUL:          begin wb_en = 1'b1; wb_data = mul_val; end
        OP_DIV, OP_SQRT: begin wb_en = 1'b1; wb_data = divres; end
        OP_STA:          begin wb_en = 1'b1; wb_data = sta_val; end
        default: ;
        endcase
    end
    wire          we_i    = (state == S_IDLE) ? host_we : (state == S_WB && wb_en);
    wire [9:0]    wa_i    = (state == S_IDLE) ? host_waddr : fd_q;
    wire [W-1:0]  wd_i    = (state == S_IDLE) ? host_wdata : wb_data;
    wire [9:0]    addr_a  = (state == S_IDLE) ? host_raddr : fa;
    always @(posedge clk) begin
        if (we_i) begin rfa[wa_i] <= wd_i; rfb[wa_i] <= wd_i; end
        rda <= rfa[addr_a];
        rdb <= rfb[fb];
    end
    assign host_rdata = rda;

    // ---- divider (2 bits / clock) and square root --------------------------------
    reg [NW-1:0] nreg;                           // dividend / radicand, shifts left
    reg [NW-1:0] qreg;                           // quotient / root
    reg [W:0]    rem;
    reg [W-1:0]  den;
    reg          neg;
    reg [7:0]    cnt;
    wire [W:0]   r1s  = {rem[W-1:0], nreg[NW-1]};
    wire         ge1  = (r1s >= {1'b0, den});
    wire [W:0]   r1   = ge1 ? r1s - {1'b0, den} : r1s;
    wire [W:0]   r2s  = {r1[W-1:0], nreg[NW-2]};
    wire         ge2  = (r2s >= {1'b0, den});
    wire [W:0]   r2   = ge2 ? r2s - {1'b0, den} : r2s;

    reg  [W+3:0] sr;
    wire [W+3:0] sr_sh  = {sr[W+1:0], nreg[NW-1:NW-2]};
    wire [W+3:0] trial  = {qreg[W+1:0], 2'b01};
    wire         sr_ge  = (sr_sh >= trial);

    always @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            state <= S_IDLE; busy <= 1'b0; pc <= 10'd0; lr <= 10'd0; acc <= 0;
        end else case (state)
        S_IDLE: if (start) begin pc <= {7'b0, cmd}; busy <= 1'b1; state <= S_RD; end
        S_RD: begin                                  // rda / rdb load from the instruction's a, b
            op_q <= op; fd_q <= fd; state <= S_EX;
        end
        S_EX: begin                                  // rda, rdb = operands
            state <= S_WB;
            pc <= pc + 10'd1;
            case (op_q)
            OP_ADD: alu <= ea + eb;
            OP_SUB: alu <= ea - eb;
            OP_AVG: alu <= (ea + eb) >>> 1;
            OP_MAX: alu <= (rda > rdb) ? ea : eb;
            OP_MOV: alu <= ea;
            OP_MUL, OP_MAC, OP_MACN: prod <= rda * rdb;
            OP_DIV: begin
                neg  <= rda[W-1] ^ rdb[W-1];
                nreg <= {(rda[W-1] ? -rda : rda), {F{1'b0}}};
                den  <= rdb[W-1] ? -rdb : rdb;
                rem  <= 0; qreg <= 0; cnt <= NW / 2;
                if (rdb == 0) begin                 // x/0 saturates by the sign of x; 0/0 = 0
                    divres <= (rda == 0) ? {W{1'b0}} : rda[W-1] ? MINV : MAXV;
                    state <= S_WB;
                end else state <= S_DIV;
            end
            OP_SQRT: begin
                nreg <= {(rda[W-1] ? {W{1'b0}} : rda), {F{1'b0}}};
                sr <= 0; qreg <= 0; cnt <= NW / 2;
                state <= S_SQRT;
            end
            OP_JMP:  begin pc <= fd_q; state <= S_RD; end
            OP_CALL: begin lr <= pc + 10'd1; pc <= fd_q; state <= S_RD; end
            OP_RET:  begin pc <= lr; state <= S_RD; end
            OP_BLT:  begin pc <= (rda <  rdb) ? fd_q : pc + 10'd1; state <= S_RD; end
            OP_BGE:  begin pc <= (rda >= rdb) ? fd_q : pc + 10'd1; state <= S_RD; end
            OP_CLR:  begin acc <= 0; state <= S_RD; end
            OP_HALT: begin busy <= 1'b0; done <= 1'b1; state <= S_IDLE; end
            OP_STA:  ;
            default: state <= S_RD;                  // NOP
            endcase
        end
        S_WB: begin
            state <= S_RD;
            case (op_q)
            OP_MAC:  acc <= acc + prod;
            OP_MACN: acc <= acc - prod;
            default: ;
            endcase
        end
        S_DIV: begin
            nreg <= {nreg[NW-3:0], 2'b00};
            rem  <= r2;
            qreg <= {qreg[NW-3:0], ge1, ge2};
            cnt  <= cnt - 8'd1;
            if (cnt == 8'd1) state <= S_FIN;
        end
        S_SQRT: begin
            nreg <= {nreg[NW-3:0], 2'b00};
            sr   <= sr_ge ? sr_sh - trial : sr_sh;
            qreg <= {qreg[NW-2:0], sr_ge};
            cnt  <= cnt - 8'd1;
            if (cnt == 8'd1) state <= S_FIN;
        end
        S_FIN: begin                                 // quotient / root is final in qreg
            state <= S_WB;
            if (op_q == OP_SQRT)                     divres <= qreg[W-1:0];
            else if (qreg[NW-1:W-1] != 0)            divres <= neg ? MINV : MAXV;   // does not fit
            else                                     divres <= neg ? -qreg[W-1:0] : qreg[W-1:0];
        end
        default: state <= S_IDLE;
        endcase
    end
endmodule
