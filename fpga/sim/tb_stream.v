// tb_stream.v - the simulated FPGA: kf_cpu driven live over stdin / stdout.
//
// The dashboard and the replay tools (model/kf_rtl.py) start this simulation as a child
// process and talk to it one line at a time: Python is the host CPU, this is the board.
//
// stdin, one request per line, "<verb> <hex> <hex>":
//     w ADDR DATA     write register ADDR (hex) with DATA (hex, Q16.40)
//     c CMD 0         run command CMD (0 RESET 1 PREDICT 2 FLOW 3 GNSS 4 ZUPT), wait for done
//     r ADDR 0        read register ADDR
// stdout:
//     c -> "D <clocks the command took>"      r -> "R <hex data>"      w -> nothing
// End of input ends the simulation.
//
//   iverilog -g2005 -I ../rtl -o tb_stream.vvp tb_stream.v ../rtl/*.v && vvp -n tb_stream.vvp
`timescale 1ns / 1ps
module tb_stream;
    localparam W = 56, F = 40;
    localparam [31:0] STDIN = 32'h8000_0000;

    reg clk = 1'b0, rst = 1'b1;
    always #10 clk = ~clk;                              // 50 MHz

    reg          host_we = 0, start = 0;
    reg  [9:0]   host_waddr = 0, host_raddr = 0;
    reg  [W-1:0] host_wdata = 0;
    reg  [2:0]   cmd = 0;
    wire [W-1:0] host_rdata;
    wire         busy, done;

`ifdef GATE_LEVEL
    kf_cpu dut (                                        // Quartus' fitted netlist has no parameters
`else
    kf_cpu #(.W(W), .F(F)) dut (
`endif
        .clk(clk), .rst(rst),
        .host_we(host_we), .host_waddr(host_waddr), .host_wdata(host_wdata),
        .host_raddr(host_raddr), .host_rdata(host_rdata),
        .start(start), .cmd(cmd), .busy(busy), .done(done));

    reg [7:0]    verb;
    reg [W-1:0]  v1, v2;
    integer      code, cyc;

    initial begin
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        repeat (2) @(posedge clk);
        forever begin
            // leading blank skips the previous newline; no trailing "\n" (it would block
            // waiting for the first character of the next line)
            code = $fscanf(STDIN, " %c %h %h", verb, v1, v2);
            if (code < 0) $finish;                  // end of input
            if (code == 3) begin
                case (verb)
                "w": begin
                    host_waddr <= v1[9:0]; host_wdata <= v2; host_we <= 1'b1;
                    @(posedge clk); host_we <= 1'b0; @(posedge clk);
                end
                "r": begin
                    host_raddr <= v1[9:0];
                    repeat (3) @(posedge clk);
                    $display("R %h", host_rdata);
                    $fflush();
                end
                "c": begin
                    cmd <= v1[2:0]; start <= 1'b1; cyc = 0;
                    @(posedge clk); start <= 1'b0;
                    while (!done) begin @(posedge clk); cyc = cyc + 1; end
                    @(posedge clk);
                    $display("D %0d", cyc + 1);
                    $fflush();
                end
                default: ;
                endcase
            end
        end
        $finish;
    end
endmodule
