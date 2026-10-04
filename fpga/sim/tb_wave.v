// tb_wave.v - replays a command script through kf_cpu and exposes inputs / outputs as real
// numbers so a waveform viewer can draw them as analog traces (see model/make_wave.py).
//   vsim work.tb_wave +script=wave_script.txt
`timescale 1ns / 1ps
module tb_wave;
    localparam W = 56, F = 40;
    `include "kf_regs.vh"

    reg clk = 1'b0, rst = 1'b1;
    always #10 clk = ~clk;                               // 50 MHz

    reg          host_we = 0, start = 0;
    reg  [9:0]   host_waddr = 0, host_raddr = 0;
    reg  [W-1:0] host_wdata = 0;
    reg  [2:0]   cmd = 0;
    wire [W-1:0] host_rdata;
    wire         busy, done;

    kf_cpu #(.W(W), .F(F)) dut (
        .clk(clk), .rst(rst),
        .host_we(host_we), .host_waddr(host_waddr), .host_wdata(host_wdata),
        .host_raddr(host_raddr), .host_rdata(host_rdata),
        .start(start), .cmd(cmd), .busy(busy), .done(done));

    // ---- readable views of the register file ------------------------------------
    real in_dt, in_ax, in_ay, in_gyro_z, in_flow_vx, in_flow_vy, in_flow_psr, in_gnss_speed;
    real out_vx, out_vy, out_bias_x, out_bias_y, out_sigma_vx, out_sigma_vy, out_status;
    reg [63:0] cmd_name;
    localparam real SC = 1099511627776.0;                // 2**40
    function real q(input signed [W-1:0] v); q = $itor(v) / SC; endfunction
    always @(posedge clk) begin
        in_dt = q(dut.rfa[R_DT]);   in_ax = q(dut.rfa[R_AX]);  in_ay = q(dut.rfa[R_AY]);
        in_gyro_z = q(dut.rfa[R_RZ]);
        in_flow_vx = q(dut.rfa[R_FVX]); in_flow_vy = q(dut.rfa[R_FVY]); in_flow_psr = q(dut.rfa[R_FQ]);
        in_gnss_speed = q(dut.rfa[R_GS]);
        out_vx = q(dut.rfa[R_X0]);  out_vy = q(dut.rfa[R_X1]);
        out_bias_x = q(dut.rfa[R_X2]); out_bias_y = q(dut.rfa[R_X3]);
        out_sigma_vx = $sqrt(q(dut.rfa[R_P00])); out_sigma_vy = $sqrt(q(dut.rfa[R_P11]));
        out_status = q(dut.rfa[R_STATUS]);               // 0 accepted, 1 gated, 2 skipped
    end
    wire [5:0] opcode = dut.op;
    wire [9:0] pc = dut.pc;

    // ---- reference: the float Python filter (kf_ref.py) after the same command ----------
    // ref_script lines: vx vy bias_x bias_y sigma_vx sigma_vy, one per command.
    real ref_vx, ref_vy, ref_bias_x, ref_bias_y, ref_sigma_vx, ref_sigma_vy;
    real err_vx, err_vy, err_sigma_vx;                   // FPGA minus Python, sampled after each command
    real max_err_vx, max_err_vy, max_err_sigma;
    integer fr, rcode, have_ref, fo;
    reg [7:0] verb;
    reg [W-1:0] v1, v2;
    integer fd, code, ncmd;
    reg [1023:0] fname, rname;
    function real absr(input real x); absr = (x < 0.0) ? -x : x; endfunction

    initial begin
        if (!$value$plusargs("script=%s", fname)) fname = "wave_script.txt";
        fd = $fopen(fname, "r");
        if (!$value$plusargs("ref=%s", rname)) rname = "ref_script.txt";
        fr = $fopen(rname, "r");
        have_ref = (fr != 0);
        fo = $fopen("fpga_waveform.csv", "w");
        $fdisplay(fo, "n,time_ps,command,in_ax,in_ay,in_gyro_z,in_flow_vx,in_flow_psr,v_x,v_y,bias_x,sigma_vx,status");
        max_err_vx = 0.0; max_err_vy = 0.0; max_err_sigma = 0.0;
        err_vx = 0.0; err_vy = 0.0; err_sigma_vx = 0.0;
        ref_vx = 0.0; ref_vy = 0.0; ref_bias_x = 0.0; ref_bias_y = 0.0; ref_sigma_vx = 0.0; ref_sigma_vy = 0.0;
        ncmd = 0;
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        repeat (4) @(posedge clk);
        code = 3;
        while (code == 3) begin
            code = $fscanf(fd, " %c %h %h", verb, v1, v2);
            if (code == 3) case (verb)
            "w": begin
                host_waddr <= v1[9:0]; host_wdata <= v2; host_we <= 1'b1;
                @(posedge clk); host_we <= 1'b0; @(posedge clk);
            end
            "c": begin
                case (v1[2:0])
                    0: cmd_name = "RESET";   1: cmd_name = "PREDICT"; 2: cmd_name = "FLOW";
                    3: cmd_name = "GNSS";    default: cmd_name = "ZUPT";
                endcase
                cmd <= v1[2:0]; start <= 1'b1;
                @(posedge clk); start <= 1'b0;
                while (!done) @(posedge clk);
                repeat (2) @(posedge clk);
                // the waveform as numbers: one row per command, what the viewer draws
                $fdisplay(fo, "%0d,%0t,%0s,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%0d",
                          ncmd, $time, cmd_name, q(dut.rfa[R_AX]), q(dut.rfa[R_AY]), q(dut.rfa[R_RZ]),
                          q(dut.rfa[R_FVX]), q(dut.rfa[R_FQ]), q(dut.rfa[R_X0]), q(dut.rfa[R_X1]),
                          q(dut.rfa[R_X2]), $sqrt(q(dut.rfa[R_P00])), $rtoi(q(dut.rfa[R_STATUS])));
                if (have_ref) begin
                    rcode = $fscanf(fr, " %f %f %f %f %f %f", ref_vx, ref_vy, ref_bias_x, ref_bias_y,
                                    ref_sigma_vx, ref_sigma_vy);
                    if (rcode == 6) begin
                        err_vx = q(dut.rfa[R_X0]) - ref_vx;
                        err_vy = q(dut.rfa[R_X1]) - ref_vy;
                        err_sigma_vx = $sqrt(q(dut.rfa[R_P00])) - ref_sigma_vx;
                        if (ncmd > 0) begin              // command 0 is the seed
                            if (absr(err_vx) > max_err_vx) max_err_vx = absr(err_vx);
                            if (absr(err_vy) > max_err_vy) max_err_vy = absr(err_vy);
                            if (absr(err_sigma_vx) > max_err_sigma) max_err_sigma = absr(err_sigma_vx);
                        end
                    end
                end
                repeat (28) @(posedge clk);              // gap so the traces are easy to read
                ncmd = ncmd + 1;
            end
            default: ;                                   // reads are not needed here
            endcase
        end
        $fclose(fo);
        $display("replayed %0d commands", ncmd);
        if (have_ref) begin
            $display("FPGA vs Python (kf_ref.py) over all %0d commands:", ncmd);
            $display("  max |v_x error|     = %g m/s", max_err_vx);
            $display("  max |v_y error|     = %g m/s", max_err_vy);
            $display("  max |sigma_vx error|= %g m/s", max_err_sigma);
        end
        $stop;
    end
endmodule
