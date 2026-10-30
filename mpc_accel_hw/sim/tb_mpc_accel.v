// Testbench: drives mpc_accel_axi only through AXI4-Lite (as the PS will) and checks g[0..19], z[0..19]
// bit-exactly against the golden-model test vectors (mem/<fmt>/tv.hex, made by gen_hw_files.py).
//
// Compile with -I mem/<fmt> (config.vh) and -DMEM_DIR=\"mem/<fmt>/\". Plusargs:
//   +NVEC=n     number of vectors to run (default all 4000)
//   +STRIDE=s   run every s-th vector (default 1)
//   +SKIP=k     start at vector k (split a regression into parallel chunks)
//   +VCD        dump waves for the first vector
`timescale 1ns / 1ps
`include "config.vh"

module tb_mpc_accel;
    localparam NIN = 92, NL = 20, NTOT = NIN + 2 * NL, NVEC_MAX = 4000;

    reg clk = 1'b0;
    reg aresetn = 1'b0;
    always #5 clk = ~clk;                        // 100 MHz

    reg  [11:0] awaddr, araddr;
    reg         awvalid, wvalid, bready, arvalid, rready;
    reg  [31:0] wdata;
    wire        awready, wready, bvalid, arready, rvalid, irq;
    wire [1:0]  bresp, rresp;
    wire [31:0] rdata;

    mpc_accel_axi #(
        .FRAC_MX(`FRAC_MX), .FRAC_MR(`FRAC_MR), .FRAC_W(`FRAC_W), .FRAC_P(`FRAC_P),
        .N_ITER_DEFAULT(`N_ITER_DEFAULT),
        .COEF_FILE({`MEM_DIR, "coef_col.mem"}), .ZMIN_FILE({`MEM_DIR, "zmin.mem"}),
        .ZMAX_FILE({`MEM_DIR, "zmax.mem"}), .PARAM_FILE({`MEM_DIR, "params.mem"})
    ) dut (
        .s_axi_aclk(clk), .s_axi_aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'b000), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'b000), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .irq(irq)
    );

    // ---------------------------------------------------------------- AXI4-Lite master
    task axi_write(input [11:0] addr, input [31:0] data);
        reg aw_done, w_done;
        begin
            @(posedge clk);
            awaddr <= addr; awvalid <= 1'b1;
            wdata  <= data; wvalid  <= 1'b1;
            aw_done = 1'b0; w_done = 1'b0;
            while (!(aw_done && w_done)) begin
                @(posedge clk);
                if (awready && awvalid) begin aw_done = 1'b1; awvalid <= 1'b0; end
                if (wready && wvalid)   begin w_done  = 1'b1; wvalid  <= 1'b0; end
            end
            bready <= 1'b1;
            @(posedge clk);
            while (!bvalid) @(posedge clk);
            bready <= 1'b0;
        end
    endtask

    task axi_read(input [11:0] addr, output [31:0] data);
        begin
            @(posedge clk);
            araddr <= addr; arvalid <= 1'b1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            arvalid <= 1'b0;
            rready  <= 1'b1;
            @(posedge clk);
            while (!rvalid) @(posedge clk);
            data = rdata;
            rready <= 1'b0;
        end
    endtask

    // ---------------------------------------------------------------- test
    reg [31:0] vec [0:NTOT-1];
    reg [31:0] rd, status;
    integer fd, k, n, i, nvec, stride, skip, ran, pass, bad_g, bad_z, cyc_min, cyc_max;
    integer first_fail;
    real    cyc_sum;

    initial begin
        awvalid = 0; wvalid = 0; bready = 0; arvalid = 0; rready = 0;
        awaddr = 0; araddr = 0; wdata = 0;
        if (!$value$plusargs("NVEC=%d", nvec)) nvec = NVEC_MAX;
        if (!$value$plusargs("STRIDE=%d", stride)) stride = 1;
        if (!$value$plusargs("SKIP=%d", skip)) skip = 0;
        if ($test$plusargs("VCD")) begin
            $dumpfile("tb_mpc_accel.vcd");
            $dumpvars(0, tb_mpc_accel);
        end

        repeat (5) @(posedge clk);
        aresetn = 1'b1;
        repeat (2) @(posedge clk);

        // register sanity
        axi_read(12'h010, rd);
        if (rd !== 32'h4D504331) begin $display("FAIL ID = %h", rd); $finish; end
        axi_read(12'h014, rd);
        $display("ID ok, CONFIG = %h (FRAC_W %0d, FRAC_MX %0d, FRAC_MR %0d, FRAC_P %0d)",
                 rd, rd[7:0], rd[15:8], rd[23:16], rd[31:24]);
        axi_read(12'h008, rd);
        $display("N_ITER = %0d", rd);
        axi_write(12'h400 + 4 * 91, 32'hDEADBEEF);
        axi_read(12'h400 + 4 * 91, rd);
        if (rd !== 32'hDEADBEEF) begin $display("FAIL input buffer readback %h", rd); $finish; end
        axi_write(12'h000, 32'h2);             // IRQ_EN

        fd = $fopen({`MEM_DIR, "tv.hex"}, "r");
        if (fd == 0) begin $display("FAIL cannot open tv.hex"); $finish; end

        ran = 0; pass = 0; first_fail = -1;
        cyc_min = 32'h7FFFFFFF; cyc_max = 0; cyc_sum = 0.0;
        for (k = 0; k < NVEC_MAX && ran < nvec; k = k + 1) begin
            for (i = 0; i < NTOT; i = i + 1) n = $fscanf(fd, "%h", vec[i]);
            if (n != 1) k = NVEC_MAX;
            else if (k >= skip && k % stride == 0) begin
                for (i = 0; i < NIN; i = i + 1) axi_write(12'h400 + 4 * i, vec[i]);
                axi_write(12'h000, 32'h3);         // START + IRQ_EN
                @(posedge clk);
                while (!irq) @(posedge clk);
                axi_read(12'h004, status);
                axi_read(12'h00C, rd);
                if (rd < cyc_min) cyc_min = rd;
                if (rd > cyc_max) cyc_max = rd;
                cyc_sum = cyc_sum + rd;

                bad_g = 0; bad_z = 0;
                for (i = 0; i < NL; i = i + 1) begin
                    axi_read(12'h900 + 4 * i, rd);
                    if (rd !== vec[NIN + i]) bad_g = bad_g + 1;
                    axi_read(12'h800 + 4 * i, rd);
                    if (rd !== vec[NIN + NL + i]) begin
                        bad_z = bad_z + 1;
                        if (first_fail < 0)
                            $display("  vec %0d z[%0d]: got %h exp %h", k, i, rd, vec[NIN + NL + i]);
                    end
                end
                if (bad_g == 0 && bad_z == 0 && status[2] == 1'b0) pass = pass + 1;
                else begin
                    if (first_fail < 0) first_fail = k;
                    $display("MISMATCH vec %0d: %0d g, %0d z wrong, SAT=%0d", k, bad_g, bad_z, status[2]);
                end
                ran = ran + 1;
                if (ran % 100 == 0) begin
                    $display("  %0d vectors, %0d pass", ran, pass);
                    $fflush;
                end
            end
        end
        $fclose(fd);

        $display("==================================================");
        $display("%0d/%0d vectors bit-exact (g and z), first fail %0d", pass, ran, first_fail);
        $display("latency: min %0d, max %0d, avg %0.1f cycles = %0.2f us @ 100 MHz",
                 cyc_min, cyc_max, cyc_sum / ran, cyc_sum / ran / 100.0);
        if (pass == ran) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("==================================================");
        $finish;
    end
endmodule
