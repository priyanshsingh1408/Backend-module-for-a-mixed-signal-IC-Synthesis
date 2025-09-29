`timescale 1ns / 1ps
`include "FPGA_model_updated.v"
//`include "backend.v"

module backend_tb();

    reg resetbFPGA;
    reg main_clk;
    reg ro_clockmodel;
    reg [3:0] adc_out;
    
    wire resetbAll, resetb_amp, resetb_core;
    wire [2:0] gain;
    wire sclk, sdin;
    wire ready, enableRO, Ibias_2x;
    wire ro_clk, core_clk;
    
    // FPGA model instantiation
    FPGA_model_updated FPGA_obj(
        .i_resetbFPGA(resetbFPGA),
        .i_ready(ready),
        .i_mainclk(main_clk),
        .o_resetbAll(resetbAll),
        .o_sclk(sclk),
        .o_sdout(sdin)
    );
    
    // Backend instantiation
    backend backend_obj(
        .i_resetbAll(resetbAll),
        .i_clk(main_clk),
        .i_sclk(sclk),
        .i_sdin(sdin),
        .i_RO_clk(ro_clk),
        .i_ADCout(adc_out),
        .o_ready(ready),
        .o_resetb_amp(resetb_amp),
        .o_gain(gain),
        .o_Ibias_2x(Ibias_2x),
        .o_enableRO(enableRO),
        .o_resetb_core(resetb_core),
        .o_core_clk(core_clk)
    );
    
    // RO clock model
    assign ro_clk = (enableRO) ? ro_clockmodel : 0;
    
    // Test signal generation
    initial begin
        $dumpfile("backend_tb.vcd");
        $dumpvars(0, backend_tb);
        
        resetbFPGA <= 0;
        main_clk <= 0;
        ro_clockmodel <= 0;
        adc_out <= 4'd10; // Initial ADC value
        
        #4 resetbFPGA <= 1;   //this means delay of 4 time units mentioned in the timescale units
        
        // Test ADC value changes
        #250 adc_out <= 4'd14; // Test high temperature case
        #200 adc_out <= 4'd6;  // Test low temperature case
        #200 adc_out <= 4'd10; // Back to medium temperature
        
        #3110 $finish;
    end
    
    // Reset when ready is detected
    always begin
        wait(ready === 1)
        begin
            #400
            resetbFPGA <= 0;
            #5 resetbFPGA <= 1;
        end
    end
    
    // Generation of main_clk (500 MHz => 2ns period)
    always #1 main_clk <= ~main_clk;
    
    // Generation of RO clock (800 MHz => 1.25ns period)
    always #0.625 ro_clockmodel <= ~ro_clockmodel;
    

endmodule
