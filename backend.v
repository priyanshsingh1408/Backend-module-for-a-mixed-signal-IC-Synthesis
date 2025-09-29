module backend( i_resetbAll,
		i_clk,
		i_sclk,
		i_sdin,
		i_RO_clk,
		i_ADCout,
		o_core_clk ,
		o_ready,
		o_resetb_amp,
		o_Ibias_2x,
		o_gain,
		o_enableRO,
		o_resetb_core
		);

// Inputs and Outputs
input i_resetbAll, i_clk, i_sclk, i_sdin, i_RO_clk ;
input [3:0]i_ADCout ;
output reg o_ready, o_Ibias_2x, o_resetb_amp, o_resetb_core, o_enableRO, o_core_clk ;
output reg [2:0] o_gain;

//Rest of the behavioral descriptions



    //  Assigning Parameters to the States of FSM used to improve readability
    parameter RESET = 3'd0;
    parameter WAIT_SERIAL = 3'd1;
    parameter ENABLE_RO = 3'd2;
    parameter WAIT_RO = 3'd3;
    parameter CHECK_ADC = 3'd4;
    parameter ENABLE_MODULES = 3'd5;
    parameter WAIT_MODULES = 3'd6;
    parameter READY = 3'd7;
    
    // Internal registers used
    reg [2:0] state;                 // Current state
    reg [2:0] counter;               // Counter for waiting states
	reg [2:0] data_counter;          // Counter for serial data bits  , it is a 3bit counter and thus can count from 0 to 7
    reg [2:0] serial_data;           // Serial data storage
    
    // Registers of ADC Filters used
    reg [6:0] ADCsum;               // Sum for moving average filter
    reg [6:0] ADCavg;               // Average value from filter is stored in ADCavg
    reg [3:0] ADC_reg [0:3];        // 4 registers for the filter
    
    // Clock divider registers
    reg [1:0] clk_div_counter;      // Counter for clock division
    
    // Initialising the ADC registers
    integer i;
    initial begin
        for (i = 0; i < 4; i = i + 1) begin
            ADC_reg[i] = 4'b0000;
        end
        ADCsum = 7'b0000000;
        ADCavg = 7'b0000000;
    end
    
    // Serial data input processing on rising edge of i_sclk
    always @(posedge i_sclk or negedge i_resetbAll) begin
        if (~i_resetbAll) begin
            data_counter <= 3'd0;
            serial_data <= 3'd0;
        end
        else if (state == WAIT_SERIAL && data_counter < 3'd5) begin
            // Shift register implementation for serial data
            if (data_counter >= 3'd2) begin  // We only care about bits d2, d3, d4 as d1 and d0 will be used later
                serial_data <= {serial_data[1:0], i_sdin}; 
            end
            data_counter <= data_counter + 3'd1;
        end
    end
    
    // Clock divider for o_core_clk 
	// clk div counter is a 2bit counter 00 -> 01-> 10-> 11
    always @(posedge i_clk or negedge i_resetbAll) begin
        if (~i_resetbAll) begin
            clk_div_counter <= 2'd0;
        end
        else begin
            clk_div_counter <= clk_div_counter + 2'd1;
        end
    end
    
    // Generating o_core_clk based on ADCavg
    always @(*) begin
        if (o_Ibias_2x == 1'b1) begin
            // Dividing clock by 4 using ternary operator and counter 
			// thus here as clk div counter is 2 bit counter it has 4 values and only for 1 value 0 we make o_core_clk as 1 and for rest 3 make it 0 , 
			// in this way the clk freq is reduced by 4 and it is slowed by 4
            o_core_clk = (clk_div_counter == 2'd0) ? 1'b1 : 1'b0;
        end
        else begin
            // Using original clock
            o_core_clk = i_clk;
        end
    end
    
    // Implementing ADC 4-tap moving average filter
    always @(posedge i_clk or negedge i_resetbAll) begin
        if (~i_resetbAll) begin
            for (i = 0; i < 4; i = i + 1) begin
                ADC_reg[i] <= 4'b0000;
            end
            ADCsum <= 7'b0000000;
            ADCavg <= 7'b0000000;
        end
        else begin
            // Shifting the ADC values through the registers
            ADC_reg[0] <= i_ADCout;
            ADC_reg[1] <= ADC_reg[0];
            ADC_reg[2] <= ADC_reg[1];
            ADC_reg[3] <= ADC_reg[2];
            
            // Calculating the sum
            ADCsum <= ADC_reg[0] + ADC_reg[1] + ADC_reg[2] + ADC_reg[3];
            
            // Calculating the average (divide by 4 = shift right by 2)
            ADCavg <= ADCsum >> 2;
        end
    end
    
    // Main FSM for startup sequence
    always @(posedge i_clk or negedge i_resetbAll) begin
        if (~i_resetbAll) begin
            // Reset state for all outputs and internal registers according to i_resetbAll
            state <= RESET;
            counter <= 3'd0;
            o_ready <= 1'b0;
            o_resetb_amp <= 1'b0;
            o_gain <= 3'd0;
            o_Ibias_2x <= 1'b0;
            o_enableRO <= 1'b0;
            o_resetb_core <= 1'b0;
	    o_core_clk <= 1'b0;
        end
        else begin
            case (state)
                RESET: begin
                    // In Reset state we wait for i_resetbAll to go high
                    state <= WAIT_SERIAL;
                end
                
                WAIT_SERIAL: begin
                    // Wait for serial data reception to complete
                    if (data_counter >= 3'd5) begin
                        o_gain <= serial_data;  // Set the gain with the received serial data
                        state <= ENABLE_RO;
                    end
                end
                
                ENABLE_RO: begin
                    // Enable the ring oscillator
                    o_enableRO <= 1'b1;
                    state <= WAIT_RO;
                    counter <= 3'd0;
                end
                
                WAIT_RO: begin
                    // This state entry point is when counter=0
                    // We want to count exactly 5 cycles (0,1,2,3,4)
                    if (counter < 3'd4) begin
                        counter <= counter + 3'd1;
                    end
                    else begin
                        // This transition happens at the fifth clock edge after the state WAIT_RO began
                        // Immediately checking ADC and setting Ibias in the same clock cycle
                        state <= CHECK_ADC;
                        if (ADCavg <= 7'd12) begin
                            o_Ibias_2x <= 1'b0;
                        end
                        else begin
                            o_Ibias_2x <= 1'b1;
                        end
                        counter <= 3'd0;
                    end
                end
                
                CHECK_ADC: begin
                    // Moving to the next state immediately after setting o_Ibias_2x
                    state <= ENABLE_MODULES;
                end
                
                ENABLE_MODULES: begin
                    // Enabling opamp and core after setting Ibias
                    o_resetb_amp <= 1'b1;
                    o_resetb_core <= 1'b1;
                    state <= WAIT_MODULES;
                    counter <= 3'd0;
                end
                
                WAIT_MODULES: begin
                    // Wait for exactly 5 clock cycles after enabling modules before ready becomes High
                    if (counter < 3'd4) begin
                        counter <= counter + 3'd1;
                    end
                    else begin
                        state <= READY;
                        o_ready <= 1'b1;  // Setting ready signal after exactly 5 cycles
                    end
                end
                
                READY: begin
                    // Continuously monitoring ADC and adjust Ibias and clock
                    if (ADCavg > 7'd12) begin
                        o_Ibias_2x <= 1'b1;
                    end
                    else if (ADCavg < 7'd8) begin
                        o_Ibias_2x <= 1'b0;
                    end
                    // If 8 <= ADCavg <= 12, retaining current values
                end
                
                default: state <= RESET;
            endcase
        end
    end
    

endmodule

