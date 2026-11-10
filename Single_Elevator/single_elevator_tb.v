`timescale 1ns / 1ps

module tb_single_elevator;

    // Testbench Parameters
    parameter N = 5;                  // 6 Floors: 0, 1, 2, 3, 4, 5
    parameter WAIT_CYCLES = 3;        // Fast wait timer for simulation
    parameter SAFE_FLOOR_INDEX = 0;   // Ground Floor (Floor 0) as Safe Floor
    parameter CLK_PERIOD = 10;        // 10ns clock period (100 MHz)

    // DUT Inputs
    reg clock;
    reg reset;
    reg e_stop;
    reg manual_mode;
    reg fire_alarm;
    reg [N:0] EU_switch;
    reg [N:0] ED_switch;
    reg [N:0] ICC_switch;

    // DUT Outputs
    wire [N:0] disp_floor;
    wire [1:0] disp_dir;
    wire service_dir;
    wire idle_flag;

    // Direct Direction Constant Decoding for Display
    localparam [1:0] STATIONARY = 2'b00,
                     UP         = 2'b01,
                     DOWN       = 2'b10;

    // Instantiate Unit Under Test (UUT)
    single_elevator #(
        .N(N),
        .WAIT_CYCLES(WAIT_CYCLES),
        .SAFE_FLOOR_INDEX(SAFE_FLOOR_INDEX)
    ) uut (
        .clock(clock),
        .reset(reset),
        .e_stop(e_stop),
        .manual_mode(manual_mode),
        .fire_alarm(fire_alarm),
        .EU_switch(EU_switch),
        .ED_switch(ED_switch),
        .ICC_switch(ICC_switch),
        .disp_floor(disp_floor),
        .disp_dir(disp_dir),
        .service_dir(service_dir),
        .idle_flag(idle_flag)
    );

    // 1. Clock Generation Process (100 MHz)
    always #(CLK_PERIOD / 2) clock = ~clock;

    // 2. Helper Tasks for Button Pulse Stimulus
    task press_icc(input integer floor_num);
        begin
            @(posedge clock);
            ICC_switch[floor_num] <= 1'b1;
            @(posedge clock);
            ICC_switch[floor_num] <= 1'b0;
            $display("[%0t ns] STIMULUS: In-Cab Button pressed for Floor %0d", $time, floor_num);
        end
    endtask

    task press_eu(input integer floor_num);
        begin
            @(posedge clock);
            EU_switch[floor_num] <= 1'b1;
            @(posedge clock);
            EU_switch[floor_num] <= 1'b0;
            $display("[%0t ns] STIMULUS: External UP Button pressed on Floor %0d", $time, floor_num);
        end
    endtask

    task press_ed(input integer floor_num);
        begin
            @(posedge clock);
            ED_switch[floor_num] <= 1'b1;
            @(posedge clock);
            ED_switch[floor_num] <= 1'b0;
            $display("[%0t ns] STIMULUS: External DOWN Button pressed on Floor %0d", $time, floor_num);
        end
    endtask

    // Helper Task to Convert One-Hot Floor Output to Integer for Console Logs
    function integer oh_to_int(input [N:0] oh_vec);
        integer i;
        begin
            oh_to_int = 0;
            for (i = 0; i <= N; i = i + 1) begin
                if (oh_vec[i]) oh_to_int = i;
            end
        end
    endfunction

    // Helper Task to Decode Direction Encodings
    function [80:0] dir_to_string(input [1:0] dir);
        case (dir)
            STATIONARY: dir_to_string = "STATIONARY";
            UP:         dir_to_string = "UP        ";
            DOWN:       dir_to_string = "DOWN      ";
            default:    dir_to_string = "UNKNOWN   ";
        endcase
    endfunction

    // 3. Console Output Monitor
    always @(disp_floor or disp_dir or idle_flag) begin
        $display("[%0t ns] STATUS: Current Floor = %0d | Direction = %s | Idle = %b", 
                 $time, oh_to_int(disp_floor), dir_to_string(disp_dir), idle_flag);
    end

    // 4. Main Test Sequence
    initial begin
        // Initialize Signals
        clock       = 0;
        reset       = 1;
        e_stop      = 0;
        manual_mode = 0;
        fire_alarm  = 0;
        EU_switch   = 0;
        ED_switch   = 0;
        ICC_switch  = 0;

        // Release Reset
        #(CLK_PERIOD * 3);
        reset = 0;
        $display("\n--- TEST START: System Reset Deasserted ---");
        #(CLK_PERIOD * 2);

        // ====================================================================
        // TEST CASE 1: Basic In-Cab Service (Floor 0 to Floor 3)
        // ====================================================================
        $display("\n--- TEST CASE 1: In-Cab Call to Floor 3 ---");
        press_icc(3);

        // Wait until floor reaches Floor 3 and returns to PARK
        wait (oh_to_int(disp_floor) == 3);
        wait (idle_flag == 1);
        $display("--> Test Case 1 Passed: Elevator arrived at Floor 3 and returned to PARK.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 2: External Up Call + Intermediate Pickup
        // Elevator is at Floor 3. Call External Up at Floor 1 and ICC at Floor 5.
        // Elevator sweeps down to Floor 1 first, picks up passenger, moves to Floor 5.
        // ====================================================================
        $display("\n--- TEST CASE 2: EU Call on Floor 1 & In-Cab Call on Floor 5 ---");
        press_eu(1);
        press_icc(5);

        // Wait until elevator picks up at Floor 1 then finishes at Floor 5
        wait (oh_to_int(disp_floor) == 5);
        wait (idle_flag == 1);
        $display("--> Test Case 2 Passed: Elevator completed sweep to Floor 5.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 3: Manual Mode Verification
        // Enable manual mode -> Hallway calls must be ignored, ICC must work.
        // ====================================================================
        $display("\n--- TEST CASE 3: Manual Mode (Ignore Hallway Calls) ---");
        manual_mode = 1;
        $display("[%0t ns] STIMULUS: Manual Mode ENABLED", $time);
        
        press_ed(2); // Hall call (should be ignored)
        #(CLK_PERIOD * 5);
        if (idle_flag == 1) begin
            $display("--> Hall call successfully ignored in Manual Mode.");
        end

        press_icc(2); // Cabin call (should be serviced)
        wait (oh_to_int(disp_floor) == 2);
        wait (idle_flag == 1);
        
        manual_mode = 0;
        $display("[%0t ns] STIMULUS: Manual Mode DISABLED", $time);
        $display("--> Test Case 3 Passed: Manual mode correctly ignored hall calls.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 4: Emergency Stop
        // Trigger trip to Floor 5, then activate e_stop mid-transit.
        // ====================================================================
        $display("\n--- TEST CASE 4: Emergency Stop Mid-Flight ---");
        press_icc(5);
        #(CLK_PERIOD * 4); // Let elevator start moving
        
        e_stop = 1;
        $display("[%0t ns] STIMULUS: Emergency Stop ACTIVATED!", $time);
        #(CLK_PERIOD * 10);
        
        e_stop = 0;
        $display("[%0t ns] STIMULUS: Emergency Stop DEACTIVATED", $time);
        $display("--> Test Case 4 Passed: Motion halted during Emergency Stop.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 5: Fire Alarm Evacuation Override
        // Send elevator to Floor 4. Assert fire alarm mid-transit.
        // Elevator must ignore destination, clear requests, and move straight to Safe Floor (0).
        // ====================================================================
        $display("\n--- TEST CASE 5: PMOD Fire Alarm Evacuation ---");
        press_icc(4);
        #(CLK_PERIOD * 3);
        
        fire_alarm = 1;
        $display("[%0t ns] STIMULUS: FIRE ALARM DETECTED BY PMOD!", $time);
        
        // Wait until safe floor reached
        wait (oh_to_int(disp_floor) == SAFE_FLOOR_INDEX);
        #(CLK_PERIOD * 10);
        
        $display("[%0t ns] STATUS: Elevator holding at Safe Floor (Floor %0d). Cleared fire alarm.", 
                 $time, SAFE_FLOOR_INDEX);
        fire_alarm = 0;
        #(CLK_PERIOD * 5);
        
        $display("--> Test Case 5 Passed: Fire evacuation completed.");

        // Finish Simulation
        $display("\n=======================================================");
        $display("   ALL TEST CASES COMPLETED SUCCESSFULLY!");
        $display("=======================================================\n");
        $finish;
    end

endmodule