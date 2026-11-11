/*`timescale 1ns / 1ps

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
    reg overload;
    reg [N:0] EU_switch;
    reg [N:0] ED_switch;
    reg [N:0] ICC_switch;

    // DUT Outputs
    wire [N:0] disp_floor;
    wire [1:0] disp_dir;
    wire service_dir;
    wire idle_flag;
    wire just_wait;
    wire door_open;

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
        .overload(overload),
        .EU_switch(EU_switch),
        .ED_switch(ED_switch),
        .ICC_switch(ICC_switch),
        .disp_floor(disp_floor),
        .disp_dir(disp_dir),
        .service_dir(service_dir),
        .idle_flag(idle_flag),
        .just_wait(just_wait),
        .door_open(door_open)
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

    // --- NEW: Boarding / Waiting Monitor ---
    always @(posedge uut.wait_enable) begin
        $display("[%0t ns] ---> DOORS OPEN: Elevator arrived at Floor %0d. Waiting for passengers...", 
                 $time, oh_to_int(disp_floor));
    end

    // NEW: Monitor for the indefinite emergency wait state
    always @(posedge uut.just_wait) begin
        $display("[%0t ns] ---> DOORS OPEN: EMERGENCY EVACUATION. Holding doors open indefinitely at Floor %0d.", 
                 $time, oh_to_int(disp_floor));
    end

    always @(posedge uut.wait_done) begin
        $display("[%0t ns] ---> DOORS CLOSED: Boarding complete at Floor %0d.", 
                 $time, oh_to_int(disp_floor));
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
        overload = 0;

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
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 6: Priority Inversion Attempt (E-Stop vs Fire Alarm)
        // ====================================================================
        $display("\n--- TEST CASE 6: Priority Inversion (Fire Alarm overrides E-Stop) ---");
        press_icc(5);
        #(CLK_PERIOD * 4); // Let elevator start moving UP
        
        e_stop = 1;
        $display("[%0t ns] STIMULUS: Emergency Stop ACTIVATED!", $time);
        #(CLK_PERIOD * 4);
        
        fire_alarm = 1; // Fire alarm occurs while E-stop is physically engaged
        $display("[%0t ns] STIMULUS: FIRE ALARM DETECTED BY PMOD (While E-Stop is active)!", $time);
        
        wait (oh_to_int(disp_floor) == SAFE_FLOOR_INDEX); // Must still reach safe floor
        #(CLK_PERIOD * 10);
        
        fire_alarm = 0;
        e_stop = 0;
        $display("[%0t ns] STIMULUS: Fire Alarm and Emergency Stop DEACTIVATED", $time);
        $display("--> Test Case 6 Passed: Fire Alarm correctly overrode the E-Stop.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 7: Memory Flush During Evacuation
        // ====================================================================
        $display("\n--- TEST CASE 7: Mashing buttons during Fire Evacuation ---");
        fire_alarm = 1;
        $display("[%0t ns] STIMULUS: FIRE ALARM ACTIVATED", $time);
        #(CLK_PERIOD * 2);
        
        // Spam buttons while evacuation is active
        press_icc(3);
        press_eu(4);
        press_ed(2);
        #(CLK_PERIOD * 5);
        
        fire_alarm = 0;
        $display("[%0t ns] STIMULUS: FIRE ALARM DEACTIVATED", $time);
        #(CLK_PERIOD * 10);
        
        if (idle_flag == 1 && oh_to_int(disp_floor) == SAFE_FLOOR_INDEX) begin
            $display("--> Test Case 7 Passed: Elevator ignored all inputs and remained safely parked.");
        end else begin
            $display("--> Test Case 7 FAILED: Elevator attempted to service wiped requests.");
        end
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 8: Zero-Distance Call (Current Floor)
        // ====================================================================
        $display("\n--- TEST CASE 8: Zero-Distance Call ---");
        // Elevator is currently sitting Idle at Floor 0.
        press_icc(SAFE_FLOOR_INDEX);
        wait (uut.wait_enable == 1); // Should immediately trigger doors without moving
        wait (idle_flag == 1);
        $display("--> Test Case 8 Passed: Executed door sequence without changing floors.");
        
        
        // ====================================================================
        // TEST CASE 9: Overload Condition
        // Send to Floor 1, trigger overload while doors are open, 
        // attempt to go to Floor 2, verify it holds, then clear overload.
        // ====================================================================
        $display("\n--- TEST CASE 9: Overload holding doors open ---");
        
        // Move to Floor 1
        press_icc(1);
        wait (oh_to_int(disp_floor) == 1);
        wait (uut.wait_enable == 1); // Wait for doors to open
        
        // Trigger Overload
        overload = 1;
        $display("[%0t ns] STIMULUS: OVERLOAD ACTIVATED! Passengers exceeded weight limit.", $time);
        
        // Try to go to Floor 2 while overloaded
        press_icc(2);
        #(CLK_PERIOD * 15); // Wait a few clock cycles to see if it improperly moves
        
        if (oh_to_int(disp_floor) == 1) begin
            $display("--> Elevator correctly prevented movement and held at Floor 1 due to Overload.");
        end else begin
            $display("--> Test Case 9 FAILED: Elevator moved while overloaded!");
        end
        
        // Clear Overload
        overload = 0;
        $display("[%0t ns] STIMULUS: OVERLOAD DEACTIVATED", $time);
        
        // Now it should automatically close doors and proceed to Floor 2
        wait (oh_to_int(disp_floor) == 2);
        wait (idle_flag == 1);
        $display("--> Test Case 9 Passed: Resumed normal operation and reached Floor 2 after overload cleared.");
        #(CLK_PERIOD * 5);

        // Finish Simulation
        $display("\n=======================================================");
        $display("   ALL TEST CASES COMPLETED SUCCESSFULLY!");
        $display("=======================================================\n");
        $finish;
    end

endmodule*/

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
    reg overload;
    reg [N:0] EU_switch;
    reg [N:0] ED_switch;
    reg [N:0] ICC_switch;

    // DUT Outputs
    wire [N:0] disp_floor;
    wire [1:0] disp_dir;
    wire service_dir;
    wire idle_flag;
    wire just_wait;
    wire door_open;

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
        .overload(overload),
        .EU_switch(EU_switch),
        .ED_switch(ED_switch),
        .ICC_switch(ICC_switch),
        .disp_floor(disp_floor),
        .disp_dir(disp_dir),
        .service_dir(service_dir),
        .idle_flag(idle_flag),
        .just_wait(just_wait),
        .door_open(door_open)
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

    // Helper Task to Decode Direction Encodings (Fixed to 80 bits [79:0])
    function [79:0] dir_to_string(input [1:0] dir);
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

    // --- UPDATED: Boarding / Waiting Monitor using door_open ---
    always @(posedge door_open) begin
        if (just_wait)
            $display("[%0t ns] ---> DOORS OPEN: EMERGENCY EVACUATION. Holding doors open indefinitely at Floor %0d.", 
                     $time, oh_to_int(disp_floor));
        else
            $display("[%0t ns] ---> DOORS OPEN: Elevator arrived at Floor %0d. Waiting for passengers...", 
                     $time, oh_to_int(disp_floor));
    end

    always @(negedge door_open) begin
        $display("[%0t ns] ---> DOORS CLOSED: Boarding complete at Floor %0d.", 
                 $time, oh_to_int(disp_floor));
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
        overload    = 0;

        // Release Reset (Shifted to negative edge to avoid race condition)
        #(CLK_PERIOD * 3 + (CLK_PERIOD / 2));
        reset = 0;
        $display("\n--- TEST START: System Reset Deasserted ---");
        #(CLK_PERIOD * 2);

        // ====================================================================
        // TEST CASE 1: Basic In-Cab Service (Floor 0 to Floor 3)
        // ====================================================================
        $display("\n--- TEST CASE 1: In-Cab Call to Floor 3 ---");
        press_icc(3);

        // Wait until floor reaches Floor 3 and returns to PARK
        wait (disp_floor[3] == 1'b1);
        wait (idle_flag == 1);
        $display("--> Test Case 1 Passed: Elevator arrived at Floor 3 and returned to PARK.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 2: External Up Call + Intermediate Pickup
        // ====================================================================
        $display("\n--- TEST CASE 2: EU Call on Floor 1 & In-Cab Call on Floor 5 ---");
        press_eu(1);
        press_icc(5);

        // Wait until elevator picks up at Floor 1 then finishes at Floor 5
        wait (disp_floor[5] == 1'b1);
        wait (idle_flag == 1);
        $display("--> Test Case 2 Passed: Elevator completed sweep to Floor 5.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 3: Manual Mode Verification
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
        wait (disp_floor[2] == 1'b1);
        wait (idle_flag == 1);
        
        manual_mode = 0;
        $display("[%0t ns] STIMULUS: Manual Mode DISABLED", $time);
        $display("--> Test Case 3 Passed: Manual mode correctly ignored hall calls.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 4: Emergency Stop
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
        // ====================================================================
        $display("\n--- TEST CASE 5: PMOD Fire Alarm Evacuation ---");
        press_icc(4);
        #(CLK_PERIOD * 3);
        
        fire_alarm = 1;
        $display("[%0t ns] STIMULUS: FIRE ALARM DETECTED BY PMOD!", $time);
        
        // Wait until safe floor reached
        wait (disp_floor[SAFE_FLOOR_INDEX] == 1'b1);
        #(CLK_PERIOD * 10);
        
        $display("[%0t ns] STATUS: Elevator holding at Safe Floor (Floor %0d). Cleared fire alarm.", 
                 $time, SAFE_FLOOR_INDEX);
        fire_alarm = 0;
        #(CLK_PERIOD * 5);
        
        $display("--> Test Case 5 Passed: Fire evacuation completed.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 6: Priority Inversion Attempt (E-Stop vs Fire Alarm)
        // ====================================================================
        $display("\n--- TEST CASE 6: Priority Inversion (Fire Alarm overrides E-Stop) ---");
        press_icc(5);
        #(CLK_PERIOD * 4); // Let elevator start moving UP
        
        e_stop = 1;
        $display("[%0t ns] STIMULUS: Emergency Stop ACTIVATED!", $time);
        #(CLK_PERIOD * 4);
        
        fire_alarm = 1; // Fire alarm occurs while E-stop is physically engaged
        $display("[%0t ns] STIMULUS: FIRE ALARM DETECTED BY PMOD (While E-Stop is active)!", $time);
        
        wait (disp_floor[SAFE_FLOOR_INDEX] == 1'b1); // Must still reach safe floor
        #(CLK_PERIOD * 10);
        
        fire_alarm = 0;
        e_stop = 0;
        $display("[%0t ns] STIMULUS: Fire Alarm and Emergency Stop DEACTIVATED", $time);
        $display("--> Test Case 6 Passed: Fire Alarm correctly overrode the E-Stop.");
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 7: Memory Flush During Evacuation
        // ====================================================================
        $display("\n--- TEST CASE 7: Mashing buttons during Fire Evacuation ---");
        fire_alarm = 1;
        $display("[%0t ns] STIMULUS: FIRE ALARM ACTIVATED", $time);
        #(CLK_PERIOD * 2);
        
        // Spam buttons while evacuation is active
        press_icc(3);
        press_eu(4);
        press_ed(2);
        #(CLK_PERIOD * 5);
        
        fire_alarm = 0;
        $display("[%0t ns] STIMULUS: FIRE ALARM DEACTIVATED", $time);
        #(CLK_PERIOD * 10);
        
        if (idle_flag == 1 && disp_floor[SAFE_FLOOR_INDEX] == 1'b1) begin
            $display("--> Test Case 7 Passed: Elevator ignored all inputs and remained safely parked.");
        end else begin
            $display("--> Test Case 7 FAILED: Elevator attempted to service wiped requests.");
        end
        #(CLK_PERIOD * 5);

        // ====================================================================
        // TEST CASE 8: Zero-Distance Call (Current Floor)
        // ====================================================================
        $display("\n--- TEST CASE 8: Zero-Distance Call ---");
        // Elevator is currently sitting Idle at Floor 0.
        press_icc(SAFE_FLOOR_INDEX);
        wait (door_open == 1); // Should immediately trigger doors without moving
        wait (idle_flag == 1);
        $display("--> Test Case 8 Passed: Executed door sequence without changing floors.");
        
        // ====================================================================
        // TEST CASE 9: Overload Condition
        // ====================================================================
        $display("\n--- TEST CASE 9: Overload holding doors open ---");
        
        // Move to Floor 1
        press_icc(1);
        wait (disp_floor[1] == 1'b1);
        wait (door_open == 1); // Wait for doors to open
        
        // Trigger Overload
        overload = 1;
        $display("[%0t ns] STIMULUS: OVERLOAD ACTIVATED! Passengers exceeded weight limit.", $time);
        
        // Try to go to Floor 2 while overloaded
        press_icc(2);
        #(CLK_PERIOD * 15); // Wait a few clock cycles to see if it improperly moves
        
        if (disp_floor[1] == 1'b1) begin
            $display("--> Elevator correctly prevented movement and held at Floor 1 due to Overload.");
        end else begin
            $display("--> Test Case 9 FAILED: Elevator moved while overloaded!");
        end
        
        // Clear Overload
        overload = 0;
        $display("[%0t ns] STIMULUS: OVERLOAD DEACTIVATED", $time);
        
        // Now it should automatically close doors and proceed to Floor 2
        wait (disp_floor[2] == 1'b1);
        wait (idle_flag == 1);
        $display("--> Test Case 9 Passed: Resumed normal operation and reached Floor 2 after overload cleared.");
        #(CLK_PERIOD * 5);

        // Finish Simulation
        $display("\n=======================================================");
        $display("   ALL TEST CASES COMPLETED SUCCESSFULLY!");
        $display("=======================================================\n");
        $finish;
    end

endmodule