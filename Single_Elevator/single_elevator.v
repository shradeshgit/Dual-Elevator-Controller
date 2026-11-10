`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.11.2026 22:31:52
// Design Name: 
// Module Name: singl_ele
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module single_elevator #(
    parameter N = 50,
    parameter WAIT_CYCLES = 5,
    parameter SAFE_FLOOR_INDEX = 0 // Parameterized safe floor index (0 to N)
)(
    input clock,
    input reset,
    input e_stop,
    input manual_mode,             // NEW: Manual Mode toggle
    input fire_alarm,              // NEW: Fire Alarm trigger from PMOD
    input [N:0] EU_switch,
    input [N:0] ED_switch,
    input [N:0] ICC_switch,
    output [N:0] disp_floor,
    output reg [1:0] disp_dir,
    output reg service_dir,
    output idle_flag,
    output reg just_wait
);

    reg [3:0] TF_sel;
    reg [N:0] TF, CF, EU_reg, ED_reg, ICC_reg;
    reg [3:0] current_state;

    assign disp_floor = CF;

    // Convert safe floor integer index to one-hot vector
    localparam [N:0] SAFE_FLOOR_OH = (1'b1 << SAFE_FLOOR_INDEX);

    

    // Direction Encodings
    localparam [1:0] STATIONARY = 2'b00,
                     UP         = 2'b01,
                     DOWN       = 2'b10;

    // FSM State Encodings
    localparam [3:0] PARK          = 4'b0000,
                     WAIT          = 4'b0001,
                     EU_SERVICE    = 4'b0010,
                     ED_SERVICE    = 4'b0011,
                     ICC_SERVICE   = 4'b0100,
                     WAIT_EU_DOWN  = 4'b0101,
                     WAIT_EU_UP    = 4'b0110,
                     WAIT_ED_DOWN  = 4'b0111,
                     WAIT_ED_UP    = 4'b1000,
                     WAIT_ICC_DOWN = 4'b1001,
                     WAIT_ICC_UP   = 4'b1010,
                     FIRE_EVAC     = 4'b1011,
                     WAIT_INDEFINITE = 4'b1100; // NEW: Non-stop Fire Evacuation State
                     
    assign idle_flag = (current_state == PARK) && 
                       (EU_reg == {(N+1){1'b0}}) && 
                       (ED_reg == {(N+1){1'b0}}) && 
                       (ICC_reg == {(N+1){1'b0}}) &&
                       !fire_alarm;
    wire wait_enable;
    wire wait_done;

    assign wait_enable = (current_state == WAIT)          ||
                         (current_state == WAIT_EU_UP)     ||
                         (current_state == WAIT_EU_DOWN)   ||
                         (current_state == WAIT_ED_UP)     ||
                         (current_state == WAIT_ED_DOWN)   ||
                         (current_state == WAIT_ICC_UP)    ||
                         (current_state == WAIT_ICC_DOWN);

    wait_counter #(.WAIT_CYCLES(WAIT_CYCLES)) u_wait_timer (
        .clock  (clock),
        .reset  (reset),
        .enable (wait_enable),
        .just_wait (just_wait),
        .done   (wait_done)
    );

    function [N:0] min_one_hot(input [N:0] req);
        begin
            min_one_hot = req & (~req + 1'b1);
        end
    endfunction

    function [N:0] max_one_hot(input [N:0] req);
        integer i;
        reg found;
        begin
            max_one_hot = {(N+1){1'b0}};
            found = 1'b0;
            for (i = N; i >= 0; i = i - 1) begin
                if (req[i] && !found) begin
                    max_one_hot[i] = 1'b1;
                    found = 1'b1;
                end
            end
        end
    endfunction

    always @(*) begin
        case (TF_sel)
            EU_SERVICE:  TF = min_one_hot(EU_reg);
            ED_SERVICE:  TF = max_one_hot(ED_reg);
            ICC_SERVICE: begin
                if (service_dir == 1'b0)
                    TF = min_one_hot(ICC_reg | ED_reg);
                else
                    TF = max_one_hot(ICC_reg | EU_reg);
            end
            default:     TF = min_one_hot(EU_reg | ICC_reg | ED_reg);
        endcase
    end

    // Sequential FSM
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            CF            <= {{N{1'b0}}, 1'b1};
            TF_sel        <= ICC_SERVICE;
            EU_reg        <= {(N+1){1'b0}};
            ED_reg        <= {(N+1){1'b0}};
            ICC_reg       <= {(N+1){1'b0}};
            service_dir   <= 1'b0;
            current_state <= PARK;
            disp_dir      <= STATIONARY;
            just_wait     <= 1'b0;
        end
        else begin
            // ------------------ REQUEST LATCHING & MODES ------------------
            if (fire_alarm) begin
                // In Fire Mode: Clear all pending calls completely
                EU_reg  <= {(N+1){1'b0}};
                ED_reg  <= {(N+1){1'b0}};
                ICC_reg <= {(N+1){1'b0}};
            end
            else if (manual_mode) begin
                // In Manual Mode: Ignore and clear all external hall calls
                EU_reg  <= {(N+1){1'b0}};
                ED_reg  <= {(N+1){1'b0}};
                ICC_reg <= ICC_reg | ICC_switch;
            end
            else begin
                // Normal Mode: Latch all calls
                EU_reg  <= EU_reg  | EU_switch;
                ED_reg  <= ED_reg  | ED_switch;
                ICC_reg <= ICC_reg | ICC_switch;
            end

            // ------------------ PRIORITY OVERRIDES ------------------
            if (fire_alarm) begin
                // FIRE ALARM (HIGHEST PRIORITY): Direct movement to SAFE_FLOOR
                current_state <= FIRE_EVAC;
                if (CF < SAFE_FLOOR_OH) begin
                    CF       <= CF << 1;
                    disp_dir <= UP;
                end
                else if (CF > SAFE_FLOOR_OH) begin
                    CF       <= CF >> 1;
                    disp_dir <= DOWN;
                end
                else begin
                    // Arrived at Safe Floor: Stop and keep doors open
                    disp_dir <= STATIONARY;
                    current_state <= WAIT_INDEFINITE;
                end
            end
            else if (e_stop) begin
                disp_dir      <= STATIONARY;
                current_state <= PARK;
            end
            else begin
                case (current_state)
                    PARK: begin
                        disp_dir <= STATIONARY;
                        if ((ICC_reg | ICC_switch) != 0) begin
                            current_state <= ICC_SERVICE;
                            TF_sel        <= ICC_SERVICE;
                        end
                        // In manual mode, EU_reg and ED_reg remain 0, automatically ignoring these:
                        else if ((EU_reg | EU_switch) != 0) begin
                            current_state <= EU_SERVICE;
                            TF_sel        <= EU_SERVICE;
                        end
                        else if ((ED_reg | ED_switch) != 0) begin
                            current_state <= ED_SERVICE;
                            TF_sel        <= ED_SERVICE;
                        end
                        else begin
                            current_state <= PARK;
                        end
                    end

                    // ------------------ FIRE EVACUATION STATE ------------------
                    FIRE_EVAC: begin
                        if (!fire_alarm) begin
                            current_state <= PARK;
                        end
                        else if (CF < SAFE_FLOOR_OH) begin
                            CF       <= CF << 1;
                            disp_dir <= UP;
                        end
                        else if (CF > SAFE_FLOOR_OH) begin
                            CF       <= CF >> 1;
                            disp_dir <= DOWN;
                        end
                        else begin
                            disp_dir <= STATIONARY;
                            current_state <= WAIT_INDEFINITE;
                        end
                    end

                    // ------------------ NORMAL SERVICE STATES ------------------
                    EU_SERVICE: begin
                        if (TF == 0) begin
                            current_state <= PARK;
                            disp_dir      <= STATIONARY;
                        end
                        else if (TF == CF) begin
                            EU_reg        <= (EU_reg  | EU_switch)  & ~CF;
                            ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                            service_dir   <= 1'b1;
                            disp_dir      <= STATIONARY;
                            current_state <= WAIT;
                        end
                        else if (TF < CF) begin
                            if ((CF & ICC_reg) != 0) begin
                                ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                                disp_dir      <= STATIONARY;
                                current_state <= WAIT_EU_DOWN;
                            end
                            else begin
                                CF       <= CF >> 1;
                                disp_dir <= DOWN;
                            end
                        end
                        else if (TF > CF) begin
                            if ((CF & (ICC_reg | EU_reg)) != 0) begin
                                ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                                EU_reg        <= (EU_reg  | EU_switch)  & ~CF;
                                disp_dir      <= STATIONARY;
                                current_state <= WAIT_EU_UP;
                            end
                            else begin
                                CF       <= CF << 1;
                                disp_dir <= UP;
                            end
                        end
                    end

                    ED_SERVICE: begin
                        if (TF == 0) begin
                            current_state <= PARK;
                            disp_dir      <= STATIONARY;
                        end
                        else if (TF == CF) begin
                            ED_reg        <= (ED_reg  | ED_switch)  & ~CF;
                            ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                            service_dir   <= 1'b0;
                            disp_dir      <= STATIONARY;
                            current_state <= WAIT;
                        end
                        else if (TF > CF) begin
                            if ((CF & ICC_reg) != 0) begin
                                ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                                disp_dir      <= STATIONARY;
                                current_state <= WAIT_ED_UP;
                            end
                            else begin
                                CF       <= CF << 1;
                                disp_dir <= UP;
                            end
                        end
                        else if (TF < CF) begin
                            if ((CF & (ICC_reg | ED_reg)) != 0) begin
                                ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                                ED_reg        <= (ED_reg  | ED_switch)  & ~CF;
                                disp_dir      <= STATIONARY;
                                current_state <= WAIT_ED_DOWN;
                            end
                            else begin
                                CF       <= CF >> 1;
                                disp_dir <= DOWN;
                            end
                        end
                    end

                    ICC_SERVICE: begin
                        if (TF == 0) begin
                            current_state <= PARK;
                            disp_dir      <= STATIONARY;
                        end
                        else if (TF == CF) begin
                            ED_reg        <= (ED_reg  | ED_switch)  & ~CF;
                            EU_reg        <= (EU_reg  | EU_switch)  & ~CF;
                            ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                            disp_dir      <= STATIONARY;
                            current_state <= WAIT;
                        end
                        else if (TF > CF) begin
                            if ((CF & (ICC_reg | EU_reg)) != 0) begin
                                ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                                EU_reg        <= (EU_reg  | EU_switch)  & ~CF;
                                disp_dir      <= STATIONARY;
                                current_state <= WAIT_ICC_UP;
                            end
                            else begin
                                CF       <= CF << 1;
                                disp_dir <= UP;
                            end
                        end
                        else if (TF < CF) begin
                            if ((CF & (ICC_reg | ED_reg)) != 0) begin
                                ICC_reg       <= (ICC_reg | ICC_switch) & ~CF;
                                ED_reg        <= (ED_reg  | ED_switch)  & ~CF;
                                disp_dir      <= STATIONARY;
                                current_state <= WAIT_ICC_DOWN;
                            end
                            else begin
                                CF       <= CF >> 1;
                                disp_dir <= DOWN;
                            end
                        end
                    end

                    WAIT: begin
                        disp_dir <= STATIONARY;
                        if (wait_done) current_state <= PARK;
                    end
                    
                    WAIT_INDEFINITE: begin
                        disp_dir <= STATIONARY;
                        just_wait <= 1'b1;
                        if (!fire_alarm) begin
                            current_state <= PARK;
                            just_wait     <= 1'b0; // Turn off the infinite counter
                        end
                    end
                       

                    WAIT_EU_UP: begin
                        disp_dir <= STATIONARY;
                        if (wait_done) begin
                            CF            <= CF << 1;
                            disp_dir      <= UP;
                            current_state <= EU_SERVICE;
                        end
                    end

                    WAIT_EU_DOWN: begin
                        disp_dir <= STATIONARY;
                        if (wait_done) begin
                            CF            <= CF >> 1;
                            disp_dir      <= DOWN;
                            current_state <= EU_SERVICE;
                        end
                    end

                    WAIT_ED_UP: begin
                        disp_dir <= STATIONARY;
                        if (wait_done) begin
                            CF            <= CF << 1;
                            disp_dir      <= UP;
                            current_state <= ED_SERVICE;
                        end
                    end

                    WAIT_ED_DOWN: begin
                        disp_dir <= STATIONARY;
                        if (wait_done) begin
                            CF            <= CF >> 1;
                            disp_dir      <= DOWN;
                            current_state <= ED_SERVICE;
                        end
                    end

                    WAIT_ICC_UP: begin
                        disp_dir <= STATIONARY;
                        if (wait_done) begin
                            CF            <= CF << 1;
                            disp_dir      <= UP;
                            current_state <= ICC_SERVICE;
                        end
                    end

                    WAIT_ICC_DOWN: begin
                        disp_dir <= STATIONARY;
                        if (wait_done) begin
                            CF            <= CF >> 1;
                            disp_dir      <= DOWN;
                            current_state <= ICC_SERVICE;
                        end
                    end

                    default: current_state <= PARK;
                endcase
            end
        end
    end

endmodule
