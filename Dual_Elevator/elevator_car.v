`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: elevator_car
//
// One elevator car. Built from the simulation version (sweep_dir, overload,
// EU/ED reversal) with these changes:
//
//   Interface / timing
//   - step_tick is an INPUT (shared tick from the top). Movement and all state
//     decisions happen on step_tick. Tie it to 1'b1 for a free-running sim.
//   - fire evacuation moves one floor per step_tick.
//   - just_wait is derived from the state (high while parked at the safe floor
//     during a fire alarm).
//   - cross_clear_EU / cross_clear_ED work: a 1 clears that floor's pending
//     hall call in this car (used by the dispatcher to take a call back).
//   - outputs for the dispatcher: sweep_dir, parked, EU_pending, ED_pending.
//
//   Behaviour
//   - ONE stop rule for every service state (EU/ED/ICC): moving up the car stops
//     at floors with an ICC or EU call, moving down at floors with an ICC or ED
//     call. So a hall call in the direction of travel is always picked up on the
//     way, which is what the dispatcher's "en route" cost assumes.
//   - ICC service is a real SCAN. service_dir is the sweep direction. The car
//     goes to the farthest request AHEAD in that direction, and reverses
//     service_dir only when nothing is left ahead but something is behind.
//   - At the end of an ICC sweep only the ICC call and the hall call matching
//     service_dir are cleared (EU for an up sweep, ED for a down sweep).
//   - No blind shift after a door wait: the car returns to its service state and
//     re-decides from the current requests.
//   - disp_dir is combinational and always shows the direction of the NEXT move
//     (STATIONARY when the car is about to stop, is waiting, or has nothing to
//     do). On every step_tick the floor moves exactly as disp_dir says.
//   - ICC switches are edge-detected: a switch left on registers once.
//////////////////////////////////////////////////////////////////////////////////

module elevator_car #(
    parameter N = 50,
    parameter WAIT_CYCLES = 5,
    parameter SAFE_FLOOR_INDEX = 0
)(
    input              clock,
    input              reset,
    input              step_tick,
    input              e_stop,
    input              manual_mode,
    input              fire_alarm,
    input              overload,

    input  [N:0]       EU_switch,
    input  [N:0]       ED_switch,
    input  [N:0]       ICC_switch,

    input  [N:0]       cross_clear_EU,
    input  [N:0]       cross_clear_ED,

    output [N:0]       disp_floor,
    output reg [1:0]   disp_dir,
    output reg         service_dir,
    output             idle_flag,
    output             just_wait,
    output             door_open,

    // for the dispatcher
    output reg [1:0]   sweep_dir,
    output             parked,
    output [N:0]       EU_pending,
    output [N:0]       ED_pending
);

    // ============================================================
    // INTERNAL REGISTERS
    // ============================================================

    reg [3:0] TF_sel;
    reg [N:0] TF, CF, EU_reg, ED_reg, ICC_reg;
    reg [3:0] current_state;
    reg [N:0] ICC_prev;

    assign disp_floor = CF;
    assign EU_pending = EU_reg;
    assign ED_pending = ED_reg;

    localparam [N:0] SAFE_FLOOR_OH = (1'b1 << SAFE_FLOOR_INDEX);

    // ============================================================
    // ENCODINGS
    // ============================================================

    localparam [1:0] STATIONARY = 2'b00,
                     UP         = 2'b01,
                     DOWN       = 2'b10;

    localparam [3:0] PARK            = 4'b0000,
                     WAIT            = 4'b0001,
                     EU_SERVICE      = 4'b0010,
                     ED_SERVICE      = 4'b0011,
                     ICC_SERVICE     = 4'b0100,
                     WAIT_EU_DOWN    = 4'b0101,
                     WAIT_EU_UP      = 4'b0110,
                     WAIT_ED_DOWN    = 4'b0111,
                     WAIT_ED_UP      = 4'b1000,
                     WAIT_ICC_DOWN   = 4'b1001,
                     WAIT_ICC_UP     = 4'b1010,
                     FIRE_EVAC       = 4'b1011,
                     WAIT_INDEFINITE = 4'b1100;

    // ============================================================
    // STATUS OUTPUTS
    // ============================================================

    assign parked = (current_state == PARK);

    assign idle_flag = (current_state == PARK) &&
                       (EU_reg  == {(N+1){1'b0}}) &&
                       (ED_reg  == {(N+1){1'b0}}) &&
                       (ICC_reg == {(N+1){1'b0}}) &&
                       !fire_alarm;

    // High while the car holds the doors open at the safe floor in fire mode.
    assign just_wait = (current_state == WAIT_INDEFINITE);

    // ============================================================
    // WAIT TIMER AND DOOR CONTROL
    // ============================================================

    wire wait_enable;
    wire wait_done;

    assign wait_enable = (current_state == WAIT)          ||
                         (current_state == WAIT_EU_UP)    ||
                         (current_state == WAIT_EU_DOWN)  ||
                         (current_state == WAIT_ED_UP)    ||
                         (current_state == WAIT_ED_DOWN)  ||
                         (current_state == WAIT_ICC_UP)   ||
                         (current_state == WAIT_ICC_DOWN);

    car_wait_counter #(
        .WAIT_CYCLES(WAIT_CYCLES)
    ) u_wait_timer (
        .clock     (clock),
        .reset     (reset),
        .enable    (wait_enable),
        .just_wait (just_wait),
        .overload  (overload),
        .done      (wait_done)
    );

    assign door_open = wait_enable || just_wait;

    // ============================================================
    // REQUEST REGISTERS WITH THIS CYCLE'S NEW PRESSES / CROSS-CLEARS
    // ============================================================

    // ICC switches are edge-detected so a switch left on registers only once
    // (otherwise it would re-latch at the stop floor and hold the car there).
    always @(posedge clock or posedge reset) begin
        if (reset) ICC_prev <= {(N+1){1'b0}};
        else       ICC_prev <= ICC_switch;
    end

    wire [N:0] ICC_new = ICC_switch & ~ICC_prev;

    wire [N:0] EU_in  = (EU_reg  | EU_switch)  & ~cross_clear_EU;
    wire [N:0] ED_in  = (ED_reg  | ED_switch)  & ~cross_clear_ED;
    wire [N:0] ICC_in = (ICC_reg | ICC_new);

    // ============================================================
    // MIN / MAX ONE-HOT SELECTION AND FLOOR MASKS
    // ============================================================

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

    // Current floor and every floor above it.
    function [N:0] mask_above_or_at(input [N:0] floor_oh);
        integer i;
        reg found;
        begin
            mask_above_or_at = {(N+1){1'b0}};
            found = 1'b0;
            for (i = 0; i <= N; i = i + 1) begin
                if (floor_oh[i]) found = 1'b1;
                if (found) mask_above_or_at[i] = 1'b1;
            end
        end
    endfunction

    // Current floor and every floor below it.
    function [N:0] mask_below_or_at(input [N:0] floor_oh);
        integer i;
        reg found;
        begin
            mask_below_or_at = {(N+1){1'b0}};
            found = 1'b0;
            for (i = N; i >= 0; i = i - 1) begin
                if (floor_oh[i]) found = 1'b1;
                if (found) mask_below_or_at[i] = 1'b1;
            end
        end
    endfunction

    wire [N:0] floors_above_or_at = mask_above_or_at(CF);
    wire [N:0] floors_below_or_at = mask_below_or_at(CF);

    // ============================================================
    // STOP RULE (same for every service state)
    // ============================================================

    wire stop_up   = ((CF & (ICC_reg | EU_reg)) != 0);   // moving up: ICC or EU call here
    wire stop_down = ((CF & (ICC_reg | ED_reg)) != 0);   // moving down: ICC or ED call here

    // ============================================================
    // TARGET FLOOR SELECTION
    // ============================================================

    wire [N:0] EU_target_reqs = (sweep_dir == UP)   ? (EU_reg & floors_above_or_at) : EU_reg;
    wire [N:0] ED_target_reqs = (sweep_dir == DOWN) ? (ED_reg & floors_below_or_at) : ED_reg;

    // ICC SCAN: requests ahead of the car in the sweep direction.
    wire [N:0] ICC_up_reqs   = (ICC_reg | EU_reg) & floors_above_or_at;
    wire [N:0] ICC_down_reqs = (ICC_reg | ED_reg) & floors_below_or_at;

    always @(*) begin
        case (TF_sel)
            EU_SERVICE:  TF = min_one_hot(EU_target_reqs);
            ED_SERVICE:  TF = max_one_hot(ED_target_reqs);
            ICC_SERVICE: TF = service_dir ? max_one_hot(ICC_up_reqs)     // farthest ahead, going up
                                          : min_one_hot(ICC_down_reqs);  // farthest ahead, going down
            default:     TF = min_one_hot(EU_reg | ICC_reg | ED_reg);
        endcase
    end

    // ============================================================
    // DIRECTION OUTPUT (combinational): direction of the NEXT move.
    // On every step_tick the floor moves exactly as this says.
    // ============================================================

    always @(*) begin
        disp_dir = STATIONARY;
        if (fire_alarm) begin
            if      (CF < SAFE_FLOOR_OH) disp_dir = UP;
            else if (CF > SAFE_FLOOR_OH) disp_dir = DOWN;
        end
        else if (!e_stop) begin
            case (current_state)
                EU_SERVICE, ED_SERVICE, ICC_SERVICE: begin
                    if (TF != 0 && TF != CF) begin
                        if (TF > CF) begin
                            if (!stop_up)   disp_dir = UP;
                        end
                        else begin
                            if (!stop_down) disp_dir = DOWN;
                        end
                    end
                end
                default: disp_dir = STATIONARY;   // PARK, WAIT_*, FIRE_EVAC, ...
            endcase
        end
    end

    // ============================================================
    // SEQUENTIAL FSM
    // ============================================================

    always @(posedge clock or posedge reset) begin

        if (reset) begin
            CF            <= {{N{1'b0}}, 1'b1};
            TF_sel        <= ICC_SERVICE;
            EU_reg        <= {(N+1){1'b0}};
            ED_reg        <= {(N+1){1'b0}};
            ICC_reg       <= {(N+1){1'b0}};
            service_dir   <= 1'b0;
            current_state <= PARK;
            sweep_dir     <= STATIONARY;
        end

        else begin

            // ---------------- REQUEST LATCHING AND MODES ----------------
            if (fire_alarm) begin
                EU_reg  <= {(N+1){1'b0}};
                ED_reg  <= {(N+1){1'b0}};
                ICC_reg <= {(N+1){1'b0}};
            end
            else if (manual_mode) begin
                // Manual mode: hall calls ignored, cabin calls accepted.
                EU_reg  <= {(N+1){1'b0}};
                ED_reg  <= {(N+1){1'b0}};
                ICC_reg <= ICC_in;
            end
            else begin
                EU_reg  <= EU_in;
                ED_reg  <= ED_in;
                ICC_reg <= ICC_in;
            end

            // ---------------- PRIORITY OVERRIDES ----------------
            if (fire_alarm) begin
                // Highest priority: evacuate to the safe floor, one floor per tick.
                sweep_dir <= STATIONARY;

                if (current_state != WAIT_INDEFINITE)
                    current_state <= FIRE_EVAC;

                if (step_tick) begin
                    if (CF < SAFE_FLOOR_OH)
                        CF <= CF << 1;
                    else if (CF > SAFE_FLOOR_OH)
                        CF <= CF >> 1;
                    else
                        current_state <= WAIT_INDEFINITE;
                end
            end

            else if (e_stop) begin
                // Stop and park; sweep_dir is kept so pending calls resume.
                current_state <= PARK;
            end

            else if (step_tick) begin

                case (current_state)

                    // ---------------------- PARK ----------------------
                    PARK: begin
                        if (overload) begin
                            current_state <= WAIT;       // doors held open
                        end
                        else if (ICC_in != 0) begin
                            current_state <= ICC_SERVICE;
                            TF_sel        <= ICC_SERVICE;
                        end
                        else if (EU_in != 0) begin
                            current_state <= EU_SERVICE;
                            TF_sel        <= EU_SERVICE;
                        end
                        else if (ED_in != 0) begin
                            current_state <= ED_SERVICE;
                            TF_sel        <= ED_SERVICE;
                        end
                        else begin
                            current_state <= PARK;
                            sweep_dir     <= STATIONARY;  // next request starts a fresh sweep
                        end
                    end

                    // Only reached after the alarm has dropped.
                    FIRE_EVAC: begin
                        current_state <= PARK;
                        sweep_dir     <= STATIONARY;
                    end

                    // ----------------- EXTERNAL UP --------------------
                    EU_SERVICE: begin
                        if (TF == 0) begin
                            if (EU_reg == 0) current_state <= PARK;
                            else             sweep_dir     <= DOWN;   // none ahead: reverse
                        end
                        else if (TF == CF) begin
                            EU_reg        <= EU_in  & ~CF;
                            ICC_reg       <= ICC_in & ~CF;
                            service_dir   <= 1'b1;
                            sweep_dir     <= UP;
                            current_state <= WAIT;
                        end
                        else if (TF > CF) begin
                            if (stop_up) begin
                                ICC_reg       <= ICC_in & ~CF;
                                EU_reg        <= EU_in  & ~CF;
                                sweep_dir     <= UP;
                                current_state <= WAIT_EU_UP;
                            end
                            else begin
                                CF        <= CF << 1;
                                sweep_dir <= UP;
                            end
                        end
                        else begin // TF < CF
                            if (stop_down) begin
                                ICC_reg       <= ICC_in & ~CF;
                                ED_reg        <= ED_in  & ~CF;
                                sweep_dir     <= DOWN;
                                current_state <= WAIT_EU_DOWN;
                            end
                            else begin
                                CF        <= CF >> 1;
                                sweep_dir <= DOWN;
                            end
                        end
                    end

                    // ----------------- EXTERNAL DOWN ------------------
                    ED_SERVICE: begin
                        if (TF == 0) begin
                            if (ED_reg == 0) current_state <= PARK;
                            else             sweep_dir     <= UP;     // none ahead: reverse
                        end
                        else if (TF == CF) begin
                            ED_reg        <= ED_in  & ~CF;
                            ICC_reg       <= ICC_in & ~CF;
                            service_dir   <= 1'b0;
                            sweep_dir     <= DOWN;
                            current_state <= WAIT;
                        end
                        else if (TF > CF) begin
                            if (stop_up) begin
                                ICC_reg       <= ICC_in & ~CF;
                                EU_reg        <= EU_in  & ~CF;
                                sweep_dir     <= UP;
                                current_state <= WAIT_ED_UP;
                            end
                            else begin
                                CF        <= CF << 1;
                                sweep_dir <= UP;
                            end
                        end
                        else begin // TF < CF
                            if (stop_down) begin
                                ICC_reg       <= ICC_in & ~CF;
                                ED_reg        <= ED_in  & ~CF;
                                sweep_dir     <= DOWN;
                                current_state <= WAIT_ED_DOWN;
                            end
                            else begin
                                CF        <= CF >> 1;
                                sweep_dir <= DOWN;
                            end
                        end
                    end

                    // ---------------- INTERNAL CABIN (SCAN) -----------
                    ICC_SERVICE: begin
                        if (TF == 0) begin
                            // Nothing ahead in the sweep direction. Reverse if
                            // something is waiting the other way, else park.
                            if (service_dir ? (ICC_down_reqs != 0) : (ICC_up_reqs != 0))
                                service_dir   <= ~service_dir;
                            else
                                current_state <= PARK;
                        end
                        else if (TF == CF) begin
                            // End of the sweep: clear the cabin call and only the
                            // hall call that matches the sweep direction.
                            ICC_reg <= ICC_in & ~CF;
                            if (service_dir) EU_reg <= EU_in & ~CF;
                            else             ED_reg <= ED_in & ~CF;
                            sweep_dir     <= service_dir ? UP : DOWN;
                            current_state <= WAIT;
                        end
                        else if (TF > CF) begin
                            if (stop_up) begin
                                ICC_reg       <= ICC_in & ~CF;
                                EU_reg        <= EU_in  & ~CF;
                                sweep_dir     <= UP;
                                current_state <= WAIT_ICC_UP;
                            end
                            else begin
                                CF        <= CF << 1;
                                sweep_dir <= UP;
                            end
                        end
                        else begin // TF < CF
                            if (stop_down) begin
                                ICC_reg       <= ICC_in & ~CF;
                                ED_reg        <= ED_in  & ~CF;
                                sweep_dir     <= DOWN;
                                current_state <= WAIT_ICC_DOWN;
                            end
                            else begin
                                CF        <= CF >> 1;
                                sweep_dir <= DOWN;
                            end
                        end
                    end

                    // -------------------- WAITS -----------------------
                    // After the door wait the car goes back to its service
                    // state and re-decides (no blind shift).
                    WAIT: begin
                        if (wait_done) current_state <= PARK;
                    end

                    // Only reached after the alarm has dropped.
                    WAIT_INDEFINITE: begin
                        if (!fire_alarm) current_state <= PARK;
                    end

                    WAIT_EU_UP,
                    WAIT_EU_DOWN: begin
                        if (wait_done) current_state <= EU_SERVICE;
                    end

                    WAIT_ED_UP,
                    WAIT_ED_DOWN: begin
                        if (wait_done) current_state <= ED_SERVICE;
                    end

                    WAIT_ICC_UP,
                    WAIT_ICC_DOWN: begin
                        if (wait_done) current_state <= ICC_SERVICE;
                    end

                    default: current_state <= PARK;
                endcase
            end
        end
    end

endmodule