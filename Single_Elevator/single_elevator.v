`timescale 1ns / 1ps

module single_elevator #(
    parameter N = 50,
    parameter WAIT_CYCLES = 5,
    parameter SAFE_FLOOR_INDEX = 0
)(
    input clock,
    input reset,
    input e_stop,
    input manual_mode,
    input fire_alarm,
    input overload,

    input [N:0] EU_switch,
    input [N:0] ED_switch,
    input [N:0] ICC_switch,

    output [N:0] disp_floor,
    output reg [1:0] disp_dir,
    output reg service_dir,
    output idle_flag,
    output reg just_wait,
    output door_open
);

    // ============================================================
    // INTERNAL REGISTERS
    // ============================================================

    reg [3:0] TF_sel;
    reg [N:0] TF, CF, EU_reg, ED_reg, ICC_reg;
    reg [3:0] current_state;

    // Remembers the current sweep direction even when
    // disp_dir becomes STATIONARY during door-open intervals.
    reg [1:0] sweep_dir;

    assign disp_floor = CF;

    // Convert safe floor integer index to one-hot vector
    localparam [N:0] SAFE_FLOOR_OH = (1'b1 << SAFE_FLOOR_INDEX);

    // ============================================================
    // DIRECTION ENCODINGS
    // ============================================================

    localparam [1:0] STATIONARY = 2'b00,
                     UP         = 2'b01,
                     DOWN       = 2'b10;

    // ============================================================
    // FSM STATE ENCODINGS
    // ============================================================

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
    // IDLE FLAG
    // ============================================================

    assign idle_flag = (current_state == PARK) &&
                       (EU_reg == {(N+1){1'b0}}) &&
                       (ED_reg == {(N+1){1'b0}}) &&
                       (ICC_reg == {(N+1){1'b0}}) &&
                       !fire_alarm;

    // ============================================================
    // WAIT TIMER AND DOOR CONTROL
    // ============================================================

    wire wait_enable;
    wire wait_done;

    assign wait_enable = (current_state == WAIT)          ||
                         (current_state == WAIT_EU_UP)     ||
                         (current_state == WAIT_EU_DOWN)   ||
                         (current_state == WAIT_ED_UP)     ||
                         (current_state == WAIT_ED_DOWN)   ||
                         (current_state == WAIT_ICC_UP)    ||
                         (current_state == WAIT_ICC_DOWN);

    wait_counter #(
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
    // MINIMUM AND MAXIMUM ONE-HOT REQUEST SELECTION
    // ============================================================

    // Returns the lowest asserted request bit.
    function [N:0] min_one_hot(input [N:0] req);
        begin
            min_one_hot = req & (~req + 1'b1);
        end
    endfunction

    // Returns the highest asserted request bit.
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

    // ============================================================
    // CURRENT-FLOOR MASK GENERATION
    // ============================================================

    // Mask: current floor and every floor above it.
    function [N:0] mask_above_or_at(input [N:0] floor_oh);
        integer i;
        reg found;
        begin
            mask_above_or_at = {(N+1){1'b0}};
            found = 1'b0;

            for (i = 0; i <= N; i = i + 1) begin
                if (floor_oh[i])
                    found = 1'b1;

                if (found)
                    mask_above_or_at[i] = 1'b1;
            end
        end
    endfunction

    // Mask: current floor and every floor below it.
    function [N:0] mask_below_or_at(input [N:0] floor_oh);
        integer i;
        reg found;
        begin
            mask_below_or_at = {(N+1){1'b0}};
            found = 1'b0;

            for (i = N; i >= 0; i = i - 1) begin
                if (floor_oh[i])
                    found = 1'b1;

                if (found)
                    mask_below_or_at[i] = 1'b1;
            end
        end
    endfunction

    // ============================================================
    // DIRECTION-DEPENDENT EXTERNAL REQUEST FILTERING
    // ============================================================

    wire [N:0] floors_above_or_at;
    wire [N:0] floors_below_or_at;

    wire [N:0] EU_target_reqs;
    wire [N:0] ED_target_reqs;

    assign floors_above_or_at = mask_above_or_at(CF);
    assign floors_below_or_at = mask_below_or_at(CF);

    // External UP calls:
    // Moving UP   -> consider calls at/above current floor.
    // Moving DOWN -> consider all pending UP calls.
    //
    // STATIONARY also considers all requests when beginning a
    // new sweep from PARK.

    assign EU_target_reqs = (sweep_dir == UP) ?
                            (EU_reg & floors_above_or_at) :
                            EU_reg;

    // External DOWN calls:
    // Moving DOWN -> consider calls at/below current floor.
    // Moving UP   -> consider all pending DOWN calls.

    assign ED_target_reqs = (sweep_dir == DOWN) ?
                            (ED_reg & floors_below_or_at) :
                            ED_reg;

    // ============================================================
    // TARGET FLOOR SELECTION
    // ============================================================

    always @(*) begin
        case (TF_sel)

            // External UP service
            EU_SERVICE:
                TF = min_one_hot(EU_target_reqs);

            // External DOWN service
            ED_SERVICE:
                TF = max_one_hot(ED_target_reqs);

            // Internal cabin call selection:
            // Preserve the original selection behaviour.
            ICC_SERVICE: begin
                if (service_dir == 1'b0)
                    TF = min_one_hot(ICC_reg | ED_reg);
                else
                    TF = max_one_hot(ICC_reg | EU_reg);
            end

            default:
                TF = min_one_hot(EU_reg | ICC_reg | ED_reg);

        endcase
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

            disp_dir      <= STATIONARY;
            sweep_dir     <= STATIONARY;

            just_wait     <= 1'b0;

        end

        else begin

            // ====================================================
            // REQUEST LATCHING AND MODE HANDLING
            // ====================================================

            if (fire_alarm) begin

                // Fire mode: clear all pending requests.
                EU_reg  <= {(N+1){1'b0}};
                ED_reg  <= {(N+1){1'b0}};
                ICC_reg <= {(N+1){1'b0}};

            end

            else if (manual_mode) begin

                // Manual mode: ignore external hall calls.
                EU_reg  <= {(N+1){1'b0}};
                ED_reg  <= {(N+1){1'b0}};

                // Cabin requests remain accepted.
                ICC_reg <= ICC_reg | ICC_switch;

            end

            else begin

                // Normal mode: latch all requests.
                EU_reg  <= EU_reg  | EU_switch;
                ED_reg  <= ED_reg  | ED_switch;
                ICC_reg <= ICC_reg | ICC_switch;

            end

            // ====================================================
            // PRIORITY OVERRIDES
            // ====================================================

            if (fire_alarm) begin

                // Highest priority: fire evacuation.
                // Normal sweep direction is no longer relevant.
                sweep_dir <= STATIONARY;

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

                    // Arrived at the safe floor.
                    disp_dir      <= STATIONARY;
                    current_state <= WAIT_INDEFINITE;

                    // Hold the doors open during fire mode.
                    just_wait <= 1'b1;

                end

            end

            else if (e_stop) begin

                // Emergency stop: stop movement and park.
                // Preserve sweep_dir so pending calls can
                // resume in the previous sweep direction.
                disp_dir      <= STATIONARY;
                current_state <= PARK;

            end

            else begin

                case (current_state)

                    // =================================================
                    // PARK STATE
                    // =================================================

                    PARK: begin

                        disp_dir <= STATIONARY;

                        if (overload) begin

                            // Keep doors open until overload clears.
                            current_state <= WAIT;

                        end

                        else if ((ICC_reg | ICC_switch) != 0) begin

                            current_state <= ICC_SERVICE;
                            TF_sel        <= ICC_SERVICE;

                        end

                        // In manual mode, normal request registers
                        // remain cleared.
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

                            // No requests remain anywhere.
                            // A future request starts a fresh sweep.
                            sweep_dir <= STATIONARY;

                        end

                    end

                    // =================================================
                    // FIRE EVACUATION STATE
                    // =================================================

                    FIRE_EVAC: begin

                        if (!fire_alarm) begin

                            current_state <= PARK;
                            sweep_dir     <= STATIONARY;

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

                            disp_dir      <= STATIONARY;
                            current_state <= WAIT_INDEFINITE;
                            just_wait     <= 1'b1;

                        end

                    end

                    // =================================================
                    // EXTERNAL UP CALL SERVICE
                    // =================================================

                    EU_SERVICE: begin

                        if (TF == 0) begin

                            if (EU_reg == 0) begin

                                // No UP calls remain.
                                current_state <= PARK;
                                disp_dir      <= STATIONARY;

                            end

                            else begin

                                // UP calls remain, but none are
                                // eligible ahead in the current UP
                                // sweep. Reverse the sweep direction.
                                //
                                // The next target calculation will
                                // consider all pending UP calls.

                                sweep_dir <= DOWN;
                                disp_dir  <= STATIONARY;

                            end

                        end

                        else if (TF == CF) begin

                            // Service the UP call at this floor.
                            EU_reg  <= (EU_reg  | EU_switch)  & ~CF;
                            ICC_reg <= (ICC_reg | ICC_switch) & ~CF;

                            service_dir   <= 1'b1;

                            disp_dir      <= STATIONARY;
                            sweep_dir     <= UP;

                            current_state <= WAIT;

                        end

                        else if (TF < CF) begin

                            // Target is below the current floor.
                            if ((CF & ICC_reg) != 0) begin

                                // Stop for an internal cabin call.
                                ICC_reg <= (ICC_reg | ICC_switch) & ~CF;

                                disp_dir      <= STATIONARY;
                                sweep_dir     <= DOWN;
                                current_state <= WAIT_EU_DOWN;

                            end

                            else begin

                                // Move down towards the lowest UP call.
                                CF       <= CF >> 1;
                                disp_dir <= DOWN;
                                sweep_dir <= DOWN;

                            end

                        end

                        else if (TF > CF) begin

                            // Target is above the current floor.
                            if ((CF & (ICC_reg | EU_reg)) != 0) begin

                                // Preserve the existing intermediate
                                // stop handling.
                                ICC_reg <= (ICC_reg | ICC_switch) & ~CF;
                                EU_reg  <= (EU_reg  | EU_switch)  & ~CF;

                                disp_dir      <= STATIONARY;
                                sweep_dir     <= UP;
                                current_state <= WAIT_EU_UP;

                            end

                            else begin

                                // Move up towards the eligible UP call.
                                CF       <= CF << 1;
                                disp_dir <= UP;
                                sweep_dir <= UP;

                            end

                        end

                    end

                    // =================================================
                    // EXTERNAL DOWN CALL SERVICE
                    // =================================================

                    ED_SERVICE: begin

                        if (TF == 0) begin

                            if (ED_reg == 0) begin

                                // No DOWN calls remain.
                                current_state <= PARK;
                                disp_dir      <= STATIONARY;

                            end

                            else begin

                                // DOWN calls remain, but none are
                                // eligible below in the current DOWN
                                // sweep. Reverse the sweep direction.
                                //
                                // The next target calculation will
                                // consider all pending DOWN calls.

                                sweep_dir <= UP;
                                disp_dir  <= STATIONARY;

                            end

                        end

                        else if (TF == CF) begin

                            // Service the DOWN call at this floor.
                            ED_reg  <= (ED_reg  | ED_switch)  & ~CF;
                            ICC_reg <= (ICC_reg | ICC_switch) & ~CF;

                            service_dir   <= 1'b0;

                            disp_dir      <= STATIONARY;
                            sweep_dir     <= DOWN;

                            current_state <= WAIT;

                        end

                        else if (TF > CF) begin

                            // Target is above the current floor.
                            if ((CF & ICC_reg) != 0) begin

                                // Stop for an internal cabin call.
                                ICC_reg <= (ICC_reg | ICC_switch) & ~CF;

                                disp_dir      <= STATIONARY;
                                sweep_dir     <= UP;
                                current_state <= WAIT_ED_UP;

                            end

                            else begin

                                // Move up towards the highest DOWN call.
                                CF       <= CF << 1;
                                disp_dir <= UP;
                                sweep_dir <= UP;

                            end

                        end

                        else if (TF < CF) begin

                            // Target is below the current floor.
                            if ((CF & (ICC_reg | ED_reg)) != 0) begin

                                // Preserve the existing intermediate
                                // stop handling.
                                ICC_reg <= (ICC_reg | ICC_switch) & ~CF;
                                ED_reg  <= (ED_reg  | ED_switch)  & ~CF;

                                disp_dir      <= STATIONARY;
                                sweep_dir     <= DOWN;
                                current_state <= WAIT_ED_DOWN;

                            end

                            else begin

                                // Move down towards the eligible DOWN call.
                                CF       <= CF >> 1;
                                disp_dir <= DOWN;
                                sweep_dir <= DOWN;

                            end

                        end

                    end

                    // =================================================
                    // INTERNAL CABIN CALL SERVICE
                    // =================================================

                    ICC_SERVICE: begin

                        if (TF == 0) begin

                            current_state <= PARK;
                            disp_dir      <= STATIONARY;

                        end

                        else if (TF == CF) begin

                            ED_reg  <= (ED_reg  | ED_switch)  & ~CF;
                            EU_reg  <= (EU_reg  | EU_switch)  & ~CF;
                            ICC_reg <= (ICC_reg | ICC_switch) & ~CF;

                            disp_dir      <= STATIONARY;
                            current_state <= WAIT;

                        end

                        else if (TF > CF) begin

                            if ((CF & (ICC_reg | EU_reg)) != 0) begin

                                ICC_reg <= (ICC_reg | ICC_switch) & ~CF;
                                EU_reg  <= (EU_reg  | EU_switch)  & ~CF;

                                disp_dir      <= STATIONARY;
                                sweep_dir     <= UP;
                                current_state <= WAIT_ICC_UP;

                            end

                            else begin

                                CF       <= CF << 1;
                                disp_dir <= UP;
                                sweep_dir <= UP;

                            end

                        end

                        else if (TF < CF) begin

                            if ((CF & (ICC_reg | ED_reg)) != 0) begin

                                ICC_reg <= (ICC_reg | ICC_switch) & ~CF;
                                ED_reg  <= (ED_reg  | ED_switch)  & ~CF;

                                disp_dir      <= STATIONARY;
                                sweep_dir     <= DOWN;
                                current_state <= WAIT_ICC_DOWN;

                            end

                            else begin

                                CF       <= CF >> 1;
                                disp_dir <= DOWN;
                                sweep_dir <= DOWN;

                            end

                        end

                    end

                    // =================================================
                    // NORMAL DOOR WAIT
                    // =================================================

                    WAIT: begin

                        disp_dir <= STATIONARY;

                        if (wait_done)
                            current_state <= PARK;

                    end

                    // =================================================
                    // INDEFINITE FIRE-MODE WAIT
                    // =================================================

                    WAIT_INDEFINITE: begin

                        disp_dir  <= STATIONARY;
                        just_wait <= 1'b1;

                        if (!fire_alarm) begin

                            current_state <= PARK;
                            just_wait     <= 1'b0;

                        end

                    end

                    // =================================================
                    // DIRECTIONAL WAIT STATES
                    // =================================================

                    WAIT_EU_UP: begin

                        disp_dir <= STATIONARY;

                        if (wait_done) begin

                            CF            <= CF << 1;
                            disp_dir      <= UP;
                            sweep_dir     <= UP;
                            current_state <= EU_SERVICE;

                        end

                    end

                    WAIT_EU_DOWN: begin

                        disp_dir <= STATIONARY;

                        if (wait_done) begin

                            CF            <= CF >> 1;
                            disp_dir      <= DOWN;
                            sweep_dir     <= DOWN;
                            current_state <= EU_SERVICE;

                        end

                    end

                    WAIT_ED_UP: begin

                        disp_dir <= STATIONARY;

                        if (wait_done) begin

                            CF            <= CF << 1;
                            disp_dir      <= UP;
                            sweep_dir     <= UP;
                            current_state <= ED_SERVICE;

                        end

                    end

                    WAIT_ED_DOWN: begin

                        disp_dir <= STATIONARY;

                        if (wait_done) begin

                            CF            <= CF >> 1;
                            disp_dir      <= DOWN;
                            sweep_dir     <= DOWN;
                            current_state <= ED_SERVICE;

                        end

                    end

                    WAIT_ICC_UP: begin

                        disp_dir <= STATIONARY;

                        if (wait_done) begin

                            CF            <= CF << 1;
                            disp_dir      <= UP;
                            sweep_dir     <= UP;
                            current_state <= ICC_SERVICE;

                        end

                    end

                    WAIT_ICC_DOWN: begin

                        disp_dir <= STATIONARY;

                        if (wait_done) begin

                            CF            <= CF >> 1;
                            disp_dir      <= DOWN;
                            sweep_dir     <= DOWN;
                            current_state <= ICC_SERVICE;

                        end

                    end

                    default: begin
                        current_state <= PARK;
                    end

                endcase

            end

        end

    end

endmodule