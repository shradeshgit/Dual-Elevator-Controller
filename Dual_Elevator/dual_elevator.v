`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: dual_elevator
//
// Two cars + hall call dispatcher.
//   Shared  : clock, reset, fire_alarm, EU_switch, ED_switch, step tick
//   Per car : ICC_switch, e_stop, manual_mode, overload
//
// Needs your existing tick_generator (clock, reset -> tick).
// One tick is shared so both cars step in lockstep.
//////////////////////////////////////////////////////////////////////////////////

module dual_elevator #(
    parameter N                = 2,
    parameter WAIT_CYCLES      = 10,
    parameter TICK_CYCLES      = 1000_000_000,
    parameter SAFE_FLOOR_INDEX = 0
)(
    input              clock,
    input              reset,
    input              fire_alarm,

    input  [N:0]       EU_switch,           // hall calls, shared
    input  [N:0]       ED_switch,

    // car 1
    input  [N:0]       ICC_switch_1,
    input              e_stop_1,
    input              manual_mode_1,
    input              overload_1,
    output [N:0]       disp_floor_1,
    output [1:0]       disp_dir_1,
    output             idle_flag_1,
    output             just_wait_1,
    output             door_open_1,

    // car 2
    input  [N:0]       ICC_switch_2,
    input              e_stop_2,
    input              manual_mode_2,
    input              overload_2,
    output [N:0]       disp_floor_2,
    output [1:0]       disp_dir_2,
    output             idle_flag_2,
    output             just_wait_2,
    output             door_open_2
);

    // ---------------- shared step tick ----------------
    wire step_tick;
    tick_generator #(.TICK_CYCLES(TICK_CYCLES)) u_step_timer (
        .clock (clock),
        .reset (reset),
        .tick  (step_tick)
    );

    // ---------------- dispatcher <-> cars ----------------
    wire [N:0] EU_to_1, EU_to_2, ED_to_1, ED_to_2;
    wire [N:0] clr_EU_1, clr_EU_2, clr_ED_1, clr_ED_2;

    wire [1:0] sweep_dir_1, sweep_dir_2;
    wire       parked_1, parked_2;
    wire [N:0] EU_pending_1, EU_pending_2, ED_pending_1, ED_pending_2;
    wire       service_dir_1, service_dir_2;

    wire avail_1 = !e_stop_1 && !manual_mode_1 && !overload_1;
    wire avail_2 = !e_stop_2 && !manual_mode_2 && !overload_2;

    hall_call_dispatcher #(.N(N)) u_dispatch (
        .clock        (clock),
        .reset        (reset),
        .fire_alarm   (fire_alarm),
        .EU_switch    (EU_switch),
        .ED_switch    (ED_switch),
        .CF_1         (disp_floor_1),
        .CF_2         (disp_floor_2),
        .sweep_dir_1  (sweep_dir_1),
        .sweep_dir_2  (sweep_dir_2),
        .parked_1     (parked_1),
        .parked_2     (parked_2),
        .avail_1      (avail_1),
        .avail_2      (avail_2),
        .EU_pending_1 (EU_pending_1),
        .EU_pending_2 (EU_pending_2),
        .ED_pending_1 (ED_pending_1),
        .ED_pending_2 (ED_pending_2),
        .EU_to_1      (EU_to_1),
        .EU_to_2      (EU_to_2),
        .ED_to_1      (ED_to_1),
        .ED_to_2      (ED_to_2),
        .clr_EU_1     (clr_EU_1),
        .clr_EU_2     (clr_EU_2),
        .clr_ED_1     (clr_ED_1),
        .clr_ED_2     (clr_ED_2)
    );

    // ---------------- car 1 ----------------
    elevator_car #(
        .N(N), .WAIT_CYCLES(WAIT_CYCLES), .SAFE_FLOOR_INDEX(SAFE_FLOOR_INDEX)
    ) u_car1 (
        .clock          (clock),
        .reset          (reset),
        .step_tick      (step_tick),
        .e_stop         (e_stop_1),
        .manual_mode    (manual_mode_1),
        .fire_alarm     (fire_alarm),
        .overload       (overload_1),
        .EU_switch      (EU_to_1),
        .ED_switch      (ED_to_1),
        .ICC_switch     (ICC_switch_1),
        .cross_clear_EU (clr_EU_1),
        .cross_clear_ED (clr_ED_1),
        .disp_floor     (disp_floor_1),
        .disp_dir       (disp_dir_1),
        .service_dir    (service_dir_1),
        .idle_flag      (idle_flag_1),
        .just_wait      (just_wait_1),
        .door_open      (door_open_1),
        .sweep_dir      (sweep_dir_1),
        .parked         (parked_1),
        .EU_pending     (EU_pending_1),
        .ED_pending     (ED_pending_1)
    );

    // ---------------- car 2 ----------------
    elevator_car #(
        .N(N), .WAIT_CYCLES(WAIT_CYCLES), .SAFE_FLOOR_INDEX(SAFE_FLOOR_INDEX)
    ) u_car2 (
        .clock          (clock),
        .reset          (reset),
        .step_tick      (step_tick),
        .e_stop         (e_stop_2),
        .manual_mode    (manual_mode_2),
        .fire_alarm     (fire_alarm),
        .overload       (overload_2),
        .EU_switch      (EU_to_2),
        .ED_switch      (ED_to_2),
        .ICC_switch     (ICC_switch_2),
        .cross_clear_EU (clr_EU_2),
        .cross_clear_ED (clr_ED_2),
        .disp_floor     (disp_floor_2),
        .disp_dir       (disp_dir_2),
        .service_dir    (service_dir_2),
        .idle_flag      (idle_flag_2),
        .just_wait      (just_wait_2),
        .door_open      (door_open_2),
        .sweep_dir      (sweep_dir_2),
        .parked         (parked_2),
        .EU_pending     (EU_pending_2),
        .ED_pending     (ED_pending_2)
    );

endmodule