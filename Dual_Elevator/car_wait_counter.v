`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: car_wait_counter
//
// Replacement for wait_counter with two changes:
//   1. overload input: while overloaded the dwell never completes and restarts
//      from zero once the overload clears.
//   2. done is a LEVEL that stays high once the dwell has elapsed (until enable
//      drops). The old done was a one-clock pulse, so the FSM only saw it if it
//      happened to coincide with a step_tick, which works only when WAIT_CYCLES
//      is a multiple of the tick period.
//////////////////////////////////////////////////////////////////////////////////

module car_wait_counter #(
    parameter WAIT_CYCLES = 5
)(
    input  clock,
    input  reset,
    input  enable,
    input  just_wait,
    input  overload,
    output done
);

    reg [31:0] count;

    assign done = enable && !overload && (count >= WAIT_CYCLES - 1);

    always @(posedge clock or posedge reset) begin
        if (reset)
            count <= 0;
        else if ((!enable && !just_wait) || overload)
            count <= 0;
        else if (just_wait)
            count <= count;                      // hold
        else if (count < WAIT_CYCLES - 1)
            count <= count + 1;                  // saturate at WAIT_CYCLES-1
    end

endmodule