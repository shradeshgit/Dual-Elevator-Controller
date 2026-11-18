`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: hall_call_dispatcher
//
// Hands each external (hall) call to exactly one of two cars.
//
// Cost for a call at floor f, per car (lowest wins, car 1 wins ties):
//   Tier 1  car already sweeping toward the call, call is ahead of it  : dist
//           (EU call: sweep_dir UP   and car floor <= f)
//           (ED call: sweep_dir DOWN and car floor >= f)
//   Tier 2  car parked                                                 : dist + (N+1)
//   Tier 3  anything else (busy, moving away)                          : dist + 2*(N+1)
//   A car that is e_stop / manual / overload is not a candidate.
//
// Assignment is sticky: a call is costed once, when it is first pressed, and
// only the chosen car latches it. The dispatcher only takes a call back (using
// the car's cross_clear port) when the owner becomes unavailable
// (e_stop / manual / overload), and then re-costs it. If no car is available
// the call is held here until one is. fire_alarm drops every call, like the
// cars do.
//
// Hall switches are edge-detected: a call is registered on the rising edge, so
// a switch left on does not re-request after it has been served.
//////////////////////////////////////////////////////////////////////////////////

module hall_call_dispatcher #(
    parameter N      = 2,
    parameter COST_W = 10
)(
    input              clock,
    input              reset,
    input              fire_alarm,

    input  [N:0]       EU_switch,
    input  [N:0]       ED_switch,

    // car status (car 1 / car 2)
    input  [N:0]       CF_1, CF_2,
    input  [1:0]       sweep_dir_1, sweep_dir_2,
    input              parked_1, parked_2,
    input              avail_1, avail_2,        // not e_stop, not manual, not overload
    input  [N:0]       EU_pending_1, EU_pending_2,
    input  [N:0]       ED_pending_1, ED_pending_2,

    // calls handed to each car (one-cycle pulses into the car's EU/ED_switch)
    output reg [N:0]   EU_to_1, EU_to_2,
    output reg [N:0]   ED_to_1, ED_to_2,
    // calls taken back from a car (into the car's cross_clear_EU/ED)
    output reg [N:0]   clr_EU_1, clr_EU_2,
    output reg [N:0]   clr_ED_1, clr_ED_2
);

    localparam [1:0] STATIONARY = 2'b00,
                     UP         = 2'b01,
                     DOWN       = 2'b10;

    localparam [COST_W-1:0] TIER2_PEN = N + 1;
    localparam [COST_W-1:0] TIER3_PEN = 2 * (N + 1);

    // ------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------
    function [31:0] onehot_to_bin(input [N:0] oh);
        integer k;
        begin
            onehot_to_bin = 0;
            for (k = 0; k <= N; k = k + 1)
                if (oh[k]) onehot_to_bin = k;
        end
    endfunction

    function [COST_W-1:0] car_cost(
        input [31:0] fl,
        input [31:0] cb,
        input        want_up,       // 1 = UP hall call, 0 = DOWN hall call
        input [1:0]  sd,
        input        is_parked
    );
        reg [31:0] dist, c;
        reg        en_route;
        begin
            dist     = (cb > fl) ? (cb - fl) : (fl - cb);
            en_route = want_up ? (sd == UP   && cb <= fl)
                               : (sd == DOWN && cb >= fl);
            if (en_route)       c = dist;
            else if (is_parked) c = dist + TIER2_PEN;
            else                c = dist + TIER3_PEN;
            car_cost = c[COST_W-1:0];
        end
    endfunction

    // ------------------------------------------------------------
    // state: per floor, per direction
    //   pend : call outstanding   asg : a car holds it   own : 0 = car 1, 1 = car 2
    // ------------------------------------------------------------
    reg [N:0] pend_EU, asg_EU, own_EU;
    reg [N:0] pend_ED, asg_ED, own_ED;
    reg [N:0] EU_prev, ED_prev;

    wire [N:0] new_EU = EU_switch & ~EU_prev;
    wire [N:0] new_ED = ED_switch & ~ED_prev;

    reg [N:0] pend_EU_n, asg_EU_n, own_EU_n;
    reg [N:0] pend_ED_n, asg_ED_n, own_ED_n;

    integer          i;
    reg [31:0]       b1, b2;
    reg [COST_W-1:0] c1, c2;
    reg              p2, anyav, owner_ok, req, serv;

    always @(*) begin
        b1    = onehot_to_bin(CF_1);
        b2    = onehot_to_bin(CF_2);
        anyav = avail_1 | avail_2;

        EU_to_1  = 0; EU_to_2  = 0; ED_to_1  = 0; ED_to_2  = 0;
        clr_EU_1 = 0; clr_EU_2 = 0; clr_ED_1 = 0; clr_ED_2 = 0;

        pend_EU_n = pend_EU; asg_EU_n = asg_EU; own_EU_n = own_EU;
        pend_ED_n = pend_ED; asg_ED_n = asg_ED; own_ED_n = own_ED;

        for (i = 0; i <= N; i = i + 1) begin

            // ================= UP hall call at floor i =================
            c1 = car_cost(i, b1, 1'b1, sweep_dir_1, parked_1);
            c2 = car_cost(i, b2, 1'b1, sweep_dir_2, parked_2);
            p2 = avail_2 && (!avail_1 || (c2 < c1));

            owner_ok = asg_EU[i] && (own_EU[i] ? avail_2 : avail_1);
            req      = pend_EU[i] | new_EU[i];
            serv     = owner_ok && !(own_EU[i] ? EU_pending_2[i] : EU_pending_1[i]);

            if (fire_alarm) begin
                pend_EU_n[i] = 1'b0;
                asg_EU_n[i]  = 1'b0;
            end
            else begin
                // owner became unavailable: take the call back from it
                if (asg_EU[i] && !owner_ok) begin
                    if (own_EU[i]) clr_EU_2[i] = 1'b1;
                    else           clr_EU_1[i] = 1'b1;
                end

                if (req && !owner_ok) begin              // needs (re)assignment
                    pend_EU_n[i] = 1'b1;
                    if (anyav) begin
                        asg_EU_n[i] = 1'b1;
                        own_EU_n[i] = p2;
                        if (p2) EU_to_2[i] = 1'b1;
                        else    EU_to_1[i] = 1'b1;
                    end
                    else begin
                        asg_EU_n[i] = 1'b0;              // hold until a car is available
                    end
                end
                else if (serv) begin                     // owner served it
                    pend_EU_n[i] = new_EU[i];
                    asg_EU_n[i]  = 1'b0;
                end
            end

            // ================ DOWN hall call at floor i ================
            c1 = car_cost(i, b1, 1'b0, sweep_dir_1, parked_1);
            c2 = car_cost(i, b2, 1'b0, sweep_dir_2, parked_2);
            p2 = avail_2 && (!avail_1 || (c2 < c1));

            owner_ok = asg_ED[i] && (own_ED[i] ? avail_2 : avail_1);
            req      = pend_ED[i] | new_ED[i];
            serv     = owner_ok && !(own_ED[i] ? ED_pending_2[i] : ED_pending_1[i]);

            if (fire_alarm) begin
                pend_ED_n[i] = 1'b0;
                asg_ED_n[i]  = 1'b0;
            end
            else begin
                if (asg_ED[i] && !owner_ok) begin
                    if (own_ED[i]) clr_ED_2[i] = 1'b1;
                    else           clr_ED_1[i] = 1'b1;
                end

                if (req && !owner_ok) begin
                    pend_ED_n[i] = 1'b1;
                    if (anyav) begin
                        asg_ED_n[i] = 1'b1;
                        own_ED_n[i] = p2;
                        if (p2) ED_to_2[i] = 1'b1;
                        else    ED_to_1[i] = 1'b1;
                    end
                    else begin
                        asg_ED_n[i] = 1'b0;
                    end
                end
                else if (serv) begin
                    pend_ED_n[i] = new_ED[i];
                    asg_ED_n[i]  = 1'b0;
                end
            end
        end
    end

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            pend_EU <= 0; asg_EU <= 0; own_EU <= 0;
            pend_ED <= 0; asg_ED <= 0; own_ED <= 0;
            EU_prev <= 0; ED_prev <= 0;
        end
        else begin
            pend_EU <= pend_EU_n; asg_EU <= asg_EU_n; own_EU <= own_EU_n;
            pend_ED <= pend_ED_n; asg_ED <= asg_ED_n; own_ED <= own_ED_n;
            EU_prev <= EU_switch;
            ED_prev <= ED_switch;
        end
    end

endmodule