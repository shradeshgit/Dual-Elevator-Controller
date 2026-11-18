`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Self-checking testbench for dual_elevator (5 floors: 0..4).
// Needs your tick_generator (one-cycle tick every TICK_CYCLES clocks).
//////////////////////////////////////////////////////////////////////////////////

module tb_dual_elevator;

    localparam N = 4;

    reg clock = 0, reset = 1, fire_alarm = 0;
    reg [N:0] EU_switch = 0, ED_switch = 0, ICC_switch_1 = 0, ICC_switch_2 = 0;
    reg e_stop_1 = 0, e_stop_2 = 0, manual_mode_1 = 0, manual_mode_2 = 0;
    reg overload_1 = 0, overload_2 = 0;

    wire [N:0] disp_floor_1, disp_floor_2;
    wire [1:0] disp_dir_1, disp_dir_2;
    wire       idle_flag_1, idle_flag_2, just_wait_1, just_wait_2, door_open_1, door_open_2;

    dual_elevator #(.N(N), .WAIT_CYCLES(6), .TICK_CYCLES(4), .SAFE_FLOOR_INDEX(0)) dut (
        .clock(clock), .reset(reset), .fire_alarm(fire_alarm),
        .EU_switch(EU_switch), .ED_switch(ED_switch),
        .ICC_switch_1(ICC_switch_1), .e_stop_1(e_stop_1), .manual_mode_1(manual_mode_1), .overload_1(overload_1),
        .disp_floor_1(disp_floor_1), .disp_dir_1(disp_dir_1), .idle_flag_1(idle_flag_1),
        .just_wait_1(just_wait_1), .door_open_1(door_open_1),
        .ICC_switch_2(ICC_switch_2), .e_stop_2(e_stop_2), .manual_mode_2(manual_mode_2), .overload_2(overload_2),
        .disp_floor_2(disp_floor_2), .disp_dir_2(disp_dir_2), .idle_flag_2(idle_flag_2),
        .just_wait_2(just_wait_2), .door_open_2(door_open_2)
    );

    always #5 clock = ~clock;

    integer errors = 0;

    // ---------------------------------------------------------------
    // invariants, checked every clock
    // ---------------------------------------------------------------
    reg [N:0] pf1 = 1, pf2 = 1;      // floor / arrow / tick as seen just before the previous edge
    reg [1:0] pd1 = 0, pd2 = 0;
    reg       ptick = 0;
    reg [N:0] exp1, exp2;

    always @(posedge clock) begin
        if (reset) begin
            pf1 <= 1; pf2 <= 1; pd1 <= 0; pd2 <= 0;
        end
        else begin
            // one-hot floors
            if (disp_floor_1 == 0 || (disp_floor_1 & (disp_floor_1 - 1)) != 0) begin
                errors = errors + 1; $display("%0t FAIL car1 floor not one-hot: %b", $time, disp_floor_1); end
            if (disp_floor_2 == 0 || (disp_floor_2 & (disp_floor_2 - 1)) != 0) begin
                errors = errors + 1; $display("%0t FAIL car2 floor not one-hot: %b", $time, disp_floor_2); end

            // The floor only changes on a step tick, and then EXACTLY as the arrow
            // said in the cycle before: UP -> one floor up, DOWN -> one floor down,
            // STATIONARY -> no move.
            exp1 = pf1; exp2 = pf2;
            if (ptick) begin
                if (pd1 == 2'b01) exp1 = pf1 << 1; else if (pd1 == 2'b10) exp1 = pf1 >> 1;
                if (pd2 == 2'b01) exp2 = pf2 << 1; else if (pd2 == 2'b10) exp2 = pf2 >> 1;
            end
            if (disp_floor_1 != exp1) begin
                errors = errors + 1; $display("%0t FAIL car1 moved %b -> %b but arrow was %0d (tick=%b)", $time, pf1, disp_floor_1, pd1, ptick); end
            if (disp_floor_2 != exp2) begin
                errors = errors + 1; $display("%0t FAIL car2 moved %b -> %b but arrow was %0d (tick=%b)", $time, pf2, disp_floor_2, pd2, ptick); end

            // doors open => arrow is stationary
            if ((door_open_1 && disp_dir_1 != 0) || (door_open_2 && disp_dir_2 != 0)) begin
                errors = errors + 1; $display("%0t FAIL arrow not stationary with doors open", $time); end
            // a hall call is never pending in both cars
            if ((dut.EU_pending_1 & dut.EU_pending_2) != 0 || (dut.ED_pending_1 & dut.ED_pending_2) != 0) begin
                errors = errors + 1; $display("%0t FAIL hall call latched in both cars", $time); end

            pf1 <= disp_floor_1; pf2 <= disp_floor_2;
            pd1 <= disp_dir_1;   pd2 <= disp_dir_2;
        end
        ptick <= dut.step_tick;
    end

    // ---------------------------------------------------------------
    // event log: presses, dispatcher decisions, floor/direction updates,
    // doors open/closed and which calls were serviced.
    //   LOG = 0 silences everything, LOG_DIR = 0 hides the arrow-change lines.
    // ---------------------------------------------------------------
    localparam LOG     = 1;
    localparam LOG_DIR = 1;

    function integer fnum(input [N:0] oh);
        integer k;
        begin
            fnum = 0;
            for (k = 0; k <= N; k = k + 1)
                if (oh[k]) fnum = k;
        end
    endfunction

    function [8*8-1:0] dname(input [1:0] d);
        begin
            case (d)
                2'b01:   dname = "UP";
                2'b10:   dname = "DOWN";
                default: dname = "STOPPED";
            endcase
        end
    endfunction

    task print_svc(input [N:0] sI, input [N:0] sEU, input [N:0] sED);
        reg first;
        begin
            first = 1;
            if (sI != 0)  begin $write("cabin call");                              first = 0; end
            if (sEU != 0) begin if (!first) $write(" + "); $write("UP hall call");   first = 0; end
            if (sED != 0) begin if (!first) $write(" + "); $write("DOWN hall call"); first = 0; end
            if (first) $write("no call (door hold)");
        end
    endtask

    integer   c, li;
    reg [N:0] a_floor [1:2];   // values seen just before this edge
    reg [1:0] a_dir   [1:2];
    reg       a_door  [1:2];
    reg       a_jw    [1:2];
    reg [N:0] a_I     [1:2];
    reg [N:0] a_EU    [1:2];
    reg [N:0] a_ED    [1:2];
    reg [N:0] p_floor [1:2];   // values seen just before the previous edge
    reg [1:0] p_dir   [1:2];
    reg       p_door  [1:2];
    reg       p_jw    [1:2];
    reg [N:0] p_I     [1:2];
    reg [N:0] p_EU    [1:2];
    reg [N:0] p_ED    [1:2];
    reg [N:0] s_I     [1:2];   // calls cleared when the current stop began
    reg [N:0] s_EU    [1:2];
    reg [N:0] s_ED    [1:2];
    reg [N:0] lp_EU = 0, lp_ED = 0, lp_I1 = 0, lp_I2 = 0;

    initial begin
        for (c = 1; c <= 2; c = c + 1) begin
            p_floor[c] = 1; p_dir[c] = 0; p_door[c] = 0; p_jw[c] = 0;
            p_I[c] = 0; p_EU[c] = 0; p_ED[c] = 0;
            s_I[c] = 0; s_EU[c] = 0; s_ED[c] = 0;
        end
    end

    always @(posedge clock) begin
        if (LOG && !reset) begin
            a_floor[1] = disp_floor_1;  a_floor[2] = disp_floor_2;
            a_dir[1]   = disp_dir_1;    a_dir[2]   = disp_dir_2;
            a_door[1]  = door_open_1;   a_door[2]  = door_open_2;
            a_jw[1]    = just_wait_1;   a_jw[2]    = just_wait_2;
            a_I[1]  = dut.u_car1.ICC_reg;  a_I[2]  = dut.u_car2.ICC_reg;
            a_EU[1] = dut.EU_pending_1;    a_EU[2] = dut.EU_pending_2;
            a_ED[1] = dut.ED_pending_1;    a_ED[2] = dut.ED_pending_2;

            // ---- button presses and dispatcher decisions ----
            for (li = 0; li <= N; li = li + 1) begin
                if (EU_switch[li]     && !lp_EU[li]) $display("[%5d ns] PRESS   : UP hall call at floor %0d", $time, li);
                if (ED_switch[li]     && !lp_ED[li]) $display("[%5d ns] PRESS   : DOWN hall call at floor %0d", $time, li);
                if (ICC_switch_1[li]  && !lp_I1[li]) $display("[%5d ns] PRESS   : Car 1 cabin call to floor %0d", $time, li);
                if (ICC_switch_2[li]  && !lp_I2[li]) $display("[%5d ns] PRESS   : Car 2 cabin call to floor %0d", $time, li);

                if (dut.EU_to_1[li])  $display("[%5d ns] ASSIGN  : UP hall call at floor %0d -> Car 1", $time, li);
                if (dut.EU_to_2[li])  $display("[%5d ns] ASSIGN  : UP hall call at floor %0d -> Car 2", $time, li);
                if (dut.ED_to_1[li])  $display("[%5d ns] ASSIGN  : DOWN hall call at floor %0d -> Car 1", $time, li);
                if (dut.ED_to_2[li])  $display("[%5d ns] ASSIGN  : DOWN hall call at floor %0d -> Car 2", $time, li);
                if (dut.clr_EU_1[li]) $display("[%5d ns] RECLAIM : UP hall call at floor %0d taken back from Car 1", $time, li);
                if (dut.clr_EU_2[li]) $display("[%5d ns] RECLAIM : UP hall call at floor %0d taken back from Car 2", $time, li);
                if (dut.clr_ED_1[li]) $display("[%5d ns] RECLAIM : DOWN hall call at floor %0d taken back from Car 1", $time, li);
                if (dut.clr_ED_2[li]) $display("[%5d ns] RECLAIM : DOWN hall call at floor %0d taken back from Car 2", $time, li);
            end
            lp_EU = EU_switch; lp_ED = ED_switch; lp_I1 = ICC_switch_1; lp_I2 = ICC_switch_2;

            // ---- per car: floor display, direction, doors, serviced calls ----
            for (c = 1; c <= 2; c = c + 1) begin
                if (a_floor[c] != p_floor[c])
                    $display("[%5d ns] CAR %0d   : floor %0d -> %0d   (moving %0s)",
                             $time, c, fnum(p_floor[c]), fnum(a_floor[c]), dname(p_dir[c]));

                if (LOG_DIR && a_dir[c] != p_dir[c])
                    $display("[%5d ns] CAR %0d   : direction %0s (floor %0d)",
                             $time, c, dname(a_dir[c]), fnum(a_floor[c]));

                if (a_door[c] && !p_door[c]) begin
                    // the calls that were cleared at the edge the stop began
                    s_I[c]  = p_I[c]  & ~a_I[c]  & a_floor[c];
                    s_EU[c] = p_EU[c] & ~a_EU[c] & a_floor[c];
                    s_ED[c] = p_ED[c] & ~a_ED[c] & a_floor[c];
                    if (a_jw[c])
                        $display("[%5d ns] CAR %0d   : doors OPEN at floor %0d (fire: holding at the safe floor)",
                                 $time, c, fnum(a_floor[c]));
                    else
                        $display("[%5d ns] CAR %0d   : doors OPEN at floor %0d", $time, c, fnum(a_floor[c]));
                end

                if (!a_door[c] && p_door[c]) begin
                    if (p_jw[c])
                        $display("[%5d ns] CAR %0d   : doors CLOSED at floor %0d (fire alarm cleared)",
                                 $time, c, fnum(a_floor[c]));
                    else begin
                        $write("[%5d ns] CAR %0d   : doors CLOSED at floor %0d -> SERVICED: ",
                               $time, c, fnum(a_floor[c]));
                        print_svc(s_I[c], s_EU[c], s_ED[c]);
                        $write("\n");
                    end
                end

                p_floor[c] = a_floor[c]; p_dir[c] = a_dir[c];
                p_door[c]  = a_door[c];  p_jw[c]  = a_jw[c];
                p_I[c] = a_I[c]; p_EU[c] = a_EU[c]; p_ED[c] = a_ED[c];
            end
        end
    end

    // ---------------------------------------------------------------
    // helpers
    // ---------------------------------------------------------------
    task press_EU(input integer f);
        begin @(negedge clock); EU_switch = (1 << f); @(negedge clock); EU_switch = 0; end
    endtask
    task press_ED(input integer f);
        begin @(negedge clock); ED_switch = (1 << f); @(negedge clock); ED_switch = 0; end
    endtask
    task press_ICC1(input integer f);
        begin @(negedge clock); ICC_switch_1 = (1 << f); @(negedge clock); ICC_switch_1 = 0; end
    endtask
    task press_ICC2(input integer f);
        begin @(negedge clock); ICC_switch_2 = (1 << f); @(negedge clock); ICC_switch_2 = 0; end
    endtask

    task wait_idle;
        integer t;
        begin
            t = 0;
            repeat (3) @(negedge clock);
            while (!(idle_flag_1 && idle_flag_2) && t < 3000) begin @(negedge clock); t = t + 1; end
            if (t >= 3000) begin errors = errors + 1; $display("FAIL timeout waiting for idle"); end
            repeat (3) @(negedge clock);
        end
    endtask

    // send both cars to floor 0 with cabin calls
    task home;
        begin
            if (disp_floor_1 != 1) press_ICC1(0);
            if (disp_floor_2 != 1) press_ICC2(0);
            wait_idle;
        end
    endtask

    task section(input [8*64-1:0] msg);
        begin
            $display("");
            $display("=== %0s ===", msg);
        end
    endtask

    task check(input cond, input [8*64-1:0] msg);
        begin
            if (!cond) begin errors = errors + 1; $display("FAIL: %0s", msg); end
            else $display("ok  : %0s", msg);
        end
    endtask

    integer t, opens4;
    reg stopped2, visited0, prev_open, first_ok;

    initial begin
        repeat (4) @(negedge clock);
        reset = 0;
        repeat (4) @(negedge clock);

        section("T1 - tie goes to car 1 (UP call at 3)");
        // T1: both parked at 0, equal cost -> car 1 takes it
        press_EU(3); wait_idle;
        check(disp_floor_1 == (1 << 3) && disp_floor_2 == 1, "T1 tie -> car1 serves EU3, car2 stays at 0");

        section("T2 - nearest parked car serves DOWN call at 1");
        // T2: car 2 is closer (parked tier, dist 1 vs 2)
        press_ED(1); wait_idle;
        check(disp_floor_1 == (1 << 3) && disp_floor_2 == (1 << 1), "T2 nearest parked car (car2) serves ED1");

        section("T3 - nearest parked car serves UP call at 0");
        // T3: car 2 at 1 is closer to 0 than car 1 at 3
        press_EU(0); wait_idle;
        check(disp_floor_1 == (1 << 3) && disp_floor_2 == 1, "T3 car2 serves EU0");

        section("T4 - car 2 e-stop: car 1 must serve UP call at 1");
        // T4: car 2 e_stop -> only car 1 can take the call
        e_stop_2 = 1;
        press_EU(1); wait_idle;
        e_stop_2 = 0;
        check(disp_floor_1 == (1 << 1) && disp_floor_2 == 1, "T4 e_stop car2 -> car1 serves EU1");

        section("T5 - car 1 goes manual: call reassigned to car 2");
        // T5: call goes to car 1, then car 1 goes manual -> call is taken back and car 2 serves it
        press_EU(4);
        manual_mode_1 = 1;
        wait_idle;
        manual_mode_1 = 0;
        check(disp_floor_2 == (1 << 4) && disp_floor_1 == (1 << 1), "T5 manual car1 -> call reassigned to car2");

        section("T6 - en-route car 1 picks up UP call at 3");
        // T6: car 1 en route up for a cabin call; EU3 appears ahead of it -> car 1 takes it (tier 1)
        press_ICC1(3);
        t = 0;
        while (disp_floor_1 != (1 << 2) && t < 500) begin @(negedge clock); t = t + 1; end
        press_EU(3);
        wait_idle;
        check(disp_floor_1 == (1 << 3) && disp_floor_2 == (1 << 4), "T6 en-route car1 takes EU3, car2 stays at 4");

        section("T7 - both cars e-stopped: call held, then served");
        // T7: both cars e_stopped -> call is held until a car is free
        e_stop_1 = 1; e_stop_2 = 1;
        press_ED(0);
        repeat (40) @(negedge clock);
        check(disp_floor_1 == (1 << 3) && disp_floor_2 == (1 << 4), "T7 held call, neither car moves");
        e_stop_2 = 0;
        wait_idle;
        e_stop_1 = 0;
        check(disp_floor_2 == 1 && disp_floor_1 == (1 << 3), "T7 call served by car2 once it is free");

        section("T8 - overloaded car 1 is skipped");
        // T8: overload on car 1 -> doors held, call goes to car 2
        overload_1 = 1;
        repeat (10) @(negedge clock);
        check(door_open_1, "T8 overload keeps car1 doors open");
        press_ED(2);
        t = 0;
        while (disp_floor_2 != (1 << 2) && t < 500) begin @(negedge clock); t = t + 1; end
        repeat (20) @(negedge clock);
        check(disp_floor_2 == (1 << 2) && disp_floor_1 == (1 << 3), "T8 overloaded car1 skipped, car2 serves ED2");
        overload_1 = 0;
        wait_idle;

        section("T9 - fire alarm: both cars go to the safe floor");
        // T9: fire alarm -> both cars step down to floor 0 and hold with doors open
        fire_alarm = 1;
        t = 0;
        while (!(just_wait_1 && just_wait_2) && t < 1000) begin @(negedge clock); t = t + 1; end
        repeat (2) @(negedge clock);
        check(just_wait_1 && just_wait_2 && disp_floor_1 == 1 && disp_floor_2 == 1, "T9 both cars at safe floor, just_wait high");
        fire_alarm = 0;
        wait_idle;
        check(!just_wait_1 && !just_wait_2 && idle_flag_1 && idle_flag_2, "T9 fire cleared, cars idle again");

        section("T10 - UP call at 2 picked up on the way to DOWN call at 3");
        // T10 (item 1): car 1 is going UP to the ED call at 3. An EU call at 2 appears
        // ahead of it; the dispatcher counts car 1 as en route, so car 1 must actually
        // stop at 2 on the way (it used to drive past).
        home;
        e_stop_2 = 1;
        press_ED(3);
        t = 0;
        while (disp_floor_1 != (1 << 1) && t < 500) begin @(negedge clock); t = t + 1; end
        e_stop_2 = 0;
        press_EU(2);
        stopped2 = 0; t = 0;
        while (!(door_open_1 && disp_floor_1 == (1 << 3)) && t < 800) begin
            @(negedge clock); t = t + 1;
            if (door_open_1 && disp_floor_1 == (1 << 2)) stopped2 = 1;
        end
        wait_idle;
        check(stopped2, "T10 car1 stops for EU2 on its way up to ED3");
        check(disp_floor_1 == (1 << 3) && disp_floor_2 == 1, "T10 car1 ends at 3, car2 never moved");

        section("T11 - ICC SCAN: keep going up to 4, then reverse to 0");
        // T11 (item 2): ICC SCAN. Car 1 last travelled up and is at 2. Cabin calls at 0 and 4
        // arrive together: it keeps going UP to 4 first, then reverses to 0.
        home;
        press_ICC1(2); wait_idle;
        @(negedge clock); ICC_switch_1 = (1 << 0) | (1 << 4);
        @(negedge clock); ICC_switch_1 = 0;
        visited0 = 0; t = 0;
        while (!(door_open_1 && disp_floor_1 == (1 << 4)) && t < 800) begin
            @(negedge clock); t = t + 1;
            if (disp_floor_1 == 1) visited0 = 1;
        end
        check(!visited0 && disp_floor_1 == (1 << 4), "T11 SCAN keeps going up to 4 first");
        wait_idle;
        check(disp_floor_1 == 1, "T11 then reverses and serves 0");

        section("T12 - no blind shift after the door wait");
        // T12 (item 3): car 1 heads up for EU4 and stops at 2 for a cabin call. While the doors
        // are open the hall call is taken away (car 1 goes manual). After the door wait it
        // must stay at 2 (it used to shift up a floor blindly).
        home;
        press_EU(4);
        t = 0;
        while (disp_floor_1 != (1 << 1) && t < 500) begin @(negedge clock); t = t + 1; end
        press_ICC1(2);
        t = 0;
        while (!(door_open_1 && disp_floor_1 == (1 << 2)) && t < 500) begin @(negedge clock); t = t + 1; end
        manual_mode_1 = 1;
        t = 0;
        while (door_open_1 && t < 500) begin @(negedge clock); t = t + 1; end
        repeat (30) @(negedge clock);
        check(disp_floor_1 == (1 << 2), "T12 car1 stays at 2 after the door wait");
        manual_mode_1 = 0;
        wait_idle;
        check(disp_floor_2 == (1 << 4) && disp_floor_1 == (1 << 2), "T12 EU4 was served by car2");

        section("T13 - up sweep ends at 4: only the up call is cleared");
        // T13 (item 5): ICC4 and ED4 pending, car 1 sweeps UP to 4. At the end of the up sweep
        // only the cabin call is cleared; the DOWN call at 4 is still pending and is served
        // in a second door cycle.
        home;
        e_stop_2 = 1;
        @(negedge clock); ICC_switch_1 = (1 << 4); ED_switch = (1 << 4);
        @(negedge clock); ICC_switch_1 = 0;        ED_switch = 0;
        opens4 = 0; first_ok = 0; prev_open = 0; t = 0;
        while (t < 800 && !(t > 20 && idle_flag_1)) begin
            @(negedge clock); t = t + 1;
            if (door_open_1 && !prev_open && disp_floor_1 == (1 << 4)) begin
                opens4 = opens4 + 1;
                if (opens4 == 1) first_ok = (dut.ED_pending_1[4] == 1'b1);
            end
            prev_open = door_open_1;
        end
        repeat (3) @(negedge clock);
        e_stop_2 = 0;
        check(first_ok, "T13 ED4 still pending after the up-sweep stop");
        check(opens4 == 2, "T13 floor 4 gets two door cycles (up, then down)");

        section("T14 - ICC switch held on registers once");
        // T14: an ICC switch left ON registers once and does not hold the car at that floor.
        home;
        @(negedge clock); ICC_switch_1 = (1 << 2);
        t = 0;
        while (!(door_open_1 && disp_floor_1 == (1 << 2)) && t < 500) begin @(negedge clock); t = t + 1; end
        t = 0;
        while (door_open_1 && t < 500) begin @(negedge clock); t = t + 1; end
        repeat (40) @(negedge clock);
        check(idle_flag_1 && disp_floor_1 == (1 << 2), "T14 held ICC switch: one stop, then idle");
        ICC_switch_1 = 0;
        wait_idle;

        $display("----------------------------------------");
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d ERROR(S)", errors);
        $finish;
    end

    initial begin
        #8_000_000;
        $display("FAIL global timeout");
        $finish;
    end

endmodule