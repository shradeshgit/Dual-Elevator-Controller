module tick_generator #(
    parameter TICK_CYCLES = 1 // 50M cycles @ 100MHz = 0.5 seconds
)(
    input clock,
    input reset,
    output tick
);
    reg [31:0] count;
    
    // Generates a 1-cycle pulse when count hits max
    assign tick = (count == TICK_CYCLES - 1);

    always @(posedge clock or posedge reset) begin
        if (reset)
            count <= 0;
        else if (tick)
            count <= 0;
        else
            count <= count + 1;
    end
endmodule