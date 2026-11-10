module wait_counter #(
    parameter WAIT_CYCLES = 5
)(
    input  clock,
    input  reset,                    
    input  enable,                   
    output done
);
    reg [31:0] count;
    assign done = enable && (count == WAIT_CYCLES - 1);
    
    always @(posedge clock or posedge reset) begin
        if (reset)
            count <= 0;                
        else if (!enable)
            count <= 0;                
        else if (done)
            count <= 0;                
        else
            count <= count + 1;
    end
endmodule