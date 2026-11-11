`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.11.2026 12:41:41
// Design Name: 
// Module Name: wait_counter
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


module wait_counter #(
    parameter WAIT_CYCLES = 5)
    (
    input  clock,
    input  reset,                      
    input  enable,
    input just_wait,
    input overload,                    
    output done);
    
    reg [31:0] count;
    assign done = enable && (count == WAIT_CYCLES - 1) && !overload;
    
    always @(posedge clock or posedge reset) begin
        if (reset)
            count <= 0;                
        else if (!enable || just_wait || overload)
            count <= 0;                
        else if (done)
            count <= 0;                
        else
            count <= count + 1;
    end
endmodule