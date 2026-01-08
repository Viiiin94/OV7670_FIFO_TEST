`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/08/2026 11:18:42 AM
// Design Name: 
// Module Name: OV7670_controller
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


module OV7670_controller(
    input wire clk,           // 100MHz System Clock
    input wire resend,        // Active High Reset
    output wire config_done,  // 설정 완료 신호
    output reg sioc,          // I2C Clock
    inout  wire siod          // I2C Data
    );

    reg [15:0] command;
    reg [7:0] rom_addr;
    
    // I2C Signal Generation
    reg [7:0] slave_addr = 8'h42; // OV7670 Write Address
    reg [23:0] data_sr;           // Shift Register (ID + Addr + Val)
    reg [5:0] bit_count;
    reg [1:0] state;
    
    // Clock Divider for I2C (Target ~200kHz)
    reg [8:0] clk_div;
    wire i2c_tick = (clk_div == 2500); // 100MHz / 500 = 200kHz toggle
    
    // Tri-state buffer control
    reg siod_out;
    reg siod_oe;
    assign siod = (siod_oe) ? siod_out : 1'bz;
    assign config_done = (command == 16'hFFFF);

    // Initial Values
    initial begin
        sioc = 1;
        siod_oe = 1;
        siod_out = 1;
        state = 0;
        rom_addr = 0;
        bit_count = 0;
        clk_div = 0;
    end

// ROM: Register Settings for QQVGA (160x120), RGB565
    always @(*) begin
        case(rom_addr)
            // 기본 설정 및 포맷
//            8'h00 : command = 16'h1280; // COM7: Reset
            8'h00 : command = 16'h1204; // COM7: RGB
            8'h01 : command = 16'h1100; // CLKRC: Internal Clock
            8'h02 : command = 16'h0C00; // COM3
            8'h03 : command = 16'h3E00; // COM14
            8'h04 : command = 16'h8C00; // RGB444 Disable
            8'h05 : command = 16'h0400; // COM1
            8'h06 : command = 16'h4010; // COM15: RGB565
            8'h07 : command = 16'h3A04; // TSLB
            8'h08 : command = 16'h1438; // COM9: AGC Ceiling
            
            // 해상도 (QQVGA) 및 스케일링
            8'h09 : command = 16'h0C04; // COM3: Enable Scaling
            8'h0A : command = 16'h3E19; // COM14: Divider
            8'h0B : command = 16'h703A; // Scaling X
            8'h0C : command = 16'h7135; // Scaling Y
            8'h0D : command = 16'h7211; // Scaling DCW
            8'h0E : command = 16'h73F0; // Scaling PCLK
            8'h0F : command = 16'hA202; // Scaling Delay

            // 끝
            default : command = 16'hFFFF;
        endcase
    end

    // I2C State Machine
    always @(posedge clk) begin
        if (resend) begin
            rom_addr <= 0;
            state <= 0;
            sioc <= 1;
            siod_oe <= 1;
            siod_out <= 1;
            clk_div <= 0;
        end else if (!config_done) begin
            // [수정된 부분] 타이머가 제대로 동작하도록 수정
            if (clk_div < 2500) begin
                clk_div <= clk_div + 1;
            end else begin
                clk_div <= 0; // Tick 발생 시에만 리셋
                
                // Tick이 발생했을 때만 FSM 동작
                case (state)
                    // 0: IDLE / Start
                    0: begin
                        sioc <= 1;
                        siod_oe <= 1;
                        if (bit_count == 0) begin // Start Condition
                            siod_out <= 0;
                            data_sr <= {slave_addr, command}; // {ID, RegAddr, RegVal}
                            state <= 1;
                        end
                    end
                    
                    // 1: Transmit Data
                    1: begin
                        // Clock Low -> Change Data
                        if (sioc == 1) begin
                            sioc <= 0;
                        end else begin
                            // Clock High -> Data Stable
                            if (bit_count == 0 || bit_count == 9 || bit_count == 18 || bit_count == 27) begin
                                // ACK bit (Ignore/Check)
                                siod_oe <= 0; // Release line for ACK
                            end else begin
                                siod_oe <= 1;
                                siod_out <= data_sr[23]; // MSB first (수정: 24비트니까 23부터)
                                data_sr <= {data_sr[22:0], 1'b0};
                            end
                            
                            sioc <= 1; // Clock High
                            bit_count <= bit_count + 1;
                            
                            if (bit_count == 28) begin // End of transmission
                                state <= 2;
                            end
                        end
                    end
                    
                    // 2: Stop Condition
                    2: begin
                        sioc <= 0;
                        siod_oe <= 1;
                        siod_out <= 0;
                        
                        // Stop: SIOC High, then SIOD Low->High
                        state <= 3;
                    end
                    
                    3: begin
                        sioc <= 1;
                        state <= 4;
                    end
                    
                    4: begin
                        siod_out <= 1; // Stop condition complete
                        bit_count <= 0;
                        state <= 0;
                        rom_addr <= rom_addr + 1; // Next Register
                    end
                endcase
            end
        end
    end
endmodule




















