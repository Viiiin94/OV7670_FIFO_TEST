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
    reg [15:0] clk_div;
    wire i2c_tick = (clk_div == 1000); // 이거 선언만 하고 직접 사용을 안하는데?
    
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
//            8'h00 : command = 16'h1280; // COM7: Reset 여기 리셋을 한 번 주고 RGB로 넘어가야 하지 않을가?
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
            if (clk_div < 1000) begin
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
                                // ACK bit (Ignore/Check) 여기 부분도 ACK 비트를 안 읽는 것 같아
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

module ov7670_sccb (
    input  wire clk,        // 100 MHz
    input  wire reset,
    output reg  sioc,
    inout  wire siod,
    output reg  done
);

    // ======================================================
    // Clock divider: 100MHz -> 100kHz (500 cycles toggle)
    // ======================================================
    reg [8:0] clk_div;
    wire scl_tick = (clk_div == 499);

    always @(posedge clk) begin
        if (reset) clk_div <= 0;
        else if (scl_tick) clk_div <= 0;
        else clk_div <= clk_div + 1;
    end

    // ======================================================
    // Open-drain SIOD
    // ======================================================
    reg siod_out;
    reg siod_oe;
    assign siod = siod_oe ? siod_out : 1'bz;
    wire siod_in = siod;

    // ======================================================
    // ROM (최소 + 안정 세트)
    // ======================================================
    reg [7:0] rom_addr;
    reg [15:0] rom_data;

    always @(*) begin
        case (rom_addr)
            8'd0: rom_data = 16'h1280; // COM7 reset
            8'd1: rom_data = 16'h1204; // RGB
            8'd2: rom_data = 16'h4010; // RGB565
            8'd3: rom_data = 16'hFFFF; // END
            default: rom_data = 16'hFFFF;
        endcase
    end

    // ======================================================
    // FSM
    // ======================================================
    localparam IDLE  = 0,
               START = 1,
               SEND  = 2,
               ACK   = 3,
               STOP  = 4;

    reg [2:0] state;
    reg [7:0] shift;
    reg [3:0] bit_cnt;
    reg [1:0] byte_sel;

    always @(posedge clk) begin
        if (reset) begin
            state    <= IDLE;
            sioc     <= 1;
            siod_out <= 1;
            siod_oe  <= 1;
            rom_addr <= 0;
            done     <= 0;
        end else if (scl_tick) begin
            case (state)

            IDLE: begin
                if (rom_data != 16'hFFFF) begin
                    state <= START;
                    done  <= 0;
                end else begin
                    done <= 1;
                end
            end

            START: begin
                siod_out <= 0;
                siod_oe  <= 1;
                sioc     <= 1;
                shift    <= 8'h42; // OV7670 write addr
                bit_cnt  <= 7;
                byte_sel <= 0;
                state    <= SEND;
            end

            SEND: begin
                sioc <= 0;
                siod_out <= shift[bit_cnt];
                sioc <= 1;
                if (bit_cnt == 0) state <= ACK;
                else bit_cnt <= bit_cnt - 1;
            end

            ACK: begin
                sioc <= 0;
                siod_oe <= 0; // release
                sioc <= 1;
                if (siod_in != 0) begin
                    state <= IDLE; // ACK 실패 → 중단
                end else begin
                    siod_oe <= 1;
                    if (byte_sel == 0) begin
                        shift <= rom_data[15:8];
                        bit_cnt <= 7;
                        byte_sel <= 1;
                        state <= SEND;
                    end else if (byte_sel == 1) begin
                        shift <= rom_data[7:0];
                        bit_cnt <= 7;
                        byte_sel <= 2;
                        state <= SEND;
                    end else begin
                        state <= STOP;
                    end
                end
            end

            STOP: begin
                sioc <= 1;
                siod_out <= 0;
                siod_out <= 1;
                rom_addr <= rom_addr + 1;
                state <= IDLE;
            end

            endcase
        end
    end
endmodule



















