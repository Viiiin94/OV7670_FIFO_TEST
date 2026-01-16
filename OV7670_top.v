`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/08/2026 10:56:21 AM
// Design Name: 
// Module Name: OV7670_top
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


module top_ov7670_rpi (
    input  wire        clk,        // 100MHz
    input  wire        reset_p,

    // ---- AL422B Read Side ----
    output wire        rclk,       // FIFO Read Clock
    output reg         rrst,       // FIFO Read Reset (Active Low)
    output reg         oe,         // FIFO Output Enable (Active Low)
    input  wire [7:0]  d_in,       // FIFO Data

    // ---- RPi Handshake ----
    output reg  [7:0]  d_out,
    output reg         valid,
    input  wire        ack,
    
    // ==========================================
    // 디버그 출력 추가
    // ==========================================
    output wire [2:0]  state_out,    // FSM 상태
    output wire [17:0] byte_cnt_out  // 바이트 카운터
);

    // ------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------
    localparam FRAME_BYTES = 320 * 240 * 2; // QVGA YUV422

    // ------------------------------------------------------------
    // ACK synchronizer (VERY IMPORTANT)
    // ------------------------------------------------------------
    reg ack_ff1, ack_ff2;
    always @(posedge clk) begin
        ack_ff1 <= ack;
        ack_ff2 <= ack_ff1;
    end
    
    // ------------------------------------------------------------
    // Read clock generator (10MHz)
    // ------------------------------------------------------------
    reg [3:0] rclk_div;
    
    always @(posedge clk) begin
        rclk_div <= rclk_div + 1;
    end
    
    assign rclk = rclk_div[3];   // ~6.25MHz (충분히 안정)
    
    reg [7:0] d_in_stable;
    always @(negedge rclk) begin
        d_in_stable <= d_in;
    end
    
    reg [7:0] d_in_sync1, d_in_sync2;
    always @(posedge clk) begin
        d_in_sync1 <= d_in_stable;
        d_in_sync2 <= d_in_sync1;  // 2-stage synchronizer
    end
    
    // ------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------
    localparam S_IDLE       = 0;
    localparam S_RRST       = 1;
    localparam S_WAIT_ACK   = 2;
    localparam S_DATA       = 3;
    localparam S_READ_NEXT  = 4;
    
    reg [3:0] state;
    reg [17:0] byte_cnt;
    reg [2:0] wait_cnt;
    
    // ==========================================
    // 디버그 출력 할당
    // ==========================================
    assign state_out = state;
    assign byte_cnt_out = byte_cnt;
    
    // ------------------------------------------------------------
    // FSM logic (rclk domain이 아니라 clk domain!)
    // ------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state    <= S_IDLE;
            rrst     <= 1'b1;
            oe       <= 1'b1;
            valid    <= 1'b0;
            d_out    <= 8'd0;
            byte_cnt <= 0;
            wait_cnt <= 0;
        end else begin
            case (state)

                // --------------------------------------------
                // IDLE
                // --------------------------------------------
                S_IDLE: begin
                    rrst     <= 1'b0;   // reset read pointer
                    oe       <= 1'b1;
                    valid    <= 1'b0;
                    byte_cnt <= 0;
                    state    <= S_RRST;
                end

                // --------------------------------------------
                // RRST release
                // --------------------------------------------
                S_RRST: begin
                    rrst  <= 1'b1;
                    oe    <= 1'b0;      // enable FIFO output
                    state <= S_WAIT_ACK;
                end

                // --------------------------------------------
                // Wait RPi ready
                // --------------------------------------------
                S_WAIT_ACK: begin
                    if (ack_ff2) begin
                        d_out <= d_in_sync2;  // DATA 먼저 고정
                        valid <= 1'b1;
                        state <= S_DATA;
                    end
                end

                // --------------------------------------------
                // DATA phase (VALID 유지)
                // --------------------------------------------
                S_DATA: begin
                    if (!ack_ff2) begin  // ACK 떨어짐 = RPi가 받았음
                        valid <= 1'b0;
                        wait_cnt <= 0;
                        state <= S_READ_NEXT;  // 새 상태 추가
                    end
                end
                
                S_READ_NEXT: begin
                    wait_cnt <= wait_cnt + 1;
                
                    if (wait_cnt >= 4) begin
                        byte_cnt <= byte_cnt + 1;
                        
                        if (byte_cnt >= FRAME_BYTES - 1) begin
                            // 프레임 완료!
                            oe <= 1'b1;      // FIFO 출력 비활성화
                            valid <= 1'b0;
                            state <= S_IDLE;
                        end else begin
                            // 다음 바이트
                            d_out <= d_in_sync2;
                            valid <= 1'b1;
                            state <= S_DATA;
                        end
                    end 
                end
            endcase
        end
    end
endmodule



//////////////////////////////////////////
// OV7670 CAPTURE
//////////////////////////////////////////
module ov7670_capture (
    input wire        clk,       // OV7670 pixel clock
    input wire        vsync,      // Frame sync
    input wire        href,       // Line valid
    input wire        cam_ready,  // SCCB configuration done
    input wire        reset_p,
    
    // FIFO Write Interface
    output reg        wr,        // FIFO write (active low)
    output reg        wrst,       // FIFO write reset (active low)
    
    // YUV422 Phase Output (for top module)
    output reg  [1:0] yuv_phase,  // 0=Y0, 1=U, 2=Y1, 3=V
    output reg        frame_done, // 1-clock pulse when frame f inished
    output reg [17:0] byte_count  // Debug: bytes written
);

    // --------------------------------------------------
    // Parameters
    // --------------------------------------------------
    localparam FRAME_BYTES = 320 * 240 * 2; // QVGA YUV422 = 153,600
    
    // YUV422 phase definitions (for clarity)
    localparam PHASE_Y0 = 2'd0;  // Luma (first pixel)
    localparam PHASE_U  = 2'd1;  // Chroma (Cb)
    localparam PHASE_Y1 = 2'd2;  // Luma (second pixel)
    localparam PHASE_V  = 2'd3;  // Chroma (Cr)

    reg vsync_d;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p)
            vsync_d <= 1'b0;
        else
            vsync_d <= vsync;
    end
    
    wire vsync_rise = (vsync && !vsync_d);
    
    // ==========================================
    // HREF Synchronization (CDC)
    // ==========================================
    reg href_d1, href_d2;
    always @(posedge clk) begin
        href_d1 <= href;
        href_d2 <= href_d1;  // 2-stage synchronizer
    end

    // --------------------------------------------------
    // FSM States
    // --------------------------------------------------
    localparam S_IDLE  = 2'd0;
    localparam S_WRST  = 2'd1;
    localparam S_FRAME = 2'd2;
    
    reg [1:0] state;
    reg [9:0] wrst_cnt;
    
    // --------------------------------------------------
    // Main Capture Logic
    // --------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state      <= S_IDLE;
            wr         <= 1'b1;
            wrst       <= 1'b1;
            byte_count <= 0;
            yuv_phase  <= PHASE_Y0;  // Start from Y0
            frame_done <= 1'b0;
            wrst_cnt   <= 0;
        end else begin
            // Default
            frame_done <= 1'b0;

            case (state)
                // ==========================================
                // IDLE: Wait for frame start
                // ==========================================
                S_IDLE: begin
                    wr         <= 1'b1;
                    wrst       <= 1'b1;
                    byte_count <= 0;
                    yuv_phase  <= PHASE_Y0;  // Reset phase
                    
                    if (cam_ready && vsync_rise) begin
                        wrst_cnt <= 1000; // long reset time
                        state <= S_WRST;
                    end
                end
                
                // ==========================================
                // WRST: Reset FIFO write pointer
                // ==========================================
                S_WRST: begin
                    wrst <= 1'b0;  // Assert reset
                    wr   <= 1'b1;
                    if (wrst_cnt > 0) begin
                        wrst_cnt <= wrst_cnt - 1;
                    end else begin
                        wrst <= 1'b1;  // Release reset
                        state <= S_FRAME;
                    end
                end
                
                // ==========================================
                // FRAME: Capture active frame
                // ==========================================
                S_FRAME: begin
                    wrst <= 1'b1;
                    wr   <= 1'b0;  // ← 항상 쓰기 활성화!
                    
                    if (vsync_rise) begin
                        wr <= 1'b1;
                        frame_done <= 1'b1;
                        state <= S_IDLE;
                    end
                end
//                S_FRAME: begin
//                    wrst <= 1'b1;  // Reset 비활성
                    
//                    // 다음 VSYNC = 프레임 종료
//                    if (vsync_rise) begin
//                        wr         <= 1'b1;  // WE = LOW
//                        frame_done <= 1'b1;
//                        state      <= S_IDLE;
//                    end
//                    // HREF HIGH = 유효 데이터 라인
//                    else if (href_d2) begin
//                        wr <= 1'b0;  // ← NAND 통과 후 WE = HIGH!
//                        byte_count <= byte_count + 1;
//                        yuv_phase  <= yuv_phase + 1'b1;
//                    end
//                    // HREF LOW = 수평 블랭킹
//                    else begin
//                        wr <= 1'b1;  // WE = LOW
//                    end
//                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

//////////////////////////////////////////
// OV7670 CONTROLL
//////////////////////////////////////////
module ov7670_sccb_ctrl (
    input  wire clk,        // 100MHz
    input  wire reset_p,

    output reg  sioc,
    inout  wire siod,

    output reg  cam_ready
);

    // --------------------------------------------------
    // SCCB parameters
    // --------------------------------------------------
    localparam SCCB_DIV = 500;   // 100MHz / 1000 = 100kHz

    // OV7670 SCCB address
    localparam SCCB_ADDR = 8'h42; // write address
    
    // --------------------------------------------------
    // Register table (QVGA + YUV422)
    // --------------------------------------------------
    localparam ROM_SIZE = 52;  // 충분한 크기
    reg [15:0] reg_rom [0:ROM_SIZE-1];
    
    initial begin
        // Reset sequence with delays
        reg_rom[0]  = 16'h1280;  // COM7: Reset
        reg_rom[1]  = 16'hFFFE;  // DELAY marker (will be handled specially)
        reg_rom[2]  = 16'h1280;  // COM7: Reset again
        reg_rom[3]  = 16'hFFFE;  // DELAY marker
        
        // Main configuration
        reg_rom[4]  = 16'h1204;  // COM7: QVGA + YUV
        reg_rom[5]  = 16'h1100;  // CLKRC: Use external clock
        reg_rom[6]  = 16'h0C00;  // COM3: Default
        reg_rom[7]  = 16'h3E00;  // COM14: Default
        reg_rom[8]  = 16'h8C00;  // RGB444: Disable
        reg_rom[9]  = 16'h0400;  // COM1: Default
        reg_rom[10] = 16'h4010;  // COM15: Output range 00-FF
        reg_rom[11] = 16'h3A04;  // TSLB: YUV output
        reg_rom[12] = 16'h1438;  // COM9: AGC ceiling
        
        // Color matrix
        reg_rom[13] = 16'h4FB3;  // MTX1
        reg_rom[14] = 16'h50B3;  // MTX2
        reg_rom[15] = 16'h5100;  // MTX3
        reg_rom[16] = 16'h523D;  // MTX4
        reg_rom[17] = 16'h53A7;  // MTX5
        reg_rom[18] = 16'h54E4;  // MTX6
        reg_rom[19] = 16'h589E;  // MTXS
        
        // Other settings
        reg_rom[20] = 16'h3DC0;  // COM13: Gamma, UV auto
        reg_rom[21] = 16'h1100;  // CLKRC
        reg_rom[22] = 16'h1711;  // HSTART
        reg_rom[23] = 16'h1861;  // HSTOP
        reg_rom[24] = 16'h32A4;  // HREF
        reg_rom[25] = 16'h1903;  // VSTART
        reg_rom[26] = 16'h1A7B;  // VSTOP
        reg_rom[27] = 16'h030A;  // VREF
        reg_rom[28] = 16'h0E61;  // COM5
        reg_rom[29] = 16'h0F4B;  // COM6
        reg_rom[30] = 16'h1602;  // Reserved
        reg_rom[31] = 16'h1E37;  // MVFP: Mirror/VFlip
        reg_rom[32] = 16'h2102;  // ADCCTR1
        reg_rom[33] = 16'h2291;  // ADCCTR2
        reg_rom[34] = 16'h2907;  // Reserved
        reg_rom[35] = 16'h330B;  // CHLF
        reg_rom[36] = 16'h350B;  // Reserved
        reg_rom[37] = 16'h371D;  // ADC
        reg_rom[38] = 16'h3871;  // ACOM
        reg_rom[39] = 16'h392A;  // OFON
        reg_rom[40] = 16'h3C78;  // COM12
        reg_rom[41] = 16'h4D40;  // Reserved
        reg_rom[42] = 16'h4E20;  // Reserved
        reg_rom[43] = 16'h6900;  // GFIX
        reg_rom[44] = 16'h6B4A;  // DBLV
        reg_rom[45] = 16'h7410;  // REG74
        reg_rom[46] = 16'h8D4F;  // Reserved
        reg_rom[47] = 16'h8E00;  // Reserved
        reg_rom[48] = 16'h8F00;  // Reserved
        reg_rom[49] = 16'h9000;  // Reserved
        reg_rom[50] = 16'h9100;  // Reserved
        reg_rom[51] = 16'hFFFF;  // End marker
    end
    
    // --------------------------------------------------
    // SCCB clock divider
    // --------------------------------------------------
    reg [9:0] div_cnt;
    reg sccb_tick_gen;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            div_cnt <= 0;
            sccb_tick_gen <= 0;
        end else begin
            if (div_cnt >= SCCB_DIV - 1) begin
                div_cnt <= 0;
                sccb_tick_gen <= 1'b1;  // 1 클럭 펄스
            end else begin
                div_cnt <= div_cnt + 1;
                sccb_tick_gen <= 1'b0;
            end
        end
    end
    
    // --------------------------------------------------
    // FSM
    // --------------------------------------------------
    localparam S_IDLE       = 4'd0;
    localparam S_START      = 4'd1;
    localparam S_ADDR_LOW   = 4'd2;   // SCL LOW phase
    localparam S_ADDR_HIGH  = 4'd3;   // SCL HIGH phase
    localparam S_ADDR_ACK   = 4'd4;   // ACK phase (don't care)
    localparam S_REG_LOW    = 4'd5;
    localparam S_REG_HIGH   = 4'd6;
    localparam S_REG_ACK    = 4'd7;
    localparam S_DATA_LOW   = 4'd8;
    localparam S_DATA_HIGH  = 4'd9;
    localparam S_DATA_ACK   = 4'd10;
    localparam S_STOP       = 4'd11;
    localparam S_DELAY      = 4'd12;
    localparam S_LONG_DELAY = 4'd13;  // For reset
    localparam S_DONE       = 4'd14;

    reg [3:0] state;
    reg [5:0] rom_idx;
    reg [7:0] shift_reg;
    reg [3:0] bit_cnt;
    reg [15:0] delay_cnt;  // 16-bit for long delays
    reg siod_oe;
    
    assign siod = siod_oe ? 1'b0 : 1'bz;
    
    // --------------------------------------------------
    // Main FSM
    // --------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state     <= S_IDLE;
            sioc      <= 1'b1;
            siod_oe   <= 1'b0;
            rom_idx   <= 0;
            bit_cnt   <= 0;
            shift_reg <= 0;
            cam_ready <= 1'b0;
            delay_cnt <= 0;
        end else if (sccb_tick_gen) begin
            case (state)
                // ============================================
                // IDLE: Check next register
                // ============================================
                S_IDLE: begin
                    sioc <= 1'b1;
                    siod_oe <= 1'b0;
                    
                    if (reg_rom[rom_idx] == 16'hFFFF) begin
                        state <= S_DONE;
                    end else if (reg_rom[rom_idx] == 16'hFFFE) begin
                        // Delay marker (for reset timing)
                        delay_cnt <= 16'd10000;  // ~100ms at 100kHz tick
                        rom_idx <= rom_idx + 1;
                        state <= S_LONG_DELAY;
                    end else begin
                        state <= S_START;
                    end
                end
                
                // ============================================
                // START Condition
                // ============================================
                S_START: begin
                    sioc <= 1'b1;
                    siod_oe <= 1'b1;    // SDA LOW
                    shift_reg <= SCCB_ADDR;
                    bit_cnt <= 8;
                    state <= S_ADDR_LOW;
                end
                
                // ============================================
                // Send Device Address (8 bits, 2 phases each)
                // ============================================
                S_ADDR_LOW: begin
                    sioc <= 1'b0;               // SCL LOW
                    siod_oe <= ~shift_reg[7];   // Set data (0=HIGH, 1=LOW)
                    state <= S_ADDR_HIGH;
                end
                
                S_ADDR_HIGH: begin
                    sioc <= 1'b1;               // SCL HIGH (sample)
                    
                    if (bit_cnt > 1) begin
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_cnt <= bit_cnt - 1;
                        state <= S_ADDR_LOW;    // Next bit
                    end else begin
                        state <= S_ADDR_ACK;    // Done, go to ACK
                    end
                end
                
                S_ADDR_ACK: begin
                    sioc <= 1'b0;
                    siod_oe <= 1'b0;            // Release SDA (don't care ACK)
                    shift_reg <= reg_rom[rom_idx][15:8];  // Load reg addr
                    bit_cnt <= 8;
                    state <= S_REG_LOW;
                end
                
                // ============================================
                // Send Register Address (8 bits)
                // ============================================
                S_REG_LOW: begin
                    sioc <= 1'b0;
                    siod_oe <= ~shift_reg[7];
                    state <= S_REG_HIGH;
                end
                
                S_REG_HIGH: begin
                    sioc <= 1'b1;
                    
                    if (bit_cnt > 1) begin
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_cnt <= bit_cnt - 1;
                        state <= S_REG_LOW;
                    end else begin
                        state <= S_REG_ACK;
                    end
                end
                
                S_REG_ACK: begin
                    sioc <= 1'b0;
                    siod_oe <= 1'b0;
                    shift_reg <= reg_rom[rom_idx][7:0];  // Load data
                    bit_cnt <= 8;
                    state <= S_DATA_LOW;
                end
                
                // ============================================
                // Send Data (8 bits)
                // ============================================
                S_DATA_LOW: begin
                    sioc <= 1'b0;
                    siod_oe <= ~shift_reg[7];
                    state <= S_DATA_HIGH;
                end
                
                S_DATA_HIGH: begin
                    sioc <= 1'b1;
                    
                    if (bit_cnt > 1) begin
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_cnt <= bit_cnt - 1;
                        state <= S_DATA_LOW;
                    end else begin
                        state <= S_DATA_ACK;
                    end
                end
                
                S_DATA_ACK: begin
                    sioc <= 1'b0;
                    siod_oe <= 1'b0;
                    state <= S_STOP;
                end
                
                // ============================================
                // STOP Condition
                // ============================================
                S_STOP: begin
                    sioc <= 1'b1;       // SCL HIGH
                    siod_oe <= 1'b0;    // SDA HIGH (stop)
                    delay_cnt <= 16'd10;  // Short delay
                    rom_idx <= rom_idx + 1;
                    state <= S_DELAY;
                end
                
                // ============================================
                // Short Delay (between registers)
                // ============================================
                S_DELAY: begin
                    if (delay_cnt > 0) begin
                        delay_cnt <= delay_cnt - 1;
                    end else begin
                        state <= S_IDLE;
                    end
                end
                
                // ============================================
                // Long Delay (after reset)
                // ============================================
                S_LONG_DELAY: begin
                    if (delay_cnt > 0) begin
                        delay_cnt <= delay_cnt - 1;
                    end else begin
                        state <= S_IDLE;
                    end
                end
                
                // ============================================
                // Done
                // ============================================
                S_DONE: begin
                    cam_ready <= 1'b1;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule


