`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/09/2026 11:26:47 AM
// Design Name: 
// Module Name: Parallel_Top
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


module Parallel_Top(
    input wire clk,             // 100MHz System Clock
    input wire reset_p,         // Center Button (Reset)

    // ==================================================
    // [Pmod JA] 카메라 제어 (FPGA -> Camera)
    // ==================================================
    output wire sioc,           // I2C Clock
    inout  wire siod,           // I2C Data
    input  wire vsync,          // VSYNC (새 프레임 알림)
    input  wire href,           // HREF (데이터 유효 구간)
    
    output wire fifo_rclk,      // (카메라용 클럭 - 사용 안 함)
    output reg  fifo_rrst,      // (FIFO 읽기 리셋)
    output wire fifo_oe,        // (Output Enable)
    output wire cam_reset,      // (Camera Reset)

    // ==================================================
    // [Pmod JB] 데이터 입력 (Camera -> FPGA)
    // ==================================================
    input wire [7:0] d_in,

    // ==================================================
    // [Pmod JC] 제어 신호 (사용 안 함)
    // ==================================================
    output wire fifo_wrst, 
    output wire fifo_wr, 
    output wire pwdn,
    
    output reg valid,           // 디버그용 LED
    input  wire ack,            // (무시함 - Free Run 모드)

    // ==================================================
    // [Pmod JXADC] 데이터 출력 (FPGA -> RPi)
    // ==================================================
    output reg [7:0] d_out,

    // [LEDs] 상태 확인
    output wire [15:0] led
    );

    // --------------------------------------------------
    // 1. 카메라 기본 신호 설정
    // --------------------------------------------------
    assign pwdn = 0;        // Power Down (0=On)
    assign cam_reset = ~reset_p;   // Reset (1=Active)
    assign fifo_oe = 0;     // Output Enable (Always On)
    assign fifo_wrst = 0;   // Write Reset
    assign fifo_wr = 0;     // Write Enable (카메라 내부 제어용)
    assign fifo_rclk = 0;   // 사용 안 함
    
//    reg [1:0] clk_25_cnt;
//    always @(posedge clk) clk_25_cnt <= clk_25_cnt + 1;
    
//    // JA7번 핀(XCLK)으로 클럭 전송
//    assign fifo_rclk = clk_25_cnt[1];

    // --------------------------------------------------
    // 2. I2C 컨트롤러 (카메라 설정)
    // --------------------------------------------------
    wire config_done;
    OV7670_controller u_ctrl (
        .clk(clk),
        .reset(reset_p),
        .config_done(config_done),
        .sioc(sioc),
        .siod(siod)
    );

    // --------------------------------------------------
    // 3. 데이터 샘플링 (Camera -> FIFO)
    // --------------------------------------------------
    // PCLK 대신 시스템 클럭으로 HREF 구간만 캡처합니다.
    // (간이 방식이지만 배선 복잡도를 줄여줍니다)
    reg [7:0] d_latch;
    reg wr_en;

    always @(posedge clk) begin
        if (href) begin
            d_latch <= d_in;
            wr_en <= 1; // HREF가 1일 때만 저장
        end else begin
            wr_en <= 0;
        end
    end

    // --------------------------------------------------
    // 4. FIFO 인스턴스 (데이터 창고)
    // --------------------------------------------------
    wire [7:0] fifo_dout;
    wire fifo_full, fifo_empty;
    reg rd_en;

    ov7670_fifo u_fifo (
        .wr_clk(clk),      
        .din(d_latch),
        .wr_en(wr_en),
        
        .rd_clk(clk),      
        .rd_en(rd_en),
        .dout(fifo_dout),
        
        .rst(reset_p),
        .full(fifo_full),
        .empty(fifo_empty)
    );

    // --------------------------------------------------
    // 5. 고속 전송 로직 (FPGA -> RPi)
    // --------------------------------------------------
    // Ack를 기다리지 않고 약 12.5MHz 속도로 계속 내보냅니다.
    
    reg [2:0] tx_div;
    always @(posedge clk) tx_div <= tx_div + 1;
    wire tx_clk = tx_div[2]; // 100MHz / 8 = 12.5MHz

    always @(posedge tx_clk) begin
        if (reset_p) begin
            rd_en <= 0;
            valid <= 0;
            d_out <= 0;
            fifo_rrst <= 1;
        end else begin
            // VSYNC(새 프레임)가 들어오면 FIFO 읽기 포인터를 리셋해서
            // 라즈베리 파이가 데이터를 놓쳐도 다음 프레임은 맞게 나오도록 함
            if (vsync) fifo_rrst <= 0; // Reset Active
            else       fifo_rrst <= 1; // Reset Release

            // 데이터가 있으면 무조건 보낸다!
            if (!fifo_empty) begin
                rd_en <= 1;
                d_out <= fifo_dout;
                valid <= 1; // LED 깜빡임용
            end else begin
                rd_en <= 0;
                valid <= 0;
                // d_out은 이전 값 유지 (SMI가 읽어가도록)
            end
        end
    end

    // --------------------------------------------------
    // 6. LED 상태 표시
    // --------------------------------------------------
    assign led[0] = config_done; // 켜져야 함 (I2C 완료)
    assign led[1] = !fifo_empty; // 켜져야 함 (카메라 데이터 들어옴)
    assign led[2] = vsync;       // 깜빡여야 함 (프레임 갱신)
    assign led[15:8] = d_out;    // 빠르게 변해야 함 (데이터 출력)

endmodule

module OV7670_TOP_MODULE (
    input  wire        clk,         // 100MHz
    input  wire        reset_p,

    // ===============================
    // OV7670 SCCB
    // ===============================
    output wire        sioc,
    inout  wire        siod,

    // ===============================
    // OV7670 Video Sync
    // ===============================
    input  wire        vsync,
    input  wire        href,        // 디버그용

    // ===============================
    // FIFO Read Side (AL422B)
    // ===============================
    output wire        rclk,
    output wire        rrst,
    output wire        oe,
    input  wire [7:0]  d_in,

    // ===============================
    // FIFO Write Side
    // ===============================
    output wire        wr,
    output wire        wrst,

    // ===============================
    // Camera Control
    // ===============================
    output wire        rst,
    output wire        pwdn,

    // ===============================
    // Raspberry Pi Interface
    // ===============================
    output wire [7:0]  d_out,
    output wire        valid,
    input  wire        ack,
    
    output wire [15:0] led
);

    assign rst  = 1'b1;
    assign pwdn = 1'b0;

    wire cam_ready;
    wire [2:0] reader_state;  // Reader FSM 상태
    wire [17:0] byte_cnt;     // Reader 바이트 카운터
    
    // ========================================
    // LED 디버그 할당
    // ========================================
    assign led[0]  = cam_ready;      // SCCB 설정 완료
    assign led[1]  = vsync;          // VSYNC 신호
    assign led[2]  = href;           // HREF 신호
    assign led[3]  = wr;             // FIFO Write
    assign led[4]  = wrst;           // FIFO Write Reset
    assign led[5]  = valid;          // RPi Valid 신호
    assign led[6]  = ack;            // RPi ACK 신호
    assign led[7]  = oe;             // FIFO Output Enable
    
    // Reader FSM 상태 (3비트)
    assign led[10:8] = reader_state;
    
    // 바이트 카운터 상위 5비트 (진행 상황)
    assign led[15:11] = byte_cnt[17:13];

    ov7670_sccb_ctrl u_sccb (
        .clk       (clk),
        .reset_p   (reset_p),
        .sioc      (sioc),
        .siod      (siod),
        .cam_ready (cam_ready)
    );
    
    ov7670_capture u_capture (
        .clk        (clk),       // FIFO write clock = pclk
        .vsync      (vsync),
        .href       (href),
        .cam_ready  (cam_ready),
        .reset_p    (reset_p),
    
        .wr         (wr),
        .wrst       (wrst),
    
        .yuv_phase  (),           // 디버그용, 미사용
        .frame_done (),
        .byte_count ()
    );
    
    top_ov7670_rpi u_rpi (
        .clk     (clk),
        .reset_p (reset_p),
    
        .rclk    (rclk),
        .rrst    (rrst),
        .oe      (oe),
        .d_in    (d_in),
    
        .d_out   (d_out),
        .valid   (valid),
        .ack     (ack),
        
        // 디버그 출력
        .state_out (reader_state),   // ← 추가!
        .byte_cnt_out (byte_cnt)     // ← 추가!
    );

endmodule










