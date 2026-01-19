`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/16/2026 05:06:21 PM
// Design Name: 
// Module Name: vga_top
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

//module OV7670_VGA_TOP (
//    input  wire        clk,
//    input  wire        reset_p,
    
//    // OV7670 SCCB
//    output wire        sioc,
//    inout  wire        siod,
    
//    // OV7670 Sync
//    input  wire        vsync,
//    input  wire        href,
    
//    // FIFO Interface
//    output wire        rclk,
//    output wire        rrst,
//    output wire        oe,
//    input  wire [7:0]  d_in,
//    output wire        wr,
//    output wire        wrst,
    
//    // Camera Control
//    output wire        rst,
//    output wire        pwdn,
    
//    // VGA Output
//    output wire [3:0]  vgaRed,
//    output wire [3:0]  vgaGreen,
//    output wire [3:0]  vgaBlue,
//    output wire        Hsync,
//    output wire        Vsync,
    
//    // Debug LED
//    output wire [15:0] led
//);

//    assign rst  = 1'b1;
//    assign pwdn = 1'b0;
    
//    wire cam_ready;
//    wire [9:0] h_count, v_count;
//    wire video_on;
    
//    // SCCB Controller
//    ov7670_sccb_ctrl u_sccb (
//        .clk(clk),
//        .reset_p(reset_p),
//        .sioc(sioc),
//        .siod(siod),
//        .cam_ready(cam_ready)
//    );
    
//    // Capture Controller
//    ov7670_capture u_capture (
//        .clk(clk),
//        .vsync(vsync),
//        .href(href),
//        .cam_ready(cam_ready),
//        .reset_p(reset_p),
//        .wr(wr),
//        .wrst(wrst),
//        .yuv_phase(),
//        .frame_done(),
//        .byte_count()
//    );
    
//    // VGA Controller
//    vga_controller u_vga (
//        .clk(clk),
//        .reset(reset_p),
//        .h_count(h_count),
//        .v_count(v_count),
//        .video_on(video_on),
//        .hsync(Hsync),
//        .vsync(Vsync)
//    );
    
//    // Camera → VGA Bridge
//    camera_vga_bridge u_bridge (
//        .clk(clk),
//        .reset(reset_p),
//        .rclk(rclk),
//        .rrst(rrst),
//        .oe(oe),
//        .fifo_data(d_in),
//        .h_count(h_count),
//        .v_count(v_count),
//        .video_on(video_on),
//        .vga_red(vgaRed),
//        .vga_green(vgaGreen),
//        .vga_blue(vgaBlue)
//    );
    
//    // Debug LED
//    assign led[7:0] = d_in;
//    assign led[8] = cam_ready;
//    assign led[9] = vsync;
//    assign led[10] = href;
//    assign led[15:11] = 0;

//endmodule

module camera_vga_direct (
    input  wire        clk,        // 100MHz
    input  wire        reset,
    
    // AL422B FIFO 읽기
    output wire        rclk,       // FIFO Read Clock
    output reg         rrst,       // FIFO Read Reset
    output reg         oe,         // Output Enable
    input  wire [7:0]  d_in,       // FIFO Data
    
    // VGA 출력
    output wire [3:0]  vgaRed,
    output wire [3:0]  vgaGreen,
    output wire [3:0]  vgaBlue,
    output wire        Hsync,
    output wire        Vsync
);

    // ==========================================
    // VGA 타이밍 (640x480 @ 60Hz, 25MHz)
    // ==========================================
    
    // 25MHz 클럭 생성
    reg [1:0] clk_div;
    wire clk_25mhz;
    
    always @(posedge clk) begin
        clk_div <= clk_div + 1;
    end
    assign clk_25mhz = clk_div[1];
    
    // VGA 카운터
    reg [9:0] h_count;
    reg [9:0] v_count;
    
    localparam H_DISPLAY = 640;
    localparam H_TOTAL   = 800;
    localparam V_DISPLAY = 480;
    localparam V_TOTAL   = 525;
    
    always @(posedge clk_25mhz or posedge reset) begin
        if (reset) begin
            h_count <= 0;
            v_count <= 0;
        end else begin
            if (h_count < H_TOTAL - 1)
                h_count <= h_count + 1;
            else begin
                h_count <= 0;
                if (v_count < V_TOTAL - 1)
                    v_count <= v_count + 1;
                else
                    v_count <= 0;
            end
        end
    end
    
    // SYNC 신호
    assign Hsync = ~((h_count >= 656) && (h_count < 752));
    assign Vsync = ~((v_count >= 490) && (v_count < 492));
    
    wire video_on = (h_count < H_DISPLAY) && (v_count < V_DISPLAY);
    
    // ==========================================
    // FIFO 읽기 제어
    // ==========================================
    
    // 카메라 좌표 (320x240 → 640x480 업스케일 2x)
    wire [8:0] cam_x = h_count[9:1];  // 640 → 320
    wire [8:0] cam_y = v_count[9:1];  // 480 → 240
    
    // 프레임 시작 시 FIFO 리셋
    reg frame_start;
    
    always @(posedge clk_25mhz or posedge reset) begin
        if (reset) begin
            frame_start <= 0;
        end else begin
            // VGA 프레임 시작 (0, 0)
            if (h_count == 0 && v_count == 0) begin
                frame_start <= 1;
            end else begin
                frame_start <= 0;
            end
        end
    end
    
    // FIFO 제어
    reg [1:0] fifo_state;
    localparam FS_RESET = 0;
    localparam FS_READY = 1;
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rrst <= 1'b0;  // FIFO reset
            oe <= 1'b1;    // Output disable
            fifo_state <= FS_RESET;
        end else begin
            case (fifo_state)
                FS_RESET: begin
                    rrst <= 1'b0;
                    oe <= 1'b1;
                    
                    // 충분히 대기
                    fifo_state <= FS_READY;
                end
                
                FS_READY: begin
                    rrst <= 1'b1;  // Reset 해제
                    oe <= 1'b0;    // Output 활성
                    
                    // 프레임 시작마다 FIFO 리셋
                    if (frame_start) begin
                        rrst <= 1'b0;
                    end
                end
            endcase
        end
    end
    
    // FIFO Read Clock (VGA 픽셀 클럭과 동기화)
    // 320x240이므로 640x480의 1/4 속도
    reg [1:0] pixel_cnt;
    reg rclk_en;
    
    always @(posedge clk_25mhz) begin
        if (video_on && cam_x < 320 && cam_y < 240) begin
            pixel_cnt <= pixel_cnt + 1;
            
            // 2x2 픽셀마다 FIFO 1바이트 읽기
            if (h_count[0] == 0 && v_count[0] == 0)
                rclk_en <= 1;
            else
                rclk_en <= 0;
        end else begin
            rclk_en <= 0;
        end
    end
    
    assign rclk = clk_25mhz & rclk_en;
    
    // ==========================================
    // VGA 출력 (FIFO 데이터 직접 사용)
    // ==========================================
    
    // FIFO 데이터 동기화
    reg [7:0] fifo_data_sync;
    
    always @(posedge clk_25mhz) begin
        fifo_data_sync <= d_in;
    end
    
    // 흑백 출력 (Y 값만 사용)
    assign vgaRed   = video_on ? fifo_data_sync[7:4] : 4'b0000;
    assign vgaGreen = video_on ? fifo_data_sync[7:4] : 4'b0000;
    assign vgaBlue  = video_on ? fifo_data_sync[7:4] : 4'b0000;

endmodule

module vga_controller (
    input  wire       clk,        // 100MHz
    input  wire       reset,
    
    output reg [9:0]  h_count,    // 0~799
    output reg [9:0]  v_count,    // 0~524
    output wire       video_on,
    output wire       hsync,
    output wire       vsync
);

    // VGA 640x480 @ 60Hz 타이밍 (25MHz 기준)
    // 100MHz → 25MHz 분주
    reg [1:0] clk_div;
    wire clk_25MHz;
    
    always @(posedge clk or posedge reset) begin
        if (reset)
            clk_div <= 0;
        else
            clk_div <= clk_div + 1;
    end
    
    assign clk_25MHz = clk_div[1];  // 25MHz
    
    // VGA 640x480 타이밍
    localparam H_DISPLAY = 640;
    localparam H_FRONT   = 16;
    localparam H_SYNC    = 96;
    localparam H_BACK    = 48;
    localparam H_TOTAL   = 800;
    
    localparam V_DISPLAY = 480;
    localparam V_FRONT   = 10;
    localparam V_SYNC    = 2;
    localparam V_BACK    = 33;
    localparam V_TOTAL   = 525;
    
    // 카운터
    always @(posedge clk_25MHz or posedge reset) begin
        if (reset) begin
            h_count <= 0;
            v_count <= 0;
        end else begin
            if (h_count < H_TOTAL - 1)
                h_count <= h_count + 1;
            else begin
                h_count <= 0;
                if (v_count < V_TOTAL - 1)
                    v_count <= v_count + 1;
                else
                    v_count <= 0;
            end
        end
    end
    
    // SYNC 신호 (negative)
    assign hsync = ~((h_count >= H_DISPLAY + H_FRONT) && 
                     (h_count < H_DISPLAY + H_FRONT + H_SYNC));
    assign vsync = ~((v_count >= V_DISPLAY + V_FRONT) && 
                     (v_count < V_DISPLAY + V_FRONT + V_SYNC));
    
    // Video ON (표시 영역)
    assign video_on = (h_count < H_DISPLAY) && (v_count < V_DISPLAY);

endmodule

module camera_vga_bridge (
    input  wire        clk,        // 100MHz
    input  wire        reset,
    
    // FIFO Read Interface
    output wire        rclk,
    output reg         rrst,
    output reg         oe,
    input  wire [7:0]  fifo_data,
    
    // VGA Interface
    input  wire [9:0]  h_count,
    input  wire [9:0]  v_count,
    input  wire        video_on,
    
    output reg [3:0]   vga_red,
    output reg [3:0]   vga_green,
    output reg [3:0]   vga_blue
);

    // 카메라 해상도: 320x240
    // VGA 해상도: 640x480
    // 업스케일: 2x (각 픽셀을 2x2로)
    
    // 현재 픽셀 위치 (카메라 좌표)
    wire [8:0] cam_x = h_count[9:1];  // 640 → 320
    wire [8:0] cam_y = v_count[9:1];  // 480 → 240
    
    // Frame buffer (320x240 YUV422)
    // YUV422: Y0 U Y1 V (4바이트 = 2픽셀)
    (* ram_style = "block" *) reg [7:0] frame_buffer [0:153599];  // 320*240*2
    
    // FIFO 읽기
    // Port A: 쓰기 전용
    reg [17:0] wr_addr;
    reg [7:0]  wr_data;
    reg        wr_en;
    
    // Port B: 읽기 전용
    reg [17:0] rd_addr;
    reg [7:0]  rd_data;
    
    // BRAM 쓰기 (Port A)
    always @(posedge clk) begin
        if (wr_en) begin
            frame_buffer[wr_addr] <= wr_data;
        end
    end
    
    // BRAM 읽기 (Port B)
    always @(posedge clk) begin
        rd_data <= frame_buffer[rd_addr];
    end
    reg [1:0] read_state;
    
    localparam RS_IDLE = 0;
    localparam RS_RRST = 1;
    localparam RS_READ = 2;
    localparam RS_DONE = 3;
    
    // FIFO Read Clock
    reg [3:0] rclk_div;
    always @(posedge clk) begin
        rclk_div <= rclk_div + 1;
    end
    assign rclk = rclk_div[3];
    
    // FIFO 읽기 FSM
    reg [3:0] wait_cnt;
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            read_state <= RS_IDLE;
            rrst <= 1'b1;
            oe <= 1'b1;
            wr_addr <= 0;
            wr_en <= 0;
            wait_cnt <= 0;
        end else begin
            case (read_state)
                RS_IDLE: begin
                    rrst <= 1'b0;
                    oe <= 1'b1;
                    wr_addr <= 0;
                    wr_en <= 0;
                    wait_cnt <= 0;
                    read_state <= RS_RRST;
                end
                
                RS_RRST: begin
                    if (wait_cnt < 10) begin
                        wait_cnt <= wait_cnt + 1;
                        rrst <= 1'b0;
                    end else begin
                        rrst <= 1'b1;
                        oe <= 1'b0;
                        wait_cnt <= 0;
                        read_state <= RS_READ;
                    end
                end
                
                RS_READ: begin
                    wait_cnt <= wait_cnt + 1;
                    
                    if (wait_cnt >= 8) begin
                        wait_cnt <= 0;
                        
                        // BRAM 쓰기
                        wr_data <= fifo_data;
                        wr_en <= 1;
                        wr_addr <= wr_addr + 1;
                        
                        if (wr_addr >= 18'd153599) begin
                            oe <= 1'b1;
                            wr_en <= 0;
                            read_state <= RS_DONE;
                        end
                    end else begin
                        wr_en <= 0;
                    end
                end
                
                RS_DONE: begin
                    wr_en <= 0;
                end
            endcase
        end
    end
    
    // VGA 출력 (YUV → RGB 변환)
    reg [17:0] pixel_addr;
    reg [7:0] Y, U, V;
    
    reg signed [15:0] R_temp, G_temp, B_temp;
    
    always @(posedge clk) begin
        if (video_on && cam_x < 320 && cam_y < 240) begin
            // YUV422 주소 계산
            // 픽셀 (x, y) → YUV422 offset
            pixel_addr = (cam_y * 320 + (cam_x & ~1)) * 2;
            
            if (cam_x[0] == 0) begin
                // 짝수 픽셀: Y0 U
                Y = frame_buffer[pixel_addr];
                U = frame_buffer[pixel_addr + 1];
                V = frame_buffer[pixel_addr + 3];
            end else begin
                // 홀수 픽셀: Y1 V
                Y = frame_buffer[pixel_addr + 2];
                U = frame_buffer[pixel_addr + 1];
                V = frame_buffer[pixel_addr + 3];
            end
            
            // YUV → RGB (간단 버전)
            // R = Y + 1.402 * (V-128)
            // G = Y - 0.344 * (U-128) - 0.714 * (V-128)
            // B = Y + 1.772 * (U-128)
            
            R_temp = Y + ((V - 128) * 91) / 64;
            G_temp = Y - ((U - 128) * 22) / 64 - ((V - 128) * 46) / 64;
            B_temp = Y + ((U - 128) * 113) / 64;
            
            vga_red   <= R_temp[7:4];
            vga_green <= G_temp[7:4];
            vga_blue  <= B_temp[7:4];
        end else begin
            vga_red   <= 4'b0000;
            vga_green <= 4'b0000;
            vga_blue  <= 4'b0000;
        end
    end

endmodule
