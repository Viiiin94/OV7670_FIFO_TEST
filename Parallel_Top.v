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
    input  wire href,           // (FIFO 버전에서는 보통 안 쓰지만 연결해둠)
    
    output wire fifo_rclk,      // JA7: Read Clock (RCK) - FPGA가 쏘는 클럭
    output reg  fifo_rrst,      // JA8: Read Reset (RRST) - FPGA가 쏘는 리셋
    output wire fifo_oe,        // JA9: Output Enable (OE)
    output wire cam_reset,      // JA10: Camera Hardware Reset

    // ==================================================
    // [Pmod JB] 데이터 입력 (Camera -> FPGA)
    // ==================================================
    input wire [7:0] d_in,

    // ==================================================
    // [Pmod JC] 제어 신호 (안 쓰는 핀 정리 + RPi 통신)
    // ==================================================
    output wire fifo_wrst,      // Write Reset (보통 카메라 내부 처리)
    output wire fifo_wr,        // Write Enable (보통 카메라 내부 처리)
    output wire pwdn,           // Power Down
    
    output reg valid,           // To RPi: "가져가!"
    input  wire ack,            // From RPi: "받았어!"

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
    assign pwdn = 0;          // 동작 모드
    assign cam_reset = 1;     // 리셋 해제 (Active Low인 경우 확인 필요, 보통 1)
    
    // OE (Output Enable): 0일 때 데이터 출력 (Active Low)
    // 항상 켜두거나, 읽을 때만 켜야 합니다. 여기선 항상 켭니다.
    assign fifo_oe = 0; 
    
    // 쓰기 관련 핀은 카메라가 알아서 하도록 둡니다 (또는 GND/VCC 처리)
    assign fifo_wrst = 0; // or 1, 보드마다 다름. 보통 0이나 1로 고정해도 됨
    assign fifo_wr = 0;   

    // --------------------------------------------------
    // 2. I2C 컨트롤러 (사용자님 코드 그대로 사용)
    // --------------------------------------------------
    wire config_done;
    OV7670_controller u_ctrl (
        .clk(clk),
        .resend(reset_p),
        .config_done(config_done),
        .sioc(sioc),
        .siod(siod)
    );

    // --------------------------------------------------
    // 3. RCK (Read Clock) 생성
    // --------------------------------------------------
    // FPGA가 카메라에게 쏘는 클럭입니다.
    // 100MHz 시스템 클럭을 나눠서 RCK를 만듭니다.
    // 너무 빠르면 신호가 뭉개지므로 적당히 12.5MHz 정도로 만듭니다.
    reg [2:0] div_cnt;
    always @(posedge clk) div_cnt <= div_cnt + 1;
    
    // JA7 핀으로 나가는 클럭 (Bit 2 = 100MHz / 8 = 12.5MHz)
    assign fifo_rclk = div_cnt[2];

    // 데이터 읽기 타이밍: RCK가 High가 되는 순간(또는 Low)
    // 여기서는 Edge 검출을 위해 펄스를 만듭니다.
    wire rck_rise = (div_cnt == 3'b011); // RCK 상승 에지 직전

    // --------------------------------------------------
    // 4. 데이터 읽기 FSM (Master Mode)
    // --------------------------------------------------
    // 순서: VSYNC 대기 -> RRST 리셋 -> RCK 쏘면서 데이터 읽기
    
    reg [2:0] read_state = 0;
    reg [18:0] pixel_cnt; // 38400 bytes (QQVGA)
    
    reg [7:0] d_latch;
    reg wr_en; // 내부 FIFO에 쓸 신호

    always @(posedge clk) begin
        if (reset_p) begin
            read_state <= 0;
            fifo_rrst <= 1; // RRST는 평소에 High (Active Low Reset)
            wr_en <= 0;
            pixel_cnt <= 0;
        end else begin
            case (read_state)
                // [0] VSYNC 대기
                0: begin
                    wr_en <= 0;
                    fifo_rrst <= 1;
                    pixel_cnt <= 0;
                    
                    // VSYNC가 High였다가 Low로 떨어지면(캡처 완료) 시작
                    // 간단히 VSYNC가 1일 때 다음 단계 준비
                    if (vsync == 1) read_state <= 1;
                end

                // [1] 읽기 포인터 리셋 (RRST Pulse)
                1: begin
                    if (vsync == 0) begin // 프레임 캡처 끝남
                        fifo_rrst <= 0; // Reset Active (Low)
                        read_state <= 2;
                    end
                end

                // [2] 리셋 해제
                2: begin
                    fifo_rrst <= 1; // Reset Inactive (High)
                    // 리셋 후 약간의 딜레이가 필요할 수 있음
                    read_state <= 3;
                end

                // [3] 데이터 읽어오기
                3: begin
                    // 우리가 만든 RCK 타이밍에 맞춰 데이터 캡처
                    if (rck_rise) begin
                        d_latch <= d_in; // JB포트에서 데이터 읽기
                        wr_en <= 1;      // 내부 FIFO에 저장
                        pixel_cnt <= pixel_cnt + 1;
                    end else begin
                        wr_en <= 0;
                    end

                    // QQVGA (160x120) * 2 byte = 38,400 bytes
                    // 조금 넉넉하게 읽고 종료
                    if (pixel_cnt >= 38400) begin
                        read_state <= 0; // 다시 VSYNC 대기
                    end
                end
            endcase
        end
    end

    // --------------------------------------------------
    // 5. FPGA 내부 버퍼 (FIFO)
    // --------------------------------------------------
    wire [7:0] fifo_dout;
    wire fifo_full, fifo_empty;
    reg rd_en;

    // 이제 쓰기 클럭도 시스템 클럭(clk)입니다!
    ov7670_fifo u_fifo (
        .wr_clk(clk),     // [중요] 시스템 클럭 사용
        .din(d_latch),
        .wr_en(wr_en),
        
        .rd_clk(clk),     // 읽기도 시스템 클럭
        .rd_en(rd_en),
        .dout(fifo_dout),
        
        .rst(reset_p),
        .full(fifo_full),
        .empty(fifo_empty)
    );

    // --------------------------------------------------
    // 6. 라즈베리 파이 전송 (Handshake)
    // --------------------------------------------------
    localparam S_TX_IDLE = 0;
    localparam S_TX_FETCH = 1;
    localparam S_TX_VALID = 2;
    localparam S_TX_ACK   = 3;

    reg [2:0] tx_state = S_TX_IDLE;

    always @(posedge clk) begin
        if (reset_p) begin
            valid <= 0;
            rd_en <= 0;
            tx_state <= S_TX_IDLE;
        end else begin
            case(tx_state)
                S_TX_IDLE: begin
                    valid <= 0;
                    rd_en <= 0;
                    // FIFO에 데이터가 있고 설정이 끝났으면
                    if (!fifo_empty) begin
                        tx_state <= S_TX_FETCH;
                    end
                end

                S_TX_FETCH: begin
                    rd_en <= 1; // FIFO에서 1바이트 꺼냄
                    tx_state <= S_TX_VALID;
                end

                S_TX_VALID: begin
                    rd_en <= 0;
                    d_out <= fifo_dout;
                    valid <= 1; // "가져가!"
                    tx_state <= S_TX_ACK;
                end

                S_TX_ACK: begin
                    // 라즈베리 파이가 받았다고 할 때까지 대기
                    if (ack == 1) begin
                        valid <= 0;
                        tx_state <= S_TX_IDLE; // 다음 데이터 준비
                    end
                end
            endcase
        end
    end

    // --------------------------------------------------
    // 7. Debug LEDs
    // --------------------------------------------------
    assign led[0] = config_done; // 켜지면 I2C 설정 완료
    assign led[1] = !fifo_empty; // 데이터가 들어오고 있음
    assign led[2] = valid;       // 데이터 전송 중
    assign led[3] = vsync;       // 카메라가 찍고 있음 (깜빡임)
    assign led[15:8] = d_out;    // 데이터 값 모니터링

endmodule
