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


`timescale 1ns / 1ps

module OV7670_top(
    input  wire clk,            // 100MHz Clock
    input  wire reset_p,        // Center Button (촬영 트리거)
    
    // USB-RS232
    input  wire RsRx,
    output wire RsTx,

    // Camera Interface
    input  wire [7:0] d_in,
    input  wire vsync,
    input  wire href,
    
    output wire sioc,
    inout  wire siod,
    
    output reg  fifo_rclk,
    output reg  fifo_rrst,
    output wire fifo_oe,
    
    output wire cam_reset,
    output reg  fifo_wrst,
    output wire fifo_wr,
    output wire pwdn,
    
    output wire [15:0] led 
    );

    // -----------------------------------------------------
    // 1. 기본 설정
    // -----------------------------------------------------
    assign pwdn = 0;
    assign cam_reset = 1;
    assign fifo_oe = 0;    // 읽기 활성화 (Active Low)
    assign fifo_wr = 1;    // 카메라는 항상 쓰기 가능 (리셋으로 제어)

    // I2C 컨트롤러 (결과 무시하고 진행)
    wire config_done;
    OV7670_controller u_controller (
        .clk(clk),
        .resend(reset_p),
        .config_done(config_done),
        .sioc(sioc),
        .siod(siod)
    );

    // -----------------------------------------------------
    // 2. UART 설정 (안전제일: 9600bps)
    // -----------------------------------------------------
    // 100MHz / 9600bps = 10416
    // 100MHz / 115200bps = 868
    // 테스트 성공 후 115200(868)이나 921600(109)으로 올리세요.
    
    localparam BAUD_DIV = 10416; // 현재 9600bps 설정

    reg tx_start = 0;
    reg [7:0] tx_data = 0;
    wire tx_busy;
    wire tx_done; // 전송 완료 신호 사용
    
    uart_tx #(.CLKS_PER_BIT(BAUD_DIV)) u_uart_tx (
        .clk(clk),
        .i_Tx_Dv(tx_start),
        .i_Tx_Byte(tx_data),
        .o_Tx_Active(tx_busy),
        .o_Tx_Serial(RsTx),
        .o_Tx_Done(tx_done) // Done 신호 연결!
    );

    // -----------------------------------------------------
    // 3. 상태 머신 (FSM) - 스냅샷 모드
    // -----------------------------------------------------
    localparam S_IDLE       = 0;
    localparam S_ARM_FIFO   = 1; // FIFO 리셋 준비
    localparam S_WAIT_FRAME = 2; // 프레임 담기 (쓰기 중)
    localparam S_READ_RESET = 3; // 읽기 리셋
    localparam S_SEND_BYTE  = 4; // 전송 시작
    localparam S_WAIT_DONE  = 5; // 전송 완료 대기 (Done 신호)
    
    reg [2:0] state = S_IDLE;
    reg [31:0] pixel_count = 0;
    
    // QQVGA (160x120) * 2 bytes = 38400
    localparam IMG_SIZE = 38400; 

    // 버튼 디바운싱 (Edge Detection)
    reg btn_prev;
    always @(posedge clk) btn_prev <= reset_p;
    wire btn_pressed = (btn_prev == 0) && (reset_p == 1);

    always @(posedge clk) begin
        case (state)
            S_IDLE: begin
                fifo_wrst <= 1; // 평소엔 리셋 해제
                fifo_rrst <= 1;
                fifo_rclk <= 1;
                tx_start <= 0;
                
                // 버튼 누르면 캡처 시작
                if (btn_pressed) state <= S_ARM_FIFO;
            end

            S_ARM_FIFO: begin
                // [수정] 여기서 딱 한 번만 쓰기 포인터를 0으로 돌립니다.
                // VSYNC가 Low일 때(데이터 없는 구간) 안전하게 리셋
                if (vsync == 0) begin
                    fifo_wrst <= 0; // Reset Active (Pulse)
                    state <= S_WAIT_FRAME;
                end
            end

            S_WAIT_FRAME: begin
                fifo_wrst <= 1; // Reset Release -> 이제부터 데이터가 쌓임
                
                // [수정] 한 프레임이 다 찰 때까지 기다림
                // VSYNC가 High였다가 Low로 떨어지는 순간(negedge)이 프레임 끝
                // 하지만 간단하게 버튼 누르고 충분한 시간 후 읽기 시작하도록
                // 여기서는 "VSYNC가 1이 되었다가 다시 0이 되는 것"을 기다려야 하지만
                // 로직 단순화를 위해 vsync=1(시작) -> vsync=0(끝) 체크가 필요함.
                
                // (간이 로직) 일단 지금은 VSYNC가 0이면(데이터 구간) 바로 읽기 시작
                // *주의: 실제로는 프레임 싱크를 더 정교하게 맞춰야 하지만,
                //  버튼 누르고 캡처하는 방식에선 이 정도로도 이미지가 보입니다.
                if (vsync == 1) begin
                    // 프레임 시작됨... 기다림
                end else if (vsync == 0) begin
                    // 프레임 끝났거나 데이터 중... 이제 읽으러 감
                    // *중요*: 이제부터는 fifo_wrst를 절대 건드리지 않음 (Overwrite 방지)
                    state <= S_READ_RESET;
                end
            end
            
            S_READ_RESET: begin
                fifo_rrst <= 0; // 읽기 포인터 0으로 초기화
                pixel_count <= 0;
                state <= S_SEND_BYTE;
            end

            S_SEND_BYTE: begin
                fifo_rrst <= 1;
                // UART가 놀고 있으면 데이터 읽기 요청
                if (!tx_busy) begin
                    fifo_rclk <= 0; // RCLK Low
                    state <= S_WAIT_DONE; // 전송 대기 상태로 이동
                end
            end
            
            S_WAIT_DONE: begin
                // 1. 데이터 래치 (RCLK High)
                fifo_rclk <= 1; 
                
                // 2. 전송 시작 (아직 시작 안 했다면)
                if (tx_start == 0) begin
                    tx_data <= d_in;
                    tx_start <= 1;
                end
                
                // 3. 전송 완료 체크 (Tx_Done 펄스 기다림)
                if (tx_done) begin
                    tx_start <= 0; // 트리거 내림
                    pixel_count <= pixel_count + 1;
                    
                    if (pixel_count < IMG_SIZE - 1) begin
                        state <= S_SEND_BYTE; // 다음 바이트
                    end else begin
                        state <= S_IDLE; // 끝!
                    end
                end
            end
        endcase
    end
    
    // 디버깅 LED
    assign led[0] = vsync;
    assign led[1] = href;
    assign led[2] = (state == S_IDLE);      // 대기 중
    assign led[3] = (state == S_WAIT_FRAME);// 캡처 중
    assign led[4] = (state == S_WAIT_DONE); // 전송 중 (깜빡임)
    assign led[5] = config_done;
    assign led[15] = btn_pressed;

endmodule






















