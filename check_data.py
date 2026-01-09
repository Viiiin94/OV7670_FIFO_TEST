import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import sys

# ==========================================
# [설정] 핀 맵핑
# ==========================================
DATA_PINS = [7, 8, 25, 24, 23, 18, 15, 14]
PIN_VALID = 20
PIN_ACK   = 21

# 영상 설정 (QQVGA)
WIDTH = 160
HEIGHT = 120
BYTES_PER_PIXEL = 2
FRAME_SIZE = WIDTH * HEIGHT * BYTES_PER_PIXEL # 38,400 bytes

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DATA_PINS, GPIO.IN)
GPIO.setup(PIN_VALID, GPIO.IN)
GPIO.setup(PIN_ACK, GPIO.OUT, initial=GPIO.LOW)

print("========================================")
print(f"🚀 이미지 수신 시작! (총 {FRAME_SIZE} 바이트)")
print("   - 파이썬 GPIO 속도 한계로 시간이 좀 걸립니다.")
print("   - 100%가 되면 이미지가 저장되고 창이 뜹니다.")
print("========================================")

try:
    raw_buffer = bytearray(FRAME_SIZE)
    
    while True:
        print("\n📸 새 프레임 수신 대기 중...", flush=True)
        byte_idx = 0
        start_time = time.time()
        
        # --------------------------------------
        # 1. 이미지 한 장 받기 (Loading Bar 표시)
        # --------------------------------------
        while byte_idx < FRAME_SIZE:
            # VALID 대기
            while GPIO.input(PIN_VALID) == 0: pass
            
            # 데이터 읽기
            val = 0
            if GPIO.input(7):  val |= 1
            if GPIO.input(8):  val |= 2
            if GPIO.input(25): val |= 4
            if GPIO.input(24): val |= 8
            if GPIO.input(23): val |= 16
            if GPIO.input(18): val |= 32
            if GPIO.input(15): val |= 64
            if GPIO.input(14): val |= 128
            
            raw_buffer[byte_idx] = val
            byte_idx += 1
            
            # ACK 전송
            GPIO.output(PIN_ACK, GPIO.HIGH)
            while GPIO.input(PIN_VALID) == 1: pass
            GPIO.output(PIN_ACK, GPIO.LOW)
            
            # [진행 상황 표시] 1000바이트마다 점 찍기
            if byte_idx % 1000 == 0:
                percent = (byte_idx / FRAME_SIZE) * 100
                sys.stdout.write(f"\r⏳ 수신 중... {percent:.1f}% ({byte_idx}/{FRAME_SIZE})")
                sys.stdout.flush()

        print("\n✅ 수신 완료! 변환 중...")

        # --------------------------------------
        # 2. 이미지 변환 (RGB565 -> BGR888)
        # --------------------------------------
        raw_data = np.frombuffer(raw_buffer, dtype=np.uint8)
        high_byte = raw_data[0::2]
        low_byte  = raw_data[1::2]
        
        # 색상 변환
        R = (high_byte & 0xF8)
        G = ((high_byte & 0x07) << 5) | ((low_byte & 0xE0) >> 3)
        B = (low_byte & 0x1F) << 3
        
        img = np.dstack((B, G, R))
        img = img.reshape((HEIGHT, WIDTH, 3))
        
        # 보기 좋게 4배 확대
        img_large = cv2.resize(img, (640, 480), interpolation=cv2.INTER_NEAREST)

        # --------------------------------------
        # 3. 결과 저장 및 출력
        # --------------------------------------
        # (1) 파일로 무조건 저장 (SSH 사용자용)
        filename = f"capture_{int(time.time())}.png"
        cv2.imwrite(filename, img_large)
        print(f"💾 이미지 파일 저장됨: {filename}")
        print("   (탐색기에서 이 파일을 열어보세요!)")

        # (2) 화면 출력 (모니터 연결된 경우)
        try:
            cv2.imshow("FPGA Camera", img_large)
            if cv2.waitKey(1000) & 0xFF == ord('q'): # 1초간 보여주고 다음 장
                break
        except Exception as e:
            print("⚠️ 화면 출력이 불가능한 환경입니다 (SSH 등). 저장된 파일을 확인하세요.")

except KeyboardInterrupt:
    print("\n종료합니다.")
finally:
    GPIO.cleanup()
    cv2.destroyAllWindows()
