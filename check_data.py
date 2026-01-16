import os
import time
import numpy as np
import cv2
import RPi.GPIO as GPIO

# ==========================================
# [설정] 핀 맵핑 (제어 신호만 GPIO로 사용)
# ==========================================
# 데이터(8~15번)는 SMI가 자동으로 가져가므로 설정 불필요!
PIN_VALID = 20
PIN_ACK   = 21

# 영상 설정
WIDTH = 160
HEIGHT = 120
FRAME_SIZE = WIDTH * HEIGHT * 2  # 38,400 bytes

# ==========================================
# [초기화] GPIO 설정 (Handshake용)
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(PIN_VALID, GPIO.IN)
GPIO.setup(PIN_ACK, GPIO.OUT, initial=GPIO.LOW)

print("------------------------------------------------")
print("🚀 SMI 모드 (Python Native) 시작!")
print("   - /dev/smi 파일을 통해 고속으로 읽습니다.")
print("------------------------------------------------")

try:
    # ★ 핵심: SMI 장치를 파일처럼 엽니다 (바이너리 읽기 모드)
    # 버퍼링을 끄기 위해 buffering=0 옵션을 줄 수도 있습니다.
    smi_file = open("/dev/smi", "rb")

    while True:
        start_time = time.time()

        # 1. FPGA가 준비될 때까지 대기 (Valid 체크)
        # (SMI가 너무 빨라서 FPGA가 준비 안 됐는데 읽으면 쓰레기 값이 들어옵니다)
        # while GPIO.input(PIN_VALID) == 0:
        #     pass

        # 2. ACK 신호 켜기 ("나 읽을 준비 됐어!")
        # GPIO.output(PIN_ACK, GPIO.HIGH)

        # 3. 데이터 읽기 (이 한 줄이 C언어 루프를 대체합니다!)
        # 38,400 바이트가 찰 때까지 커널이 알아서 기다렸다가 가져옵니다.
        raw_data_bytes = smi_file.read(FRAME_SIZE)

        # 4. ACK 신호 끄기
        # GPIO.output(PIN_ACK, GPIO.LOW)

        # 5. FPGA가 Valid 끌 때까지 대기 (동기화)
        # while GPIO.input(PIN_VALID) == 1:
            # pass
            
        # ==========================================
        # [변환] 바이트 -> 이미지 (이전과 동일)
        # ==========================================
        if not raw_data_bytes or len(raw_data_bytes) != FRAME_SIZE:
            continue # 데이터가 덜 들어왔으면 스킵

        # 문자열(bytes)을 숫자 배열(numpy)로 변환
        raw_data = np.frombuffer(raw_data_bytes, dtype=np.uint8)
        
        high_byte = raw_data[0::2]
        low_byte  = raw_data[1::2]
        
        # RGB565 -> BGR888
        R = (high_byte & 0xF8)
        G = ((high_byte & 0x07) << 5) | ((low_byte & 0xE0) >> 3)
        B = (low_byte & 0x1F) << 3
        
        img = np.dstack((B, G, R))
        img = img.reshape((HEIGHT, WIDTH, 3))
        
        # 화면 출력
        img_large = cv2.resize(img, (640, 480), interpolation=cv2.INTER_NEAREST)
        
        # FPS 출력
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(img_large, f"FPS: {fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("SMI Python Stream", img_large)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n종료합니다.")
except FileNotFoundError:
    print("\n❌ 오류: /dev/smi 파일을 찾을 수 없습니다.")
    print("   /boot/config.txt 에 'dtoverlay=smi-dev'를 추가하고 재부팅했는지 확인해주세요!")
finally:
    if 'smi_file' in locals():
        smi_file.close()
    GPIO.cleanup()
    cv2.destroyAllWindows()