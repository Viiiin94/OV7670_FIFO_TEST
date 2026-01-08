import serial
import numpy as np
import cv2
import time

# ==========================================
# [중요] 설정 확인
# ==========================================
PORT = '/dev/ttyUSB1'  # Moserial에서 썼던 포트 이름 (윈도우는 'COM3' 등)
BAUD = 9600  # [핵심] FPGA와 동일하게 9600으로 설정!
WIDTH = 160  # QQVGA 너비
HEIGHT = 120  # QQVGA 높이
IMG_SIZE = WIDTH * HEIGHT * 2  # 38,400 bytes

try:
    # 시리얼 포트 열기
    ser = serial.Serial(PORT, BAUD, timeout=300)  # 타임아웃 넉넉하게
    print(f"✅ Connected to {PORT} at {BAUD}bps")
    print("🚀 FPGA의 가운데 버튼(Reset)을 눌러주세요! (약 40초 소요)")

    while True:
        # 데이터가 들어올 때까지 대기
        if ser.in_waiting > 0:
            print("📥 데이터 수신 시작... 기다려주세요!")

            # 이미지 데이터 전체 읽기 (오래 걸림)
            raw_data = ser.read(IMG_SIZE)

            if len(raw_data) == IMG_SIZE:
                print("✨ 이미지 수신 완료! 변환 중...")

                # 바이트 -> 넘파이 배열
                arr = np.frombuffer(raw_data, dtype=np.uint8)

                # RGB565 복원 (High Byte + Low Byte)
                high_bytes = arr[0::2]
                low_bytes = arr[1::2]
                pixel16 = (high_bytes.astype(np.uint16) << 8) | low_bytes.astype(np.uint16)

                # RGB565 -> RGB888 변환
                r = ((pixel16 >> 11) & 0x1F) * 255 // 31
                g = ((pixel16 >> 5) & 0x3F) * 255 // 63
                b = (pixel16 & 0x1F) * 255 // 31

                # 이미지 생성 (OpenCV는 BGR 순서)
                img = np.dstack((b, g, r)).astype(np.uint8)
                img = img.reshape((HEIGHT, WIDTH, 3))

                # 보기 좋게 3배 확대
                # img_large = cv2.resize(img, (WIDTH * 3, HEIGHT * 3), interpolation=cv2.INTER_NEAREST)

                # 화면 표시
                cv2.imshow('Basys3 Camera Snapshot', img)
                print("📸 찰칵! 다음 사진을 찍으려면 다시 버튼을 누르세요.")
            else:
                print(f"⚠️ 데이터 부족: {len(raw_data)} / {IMG_SIZE} bytes")

        # 'q' 키를 누르면 종료
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"❌ 에러 발생: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()