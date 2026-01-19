#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import numpy as np

# ==========================================
# GPIO 핀 설정 (확정!)
# ==========================================
# 데이터 핀 (순서대로 D0~D7)
DATA_PINS = [8, 9, 10, 11, 12, 13, 14, 15]

# 제어 핀 (실제 연결 확인 필요! - 임시값)
VALID_PIN = 20  
ACK_PIN = 21    

# Frame settings
WIDTH = 320
HEIGHT = 240
FRAME_SIZE = WIDTH * HEIGHT * 2  # 153,600 bytes

# ==========================================
# GPIO 초기화
# ==========================================
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 데이터 핀 (입력)
    for pin in DATA_PINS:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # 제어 핀
    GPIO.setup(VALID_PIN, GPIO.IN)
    GPIO.setup(ACK_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    print("✅ GPIO Setup Complete")
    print("=" * 70)
    print(f"   Data Pins (D0-D7): {DATA_PINS}")
    print(f"   Valid Pin: GPIO {VALID_PIN}")
    print(f"   ACK Pin: GPIO {ACK_PIN}")
    print("=" * 70)

# ==========================================
# 신호 상태 확인
# ==========================================
def check_signals():
    print("\n🔍 Checking FPGA Signal Status:")
    print("-" * 70)
    
    # VALID 신호 확인
    valid_state = GPIO.input(VALID_PIN)
    print(f"   VALID pin (GPIO {VALID_PIN}): {'HIGH ✅' if valid_state else 'LOW'}")
    
    # 데이터 핀 확인
    print(f"\n   Data pins:")
    data_val = 0
    for i, pin in enumerate(DATA_PINS):
        state = GPIO.input(pin)
        data_val |= (state << i)
        print(f"      D{i} (GPIO {pin:2d}): {'1' if state else '0'}")
    
    print(f"\n   Current data byte: 0x{data_val:02X} ({data_val:3d})")
    print("-" * 70)

# ==========================================
# 1바이트 읽기 (핸드셰이크)
# ==========================================
def read_byte_handshake():
    """FPGA와 핸드셰이크하며 1바이트 읽기"""
    
    # 1. ACK HIGH → "나 준비됨!"
    GPIO.output(ACK_PIN, GPIO.HIGH)
    
    # 2. VALID HIGH 대기 (최대 1초)
    timeout = 0
    while GPIO.input(VALID_PIN) == GPIO.LOW:
        timeout += 1
        if timeout > 1000000:
            GPIO.output(ACK_PIN, GPIO.LOW)
            return None, "VALID_TIMEOUT"
        time.sleep(0.000001)  # 1us
    
    # 3. 데이터 읽기
    byte_val = 0
    for i, pin in enumerate(DATA_PINS):
        if GPIO.input(pin):
            byte_val |= (1 << i)
    
    # 4. ACK LOW → "받았어!"
    GPIO.output(ACK_PIN, GPIO.LOW)
    
    # 5. VALID LOW 대기
    timeout = 0
    while GPIO.input(VALID_PIN) == GPIO.HIGH:
        timeout += 1
        if timeout > 1000000:
            return byte_val, "VALID_LOW_TIMEOUT"
        time.sleep(0.000001)
    
    return byte_val, "OK"

# ==========================================
# N바이트 테스트 읽기
# ==========================================
def test_read_bytes(num_bytes=100):
    print(f"\n📊 Testing: Reading {num_bytes} bytes")
    print("=" * 70)
    
    received_data = []
    errors = []
    
    start_time = time.time()
    
    for i in range(num_bytes):
        byte_val, status = read_byte_handshake()
        
        if status != "OK":
            error_msg = f"Byte {i}: {status}"
            errors.append(error_msg)
            print(f"❌ {error_msg}")
            
            if len(errors) >= 5:
                print(f"\n⚠️ Too many errors, stopping...")
                break
            continue
        
        received_data.append(byte_val)
        
        # Progress (매 10 바이트마다)
        if (i + 1) % 10 == 0:
            print(f"  [{i+1:4d}/{num_bytes}] Last byte: 0x{byte_val:02X} ({byte_val:3d})  {byte_val:08b}")
    
    elapsed = time.time() - start_time
    
    print("=" * 70)
    print(f"✅ Received {len(received_data)}/{num_bytes} bytes in {elapsed:.3f}s")
    if errors:
        print(f"⚠️ Errors: {len(errors)}")
    
    return received_data, errors

# ==========================================
# 데이터 분석
# ==========================================
def analyze_yuv_data(data):
    if len(data) < 4:
        print("❌ Not enough data for analysis")
        return
    
    print("\n📈 YUV422 Data Analysis:")
    print("=" * 70)
    print(f"  Total bytes: {len(data)}")
    print(f"  Value range: 0x{min(data):02X} - 0x{max(data):02X} ({min(data)} - {max(data)})")
    print(f"  Average: {sum(data)/len(data):.1f}")
    
    # YUV422 패턴 확인 (Y0 U Y1 V)
    print("\n  YUV422 Pattern (first 5 pixel pairs):")
    print("  " + "-" * 66)
    print(f"  {'Pair':>4} | {'Y0':>3} {'U':>3} {'Y1':>3} {'V':>3} | Binary")
    print("  " + "-" * 66)
    
    for i in range(min(5, len(data)//4)):
        idx = i * 4
        if idx + 3 < len(data):
            Y0, U, Y1, V = data[idx:idx+4]
            print(f"  {i:4d} | {Y0:3d} {U:3d} {Y1:3d} {V:3d} | "
                  f"{Y0:08b} {U:08b} {Y1:08b} {V:08b}")
    
    print("  " + "-" * 66)
    
    # 통계
    if len(data) >= 4:
        Y_values = [data[i] for i in range(0, len(data), 2)]  # Y0, Y1
        U_values = [data[i] for i in range(1, len(data), 4)]  # U
        V_values = [data[i] for i in range(3, len(data), 4)]  # V
        
        print(f"\n  Component Statistics:")
        print(f"    Y (Luma):      avg={sum(Y_values)/len(Y_values):.1f}, "
              f"range={min(Y_values)}-{max(Y_values)}")
        if U_values:
            print(f"    U (Chroma Cb): avg={sum(U_values)/len(U_values):.1f}, "
                  f"range={min(U_values)}-{max(U_values)}")
        if V_values:
            print(f"    V (Chroma Cr): avg={sum(V_values)/len(V_values):.1f}, "
                  f"range={min(V_values)}-{max(V_values)}")
    
    print("=" * 70)

# ==========================================
# 전체 프레임 캡처 (선택적)
# ==========================================
def capture_full_frame():
    print(f"\n🎥 Capturing FULL FRAME ({FRAME_SIZE} bytes)")
    print("⚠️  This will take a LONG time (~2-10 minutes)!")
    
    confirm = input("Continue? (yes/no): ").lower()
    if confirm != 'yes':
        return None
    
    print("\n📸 Starting capture...")
    print("=" * 70)
    
    frame_data = bytearray(FRAME_SIZE)
    start_time = time.time()
    
    for i in range(FRAME_SIZE):
        byte_val, status = read_byte_handshake()
        
        if status != "OK":
            print(f"\n❌ Error at byte {i}/{FRAME_SIZE}: {status}")
            return None
        
        frame_data[i] = byte_val
        
        # Progress (매 15,360 바이트 = 10%)
        if (i + 1) % 15360 == 0:
            progress = (i + 1) / FRAME_SIZE * 100
            elapsed = time.time() - start_time
            eta = elapsed / (i + 1) * FRAME_SIZE - elapsed
            print(f"  Progress: {progress:5.1f}% ({i+1}/{FRAME_SIZE}) "
                  f"- Elapsed: {elapsed:.1f}s, ETA: {eta:.1f}s")
    
    elapsed = time.time() - start_time
    print("=" * 70)
    print(f"✅ Frame captured in {elapsed:.2f}s")
    
    return np.frombuffer(frame_data, dtype=np.uint8)

# ==========================================
# 메인 함수
# ==========================================
def main():
    print("=" * 70)
    print("🧪 OV7670 FPGA Communication Test")
    print("   Raspberry Pi 4 - GPIO Mode")
    print("=" * 70)
    
    setup_gpio()
    
    try:
        # 1. 신호 상태 확인
        check_signals()
        
        input("\nPress Enter to start testing...")
        
        # 2. 소량 데이터 테스트 (100 바이트)
        print("\n" + "="*70)
        print("TEST 1: Reading 100 bytes")
        print("="*70)
        
        data, errors = test_read_bytes(100)
        
        if data:
            analyze_yuv_data(data)
        
        # 3. 추가 테스트 메뉴
        while True:
            print("\n" + "="*70)
            print("Options:")
            print("  1. Read more bytes (custom amount)")
            print("  2. Capture full frame (153,600 bytes - SLOW!)")
            print("  3. Check signals again")
            print("  4. Exit")
            print("="*70)
            
            choice = input("Select option (1-4): ").strip()
            
            if choice == '1':
                try:
                    num = int(input("How many bytes? (1-10000): "))
                    if 1 <= num <= 10000:
                        data, errors = test_read_bytes(num)
                        if data:
                            analyze_yuv_data(data)
                    else:
                        print("⚠️ Please enter a number between 1 and 10000")
                except ValueError:
                    print("⚠️ Invalid number")
            
            elif choice == '2':
                yuv_data = capture_full_frame()
                if yuv_data is not None:
                    print("\n💾 Saving frame data...")
                    np.save('captured_frame.npy', yuv_data)
                    print("✅ Saved: captured_frame.npy")
                    analyze_yuv_data(yuv_data.tolist()[:100])  # 처음 100바이트 분석
            
            elif choice == '3':
                check_signals()
            
            elif choice == '4':
                break
            
            else:
                print("⚠️ Invalid option")
    
    except KeyboardInterrupt:
        print("\n\n⚠️ Interrupted by user")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        GPIO.cleanup()
        print("\n✅ GPIO Cleanup Complete")

if __name__ == "__main__":
    main()