#!/usr/bin/env python3
import numpy as np
import cv2

def yuv422_to_bgr_fast(yuv_data):
    """YUV422 → BGR 변환 (빠른 버전)"""
    
    WIDTH = 320
    HEIGHT = 240
    
    # YUV 분리
    Y0 = yuv_data[0::4].astype(np.int32)
    U  = yuv_data[1::4].astype(np.int32)
    Y1 = yuv_data[2::4].astype(np.int32)
    V  = yuv_data[3::4].astype(np.int32)
    
    # YUV → RGB 변환
    C0 = Y0 - 16
    C1 = Y1 - 16
    D  = U - 128
    E  = V - 128
    
    # Pixel 0
    R0 = np.clip((298 * C0 + 409 * E + 128) >> 8, 0, 255).astype(np.uint8)
    G0 = np.clip((298 * C0 - 100 * D - 208 * E + 128) >> 8, 0, 255).astype(np.uint8)
    B0 = np.clip((298 * C0 + 516 * D + 128) >> 8, 0, 255).astype(np.uint8)
    
    # Pixel 1
    R1 = np.clip((298 * C1 + 409 * E + 128) >> 8, 0, 255).astype(np.uint8)
    G1 = np.clip((298 * C1 - 100 * D - 208 * E + 128) >> 8, 0, 255).astype(np.uint8)
    B1 = np.clip((298 * C1 + 516 * D + 128) >> 8, 0, 255).astype(np.uint8)
    
    # 인터리브
    total_pixels = WIDTH * HEIGHT
    B = np.empty(total_pixels, dtype=np.uint8)
    G = np.empty(total_pixels, dtype=np.uint8)
    R = np.empty(total_pixels, dtype=np.uint8)
    
    B[0::2] = B0
    B[1::2] = B1
    G[0::2] = G0
    G[1::2] = G1
    R[0::2] = R0
    R[1::2] = R1
    
    # BGR 이미지 생성
    bgr_image = np.dstack((B, G, R))
    bgr_image = bgr_image.reshape((HEIGHT, WIDTH, 3))
    
    return bgr_image

# 메인
if __name__ == "__main__":
    print("📸 Converting captured frame to image...")
    
    # 로드
    yuv_data = np.load('captured_frame.npy')
    print(f"✅ Loaded: {len(yuv_data)} bytes")
    
    # 변환
    print("🎨 Converting YUV422 to BGR...")
    bgr_image = yuv422_to_bgr_fast(yuv_data)
    
    # 원본 저장
    cv2.imwrite('captured_320x240.png', bgr_image)
    print("💾 Saved: captured_320x240.png")
    
    # 확대 버전 저장
    large = cv2.resize(bgr_image, (640, 480), interpolation=cv2.INTER_LINEAR)
    cv2.imwrite('captured_640x480.png', large)
    print("💾 Saved: captured_640x480.png")
    
    # 더 큰 버전
    xlarge = cv2.resize(bgr_image, (1280, 960), interpolation=cv2.INTER_LINEAR)
    cv2.imwrite('captured_1280x960.png', xlarge)
    print("💾 Saved: captured_1280x960.png")
    
    print("\n✅ Done! Open the PNG files to see your image!")
    print("   - captured_320x240.png (original)")
    print("   - captured_640x480.png (2x)")
    print("   - captured_1280x960.png (4x)")