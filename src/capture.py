import cv2
import numpy as np
import urllib.request
import threading
import time

# 1. STREAM READER SETUP
CAM_URL = "http://192.168.137.211:81/stream"  # Update to current ESP IP
latest_frame = None
lock = threading.Lock()

def reader():
    global latest_frame
    while True:
        try:
            stream = urllib.request.urlopen(CAM_URL, timeout=15)
            buf = b""
            while True:
                chunk = stream.read(4096)
                if not chunk:
                    break
                buf += chunk
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a + 2) if a != -1 else -1
                if a != -1 and b != -1:
                    jpg = buf[a:b + 2]
                    buf = buf[b + 2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with lock:
                            latest_frame = frame
        except Exception:
            time.sleep(2)

threading.Thread(target=reader, daemon=True).start()

# 2. COLOR CLASSIFICATION
def classify_color(h, s, v):
    if v < 15:                 return "Black"
    if v > 75 and s < 15:      return "White"
    if s < 20:                 return "Gray"
    
    if h < 15 or h >= 345:     return "Red"
    if 15 <= h < 45:           return "Orange"
    if 45 <= h < 75:           return "Yellow"
    if 75 <= h < 165:          return "Green"
    if 165 <= h < 195:         return "Cyan"
    if 195 <= h < 255:         return "Blue"
    if 255 <= h < 285:         return "Purple"
    if 285 <= h < 345:         return "Pink" if s > 50 else "Magenta"
    return "Unknown"

# 3. OPTIMIZED VISION PIPELINE
def process_pipeline(frame):
    height, width = frame.shape[:2]
    
    # Blur to reduce camera noise
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # STRATEGY: Target extreme vividness.
    # Hue: 0-179 (Accept ANY color)
    # Saturation: 120-255 (Must be highly vibrant, ignores washed-out backgrounds)
    # Value: 80-255 (Must be bright, ignores shadows and black objects)
    lower_bound = np.array([0, 120, 80])
    upper_bound = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Clean up the mask (remove tiny background specks, fill small holes in the object)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    error = 0.0
    detected_color = "None"
    
    if contours:
        # Grab the absolute largest highly-saturated blob
        largest_blob = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_blob) > 800: # 800 ignores small noisy colored spots
            M = cv2.moments(largest_blob)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Sample the exact color in a 20x20 box at the object's center
                y1, y2 = max(0, cy-10), min(hsv.shape[0], cy+10)
                x1, x2 = max(0, cx-10), min(hsv.shape[1], cx+10)
                region = hsv[y1:y2, x1:x2]
                
                if region.size > 0:
                    h_cpp = np.mean(region[:,:,0]) * 2
                    s_cpp = (np.mean(region[:,:,1]) / 255) * 100
                    v_cpp = (np.mean(region[:,:,2]) / 255) * 100
                    detected_color = classify_color(h_cpp, s_cpp, v_cpp)
                
                # Calculate Error (-1.0 to +1.0)
                center_image = width / 2
                error = (cx - center_image) / center_image
                
                # Visuals: Draw a tight bounding box and a red crosshair target
                x, y, w, h_box = cv2.boundingRect(largest_blob)
                cv2.rectangle(frame, (x, y), (x + w, y + h_box), (0, 255, 0), 2)
                cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 15, 2)
                
                text = f"{detected_color} (Err: {error:.2f})"
                cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, mask

# 4. MAIN LOOP
last_shown = None

while True:
    with lock:
        frame = latest_frame
        
    if frame is not None and frame is not last_shown:
        last_shown = frame
        processed_frame, debug_mask = process_pipeline(frame.copy())
        
        cv2.imshow("ESP32-CAM Vision - Most Colored Object", processed_frame)
        cv2.imshow("Saturation Mask", debug_mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()