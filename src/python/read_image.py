import cv2
import numpy as np

def grab_info(cap, rows=100, cols=100, rlength=900, clength=900): # Grid size
    # Wrong inputs
    if rlength % rows != 0 or clength % cols != 0:
        raise ValueError("rows and cols must divide length evenly.")
    
    ret, frame = cap.read()

    # Failed to grab frame
    if not ret:
        return None
    
    # Resize for consistent processing
    frame = cv2.resize(frame, (rlength, clength))
    h, w, _ = frame.shape

    # Initialize classification grid
    cell_h, cell_w = h // rows, w // cols
    classification = [['U' for _ in range(cols)] for _ in range(rows)]

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks
    mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 90]))
    mask_white = cv2.inRange(hsv, np.array([0, 0, 120]), np.array([180, 50, 255]))
    mask_green = cv2.inRange(hsv, np.array([40, 100, 100]), np.array([85, 255, 255]))
    mask_blue = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([130, 255, 255]))

    # Analyze each grid cell
    threshold = cell_h * cell_w * 2 // 3 # Threshold for minimum pixel count
    for i in range(rows):
        for j in range(cols):
            y1, y2 = i * cell_h, (i + 1) * cell_h
            x1, x2 = j * cell_w, (j + 1) * cell_w

            red = mask_red[y1:y2, x1:x2]
            black = mask_black[y1:y2, x1:x2]
            white = mask_white[y1:y2, x1:x2]
            green = mask_green[y1:y2, x1:x2]
            blue = mask_blue[y1:y2, x1:x2]

            red_count = cv2.countNonZero(red)
            black_count = cv2.countNonZero(black)
            white_count = cv2.countNonZero(white)
            green_count = cv2.countNonZero(green)
            blue_count = cv2.countNonZero(blue)

            if red_count > black_count and red_count > white_count and red_count > threshold:
                classification[i][j] = 'R'
                # Draw color blocks on frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            elif green_count > max(red_count, black_count, white_count) and green_count > threshold:
                classification[i][j] = 'G'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
            elif black_count > white_count and black_count > threshold:
                classification[i][j] = 'D'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 1)
            elif white_count > threshold:
                classification[i][j] = 'W'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)
            elif blue_count > threshold:
                classification[i][j] = 'B' 
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)

    # Show result
    cv2.imshow("Webcam Maze Detection", frame)
    return classification
    
if __name__ == "__main__":
    # Open webcam (0 is usually the default camera)
    cap = cv2.VideoCapture(0)

    # print("Testing camera indices with AVFoundation:")
    # for i in range(5):
    #     cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
    #     if cap.isOpened():
    #         print(f"Camera index {i} works!")
    #         cap.release()

    cnt = 1
    while True:
        current_grid = grab_info(cap=cap, rows=150, cols=150, rlength=600, clength=600)
        if current_grid is None:
            continue
        
        if cnt % 100000:
            # print(current_grid)
            cnt = 0
        cnt += 1

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()