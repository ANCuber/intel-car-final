import cv2
import numpy as np

def grab_info(cap, rows=216, cols=384): 
    """Original grab_info function that just returns the classification grid"""
    # desired_width = 640
    # desired_height = 480
    # desired_width = 256 * 4 // 3
    # desired_height = 192 * 4 // 3

    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

    ret, frame = cap.read()

    # Failed to grab frame
    if not ret:
        return None, None, None
    
    # Get frame dimensions directly
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

    mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 120]))
    mask_white = cv2.inRange(hsv, np.array([0, 0, 120]), np.array([180, 50, 255]))
    mask_green = cv2.inRange(hsv, np.array([40, 100, 100]), np.array([85, 255, 255]))
    mask_blue = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([130, 255, 255]))
    mask_orange = cv2.inRange(hsv, np.array([5, 100, 100]), np.array([25, 255, 255]))

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
            orange = mask_orange[y1:y2, x1:x2]

            red_count = cv2.countNonZero(red)
            black_count = cv2.countNonZero(black)
            white_count = cv2.countNonZero(white)
            green_count = cv2.countNonZero(green)
            blue_count = cv2.countNonZero(blue)
            orange_count = cv2.countNonZero(orange)

            if orange_count > threshold:
                classification[i][j] = 'O'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 1)
            elif red_count > threshold:
                classification[i][j] = 'R'
                # Draw color blocks on frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            elif green_count > threshold:
                classification[i][j] = 'G'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
            elif black_count > threshold:
                classification[i][j] = 'D'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 1)
            elif white_count > threshold:
                classification[i][j] = 'W'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)
            elif blue_count > threshold:
                classification[i][j] = 'B' 
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)
            
    # Show result without path visualization yet
    # We'll show the final visualization from main.py
    return classification, frame, cell_h, cell_w

# Adding this function for compatibility with your requested code structure
def grab_info_and_visualize(cap, rows=216, cols=384):
    """Alias for grab_info - returns grid, frame, cell_h, cell_w"""
    return grab_info(cap, rows, cols)

if __name__ == "__main__":
    # Test code
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video capture.")
        exit()

    import time
    time.sleep(1.0)  # Sleep for 1 second

    while True:
        num_rows = 216
        num_cols = 384
            
        current_grid, frame, cell_h, cell_w = grab_info(cap=cap, rows=num_rows, cols=num_cols)
        if current_grid is None:
            print("Failed to grab frame.")
            continue
        
        cv2.imshow("Webcam Maze Detection", frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()