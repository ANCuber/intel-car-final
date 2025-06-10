import cv2
import numpy as np

def grab_info(cap, rows=150, cols=200): # Grid size, rlength and clength removed
    desired_width = 640
    desired_height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

    # Verify the resolution (optional)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # print(f"Attempted to set resolution to {desired_width}x{desired_height}")
    # print(f"Actual resolution set to {actual_width}x{actual_height}")

    ret, frame = cap.read()

    # Failed to grab frame
    if not ret:
        return None
    
    # Get frame dimensions directly
    h, w, _ = frame.shape

    # Wrong inputs for grid division
    if h % rows != 0 or w % cols != 0:
        # You might want to handle this more gracefully, 
        # e.g., by adjusting rows/cols or cropping, 
        # or inform the user that the chosen grid doesn't fit the resolution.
        # For now, we'll raise an error or log a warning.
        print(f"Warning: Frame dimensions {w}x{h} are not evenly divisible by cols={cols}, rows={rows}. Grid cells might not be uniform.")
        # Or, uncomment to raise an error:
        # raise ValueError(f"Frame dimensions {w}x{h} must be evenly divisible by cols={cols} and rows={rows}.")

    # Initialize classification grid
    cell_h, cell_w = h // rows, w // cols
    classification = [['U' for _ in range(cols)] for _ in range(rows)]

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks
    mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 100]))
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
            
    # Show result
    cv2.imshow("Webcam Maze Detection", frame)
    return classification
    
if __name__ == "__main__":
    # Option 1: Try with a different backend like CAP_DSHOW
    # cap = cv2.VideoCapture(1, cv2.CAP_DSHOW) 
    # If the line above doesn't work or you want to stick to the default backend first, use:
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video capture.")
        exit()

    # Add a small delay to allow the camera to initialize with the new settings
    import time
    time.sleep(1.0) # Sleep for 1 second

    # print("Testing camera indices with AVFoundation:")
    # for i in range(5):
    #     cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
    #     if cap.isOpened():
    #         print(f"Camera index {i} works!")
    #         cap.release()

    cnt = 1
    while True:
        # Call grab_info without rlength and clength, adjust rows/cols as needed
        # Ensure rows and cols are compatible with the actual_width and actual_height
        # For example, if actual_width=320, actual_height=240:
        # You could use rows=24, cols=32 for 10x10 cells
        # Or rows=10, cols=10 if you want fewer, larger cells (adjust cell_h, cell_w logic or accept non-square cells)
        # For this example, let's make rows and cols smaller to fit the new resolution
        # and be divisible.
        # For 320x240, let's try a 24x32 grid (10x10 pixel cells)
        # Or, to keep it simple, let's try to make them somewhat square-ish and divisible
        # e.g. rows=24, cols=32 (cells of 10x10)
        # or rows=12, cols=16 (cells of 20x20)
        
        # Adjust these based on your needs and the actual camera resolution
        num_rows = 96 
        num_cols = 128
            
        current_grid = grab_info(cap=cap, rows=num_rows, cols=num_cols)
        if current_grid is None:
            print("Failed to grab frame.")
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