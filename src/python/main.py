from asyncio import sleep
import cv2, logging
import numpy as np
import serial
import time

from maze import process_grid, breadth_first_search, visualize_path_on_frame
from read_image import grab_info

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s", level=logging.INFO
)

log = logging.getLogger(__name__)

def get_instruction(cap):
    # Read the current frame from the webcam
    current_grid, frame, cell_h, cell_w = grab_info(cap=cap, rows=216, cols=384)
    print("Current grid shape:", np.shape(current_grid))
    if current_grid is None:
        logging.error("Failed to grab grid information from webcam.")
        return None, (-1001, -1001)

    # Process the grid to find the ball and target positions
    graph, ball_pos, tar_pos = process_grid(current_grid)
    if ball_pos is None or tar_pos is None:
        logging.error("Ball or target position not found in the grid.")
        return frame, (-1001, -1001)

    # BFS - now returns both path and direction
    path, overall_direction = breadth_first_search(graph=graph, grid=current_grid, source=tar_pos, target=ball_pos)
    
    # Visualize the path on the frame
    if path:
        frame = visualize_path_on_frame(frame, path, cell_h, cell_w)
    
    # Display the frame with the path
    cv2.imshow("Ball Path Visualization", frame)
    
    return overall_direction

def send_instruction(arduino, instruction: str):
    arduino.write(instruction.encode())

def read_message(arduino):
    while True:
        response = arduino.readline().decode().strip()
        if response:
            return response

def main(port: str, cam_id: int, baudrate: int = 115200, sleep_time: int = 1.5):
    # Open webcam
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")
    
    # Set resolution for better visualization
    # desired_width = 640
    # desired_height = 480
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
    
    # Allow camera to initialize
    time.sleep(0.5)

    # Build serial connection
    arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    time.sleep(sleep_time)  # Wait for the connection to establish

    logging.info("Arduino resetting...")
    send_instruction(arduino=arduino, instruction="0,0") # Reset the Arduino
    _ = read_message(arduino)
    logging.info("Arduino reset complete.")

    # Main loop
    previouse_instruction = (-1001, -1001)
    beta = 1
    while True:
        # Get the instruction
        instruction = get_instruction(cap=cap)
        
        # Send the instruction
        if instruction == (-1001, -1001):
            instruction_to_send = "-1001,-1001"
        else:
            # instruction_to_send = f"{(abs(instruction[1])/instruction[1]) * np.sqrt(abs(instruction[1])) * 4 // 2},{-(abs(instruction[0])/instruction[0]) * np.sqrt(abs(instruction[0])) * 4 // 2}"
            if (instruction[1], -instruction[0]) == previouse_instruction:
                beta += 1
            else:
                beta = 1
            instruction_to_send = f"{beta*instruction[1]},{-beta*instruction[0]}"
            previouse_instruction = (instruction[1], -instruction[0])
        print("Current instruction:", instruction_to_send)
        print("Previous instruction:", previouse_instruction)
        send_instruction(arduino=arduino, instruction=instruction_to_send)
        logging.info(f"Instruction sent: {instruction_to_send}")

        _ = read_message(arduino)
        logging.info(f"Arduino received: {_}")

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()

if __name__ == "__main__":
    # Update port and cam_id for your system
    main(port="/dev/cu.usbserial-120", cam_id=0)