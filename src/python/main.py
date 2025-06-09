from asyncio import sleep
import cv2, logging
import numpy as np
import serial
import time

from maze import process_grid, breadth_first_search
from read_image import grab_info

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s", level=logging.INFO
)

log = logging.getLogger(__name__)

def get_instruction(cap):
    # Read the current frame from the webcam
    current_grid = grab_info(cap=cap)
    if current_grid is None:
        return None

    # Process the grid to find the ball and target positions
    graph, ball_pos, tar_pos = process_grid(current_grid)
    if ball_pos is None or tar_pos is None:
        return None

    # BFS
    overall_direction = breadth_first_search(graph, tar_pos, ball_pos)
    return overall_direction

def send_instruction(arduino, instruction: str):
    arduino.write(instruction.encode())

def read_message(arduino):
    while True:
        response = arduino.readline().decode().strip()
        if response:
            return response

def main(port: str, baudrate: int = 9600, sleep_time: int = 1.5):
    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")

    # Build serial connection
    arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    time.sleep(sleep_time)  # Wait for the connection to establish

    # Main loop
    while True:
        # Get the instruction
        instruction = get_instruction(cap=cap)
        if instruction is None:
            continue
        
        # Send the instruction
        logging.info(f"Sending instruction: {instruction}")
        arduino.write(str(instruction[0]).encode())
        arduino.write(str(instruction[1]).encode())
        _ = read_message(arduino)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release stuffs
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()


def testSerial(port: str, baudrate: int = 9600, sleep_time: int = 1.5):
    arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    time.sleep(sleep_time)  # Wait for the connection to establish

    for i in range(5):
        send_instruction(arduino=arduino, instruction=str(i))
        
        response = read_message(arduino)
        logging.info(f"Arduino says: {response}")
    
    arduino.close()

if __name__ == "__main__":
    testSerial(port="/dev/cu.usbserial-120")
    exit(0)
    main(port="/dev/cu.usbserial-120")