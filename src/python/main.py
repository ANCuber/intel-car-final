import cv2, logging
import numpy as np

from BTinterface import BTInterface
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

def main(bt_port: str):
    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")


    #  Bluetooth 
    interface = BTInterface(port=bt_port)
    interface.send_instruction("R")

    while True:
        # Get the instruction
        instruction = get_instruction(cap=cap)
        if instruction is None:
            continue
        
        # Send the instruction
        print(instruction)
        interface.send_instruction(str(instruction[0]))
        interface.send_instruction(str(instruction[1]))
        _ = interface.fetch_info()

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

def testBT(bt_port):
    interface = BTInterface(port=bt_port)
    interface.send_instruction("R")
    msg = interface.fetch_info()
    print(msg)

if __name__ == "__main__":
    # Change the port to your Bluetooth device
    testBT(bt_port="/dev/cu.HC-05")
    exit(0)
    main()