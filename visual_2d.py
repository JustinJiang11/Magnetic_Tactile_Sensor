import pygame
import sys
import math
import numpy as np
import time
import serial.serialutil
import torch
import torch.nn as nn
from my_sensor import MySensorBase
from low_pass_filter import LPFilter

class ReSkinMLP(nn.Module):
    def __init__(self):
        super(ReSkinMLP, self).__init__()
        self.layer1 = nn.Linear(15, 200)
        self.relu1 = nn.ReLU()
        self.layer2 = nn.Linear(200, 200)
        self.layer3 = nn.Linear(200, 40)
        self.layer4 = nn.Linear(40, 200)
        self.relu4 = nn.ReLU()
        self.layer5 = nn.Linear(200, 200)
        self.relu5 = nn.ReLU()
        self.output_layer = nn.Linear(200, 3)

    def forward(self, x):
        x = self.relu1(self.layer1(x))
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.relu4(self.layer4(x))
        x = self.relu5(self.layer5(x))
        x = self.output_layer(x)
        return x

def init_pygame():
    """Initializes Pygame window and assets."""
    time.sleep(1)
    pygame.init()
    clock = pygame.time.Clock()
    screen_width, screen_height = 425, 600
    # screen_width, screen_height = 2267, 3200
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption('My Tactile Sensor Visualizer')
    pygame.mouse.set_visible(1)

    font = pygame.font.Font(None, 45)
    
    # Load background image
    try:
        bg = pygame.image.load("./pcb2.png")
    except pygame.error:
        print("Warning: Image not found. Using a white background instead.")
        bg = pygame.Surface((screen_width, screen_height))
        bg.fill((255, 255, 255))
        
    return clock, screen, bg, font


def draw_arrowhead(screen, color, start_pos, end_pos, size):
    """Draws a triangle arrowhead at the end of a line."""
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    angle = math.atan2(dy, dx)
    p1 = (end_pos[0] - size * math.cos(angle - math.pi/6), end_pos[1] - size * math.sin(angle - math.pi/6))
    p2 = (end_pos[0] - size * math.cos(angle + math.pi/6), end_pos[1] - size * math.sin(angle + math.pi/6))
    
    pygame.draw.polygon(screen, color, [end_pos, p1, p2])

def draw_text(screen, font, Fx, Fy, Fz):
    """Draws the predicted force values on the screen."""
    text_color = pygame.Color(255, 255, 255) 
    
    # Calculate screen position for the text block (e.g., bottom left)
    x_pos = 10
    y_start = 450
    line_height = 35

    text_lines = [
        f"Fx: {Fx:.2f} N",
        f"Fy: {Fy:.2f} N",
        f"Fz:  {Fz:.2f} N"
    ]
    
    for i, line in enumerate(text_lines):
        text_surface = font.render(line, True, text_color)
        screen.blit(text_surface, (x_pos, y_start + i * line_height))


if __name__ == '__main__':
    RED = pygame.Color(255, 0, 0)
    GREEN = pygame.Color(0, 255, 0)
    BLUE = pygame.Color(0, 0 ,255)
    BLACK = pygame.Color(0, 0, 0)

    num_mags = 5
    SCALE = 15
    LENGTH = 30
    arrowhead_size = 10

    # Chip locations in pixels on your visual board
    center_arrow = [215, 206]

    # Attempt to connect to your sensor using the MySensorBase class
    try:
        viz_sensor = MySensorBase(
            num_mags=num_mags,
            port='/dev/ttyACM0',  
            baudrate=115200
        )
    except serial.serialutil.SerialException as e:
        print(f"ERROR: Failed to connect to sensor. Exiting. Details: {e}")
        sys.exit(1)
    
    # Initialize Pygame and get the screen and clock
    clock, screen, bg, font = init_pygame()

    # load model
    MODEL_PATH = "model/model_try.pth"
    try:
        model = ReSkinMLP()
        model.load_state_dict(torch.load(MODEL_PATH))
        model.eval()
        print("Model loaded successfully.")
    except FileNotFoundError:
        print(f"ERROR: Model file not found at '{MODEL_PATH}'. Please ensure the model has been saved.")
        exit()
    
    # Read the first 100 samples as the baseline
    np.set_printoptions(precision=4, suppress=True)
    baseline = viz_sensor.get_baseline(num_samples=100)
    print("Baseline: ", baseline)

    # Initialize filter 
    ALPHA = 1.0
    lp_filter = LPFilter(alpha=ALPHA)

    # Main visualization loop
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == ord('b'):
                        baseline = viz_sensor.get_baseline(num_samples=100)
                        print("Baseline: ", baseline)

             # keep reading from sensor
            sample = viz_sensor.get_sample()
            while (sample is None or sample.data is None or any(val == 0.0 for val in sample.data)):
                sample = viz_sensor.get_sample()
            data = (sample.data - baseline).tolist()
            input_tensor = torch.tensor(data, dtype=torch.float32).unsqueeze(0) # Add batch dimension

             # Predict with the model
            with torch.no_grad():
                prediction = model(input_tensor)
            raw_forces = prediction.numpy().flatten()
            # Apply the low-pass filter to each force component
            filtered_fx = lp_filter.next(raw_forces[0])
            filtered_fy = lp_filter.next(raw_forces[1])
            filtered_fz = lp_filter.next(raw_forces[2])
            predicted_forces = np.array([filtered_fx, filtered_fy, filtered_fz])
            input_data = [predicted_forces[0], predicted_forces[1], -1*predicted_forces[2]]

            screen.blit(bg, (0,0))
            # --- Draw the COMBINED vector of Bx and By ---
            end_combined_x = int(center_arrow[0] + input_data[0] * LENGTH)
            end_combined_y = int(center_arrow[1] + input_data[1] * LENGTH)
            pygame.draw.line(screen, BLACK, center_arrow, (end_combined_x, end_combined_y), 6)
            # Draw the arrowhead at the end of the line
            draw_arrowhead(screen, BLACK, center_arrow, (end_combined_x, end_combined_y), arrowhead_size)

            # --- Draw the Bz circle (Normal force) ---
            r = abs(input_data[2]) * SCALE
            r = max(5, int(r))
            pygame.draw.circle(screen, RED, center_arrow, r, 3)

            # text predictions in normal xyz frame
            # draw_text(screen, font, input_data[0], -1*input_data[1], -1*input_data[2])
            draw_text(screen, font, input_data[0], -1*input_data[1] + 0.04, -1*input_data[2] + 0.14) 

            pygame.display.update()

    except KeyboardInterrupt:
        print("\nVisualization stopped.")
        viz_sensor.close()
        pygame.quit()
        sys.exit()

