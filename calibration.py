import time
import threading
import json
import os
from pathlib import Path
from datetime import datetime

import numpy as np
import zenoh
from loguru import logger
from tqdm import tqdm

from robot_arm_calib.multiproc.znh.znh_container import (
    parse_timed_msg,
)
from robot_arm_calib.multiproc.znh.znh_pb2 import Position
from .xarm_controller import (
    xArm7ControlConfig, 
    xArm7CalibController,
    ACTUAL_EE_POS,
)


class TactileCalibration(xArm7CalibController):
    def __init__(self, config: xArm7ControlConfig):
        super().__init__(config)

        self._lock = threading.Lock()
        self.ft_data_arr: np.ndarray | None = None
        self.tactile_data_arr: np.ndarray | None = None
        self.origin = ACTUAL_EE_POS
        
        # Data collection attributes
        self._collecting_data = False
        self._collected_data = []
        self._collection_thread = None

        z_conf = zenoh.Config()
        self.session = zenoh.open(z_conf)
        self.sub = self.session.declare_subscriber("ft_sensor/data", self.ft_callback)
        self.sub_tactile = self.session.declare_subscriber("tactile/single_sensor", self.tactile_callback)

    def ft_callback(self, msg):
        ft_data_arr, _ = parse_timed_msg(msg, Position)
        # logger.info(f"Received FT sensor data: {ft_data}")

        with self._lock:
            self.ft_data_arr = ft_data_arr

    def tactile_callback(self, msg):
        tactile_data_arr, _ = parse_timed_msg(msg, Position)
        # logger.info(f"Received tactile sensor data: {tactile_data_arr}")

        with self._lock:
            self.tactile_data_arr = tactile_data_arr

    def wait_for_required_data(self):
        logger.info("Waiting for FT sensor and tactile sensor data...")
        while True:
            with self._lock:
                if self.ft_data_arr is not None and self.tactile_data_arr is not None:
                # if self.ft_data_arr is not None:
                    break
            time.sleep(0.5)
        logger.info("Received FT and tactile sensor data, proceeding with calibration.")

    def move_down_until_contact(self, force_threshold: float = 0.3):
        for i in range(4):
            logger.info(f"Moving down by 1mm, iteration {i+1}/4")
            self.move_linear_quasi_static(np.array([0, 0, -0.001]), n_steps=5)

        for i in range(20):
            input("Press Enter to move down by 0.2mm and check for contact...")
            logger.info(f"Moving down by 0.2mm, iteration {i+1}/20")
            self.move_linear_quasi_static(np.array([0, 0, -0.0002]), n_steps=2)

            with self._lock:
                ft_data_arr = self.ft_data_arr

            if ft_data_arr[2] < -force_threshold:
                logger.info(f"Contact detected with force {ft_data_arr[2]}N")
                break
    
    def move_down(self, depth_mm: float):
        """Move from above-surface start position to the target depth below contact.

        How ``num_iterations`` controls depth:
        The ``+4`` term is the fixed offset used to reach the contact level from
        the standard start height. Therefore:
        - At ``depth_mm = 0``, ``num_iterations = 4`` and the fine stage reaches
            the calibrated contact depth (0 mm).
        - For every additional 0.2 mm in ``depth_mm``, one more fine iteration is
            added, increasing penetration depth by 0.2 mm.
        """
        # Calculate the number of iterations based on the offset: (depth / 0.2) + 3
        num_iterations = int(round(depth_mm / 0.2)) + 4

        for i in range(4):
            logger.info(f"Moving down by 1mm, iteration {i+1}/4")
            self.move_linear_quasi_static(np.array([0, 0, -0.001]), n_steps=5)

        for i in range(num_iterations):
            logger.info(f"Moving down (target depth {depth_mm}mm), iteration {i+1}/{num_iterations}")
            self.move_linear_quasi_static(np.array([0, 0, -0.0002]), n_steps=2)

    def _collection_thread_func(self):
        """Background thread function for collecting sensor data"""
        start_time = time.time()
        
        while self._collecting_data:
            with self._lock:
                ft_data = self.ft_data_arr.copy() if self.ft_data_arr is not None else None
                tactile_data = self.tactile_data_arr.copy() if self.tactile_data_arr is not None else None
            
            if ft_data is not None and tactile_data is not None:
                sample = {
                    'timestamp': time.time(),
                    'elapsed_time': time.time() - start_time,
                    'ft_data': ft_data,
                    'tactile_data': tactile_data,
                }
                self._collected_data.append(sample)
            
            time.sleep(0.01)  # Sample at ~100Hz

    def start_collecting_data(self):
        """Start collecting sensor data in a background thread"""
        self._collecting_data = True
        self._collected_data = []
        self._collection_thread = threading.Thread(target=self._collection_thread_func, daemon=True)
        self._collection_thread.start()
        logger.info("Started data collection thread")

    def stop_collecting_data(self):
        """Stop collecting sensor data and return the collected data"""
        self._collecting_data = False
        if self._collection_thread is not None:
            self._collection_thread.join()
        logger.info(f"Stopped data collection. Collected {len(self._collected_data)} samples")
        return self._collected_data

    def save_sensor_data_to_json(self, depth: float, x: float, y: float, direction: str, collected_data: list):
        script_dir = Path(__file__).parent 
        data_path = script_dir / "data"/ "shear" 
        filename = f"shear_calib_{depth}mm_X{x*1000:.1f}_Y{y*1000:.1f}_{direction}.json"
        filepath = data_path / filename
        
        # Convert numpy arrays to lists for JSON serialization
        data_to_save = []
        for sample in collected_data:
            sample_dict = {
                'timestamp': sample['timestamp'],
                'elapsed_time': sample['elapsed_time'],
                'ft_data': sample['ft_data'].tolist() if isinstance(sample['ft_data'], np.ndarray) else sample['ft_data'],
                'tactile_data': sample['tactile_data'].tolist() if isinstance(sample['tactile_data'], np.ndarray) else sample['tactile_data'],
            }
            data_to_save.append(sample_dict)
        
        # Save to JSON file
        with open(filepath, 'w') as f:
            json.dump(data_to_save, f, indent=2)
        
        logger.info(f"Saved {len(collected_data)} samples to {filepath}")
        return str(filepath)

# Direction mapping for shear calibration
distance = 0.002  # move in this direction for 2mm
SHEAR_DIRECTIONS = {
    'right': np.array([0, distance, 0]),          # positive y-direction
    'bottom': np.array([distance, 0, 0]),         # positive x-direction
    'left': np.array([0, -distance, 0]),          # negative y-direction
    'top': np.array([-distance, 0, 0]),           # negative x-direction
    'top_left': np.array([-distance, -distance, 0]), # negative x, negative y
    'top_right': np.array([-distance, distance, 0]), # negative x, positive y
    'bottom_left': np.array([distance, -distance, 0]), # positive x, negative y
    'bottom_right': np.array([distance, distance, 0]), # positive x, positive y
}

def calib(calib_controller, depth, x, y, direction: str = 'right'):
    """Calibrate at a specific start point and direction by moving down to the target depth, 
    then shearing in the specified direction while collecting sensor data.
    """
    calib_controller._move_to_init_ee_pose() 
    calib_controller.wait_for_required_data()
    time.sleep(0.5)

    # Move to the start position above the object
    start_pos = ACTUAL_EE_POS + np.array([x, y, 0.005])  
    calib_controller.move_to_ee_pos_quasi_static(start_pos, n_steps=10)
    time.sleep(0.5)

    # Move down until contact is detected
    calib_controller.move_down(depth_mm=depth)
    time.sleep(1)

    # Start collecting data before moving to the edge
    movement = SHEAR_DIRECTIONS[direction]
    logger.info(f"Moving to {direction} edge...")
    calib_controller.start_collecting_data()
    calib_controller.move_linear_quasi_static(movement, n_steps=1)
    collected_data = calib_controller.stop_collecting_data()
   
    json_filepath = calib_controller.save_sensor_data_to_json(depth=depth, x=x, y=y, direction=direction, collected_data=collected_data)
    logger.info(f"Data saved to: {json_filepath}")
    time.sleep(0.5)


if __name__ == "__main__":
    import tyro
    np.set_printoptions(precision=4, suppress=True)

    # Initialize the robot ONCE before the loops start
    robot_config = tyro.cli(xArm7ControlConfig)
    robot_controller = TactileCalibration(robot_config)

    try:
        depth = 0.2  # [0.2mm, 0.6mm, 1.0mm, 1.4mm] collect at these 4 depths
        start_points_mm = [
            (0, 0),
            (0, 5),
            (5, 0),
            (0, -5),
            (-5, 0),
            (-5, 5),
            (5, -5),
            (5, 5),
            (-5, -5),
        ]  # total 9 start points

        total_runs = len(start_points_mm) * len(SHEAR_DIRECTIONS)
        with tqdm(total=total_runs, desc="Calibration progress", unit="calib") as pbar:
            for sx_mm, sy_mm in start_points_mm:
                start_x = sx_mm / 1000.0
                start_y = sy_mm / 1000.0
                logger.info(f"Starting start point X={sx_mm}mm, Y={sy_mm}mm")

                for direction in SHEAR_DIRECTIONS.keys():
                    logger.info(f"Starting calibration for {direction} direction")
                    calib(robot_controller, depth=depth, x=start_x, y=start_y, direction=direction)
                    pbar.update(1)
                    logger.info(f"Finished calibration for {direction} direction")

    except KeyboardInterrupt:
        logger.info("Ctrl C detected. Stopping the robot.")