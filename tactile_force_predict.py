import serial
import time
import threading
from dataclasses import dataclass

import numpy as np
import zenoh
from loguru import logger
import torch
import torch.nn as nn
import joblib
from pathlib import Path

from robot_arm_calib.multiproc.znh.znh_container import (
    create_timed_msg,
)
from robot_arm_calib.multiproc.znh.znh_pb2 import Position




# --- Model Definition ---
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


@dataclass
class TactileData:
    x: float
    y: float
    z: float

    def as_array(self):
        return np.array([self.x, self.y, self.z])
    

@dataclass
class TactileReaderConfig:
    port: str = '/dev/ttyACM0'
    baudrate: int = 500000
    num_mlx: int = 5
    pub_freq_hz: float = 50.0

    tactile_topic: str = "tactile/single_sensor"
    
    CURRENT_DIR = Path(__file__).parent
    model_path = CURRENT_DIR / "model" / "3d" / "model.pth"
    scaler_path = CURRENT_DIR / "model" / "3d" / "scaler.pkl"


class TactileReader:
    def __init__(
        self, config: TactileReaderConfig
    ):
        self.config = config

        # --- 1. Load the unified 3D model and scaler ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        try:
            self.scaler = joblib.load(self.config.scaler_path)
            self.model = ReSkinMLP().to(self.device)
            self.model.load_state_dict(torch.load(self.config.model_path, weights_only=True))
            self.model.eval()

            logger.info(f"Successfully loaded unified 3D model and scaler on {self.device}.")
        except Exception as e:
            logger.error(f"Failed to load model or scaler. Check paths! Error: {e}")
            raise

        # --- Serial Connection ---
        try:
            self.ser = serial.Serial(self.config.port, self.config.baudrate)
            print(f"Connected to {self.config.port} at {self.config.baudrate} baud.")
        except serial.SerialException as e:
            print(f"ERROR: Could not open serial port {self.config.port}. Please check the connection.")
            raise serial.serialutil.SerialException(f"Serial port error: {e}")
        
        time.sleep(2)  # Wait for the serial connection to establish
        self.ser.flushInput()

        self.cur_data: list[TactileData] = []
        self._lock = threading.Lock()
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        time.sleep(1)  # Wait a moment to ensure the read thread has started
        z_conf = zenoh.Config()
        self.session = zenoh.open(z_conf)
        self.pub = self.session.declare_publisher(self.config.tactile_topic)
        logger.info(f"Published to topic: {self.config.tactile_topic}")
        self.last_pub_time = None

    def _read_loop(self):
        while True:
            line_str = self._read_line()
            if line_str is None:
                continue
            values_str = [num.strip() for num in line_str.split(',') if num.strip()]

            try:
                values_float = [float(num) for num in values_str]
            except ValueError as e:
                print(f"Error parsing line: {line_str}. Error: {e}")
                continue

            if len(values_float) != self.config.num_mlx * 3:
                print(f"Unexpected number of values: {len(values_float)}. Line: {line_str}")
                continue

            with self._lock:
                self.cur_data = []
                for i in range(self.config.num_mlx):
                    x_i = values_float[i * 3]
                    y_i = values_float[i * 3 + 1]
                    z_i = values_float[i * 3 + 2]
                    tac_data = TactileData(x_i, y_i, z_i)
                    self.cur_data.append(tac_data)

    def _read_line(self):
        try:
            line_bytes = self.ser.readline()
            if not line_bytes:
                return None
            line_str = line_bytes.decode('utf-8').strip()
            return line_str
        except Exception as e:
            print(f"Error reading from serial port: {e}")
            return None
        
    def wait_until_next_pub(self):
        if self.last_pub_time is None:
            self.last_pub_time = time.perf_counter()
        else:
            dt = time.perf_counter() - self.last_pub_time
            target_dt = 1.0 / self.config.pub_freq_hz
            if dt < target_dt:
                time.sleep(target_dt - dt)
            else:
                logger.warning(
                    f"Publish dt: {target_dt} can not be reached, actual dt: {dt}"
                )
            self.last_pub_time = time.perf_counter()

    def publish_tactile_data(self, verbose=False):
        with self._lock:
            tac_data_arr = np.array([tac_data.as_array() for tac_data in self.cur_data])

        if len(tac_data_arr) == 0:
            logger.warning("No tactile data to publish.")
            return

        # --- 2. Live Inference ---
        # Flatten the 5x3 array into 15 elements, and reshape for the scaler: shape (1, 15)
        flat_data = tac_data_arr.flatten().reshape(1, -1)
        
        # Scale and convert to tensor
        scaled_data = self.scaler.transform(flat_data)
        tensor_data = torch.tensor(scaled_data, dtype=torch.float32).to(self.device)

        # Predict 3D force (Fx, Fy, Fz)
        with torch.no_grad():
            predicted_force = self.model(tensor_data).cpu().numpy()[0]

        predicted_fx, predicted_fy, predicted_fz = predicted_force
        
        if verbose:
            logger.debug(f"Tactile data array:\n{tac_data_arr}")
            logger.info(
                f"==> Fx: {predicted_fx:.4f} N, "
                f"Fy: {-1 * predicted_fy:.4f} N, "
                f"Fz: {predicted_fz:.4f} N"
            )

        # Note: Right now we only publish the raw tactile array to the Position proto.
        # If you want to broadcast predicted forces over Zenoh, pack the predicted
        # 3D force vector into a new protobuf message or modify the existing one here.
        tac_msg = create_timed_msg(tac_data_arr, time.time(), Position)
        self.pub.put(tac_msg.SerializeToString())


def main():
    import tyro
    np.set_printoptions(precision=0, suppress=True)

    config = tyro.cli(TactileReaderConfig)
    reader = TactileReader(config=config)
    while True:
        reader.wait_until_next_pub()
        reader.publish_tactile_data(verbose=True)


if __name__ == "__main__":
    main()