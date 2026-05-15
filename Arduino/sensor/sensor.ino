#include <Wire.h>
#include <MLX90393.h> 

MLX90393 mlx0;
MLX90393 mlx1;
MLX90393 mlx2;
MLX90393 mlx3;
MLX90393 mlx4;


MLX90393::txyz data0 = {0,0,0,0};
MLX90393::txyz data1 = {0,0,0,0};
MLX90393::txyz data2 = {0,0,0,0};
MLX90393::txyz data3 = {0,0,0,0};
MLX90393::txyz data4 = {0,0,0,0};


uint8_t mlx0_i2c = 0x0C; // these are the I2C addresses of the five chips that share one I2C bus
uint8_t mlx1_i2c = 0x0D;
uint8_t mlx2_i2c = 0x0E;
uint8_t mlx3_i2c = 0x0F; 
uint8_t mlx4_i2c = 0x18;


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(5);
  }

  Wire.begin();
  Wire.setClock(400000);
  delay(10);
  
  byte status = mlx0.begin(mlx0_i2c, -1, Wire);
  status = mlx1.begin(mlx1_i2c, -1, Wire);
  status = mlx2.begin(mlx2_i2c, -1, Wire);
  status = mlx3.begin(mlx3_i2c, -1, Wire);
  status = mlx4.begin(mlx4_i2c, -1, Wire);


  mlx0.startBurst(0xF);
  mlx1.startBurst(0xF);
  mlx2.startBurst(0xF);
  mlx3.startBurst(0xF);
  mlx4.startBurst(0xF);

  

}

void loop() {

  // Read and print sensor data
  mlx0.readBurstData(data0);
  mlx1.readBurstData(data1);
  mlx2.readBurstData(data2);
  mlx3.readBurstData(data3);
  mlx4.readBurstData(data4);

  //write string data over serial
  Serial.print("X0: ");
  Serial.print(data0.x);
  Serial.print("\t");
  Serial.print("Y0: ");
  Serial.print(data0.y);
  Serial.print("\t");
  Serial.print("Z0: ");
  Serial.print(data0.z);
  Serial.print("\t");
  
  Serial.print("X1: ");
  Serial.print(data1.x);
  Serial.print("\t");
  Serial.print("Y1: ");
  Serial.print(data1.y);
  Serial.print("\t");
  Serial.print("Z1: ");
  Serial.print(data1.z);
  Serial.print("\t");
  
  Serial.print("X2: ");
  Serial.print(data2.x);
  Serial.print("\t");
  Serial.print("Y2: ");
  Serial.print(data2.y);
  Serial.print("\t");
  Serial.print("Z2: ");
  Serial.print(data2.z);
  Serial.print("\t");
  
  Serial.print("X3: ");
  Serial.print(data3.x);
  Serial.print("\t");
  Serial.print("Y3: ");
  Serial.print(data3.y);
  Serial.print("\t");
  Serial.print("Z3: ");
  Serial.print(data3.z);
  Serial.print("\t");

  Serial.print("X4: ");
  Serial.print(data4.x);
  Serial.print("\t");
  Serial.print("Y4: ");
  Serial.print(data4.y);
  Serial.print("\t");
  Serial.print("Z4: ");
  Serial.print(data4.z);
  Serial.print("\t");


  // delay(100);

  Serial.println();
  delayMicroseconds(500); // Adjust sampling rate, 2000Hz
}