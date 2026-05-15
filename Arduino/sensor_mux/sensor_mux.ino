#include <Wire.h>
#include <MLX90393.h> 

// --- Mux Configuration ---
#define MUX_ADDR 0x70

// --- Device A (Mux Channel 0) Instances ---
MLX90393 mlx0A;
MLX90393 mlx1A;
MLX90393 mlx2A;
MLX90393 mlx3A;
MLX90393 mlx4A;
// --- Device B (Mux Channel 1) Instances ---
MLX90393 mlx0B;
MLX90393 mlx1B;
MLX90393 mlx2B;
MLX90393 mlx3B;
MLX90393 mlx4B;
// --- Device C (Mux Channel 2) Instances ---
MLX90393 mlx0C;
MLX90393 mlx1C;
MLX90393 mlx2C;
MLX90393 mlx3C;
MLX90393 mlx4C;
// --- Device D (Mux Channel 3) Instances ---
MLX90393 mlx0D;
MLX90393 mlx1D;
MLX90393 mlx2D;
MLX90393 mlx3D;
MLX90393 mlx4D;
// --- Device E (Mux Channel 4) Instances ---
// MLX90393 mlx0E;
// MLX90393 mlx1E;
// MLX90393 mlx2E;
// MLX90393 mlx3E;
// MLX90393 mlx4E;
// --- Device A Data Structures ---
MLX90393::txyz data0A = {0,0,0,0};
MLX90393::txyz data1A = {0,0,0,0};
MLX90393::txyz data2A = {0,0,0,0};
MLX90393::txyz data3A = {0,0,0,0};
MLX90393::txyz data4A = {0,0,0,0};
// --- Device B Data Structures ---
MLX90393::txyz data0B = {0,0,0,0};
MLX90393::txyz data1B = {0,0,0,0};
MLX90393::txyz data2B = {0,0,0,0};
MLX90393::txyz data3B = {0,0,0,0};
MLX90393::txyz data4B = {0,0,0,0};
// --- Device C Data Structures ---
MLX90393::txyz data0C = {0,0,0,0};
MLX90393::txyz data1C = {0,0,0,0};
MLX90393::txyz data2C = {0,0,0,0};
MLX90393::txyz data3C = {0,0,0,0};
MLX90393::txyz data4C = {0,0,0,0};
// --- Device D Data Structures ---
MLX90393::txyz data0D = {0,0,0,0};
MLX90393::txyz data1D = {0,0,0,0};
MLX90393::txyz data2D = {0,0,0,0};
MLX90393::txyz data3D = {0,0,0,0};
MLX90393::txyz data4D = {0,0,0,0};
// --- Device E Data Structures ---
// MLX90393::txyz data0E = {0,0,0,0};
// MLX90393::txyz data1E = {0,0,0,0};
// MLX90393::txyz data2E = {0,0,0,0};
// MLX90393::txyz data3E = {0,0,0,0};
// MLX90393::txyz data4E = {0,0,0,0};

// --- Shared Sensor I2C Addresses ---
uint8_t mlx0_i2c = 0x0C;
uint8_t mlx1_i2c = 0x0D; 
uint8_t mlx2_i2c = 0x0E; 
uint8_t mlx3_i2c = 0x0F; 
uint8_t mlx4_i2c = 0x18;


// Function to select a TCA9548A Mux channel (0-7)
void tcaSelect(uint8_t i) {
  if (i > 7) return; 
  
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << i); 
  Wire.endTransmission();
}

void setup() {
  // *** CHANGE 1: INCREASE BAUD RATE ***
  Serial.begin(1000000); // Changed from 115200 to 500000 for speed 
  while (!Serial) {
    delay(5);
  }

  Wire.begin();
  Wire.setClock(400000);
  delay(10);
  
  // --- Initialize Device A (Mux Channel 0) ---
  tcaSelect(0);
  byte status = mlx0A.begin(mlx0_i2c, -1, Wire);
  status = mlx1A.begin(mlx1_i2c, -1, Wire);
  status = mlx2A.begin(mlx2_i2c, -1, Wire);
  status = mlx3A.begin(mlx3_i2c, -1, Wire);
  status = mlx4A.begin(mlx4_i2c, -1, Wire);

  // --- Initialize Device B (Mux Channel 1) ---
  tcaSelect(1); 
  status = mlx0B.begin(mlx0_i2c, -1, Wire);
  status = mlx1B.begin(mlx1_i2c, -1, Wire);
  status = mlx2B.begin(mlx2_i2c, -1, Wire);
  status = mlx3B.begin(mlx3_i2c, -1, Wire);
  status = mlx4B.begin(mlx4_i2c, -1, Wire);

  // --- Initialize Device C (Mux Channel 2) ---
  tcaSelect(2); 
  status = mlx0C.begin(mlx0_i2c, -1, Wire);
  status = mlx1C.begin(mlx1_i2c, -1, Wire);
  status = mlx2C.begin(mlx2_i2c, -1, Wire);
  status = mlx3C.begin(mlx3_i2c, -1, Wire);
  status = mlx4C.begin(mlx4_i2c, -1, Wire);

  // --- Initialize Device D (Mux Channel 3) ---
  tcaSelect(3); 
  status = mlx0D.begin(mlx0_i2c, -1, Wire);
  status = mlx1D.begin(mlx1_i2c, -1, Wire);
  status = mlx2D.begin(mlx2_i2c, -1, Wire);
  status = mlx3D.begin(mlx3_i2c, -1, Wire);
  status = mlx4D.begin(mlx4_i2c, -1, Wire);

  // --- Initialize Device E (Mux Channel 4) ---
  // tcaSelect(4); 
  // status = mlx0E.begin(mlx0_i2c, -1, Wire);
  // status = mlx1E.begin(mlx1_i2c, -1, Wire);
  // status = mlx2E.begin(mlx2_i2c, -1, Wire);
  // status = mlx3E.begin(mlx3_i2c, -1, Wire);
  // status = mlx4E.begin(mlx4_i2c, -1, Wire);

  // --- Start Burst Mode for ALL 25 sensors ---
  // Device A
  tcaSelect(0);
  mlx0A.startBurst(0xF);
  mlx1A.startBurst(0xF);
  mlx2A.startBurst(0xF);
  mlx3A.startBurst(0xF);
  mlx4A.startBurst(0xF);
  
  // Device B
  tcaSelect(1);
  mlx0B.startBurst(0xF);
  mlx1B.startBurst(0xF);
  mlx2B.startBurst(0xF);
  mlx3B.startBurst(0xF);
  mlx4B.startBurst(0xF);

  // Device C
  tcaSelect(2);
  mlx0C.startBurst(0xF);
  mlx1C.startBurst(0xF);
  mlx2C.startBurst(0xF);
  mlx3C.startBurst(0xF);
  mlx4C.startBurst(0xF);

  // Device D
  tcaSelect(3);
  mlx0D.startBurst(0xF);
  mlx1D.startBurst(0xF);
  mlx2D.startBurst(0xF);
  mlx3D.startBurst(0xF);
  mlx4D.startBurst(0xF);

  // Device E
  // tcaSelect(4);
  // mlx0E.startBurst(0xF);
  // mlx1E.startBurst(0xF);
  // mlx2E.startBurst(0xF);
  // mlx3E.startBurst(0xF);
  // mlx4E.startBurst(0xF);
}

void printData(const MLX90393::txyz& data) {
  // Only print the raw X, Y, Z values separated by a comma.
  Serial.print(data.x);
  Serial.print(",");
  Serial.print(data.y);
  Serial.print(",");
  Serial.print(data.z);
  Serial.print(","); // Trailing comma
}

void loop() {
  
  // --- Read Device A (Channel 0) ---
  tcaSelect(0);
  mlx0A.readBurstData(data0A);
  mlx1A.readBurstData(data1A);
  mlx2A.readBurstData(data2A);
  mlx3A.readBurstData(data3A);
  mlx4A.readBurstData(data4A);

  // --- Read Device B (Channel 1) ---
  tcaSelect(1);
  mlx0B.readBurstData(data0B);
  mlx1B.readBurstData(data1B);
  mlx2B.readBurstData(data2B);
  mlx3B.readBurstData(data3B);
  mlx4B.readBurstData(data4B);

  // --- Read Device C (Channel 2) ---
  tcaSelect(2);
  mlx0C.readBurstData(data0C);
  mlx1C.readBurstData(data1C);
  mlx2C.readBurstData(data2C);
  mlx3C.readBurstData(data3C);
  mlx4C.readBurstData(data4C);

  // --- Read Device D (Channel 3) ---
  tcaSelect(3);
  mlx0D.readBurstData(data0D);
  mlx1D.readBurstData(data1D);
  mlx2D.readBurstData(data2D);
  mlx3D.readBurstData(data3D);
  mlx4D.readBurstData(data4D);

  // --- Read Device E (Channel 4) ---
  // tcaSelect(4);
  // mlx0E.readBurstData(data0E);
  // mlx1E.readBurstData(data1E);
  // mlx2E.readBurstData(data2E);
  // mlx3E.readBurstData(data3E);
  // mlx4E.readBurstData(data4E);

  // --- Print All Sensor Data on a single line (75 comma-separated values) ---
  
  // Device A data
  printData(data0A);
  printData(data1A);
  printData(data2A);
  printData(data3A);
  printData(data4A);
  
  // Device B data
  printData(data0B);
  printData(data1B);
  printData(data2B);
  printData(data3B);
  printData(data4B);

  // Device C data
  printData(data0C);
  printData(data1C);
  printData(data2C);
  printData(data3C);
  printData(data4C);

  // Device D data
  printData(data0D);
  printData(data1D);
  printData(data2D);
  printData(data3D);
  printData(data4D);

  // Device E data
  // printData(data0E);
  // printData(data1E);
  // printData(data2E);
  // printData(data3E);
  // printData(data4E);
  
  Serial.println(); // Sends the final newline
  delayMicroseconds(500); 
}