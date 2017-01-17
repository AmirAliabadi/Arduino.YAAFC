///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 10000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 7;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void calibrate()
{
  // initialize device
  accelgyro.initialize();

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()) {
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again

  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  delay(2000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  if (state == 0) {
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state == 1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state == 2) {
    meansensors();

    process = &calibrate_done;

  }
}


void calibrate_done()
{
  Serial.println("\nFINISHED!");
  Serial.print("\nSensor readings with offsets:\t");
  Serial.print(mean_ax);
  Serial.print("\t");
  Serial.print(mean_ay);
  Serial.print("\t");
  Serial.print(mean_az);
  Serial.print("\t");
  Serial.print(mean_gx);
  Serial.print("\t");
  Serial.print(mean_gy);
  Serial.print("\t");
  Serial.println(mean_gz);

  Serial.print("Your offsets:\t");
  Serial.print(ax_offset);
  Serial.print("\t");
  Serial.print(ay_offset);
  Serial.print("\t");
  Serial.print(az_offset);
  Serial.print("\t");
  Serial.print(gx_offset);
  Serial.print("\t");
  Serial.print(gy_offset);
  Serial.print("\t");
  Serial.println(gz_offset);

  Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
  Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
  Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");

  while (Serial.available() && Serial.read()); // empty buffer
  Serial.println(F("Press any key to store in Egit EPROM\n"));
  while (!Serial.available()) {
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again

  store_in_eeprom();

  while (Serial.available() && Serial.read()); // empty buffer
  Serial.println(F("Press any key to test mpu"));
  while (!Serial.available()) {
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again

  process = &run_mpu_loop;
}



void test_raw_motion6_results()
{
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);

  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);

  delay(200);
}

