// Header file for MMA8452 functions

// Initialize the MMA8452 registers
// See the many application notes for more info on setting all of these registers:
// http://www.nxp.com/products/sensors/accelerometers/3-axis-accelerometers/2g-4g-8g-low-g-12-bit-digital-accelerometer:MMA8452Q?tab=Documentation_Tab
// Feel free to modify any values, these are settings that work well for me.


// Read a single byte from address and return it as a byte
byte readRegister(int I2CAddress, byte address)
{
  //Send a request
  //Start talking to the device at the specified address
  Wire.beginTransmission(I2CAddress);
  //Send a bit asking for requested register address
  Wire.write(address);
  //Complete Transmission
  Wire.endTransmission(false);
  //Read the register from the device
  //Request 1 Byte from the specified address
  Wire.requestFrom(I2CAddress, 1);
  //wait for response
  while(Wire.available() == 0);
  // Get the temp and read it into a variable
  byte data = Wire.read();
  return data;
}

// Writes a single byte (data) into address
void writeRegister(int I2CAddress, unsigned char address, unsigned char data)
{
  //Send a request
  //Start talking to the device at the specified address
  Wire.beginTransmission(I2CAddress);
  //Send a bit asking for requested register address
  Wire.write(address);
  Wire.write(data);
  //Complete Transmission
  Wire.endTransmission(false);
}


// Sets the MMA8452 to standby mode.
// It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(MMA8452_ADDRESS,0x2A);
  writeRegister(MMA8452_ADDRESS,0x2A, c & ~(0x01));
}

// Sets the MMA8452 to active mode.
// Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(MMA8452_ADDRESS,0x2A);
  writeRegister(MMA8452_ADDRESS,0x2A, c | 0x01);
}



void initMMA8452(byte fsr, byte dataRate)
{
  byte setSensitivity = FRAMread8(SENSITIVITYADDR);
  MMA8452Standby();  // Must be in standby to change registers
  // Set up the full scale range to 2, 4, or 8g.
  if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(MMA8452_ADDRESS, 0x0E, fsr >> 2);
  else
    writeRegister(MMA8452_ADDRESS,0x0E, 0);
  // Setup the 3 data rate bits, from 0 to 7
  writeRegister(MMA8452_ADDRESS, 0x2A, readRegister(MMA8452_ADDRESS,0x2A) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(MMA8452_ADDRESS,0x2A, readRegister(MMA8452_ADDRESS,0x2A) | (dataRate << 3));

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://www.nxp.com/assets/documents/data/en/application-notes/AN4072.pdf */
  writeRegister(MMA8452_ADDRESS, 0x21, 0x55);           // 1. single taps only on all axes
  writeRegister(MMA8452_ADDRESS, 0x23, setSensitivity);    // 2. x thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(MMA8452_ADDRESS, 0x24, setSensitivity);    // 2. y thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(MMA8452_ADDRESS, 0x25, setSensitivity);    // 2. z thresh at .5g (0x08), multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(MMA8452_ADDRESS, 0x26, 0xFF);           // 3. Max time limit as this is very dependent on data rate, see the app note
  writeRegister(MMA8452_ADDRESS, 0x27, 0x64);           // 4. 1000ms (at 100Hz odr, Normal, and LPF Disabled) between taps min, this also depends on the data rate
  writeRegister(MMA8452_ADDRESS, 0x28, 0xFF);           // 5. Mmax value between taps max
  writeRegister(MMA8452_ADDRESS, 0x2C, 0x02);           // Active high, push-pull interrupts
  writeRegister(MMA8452_ADDRESS, 0x2D, 0x08);           // Tap ints enabled
  writeRegister(MMA8452_ADDRESS, 0x2E, 0xB7);           // Taps on INT2 everyhting else to INT1
  MMA8452Active();  // Set to active to start reading
}
