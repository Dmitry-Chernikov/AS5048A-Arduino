#include "AS5048A.h"
//#define AS5048A_DEBUG
const int ERROR_FLAG_BIT                             = 14;     // Error bit index
const int AS5048A_NOP                                = 0x0000; // Dummy operation, no information.
const int AS5048A_CLEAR_ERROR_FLAG                   = 0x0001; // Error register. All errors are cleared by access.
const int AS5048A_PROGRAMMING_CONTROL                = 0x0003; // Programming control register. Programming must be enabled before burning memory. A check is mandatory before programming. See the programming procedure.
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH        = 0x0016; // Zero position 8 bits high
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW         = 0x0017; // Zero position 6 bits low
const int AS5048A_DIAG_AGC                           = 0x3FFD; // (0-7) Automatic gain control value. 0 decimal represents a high magnetic field, 255 decimal represents a low magnetic field. (8-13) Diagnostic flags
const int AS5048A_MAGNITUDE                          = 0x3FFE; // CORDIC output power value
const int AS5048A_ANGLE                              = 0x3FFF; // Angular output value, including zero position correction Resolution_ADC 14-bit resolution (0.0219°/LSB)
const int AS5048A_ANGLE_HALF                         = AS5048A_ANGLE / 2; // Half range
/**
 * Constructor
 * Initialization of the AS5048A library
 */
AS5048A::AS5048A(byte argCs) {
  _cs = argCs;
  _errorFlag = false;
  _position = 0;
}
/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::init() {
  /**
   * 1MHz clock (AMS should be able to accept up to 10MHz)
   * mySetting (speedMaximum, dataOrder, dataMode)
   * speedMaximum - maximum communication speed. For SPI chip rated for frequencies up to 20 MHz, use 20000000.
   * dataOrder - order of data output in/from the SPI bus, can be LSBFIRST (least significant bit first) or MSBFIRST (most significant bit first)
   * dataMode - sets the operating mode of the SPI bus by specifying the signal level and phase
   * SPI_MODE0 (Signal Level (CPOL)-0, Phase (CPHA)-0)
   * SPI_MODE1 (Signal Level (CPOL)-0, Phase (CPHA)-1)
   * SPI_MODE2 (Signal Level (CPOL)-1, Phase (CPHA)-0)
   * SPI_MODE3 (Signal Level (CPOL)-1, Phase (CPHA)-1)
   * f(sample) = Min-10.2, Typ-11.25, Max-12.4. (kHz)
   * Supported I2C modes for AS5048B:
   * • Random / sequential reading
   * • Byte / Page write
   * • Standard: 0 to 100 kHz, clock frequency (slave mode)
   * • Fast mode: clock frequency from 0 to 400 kHz (slave mode)
   * • High speed: clock frequency from 0 to 3.4 MHz (slave mode)
   */
  settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
  // Initialize the Slave Select pin, if LOW the slave interacts with the master, if HIGH the slave ignores signals from the master
  pinMode(_cs, OUTPUT);
  // SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
  SPI.begin();
}
/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void AS5048A::close() {
  SPI.end();
}
/**
 * Utility function used to calculate even parity of word
 * Calculates the parity bit of a 14-bit address and writes it to the 15th bit of the returned 16-bit word
 */
byte AS5048A::spiCalcEvenParity(word valueADC) {
  byte cnt = 0;
  byte i;
  for (i = 0; i < 15; i++) {
    if (valueADC & 0x1) {
      cnt++;
    }
    valueADC >>= 1;
  }
  return cnt & 0x1;
  // byte operandCompare =  bitRead(valueADC,0);
  // byte i = 1;
  // do{
  //   operandCompare ^= bitRead(valueADC,i);
  // } while ((i++) < 14);
  // return operandCompare & 0x1;
}
/**
 * Get the rotation of the sensor relative to the zero _position.
 * @return {int} between -2^13 and 2^13
 */
int AS5048A::getRotation() {
  word data;
  int rotation;
  data = getRawRotation();
  rotation = (int)data - (int)_position;
  if (rotation > 8191)
    rotation = -((0x3FFF) - rotation); // more than -180
  // if(rotation < -0x1FFF) rotation = rotation+0x3FFF;
  return rotation;
}
/**
 * Returns the raw angle directly from the sensor
 * Returns the angular binary 14-bit angle value (DEC 16383)
 * Angular output value, including zero position correction.
 */
word AS5048A::getRawRotation(byte sizeValues, bool median, bool circular) {
  return readAveragingUniversal(static_cast<word>(AS5048A_ANGLE), sizeValues, median, circular);
}
/**
 * Returns the physical quantity in angular degrees obtained from a 14-bit binary ADC number
 */
float AS5048A::rotationRawToAngle(word discreteCode) {
  return discreteCode * (360.0 / float(AS5048A_ANGLE));
}
/**
 * Returns the physical quantity in angular radians obtained from a 14-bit binary ADC number
 */
float AS5048A::rotationRawToRadian(word discreteCode) {
  return static_cast<float>(discreteCode) * ((2 * PI) / float(AS5048A_ANGLE));
}
/**
 * Normalization of the angle
 */
double AS5048A::normalizeAngle(double radians) {
/*    // Нормализуем угол в диапазоне от 0 до 2π
    while (radians < 0) {
        radians += 2 * M_PI; // Приводим к положительному значению
    }
    while (radians >= 2 * M_PI) {
        radians -= 2 * M_PI; // Приводим к диапазону 0 - 2π
    }
    return radians;
*/
  radians = fmod(radians, 2 * M_PI); // Применяем модуль
  if (radians < 0) {
    radians += 2 * M_PI; // Приводим к положительному значению
  }
  return radians;
}
/**
 * Function RadianToRotationRaw: This function takes an angle in radians and returns the value
 * corresponding to this angle in the format used by the ADC (or in discrete code).
 */
word AS5048A::radianToRotationRaw(double radians) {
  return static_cast<word>(radians * (AS5048A_ANGLE / (2 * PI)));
}
/**
 * Returns the incremental and decremental rotation angle in the rotationAngle variable, pointers to the variables are passed to the procedure
 */
void AS5048A::absoluteAngleRotation(float *rotationAngle, float *angleCurrent, float *anglePrevious) {
  if (*angleCurrent != *anglePrevious) {
    // A full circle has been made increasing from 360 to 1
    if ((*angleCurrent < 90) && (*anglePrevious > 270) /*|| (*angleCurrent < 1.5707963267948966192313216916398) && (*anglePrevious > 4.7123889803846898576939650749193) */) {
      *rotationAngle += abs(360 - abs(*angleCurrent - *anglePrevious));
      _reverse = true;
    }
    // A full circle has been made decreasing from 1 to 360
    if ((*anglePrevious < 90) && (*angleCurrent > 270) /*|| (*anglePrevious < 1.5707963267948966192313216916398) && (*angleCurrent > 4.7123889803846898576939650749193) */) {
      *rotationAngle -= abs(360 - abs(*angleCurrent - *anglePrevious));
      _reverse = false;
    }
    // Moving around in increasing direction
    if (*angleCurrent > *anglePrevious && ((*angleCurrent < 90) && (*anglePrevious > 270)) != true && ((*anglePrevious < 90) && (*angleCurrent > 270)) != true /*|| *angleCurrent > *anglePrevious && ((*angleCurrent < 1.5707963267948966192313216916398) && (*anglePrevious > 4.7123889803846898576939650749193)) != true && ((*anglePrevious < 1.5707963267948966192313216916398) && (*angleCurrent > 4.7123889803846898576939650749193)) != true*/) {
      *rotationAngle += abs(*angleCurrent - *anglePrevious);
      _reverse = true;
    }
    // Moving around in decreasing direction
    if (*anglePrevious > *angleCurrent && ((*angleCurrent < 90) && (*anglePrevious > 270)) != true && ((*anglePrevious < 90) && (*angleCurrent > 270)) != true /*|| *anglePrevious > *angleCurrent && ((*angleCurrent < 1.5707963267948966192313216916398) && (*anglePrevious > 4.7123889803846898576939650749193)) != true && ((*anglePrevious < 1.5707963267948966192313216916398) && (*angleCurrent > 4.7123889803846898576939650749193)) != true*/) {
      *rotationAngle -= abs(*anglePrevious - *angleCurrent);
      _reverse = false;
    }
  }
  *anglePrevious = *angleCurrent;
}
float AS5048A::absoluteAngleRotation(float *rotationAngle, float angleCurrent, float *anglePrevious) {
  if (angleCurrent != *anglePrevious) {
    // A full circle has been made increasing from 360 to 1
    if ((angleCurrent < 90) && (*anglePrevious > 270)) {
      *rotationAngle += abs(360 - abs(angleCurrent - *anglePrevious));
    }
    // A full circle has been made decreasing from 1 to 360
    if ((*anglePrevious < 90) && (angleCurrent > 270)) {
      *rotationAngle -= abs(360 - abs(angleCurrent - *anglePrevious));
    }
    // Moving around in increasing direction
    if (angleCurrent > *anglePrevious && ((angleCurrent < 90) && (*anglePrevious > 270)) != true && ((*anglePrevious < 90) && (angleCurrent > 270)) != true) {
      *rotationAngle += abs(angleCurrent - *anglePrevious);
    }
    // Moving around in decreasing direction
    if (*anglePrevious > angleCurrent && ((angleCurrent < 90) && (*anglePrevious > 270)) != true && ((*anglePrevious < 90) && (angleCurrent > 270)) != true) {
      *rotationAngle -= abs(*anglePrevious - angleCurrent);
    }
  }
  *anglePrevious = angleCurrent;
  return *rotationAngle;
}
/**
 * Returns angular minutes
 */
float AS5048A::getAngularMinutes(float angleAbsolute) {
  return (angleAbsolute - int(angleAbsolute)) * 60;
}
/**
 * Returns angular seconds
 */
float AS5048A::getAngularSeconds(float angleAbsolute) {
  return (getAngularMinutes(angleAbsolute) - static_cast<int>(getAngularMinutes(angleAbsolute))) * 60;
}
/**
 * Returns the displacement of a straight-toothed rack in mm
 * wheelRotationAngle - Wheel rotation angle
 * normalModule - Normal module
 * numberGearTeeth - Number of teeth on the wheel or number of worm threads
 * (PI * normalModule) - End face pitch
 * AngleTiltTooth Tooth tilt angle, default arguments 20
 */
float AS5048A::linearDisplacementRack(float wheelRotationAngle, float normalModule, float numberGearTeeth) {
  return wheelRotationAngle * (((PI * normalModule) * numberGearTeeth) / 360);
}
float AS5048A::linearDisplacementRack(float wheelRotationAngle, float normalModule, float numberGearTeeth, float angleTiltTooth = 20) {
  return wheelRotationAngle * ((((PI * normalModule) / cos(radians(angleTiltTooth))) * numberGearTeeth) / 360);
}
/**
 * Returns the displacement of the screw transmission in mm
 * stepGroove - screw thread pitch
 * screwRotationAngle - screw rotation angle
 */
float AS5048A::linearMotionHelicalGear(float screwRotationAngle, float stepGroove) {
  return (screwRotationAngle * (stepGroove / 360));
}
/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 * Returns the value of the sensor's diagnostic register
 * sized 16 bits, of which 13 are significant (example 1101100110101)
 */
word AS5048A::getState() {
  return read(static_cast<word>(AS5048A_DIAG_AGC)) & ~0xC000;
}
/**
 * Print the diagnostic register of the sensor
 * Output the value of the sensor's diagnostic register to the Serial port
 */
void AS5048A::printState() {
  word data;
  data = getState();
  if (AS5048A::error()) {
    Serial.println("Error bit was set! (function printState register Diagnostics + Automatic Gain Control (AGC) )");
  }
  Serial.println(" ");
  Serial.println("\tAutomatic gain control value of the magnetic field");
  Serial.println("\t255 - represents a low magnetic field");
  Serial.println("\t0 - represents a high magnetic field");
  Serial.print("\tBIN Value: ");
  Serial.println(lowByte(data), BIN);
  Serial.print("\tDEC Value: ");
  Serial.println(lowByte(data), DEC);
  /** Diagnostic functions of AS5048
   * AS5048 provides diagnostic functions for the IC as well as for the input magnetic field. The following diagnostic flags are available: see Figure 22 register address x3FFD (AS5048A) or
   * address 31 address 251 deci (AS5048B) • OCF (Offset compensation completed), logical high indicates that the offset compensation algorithm has finished. After activation, the flag remains
   * logically high. • COF (CORDIC Overflow), logical high indicates an out-of-range error in the CORDIC part. When this bit is set, angle and magnitude data are invalid. Absolute
   * output retains the last valid angle value. • COMP low indicates a high magnetic field. It is recommended to additionally monitor the magnitude value. • COMP high indicates
   * a weak magnetic field. It is recommended to monitor the magnitude value.
   */
  Serial.print("Diagnostic flags");
  Serial.print(" OCF-");
  Serial.print(bitRead(data, 8), DEC);
  Serial.print(" COF-");
  Serial.print(bitRead(data, 9), DEC);
  Serial.print(" Comp Low-");
  Serial.print(bitRead(data, 10), DEC);
  Serial.print(" Comp High-");
  Serial.println(bitRead(data, 11), DEC);
  //Serial.println(" ");
  //Serial.println(data, BIN);
}
/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 * Returns the value of the Automatic Gain Control diagnostic register
 */
byte AS5048A::getGain() {
  word data = getState();
  if (error()) {
    Serial.print("Error bit was set! (function getGain register Diagnostics + Automatic Gain Control (AGC) )");
  }
  return (byte)data & 0xFF;
}
/**
 * Get and clear the error register by reading it
 * Clear the error flag and return three bits (0 bit Framing Error, 1 bit Command Invalid, 2 bit Parity Error)
 * Error register. All errors are cleared by access
 * Possible conditions that cause the ERROR FLAG to be set:
 * • Invalid parity
 * • Incorrect number of clocks (without a complete transmission cycle or too many clocks)
 * • Invalid command
 * • Framing error
 * Note(s): If the error flag is set high due to
 * communication issues, the flag remains set until it is
 * cleared by the CLEAR ERROR FLAG command.
 */
word AS5048A::getErrors() {
  return read(static_cast<word>(AS5048A_CLEAR_ERROR_FLAG)) & ~0xC000;
}
/**
 * Get and clear the error register and output the value of the register to the Serial port
 */
void AS5048A::printErrors() {
  word data;
  data = getErrors();
  if (error()) {
    Serial.println("Error bit was set! (function printErrors register Clear Error Flag)");
  }
  Serial.println("Error register");
  Serial.print("Framing (packet) command error ");
  Serial.println(bitRead(data, 0), DEC);
  Serial.print("Invalid command ");
  Serial.println(bitRead(data, 1), DEC);
  Serial.print("Parity error ");
  Serial.println(bitRead(data, 2), DEC);
  Serial.println(" ");
  // Serial.println(data, BIN);
}
/**
 * The function sends a NOP command and returns the contents of the register. The NOP command is a dummy
 * write to the x0000 register of the AS5048 sensor
 */
word AS5048A::dummyOperNoInf() {
  return read(static_cast<word>(AS5048A_NOP));
}
/**
 * The procedure writes the absolute value measured by the AS5048 sensor of a randomly positioned magnet on the axis of rotation,
 * as the zero position of the angle
 * Programming AS5048
 * Programming the zero position: the absolute angle position can be programmed via the interface. This can be useful for randomly placing the magnet on the axis of rotation.
 * Reading at the mechanical zero position can be performed and written back to the IC. With permanent programming, the position is irreversible, stored in the IC.
 * This programming can only be done once. To simplify the calculation of the zero position, it is only necessary to write the value to the IC that was read earlier from the angle register.
 * Programming sequence with checking: to program the zero position, the following sequence must be performed:
 * 1. Write 0 to the OTP zero position register to clear it.
 * 2. Read the current angle information
 * 3. Write the read angle position to the OTP zero position register.
 * Now writing the zero position. If you want to write the OTP register value, send:
 * 4. Set the programming bit (Programming Enable) to write the OTP control register value.
 * 5. Set the burn bit to start the automatic programming procedure.
 * 6. Read the current angle information if (equals 0) then.
 * 7. Set the Verify bit to reload OTP data into internal registers.
 * 8. Read the current angle information for verification (equals 0).
 * Programming can be performed in 5V mode using the internal LDO or 3V, but with a minimum supply voltage of 3.3V. In the case of 3V operation, a 10 µF capacitor is also required on the VDD3 pin.
 */
void AS5048A::progAbsoluteAngleZeroPosit() {
  word rotationzero = 0b0000000000000000;
  word programcontrol = 0b00000000000000;
  write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, AS5048A_NOP & ~0xFF00);
  write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, AS5048A_NOP & ~0xFFC0);
  rotationzero |= getRawRotation();
  write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, (rotationzero >> 6) & 0xFF);
  write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, rotationzero & 0x3F);
  write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol, 0));
  write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol, 3));
  if ((-1 < getRawRotation()) && (getRawRotation() < 1)) {
    write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol, 6));
  }
  Serial.println(getRawRotation(), DEC);
}
/**
 * Set the zero _position
 */
void AS5048A::setZeroPosition(word argPosition) {
  _position = argPosition % 0x3FFF;
}
/**
 * Returns the current zero _position
 */
word AS5048A::getZeroPosition() {
  return _position;
}
/**
 * Function for sorting in ascending order
 */
void AS5048A::quickSort(word *adcValues, int left, int right) {
  // Check if the array is empty or has one element
  if (left >= right) {
    return;
  }
  int i = left, j = right;
  int tmp;                                    // for swap
  word pivot = adcValues[(left + right) / 2]; // Pivot element
  // Partition the array
  while (i <= j) {
    while (adcValues[i] < pivot)
      i++;
    while (adcValues[j] > pivot)
      j--;
    if (i <= j) {
      // Swap values
      tmp = adcValues[i];
      adcValues[i] = adcValues[j];
      adcValues[j] = tmp;
      i++;
      j--;
    }
  }
  // Recursive calls to sort subarrays
  if (left < j)
    quickSort(adcValues, left, j);
  if (i < right)
    quickSort(adcValues, i, right);
}
/**
 * Check if an error has been encountered.
 * Error flag indicating a transmission error in the previous transmission of the master device
 */
bool AS5048A::error() {
  return _errorFlag;
}
/**
 * Calculates the median average for a given array
 */
word AS5048A::calculateMedian(word *adcValues, int sizeArray) {
  // Sort the array
  quickSort(adcValues, 0, sizeArray - 1);
  // Find the maximum gap for zero crossing correction
  word maxGap = 0;
  int maxGapIndex = 0;
  for (int i = 1; i < sizeArray; i++) {
    word gap = adcValues[i] - adcValues[i - 1];
    if (gap > maxGap) { // Find the start of the gap
      maxGap = gap;
      maxGapIndex = i;
    }
  }
  // Zero crossing correction
  if (maxGap > AS5048A_ANGLE_HALF) {
    for (int i = 0; i < maxGapIndex; i++) {
      adcValues[i] += AS5048A_ANGLE;
    }
    quickSort(adcValues, 0, sizeArray - 1);
  }
  // Calculate the median value
  float median;
  if (sizeArray > 1) {
    if ((sizeArray % 2) > 0) {
      median = (adcValues[(sizeArray / 2) - 1] + adcValues[sizeArray / 2] + adcValues[(sizeArray / 2) + 1]) / 3;
    } else {
      median = (adcValues[(sizeArray / 2) - 1] + adcValues[sizeArray / 2]) / 2;
    }
  } else {
    median = adcValues[0];
  }
  return static_cast<word>(median);
}
/**
 * Calculates the circular average for a given array
 */
word AS5048A::calculateCircular(word *adcValues, int sizeArray) {
  double sinSum = 0.0;
  double cosSum = 0.0;
  for (int i = 0; i < sizeArray; ++i) {
    // Convert ADC value to radians
    sinSum += sin(static_cast<double>(rotationRawToRadian(adcValues[i])));
    cosSum += cos(static_cast<double>(rotationRawToRadian(adcValues[i])));
  }
  // Calculate circular average using arctan2
  // Convert average back to ADC code  
  return radianToRotationRaw( normalizeAngle(atan2(sinSum,cosSum)) );
}
/**
 * Calculates the circular average with median for a given array
 */
word AS5048A::calculateCircularMedian(word *adcValues, int sizeArray) {
  double sinSum = 0.0;
  double cosSum = 0.0;
  // Sort the array
  quickSort(adcValues, 0, sizeArray - 1);
  // Calculate the median value
  if (sizeArray > 1) {
    if ((sizeArray % 2) > 0) {
      sinSum = (sin(static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + sin(static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) ) + sin(static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) + 1])) ) ) / 3;
      cosSum = (cos(static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + cos(static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) ) + cos(static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) + 1])) ) ) / 3;
    } else {
      sinSum = (sin(static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + sin(static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) )) / 2;
      cosSum = (cos(static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + cos(static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) )) / 2;
    }
  } else {
    sinSum = sin(static_cast<double>(rotationRawToRadian(adcValues[0])));
    cosSum = cos(static_cast<double>(rotationRawToRadian(adcValues[0])));
  }
  // Calculate circular average using arctan2
  // Convert average back to ADC code
  return radianToRotationRaw( normalizeAngle(atan2(sinSum,cosSum)) );
}
/**
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 * Sending a command to read the registers of the AS5048A sensor
 * median enables median average mode
 * sizeValues number of measurements for median average mode
 */
word AS5048A::readAveragingMedian(word registerAddress, byte sizeValues, bool median) {
  word readData;
  if (sizeValues <= 0) {
    sizeValues = 1;
  }
  word adcValues[sizeValues];
  word command = 0b0100000000000000; // PAR=0 R/W=R
  command |= registerAddress;
  // Add a parity bit on the MSB
  command |= ((word)spiCalcEvenParity(command) << 15);
  // SPI - begin transaction
  SPI.beginTransaction(settings);
  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
#ifdef AS5048A_DEBUG
  Serial.print("Read address (0x");
  Serial.print(registerAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif
  // Send the command and Now read the response
  if (median == true) {
    for (byte i = 0; i < sizeValues - 1; i++) {
      digitalWrite(_cs, LOW);
      adcValues[i] = SPI.transfer16(command) & ~0xC000;
      digitalWrite(_cs, HIGH);
      // Serial.println(adcValues[i], BIN);
    }
    SPI.endTransaction();
    // SPI - end transaction
    quickSort(adcValues, 0, sizeValues - 1); //(sizeof(adcValues) / sizeof(adcValues[0]))
    if (sizeValues > 1) {
      if ((sizeValues % 2) > 0) {
        readData = (adcValues[(sizeValues / 2) - 1] + adcValues[sizeValues / 2] + adcValues[(sizeValues / 2) + 1]) / 3;
      } else {
        readData = (adcValues[(sizeValues / 2) - 1] + adcValues[sizeValues / 2]) / 2;
      }
    } else {
      readData = adcValues[0];
    }
    // Return the data, stripping the parity and error bits
    return readData;
  } else {
    digitalWrite(_cs, LOW);
    readData = SPI.transfer16(command);
    digitalWrite(_cs, HIGH);
#ifdef AS5048A_DEBUG
  Serial.print("Read returned:\n");
  Serial.print("\thighByte ");
  Serial.println(highByte(readData), BIN);
  Serial.print("\tlowByte ");
  Serial.println(lowByte(readData), BIN);
  Serial.print("\tReadValueBIN: ");
  Serial.println(readData, BIN);
  Serial.print("\tReadValueDEC: ");
  Serial.println(readData, DEC);
  Serial.println("");
#endif
    // Check if the error bit is set
    // If the 15th bit is set to 1 (transmission error in the previous transmission of the master device) set _errorFlag to 1 otherwise to 0
    if (bitRead(readData, ERROR_FLAG_BIT)) {
#ifdef AS5048A_DEBUG
    Serial.println("Setting error bit");
    Serial.println("Error bit set");
#endif
    _errorFlag = true; // Set the error flag
  } else {
    _errorFlag = false; // Reset the error flag
  }
    SPI.endTransaction();
    // SPI - end transaction
    // Return the data, stripping the parity and error bits
    return readData & ~0xC000;
  }
}
/**
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 * Sending a command to read the registers of the AS5048A sensor
 * median enables median average mode or
 * circular enables circular average mode
 * if both modes are enabled, the sensor is simply polled without averaging.
 * sizeValues number of measurements for median average mode
 */
word AS5048A::readAveragingUniversal(word registerAddress, byte sizeValues, bool median, bool circular) {
  if ((sizeValues <= 0) || (!median && !circular)) {
    sizeValues = 1;
  }
  byte errorCount = 0;
  // Allocate memory for the array
  word adcValues[sizeValues];
  for (byte i = 0; i < sizeValues;) {
    // Read data from the register
    word readData = read(registerAddress);
    // Check the error flag
    if (_errorFlag) {
      if (errorCount >= sizeValues) {
        // If the number of read errors exceeds the size of the array, exit the loop and return 0 from the function
#ifdef AS5048A_DEBUG
        Serial.println("Read error, angle values are not valid...");
#endif
        return 0;
      }
       // If the error flag is set, repeat the read
#ifdef AS5048A_DEBUG
      Serial.println("Read error, retrying...");
#endif
      errorCount++;
      continue; // Continue the loop without increasing the index
    }
    // If there are no errors, save the data
    adcValues[i] = readData;
    i++; // Increase the index only if the read was successful
  }
  if (median && !circular)
    return calculateMedian(adcValues, sizeValues);
  if (circular && !median)
    return calculateCircular(adcValues, sizeValues);
  if (circular && median)
    return calculateCircularMedian(adcValues, sizeValues);
  return adcValues[0];
}
/**
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
word AS5048A::read(word registerAddress) {
  word readData;
  word command = 0b0100000000000000; // PAR=0 R/W=R
  command |= registerAddress;
  // Add a parity bit on the MSB
  // Adds a parity bit to the most significant bit (MSB).
  command |= ((word)spiCalcEvenParity(command) << 15);
#ifdef AS5048A_DEBUG
  Serial.print("Read address (0x");
  Serial.print(registerAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif
  // SPI - begin transaction
  // SPI - start transaction
  SPI.beginTransaction(settings);
  // Send the command target address
  // Send the read command
  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);
  // Send a NOP to get the new data in the register
  // Send a NOP command to get new data in the register.
  digitalWrite(_cs, LOW);
  readData = SPI.transfer16(AS5048A_NOP); // Send an empty byte to get the data
  digitalWrite(_cs, HIGH);
  // SPI - end transaction
  // SPI - end transaction
  SPI.endTransaction();
#ifdef AS5048A_DEBUG
  Serial.print("Read returned:\n");
  Serial.print("\thighByte ");
  Serial.println(highByte(readData), BIN);
  Serial.print("\tlowByte ");
  Serial.println(lowByte(readData), BIN);
  Serial.print("\tReadValueBIN: ");
  Serial.println(readData, BIN);
  Serial.print("\tReadValueDEC: ");
  Serial.println(readData, DEC);
  Serial.println("");
#endif
  // Check if the error bit is set
  // Checks if the 14th bit is set in the readData variable,
  // and sets the error flag _errorFlag accordingly
  if (bitRead(readData, ERROR_FLAG_BIT)) {
#ifdef AS5048A_DEBUG
    Serial.println("Setting error bit");
    Serial.println("Error bit set");
#endif
    _errorFlag = true; // Set the error flag
  } else {
    _errorFlag = false; // Reset the error flag
  }
  // Return the data, stripping the parity and error bits
  // Returns the data, removing the parity and error bits
  return readData & ~0xC000;
}
/**
 * Write to a register
 * Takes the 16-bit address of the target register and the 16 bit word of writeData
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a successful write.
 */
word AS5048A::write(word registerAddress, word writeData) {
  word command = 0b0000000000000000; // PAR=0 R/W=W
  word dataToSend = 0b0000000000000000;
  command |= registerAddress;
  dataToSend |= writeData;
  // Add a parity bit on the MSB
  command |= ((word)spiCalcEvenParity(command) << 15);
  // Craft another packet including the data and parity
  dataToSend |= ((word)spiCalcEvenParity(dataToSend) << 15);
#ifdef AS5048A_DEBUG
  Serial.print("Read address (0x");
  Serial.print(registerAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif
  // SPI - begin transaction
  SPI.beginTransaction(settings);
  // Start the write command with the target address
  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);
#ifdef AS5048A_DEBUG
  Serial.print("Sending data to write: ");
  Serial.println(dataToSend, BIN);
#endif
  // Now send the data packet
  digitalWrite(_cs, LOW);
  SPI.transfer16(dataToSend);
  digitalWrite(_cs, HIGH);
  // Send a NOP to get the new data in the register
  digitalWrite(_cs, LOW);
  dataToSend = SPI.transfer16(AS5048A_NOP); // Send an empty byte to get the data
  digitalWrite(_cs, HIGH);
  // SPI - end transaction
  SPI.endTransaction();
  // Return the data, stripping the parity and error bits
  return dataToSend & ~0xC000;
}