#pragma once
#define LIBRARY_VERSION 1.0.1
#include <Arduino.h>
#include <SPI.h>
// #include <math.h>
// #include <stdio.h>
// #include <stdlib.h>
class AS5048A {
  bool _errorFlag;
  byte _cs;
  // byte cs;
  // byte dout;
  // byte din;
  // byte clk;
  // word transaction(word data);
  word _position;
  bool _reverse; // Flag for the direction of the machine slider movement
  SPISettings settings;
public:
  /**
   *  Constructor
   */
  AS5048A(byte argCs);
  /**
   * Initialiser
   * Sets up the SPI interface
   */
  void init();
  /**
   * Closes the SPI connection
   */
  void close();
  /**
   * Read a register from the sensor
   * Takes the address of the register as a 16 bit word
   * Returns the value of the register
   * MeaValueMedian allows finding the median average from 16 measurements since
   * after 16 CLK cycles, CSn needs to be returned to a high state to reset
   * some parts of the interface core.
   * median enables median average mode
   * sizeValues number of measurements for median average mode
   */
  // Marking the function as deprecated
  [[deprecated("This function is deprecated, use the new function.")]]
  word readAveragingMedian(word registerAddress, byte sizeValues = 16, bool median = false);
  word readAveragingUniversal(word registerAddress, byte sizeValues = 16, bool median = false, bool circular = false);
  word read(word registerAddress);
  /**
   * Write to a register
   * Takes the 16-bit address of the target register and the 16-bit word of data
   * to be written to that register
   * Returns the value of the register after the write has been performed. This
   * is read back from the sensor to ensure a successful write.
   */
  word write(word registerAddress, word writeData);
  /**
   * Get the rotation of the sensor relative to the zero position.
   *
   * @return {int} between -2^13 and 2^13
   */
  int getRotation();
  /**
   * Returns the raw angle directly from the sensor
   */
  word getRawRotation(byte sizeValues = 0, bool median = false, bool circular = false);
  /**
   * Returns the physical quantity in angular degrees obtained from a 14-bit binary ADC number
   */
  float rotationRawToAngle(word discreteCode);
  /**
   * Returns the physical quantity in angular radians obtained from a 14-bit binary ADC number
   */
  float rotationRawToRadian(word discreteCode);
  /**
   * Function RadianToRotationRaw: This function takes an angle in radians and returns the value
   * corresponding to this angle in the format used by the ADC (or in discrete code).
   */
  word radianToRotationRaw(double radians);
  /**
   * Normalization of the angle
   */
  double normalizeAngle(double radians);
  /**
   * Returns the incremental and decremental rotation angle in the RotationAngle variable, pointers to the variables are passed to the procedure
   */
  void absoluteAngleRotation(float *rotationAngle, float *angleCurrent, float *anglePrevious);
  /**
   * Returns the incremental and decremental rotation angle in the RotationAngle variable, pointers to the variables are passed to the procedure
   */
  float absoluteAngleRotation(float *rotationAngle, float angleCurrent, float *anglePrevious);
  /**
   * Function for sorting in ascending order
   */
  void quickSort(word *adcValues, int left, int right);
  /**
   * Function for calculating the median
   */
  word calculateMedian(word *adcValues, int sizeValues);
  /**
   * Calculates the circular average for a given array
   */
  word calculateCircular(word *adcValues, int sizeArray);
  /**
   * Calculates the circular average with median for a given array
   */
  word calculateCircularMedian(word *adcValues, int sizeArray);
  /**
   * returns angular minutes
   */
  float getAngularMinutes(float angleAbsolute);
  /**
   * Returns angular seconds
   */
  float getAngularSeconds(float angleAbsolute);
  /*!
   * @brief returns the displacement of a straight-toothed rack in mm
   * @param WheelRotationAngle - Wheel rotation angle
   * @param NormalModule - Normal module
   * @param NumberGearTeeth - Number of teeth on the wheel or number of worm threads
   * @param (PI * NormalModule) - End face pitch
   * @return float displacement in mm
   */
  float linearDisplacementRack(float wheelRotationAngle, float normalModule, float numberGearTeeth);
  /*!
   * @brief returns the displacement of a straight-toothed rack in mm
   * @param WheelRotationAngle - Wheel rotation angle
   * @param NormalModule - Normal module
   * @param NumberGearTeeth - Number of teeth on the wheel or number of worm threads
   * @param (PI * NormalModule) - End face pitch
   * @param AngleTiltTooth Tooth tilt angle, default argument 20
   * @return float displacement in mm
   */
  float linearDisplacementRack(float wheelRotationAngle, float normalModule, float numberGearTeeth, float angleTiltTooth);
  /**
   * Returns the displacement of the screw transmission in mm
   * StepGroove - screw thread pitch
   * ScrewRotationAngle - screw rotation angle
   */
  float linearMotionHelicalGear(float screwRotationAngle, float stepGroove);
  /**
   * returns the value of the state register
   * @return 16 bit word containing flags
   */
  word getState();
  /**
   * Print the diagnostic register of the sensor
   */
  void printState();
  /**
   * Returns the value used for Automatic Gain Control (Part of diagnostic
   * register)
   */
  byte getGain();
  /**
   * Get and clear the error register by reading it
   */
  word getErrors();
  /**
   * Get and clear the error register and output the value of the register to the Serial port
   */
  void printErrors();
  /**
   * The function sends a NOP command and returns the contents of the register. The NOP command is a dummy
   * write to the x0000 register of the AS5048 sensor
   */
  word dummyOperNoInf();
  /**
   * The procedure writes the absolute value measured by the AS5048 sensor of a randomly positioned magnet on the axis of rotation,
   * as the zero position of the angle
   */
  void progAbsoluteAngleZeroPosit();
  /**
   * Set the zero position
   */
  void setZeroPosition(word argPosition);
  /**
   * Returns the current zero position
   */
  word getZeroPosition();
  /**
   * Check if an error has been encountered.
   */
  bool error();
private:
  /**
   * returns the parity bit
   */
  byte spiCalcEvenParity(word valueADC);
};