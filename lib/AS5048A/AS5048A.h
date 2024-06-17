#ifndef as5048_h
#define as5048_h
#define LIBRARY_VERSION 1.0.1

#include <Arduino.h>
#include <SPI.h>

//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>

class AS5048A{

  bool _errorFlag;
  byte _cs;
  //byte cs;
  //byte dout;
  //byte din;
  //byte clk;
  //word transaction(word data);
  word _position;
  bool _reverse; //Флаг напраления движения ползуна станка

  SPISettings settings;

  public:

  /**
   *  Constructor
   */
  AS5048A(byte Arg_Cs);

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
   * MeaValueMedian разрешает найти медианное среднее значение из 16 измерений так как
   * после 16 тактов CLK циклов, CSn необходимо вернуть к высокому состоянию, чтобы сбросить
   * некоторые части ядра интерфейса.
   * MeanValueMedian включить режим медианного среднего
   * NumberFunctionValues количество измерений для режима медианного среднего
   */
  word read(word RegisterAddress, bool MeanValueMedian = false, byte NumberFunctionValues = 16);

  /**
   * Write to a register
   * Takes the 16-bit  address of the target register and the 16 bit word of data
   * to be written to that register
   * Returns the value of the register after the write has been performed. This
   * is read back from the sensor to ensure a sucessful write.
   */
  word write(word RegisterAddress, word WriteData);

  /**
   * Get the rotation of the sensor relative to the zero position.
   *
   * @return {int} between -2^13 and 2^13
   */
  int getRotation();

  /**
   * Returns the raw angle directly from the sensor
   */
  word getRawRotation(bool EnableMedianValue = false, byte NumberFunctionValues = 16);

  /**
   * Возвращает физическую величину в угловых градусах, полученное из двоичного 14 битного числа АЦП
   */
  float RotationRawToAngle (word DiscreteCode);

  /**
   *Возвращает физическую величину в угловых радианах, полученное из двоичного 14 битного числа АЦП
   */
  float RotationRawToRadian(word DiscreteCode);

  /**
  * Возвращает инкрементный и декрементный угол поворота в переменную RotationAngle в процедуру передаются адреса переменных
  */
  void AbsoluteAngleRotation (float *RotationAngle, float *AngleCurrent, float *AnglePrevious);

  /**
  * Возвращает инкрементный и декрементный угол поворота в переменную RotationAngle в процедуру передаются адреса переменных
  */
  float AbsoluteAngleRotation (float *RotationAngle, float AngleCurrent, float *AnglePrevious);

  /**
  *функция для сортировки по возрастанию
  */
  void quickSort(word *Arr, int Left, int Right);

  /**
  *возвращает минуты угла
  */
  float GetAngularMinutes (float AngleAbsolute);

  /**
  *возвращает секунды угла
  */
  float GetAngularSeconds (float AngleAbsolute);

  /*!
  * @brief возвращает перемещение прямозубой зубчатой рейки в мм
  * @param WheelRotationAngle - Угол поворота колеса
  * @param NormalModule - Модуль нормальный
  * @param NumberGearTeeth - Число зубьев колеса или число заходов червяка
  * @param (PI * NormalModule) - Шаг торцовый
  * @return float перемещение мм
  */
  float LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth);

  /*!
  * @brief возвращает перемещение прямозубой зубчатой рейки в мм
  * @param WheelRotationAngle - Угол поворота колеса
  * @param NormalModule - Модуль нормальный
  * @param NumberGearTeeth - Число зубьев колеса или число заходов червяка
  * @param (PI * NormalModule) - Шаг торцовый
  * @param AngleTiltTooth Угол наклона зуба, аргумент по умолчанию 20
  * @return float перемещение мм
  */
  float LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth, float AngleTiltTooth);

  /**
  *возвращает перемещение винтовой передачи в мм
  *StepGroove - шаг резьбы винта
  *ScrewRotationAngle - угол поворота винта
  */
  float LinearMotionHelicalGear ( float ScrewRotationAngle, float StepGroove);

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
   * Получить и очистить регистр ошибок и вывести значение регистра в Serial порт
   */
  void printErrors();

  /**
   *Функция посылает команда NOP и возвращает содержимое регистра. Команда NOP представляет собой фиктивную
   *запись в регистр x0000 сенсора AS5048
   */
  word DummyOperNoInf();

  /**
   *Процедура записывает абсолютное значен измеренно сенсором AS5048, случайно расположенного магнита на оси вращения,
   *как нулевую позицию угла
   */
  void ProgAbsolAngleZeroPosit ();

  /**
   * Set the zero position
   */
  void setZeroPosition(word Arg_Position);

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
   * возвращает бит чётности
   */
  byte spiCalcEvenParity(word);
};
#endif
