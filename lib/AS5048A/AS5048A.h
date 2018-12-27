#ifndef as5048_h
#define as5048_h
#define LIBRARY_VERSION 1.0.1

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
   * MeaValueMedian разрешает найти медианое среднее значение из 16 измерений так как
   * после 16 тактов CLK циклов, CSn необходимо вернуть к высокому состоянию, чтобы сбросить
     некоторые части ядра интерфейса.
   */
  word read(word RegisterAddress, bool MeanValueMedian = false);

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
  word getRawRotation(bool EnableMedianValue = false);

  /**
   * Возвращает физическую величину в угловых градусах, олученное из двоичного 14 битного числа АЦП 
   */
  float RotationRawToAngle (word DiscreteCode);
  
  /**
   *Возвращает физическую величину в угловых радианах, полученное из двоичного 14 битного числа АЦП
   */
  float RotationRawToRadian(word DiscreteCode);

  /**
  * Возвращает инкрементный и декрементный угол поворота в переменную RotationAngle в процедуру прередають адреса переменных 
  */
  void AbsoluteAngleRotation (float *RotationAngle, float *AngleCurrent, float *AnglePrevious);

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
  
  /**
  *возвращает перемещение прямозубой зубчатой рекйки в мм
  *WheelRotationAngle - Угол поворота колеса
  *NormalModule - Модуль нормальный
  *NumberGearTeeth - Число зубьев колеса или число заходов червяка
  *(PI * NormalModule) - Шаг торцовый
  *20 - Угол наклона зуба
  */ 
  float LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth);
  
  /**
  *возвращает перемещение винтовой предачи в мм
  *StepGroove - шаг резьбы винта
  *ScrewRotationAngle - eгол поворота винта
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
   *запись в регитр x0000 сенсора AS5048
   */
  word DummyOperNoInf();
  
  /**
   *Процидура записывает абсолютное значен измернное сенсером AS5048, случайно расположеного магнита на оси вращения,
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
