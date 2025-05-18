#include "AS5048A.h"

//#define AS5048A_DEBUG

const int ERROR_FLAG_BIT 						= 14;     // Индекс бита ошибки
const int AS5048A_NOP 							= 0x0000; // Фиктивная операция, нет информации.
const int AS5048A_CLEAR_ERROR_FLAG 				= 0x0001; // Регистр ошибок. Все ошибки очищаются путем доступа.
const int AS5048A_PROGRAMMING_CONTROL 			= 0x0003; // Регистр управления программированием. Программирование должно быть включено до прожига памяти. Перед программированием проверка является обязательной. См. Процедуру программирования.
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH 	= 0x0016; // Нулевое значение 8 бит старших
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW 	= 0x0017; // Нулевая значение 6 бит младших
const int AS5048A_DIAG_AGC 						= 0x3FFD; // (0-7)Значение автоматического регулирования усиления. 0 десятичной представляет высокое магнитное поле, 255 десятичных представляет низкое магнитное поле. (8-13)Флаги диагностики
const int AS5048A_MAGNITUDE 					= 0x3FFE; // Значение выходной мощности CORDIC
const int AS5048A_ANGLE 						= 0x3FFF; // Угловое выходное значение, включая коррекцию нулевой позиции Resolution_ADC 14-bit resolution (0.0219°/LSB)
const int AS5048A_ANGLE_HALF 					= AS5048A_ANGLE / 2; // Половина диапазона

/**
 * Constructor
 * Инициализация библиотеки AS5048A
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
   * mySettting (speedMaximum, dataOrder, dataMode)
   * speedMaximum - максимальная скорость связи. Для чипа SPI, рассчитанного на частоту до 20 МГц , используйте 20000000.
   * dataOrder - порядок вывода данных в/из шины SPI,  может быть LSBFIRST (наименьший разряд(бит) первый) или MSBFIRST (старший разряд первый)
   * dataMode - устанавливает режим работы шины SPI, задавая уровень сигнала синхронизации и фазу синхронизации
   * SPI_MODE0 (Уровень сигнала (CPOL)-0, Фаза (CPHA)-0)
   * SPI_MODE1 (Уровень сигнала (CPOL)-0, Фаза (CPHA)-1)
   * SPI_MODE2 (Уровень сигнала (CPOL)-1, Фаза (CPHA)-0)
   * SPI_MODE3 (Уровень сигнала (CPOL)-1, Фаза (CPHA)-1)
   * f(sample) = Min-10.2, Typ-11.25, Max-12.4. (kHz)
   * Поддерживаемые режимы  I2C AS5048B:
   * • Случайное / последовательное чтение
   * • Байт / Запись страницы
   * • Стандартный: от 0 до 100 кГц, тактовая частота (ведомый режим)
   * • Быстрый режим: тактовая частота от 0 до 400 кГц (ведомый режим)
   * • Высокая скорость: от 0 до 3,4 МГц тактовой частоты (ведомый режим)
   */
  settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
  // инициализация пина Slave Select если LOW ведомый взаимодействует с ведущим если HIGH ведомый игнорирует сигналы от ведущего
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
 * Вычисление бита чётности 14 битного адресса и запись в 15-й бит возвращаемого 16 битного слова
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
 * Возвращает угловое двоичное 14 битное угловое значение (DEC 16383)
 * Угловое выходное значение, включая коррекцию нулевой позиции.
 */
word AS5048A::getRawRotation(byte sizeValues, bool median, bool circular) {
  return readAveragingUniversal(static_cast<word>(AS5048A_ANGLE), sizeValues, median, circular);
}

/**
 * Возвращает физическую величину в угловых градусах, полученное из двоичного 14 битного числа АЦП
 */
float AS5048A::rotationRawToAngle(word discreteCode) {
  return discreteCode * (360.0 / float(AS5048A_ANGLE));
}

/**
 * Возвращает физическую величину в угловых радианах, полученное из двоичного 14 битного числа АЦП
 */
float AS5048A::rotationRawToRadian(word discreteCode) {
  return static_cast<float>(discreteCode) * ((2 * PI) / float(AS5048A_ANGLE));
}

/**
 * Функция RadianToRotationRaw: Эта функция принимает угол в радианах и возвращает значение,
 * соответствующее этому углу в формате, который использует АЦП (или в дискретном коде).
 */
word AS5048A::radianToRotationRaw(double radians) {
  return static_cast<word>(radians * (AS5048A_ANGLE / (2 * PI)));
}

/**
 * Возвращает инкрементный и декрементный угол поворота в переменную rotationAngle в процедуру прередаються адреса переменных
 */
void AS5048A::absoluteAngleRotation(float *rotationAngle, float *angleCurrent, float *anglePrevious) {

  if (*angleCurrent != *anglePrevious) {
    // сделан круг на возрастание с 360 на 1
    if ((*angleCurrent < 90) && (*anglePrevious > 270) /*|| (*angleCurrent < 1.5707963267948966192313216916398) && (*anglePrevious > 4.7123889803846898576939650749193) */) {
      *rotationAngle += abs(360 - abs(*angleCurrent - *anglePrevious));
      _reverse = true;
    }

    // сделан круг на убывание с 1 на 360
    if ((*anglePrevious < 90) && (*angleCurrent > 270) /*|| (*anglePrevious < 1.5707963267948966192313216916398) && (*angleCurrent > 4.7123889803846898576939650749193) */) {
      *rotationAngle -= abs(360 - abs(*angleCurrent - *anglePrevious));
      _reverse = false;
    }

    // ход по кругу на возрастание
    if (*angleCurrent > *anglePrevious && ((*angleCurrent < 90) && (*anglePrevious > 270))!=true && ((*anglePrevious < 90) && (*angleCurrent > 270))!=true /*|| *angleCurrent > *anglePrevious && ((*angleCurrent < 1.5707963267948966192313216916398) && (*anglePrevious > 4.7123889803846898576939650749193))!=true && ((*anglePrevious < 1.5707963267948966192313216916398) && (*angleCurrent > 4.7123889803846898576939650749193))!=true*/){
      *rotationAngle += abs(*angleCurrent - *anglePrevious);
      _reverse = true;
    }

    // ход по кругу на убывание
    if (*anglePrevious > *angleCurrent && ((*angleCurrent < 90) && (*anglePrevious > 270))!=true && ((*anglePrevious < 90) && (*angleCurrent > 270))!=true /*|| *anglePrevious > *angleCurrent && ((*angleCurrent < 1.5707963267948966192313216916398) && (*anglePrevious > 4.7123889803846898576939650749193))!=true && ((*anglePrevious < 1.5707963267948966192313216916398) && (*angleCurrent > 4.7123889803846898576939650749193))!=true*/){
      *rotationAngle -= abs(*anglePrevious - *angleCurrent);
      _reverse = false;
    }
  }

  *anglePrevious = *angleCurrent;
}

float AS5048A::absoluteAngleRotation(float *rotationAngle, float angleCurrent, float *anglePrevious) {

  if (angleCurrent != *anglePrevious) {
    // сделан круг на возрастание с 360 на 1
    if ((angleCurrent < 90) && (*anglePrevious > 270)) {
      *rotationAngle += abs(360 - abs(angleCurrent - *anglePrevious));
    }

    // сделан круг на убывание с 1 на 360
    if ((*anglePrevious < 90) && (angleCurrent > 270)) {
      *rotationAngle -= abs(360 - abs(angleCurrent - *anglePrevious));
    }

    // ход по кругу на возрастание
    if (angleCurrent > *anglePrevious && ((angleCurrent < 90) && (*anglePrevious > 270)) != true && ((*anglePrevious < 90) && (angleCurrent > 270)) != true) {
      *rotationAngle += abs(angleCurrent - *anglePrevious);
    }

    // ход по кругу на убывание
    if (*anglePrevious > angleCurrent && ((angleCurrent < 90) && (*anglePrevious > 270)) != true && ((*anglePrevious < 90) && (angleCurrent > 270)) != true) {
      *rotationAngle -= abs(*anglePrevious - angleCurrent);
    }
  }

  *anglePrevious = angleCurrent;

  return *rotationAngle;
}

/**
 * Возвращает минуты угла
 */
float AS5048A::getAngularMinutes(float angleAbsolute) {
  return (angleAbsolute - int(angleAbsolute)) * 60;
}

/**
 * Возвращает секунды угла
 */
float AS5048A::getAngularSeconds(float angleAbsolute) {
  return ( getAngularMinutes(angleAbsolute) - static_cast<int>(getAngularMinutes(angleAbsolute)) ) * 60;
}

/**
 * Возвращает перемещение прямозубой зубчатой рейки в мм
 * wheelRotationAngle - Угол поворота колеса
 * normalModule - Модуль нормальный
 * numberGearTeeth - Число зубьев колеса или число заходов червяка
 * (PI * normalModule) - Шаг торцовый
 * AngleTiltTooth Угол наклона зуба, аргументы по умолчанию 20
 */
float AS5048A::linearDisplacementRack(float wheelRotationAngle, float normalModule, float numberGearTeeth) {
  return wheelRotationAngle * (((PI * normalModule) * numberGearTeeth) / 360);
}

float AS5048A::linearDisplacementRack(float wheelRotationAngle, float normalModule, float numberGearTeeth, float angleTiltTooth = 20) {
  return wheelRotationAngle * ((((PI * normalModule) / cos(radians(angleTiltTooth))) * numberGearTeeth) / 360);
}

/**
 * Возвращает перемещение винтовой передачи в мм
 * stepGroove - шаг резьбы винта
 * screwRotationAngle - угол поворота винта
 */
float AS5048A::linearMotionHelicalGear(float screwRotationAngle, float stepGroove) {
  return (screwRotationAngle * (stepGroove / 360));
}

/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 * Возвращает значение диагностического регистра датчика
 * размером 16 бит из них 13 значащих (пример 1101100110101)
 */
word AS5048A::getState() {
  return read(static_cast<word>(AS5048A_DIAG_AGC)) & ~0xC000;
}

/**
 * Print the diagnostic register of the sensor
 * Вывести в порт Serial значение диагностического регистра датчика
 */
void AS5048A::printState() {
  word data;
  data = getState();

  if (AS5048A::error()) {
    Serial.println("Error bit was set! (function printState register Diagnostics + Automatic Gain Control (AGC) )");
  }

  Serial.println(" ");
  Serial.println("\tЗначение автоматического регулирования усиления магнитного поля");
  Serial.println("\t255 - представляет собой низкое магнитное поле");
  Serial.println("\t0 - представляет собой высокое магнитное поле");
  Serial.print("\tЗначение BIN: ");
  Serial.println(lowByte(data), BIN);
  Serial.print("\tЗначение DEC: ");
  Serial.println(lowByte(data), DEC);

  /**Диагностические функции AS5048
   * AS5048 обеспечивает диагностические функции ИС, а также диагностические функции магнитного поля ввода. Доступны следующие диагностические флаги: см. Рис. 22 адрес регистра x3FFD (AS5048A) или
   * адрес 31 адреса адреса 251 деци (AS5048B) • OCF (Компенсация смещения завершена), логический максимум указывает законченный алгоритм компенсации смещения. После включения флаг всегда остается
   * логически высоким. • COF (CORDIC Overflow), логический максимум указывает на ошибку вне диапазона в части CORDIC. Когда этот бит установлен, данные угла и величины недействительны. Абсолютный
   * выход сохраняет последнее действительное угловое значение. • COMP low, указывает на высокое магнитное поле. Рекомендуется дополнительно контролировать величину величины. • COMP высокий, указывает
   * на слабое магнитное поле. Рекомендуется контролировать величину величины.
   */
  Serial.print("Флаги диагностики");
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
 * Возвращает значение Автоматического контроля усиления диагностического регистра
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
 * Очистить флаг ошибки и возврат три бита (0бит Framing Error, 1бит Command Invalid, 2бит Parity Error)
 * Регистр ошибок. Все ошибки очищаются путем доступа
 * Возможные условия, которые заставляют установить ERROR FLAG:
 * • Неверный паритет
 * • Неправильное количество часов (без полного цикла передачи или слишком много часов)
 * • Недействительная команда
 * • Ошибка кадра
 * Примечание (ы): Если флаг ошибки установлен на высокий из-за
 * проблема связи, флаг остается установленным до тех пор, пока он не будет
 * очищается командой CLEAR ERROR FLAG.
 */
word AS5048A::getErrors() {
  return read(static_cast<word>(AS5048A_CLEAR_ERROR_FLAG)) & ~0xC000;
}

/**
 * Получить и очистка регистра ошибок и вывести значение регистра в Serial порт
 */
void AS5048A::printErrors() {
  word data;
  data = getErrors();
  if (error()) {
    Serial.println("Error bit was set! (function printErrors register Clear Error Flag)");
  }
  Serial.println("Регистр ошибок");
  Serial.print("Ошибка кадра (пакета) команды ");
  Serial.println(bitRead(data, 0), DEC);
  Serial.print("Неверная команда ");
  Serial.println(bitRead(data, 1), DEC);
  Serial.print("Ошибка бита четности ");
  Serial.println(bitRead(data, 2), DEC);
  Serial.println(" ");
  // Serial.println(data, BIN);
}

/**
 * Функция посылает команда NOP и возвращает содержимое регистра. Команда NOP представляет собой фиктивную
 * запись в регистр x0000 сенсора AS5048
 */
word AS5048A::dummyOperNoInf() {
  return read(static_cast<word>(AS5048A_NOP));
}

/**
 * Процедура записывает абсолютное значен измеренное сенсором AS5048, случайно расположенного магнита на оси вращения,
 * как нулевую позицию угла

 * Программирование AS5048
 * Программирование нулевого положения: абсолютное положение угла может быть запрограммировано по интерфейсу. Это может быть полезно для случайного размещения магнита на оси вращения.
 * Считывание в механическом нулевом положении может быть выполнено и записано обратно в ИС. При постоянном программировании позиция не обратима, хранящаяся в ИС.
 * Это программирование может выполняться только один раз. Чтобы упростить вычисление нулевой позиции, необходимо только записать значение в ИС, которое было зачитано ранее из регистра угла.

 * Последовательность программирования с проверкой: для программирования нулевой позиции необходимо выполнить следующую последовательность:
 * 1. Запишите 0 в регистр нулевой позиции OTP, чтобы очистить.
 * 2. Считать информацию текущего угла
 * 3. Запишите считанное положение угла в регистр нулевой позиции OTP.

 * Теперь запись нулевого положение. Если вы хотите записать значение регистра OTP, отправьте:
 * 4. Установите бит программирования (Programming Enable) чтобы записать значение регистра управления OTP.
 * 5. Установите бит записи (Burn), чтобы запустить процедуру автоматического программирования.
 * 6. Считайте информацию текущего угла если (равно 0) то.
 * 7. Установите бит Verify для повторной загрузки OTP данных во внутренние регистры.
 * 8. Считайте информацию текущего угла для проверки (равно 0).

 * Программирование может быть выполнено в режиме 5 В с использованием внутреннего LDO или 3V, но с минимальным напряжением питания 3,3 В. В случае работы 3 В также требуется конденсатор 10 мкФ на выводе VDD3.
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
 * Функция для сортировки по возрастанию
 */
void AS5048A::quickSort(word *adcValues, int left, int right) {
  // Проверка на случай, если массив пустой или имеет один элемент
  if (left >= right) {
    return;
  }

  int i = left, j = right;
  int tmp;                                    // для swap
  word pivot = adcValues[(left + right) / 2]; // Опорный элемент

  // Разделение массива
  while (i <= j) {
    while (adcValues[i] < pivot)
      i++;
    while (adcValues[j] > pivot)
      j--;
    if (i <= j) {
      // Обмен значениями
      tmp = adcValues[i];
      adcValues[i] = adcValues[j];
      adcValues[j] = tmp;
      i++;
      j--;
    }
  }

  // Рекурсивные вызовы для сортировки под массивов
  if (left < j)
    quickSort(adcValues, left, j);
  if (i < right)
    quickSort(adcValues, i, right);
}

/**
 * Check if an error has been encountered.
 * Флаг ошибки, указывающий на ошибку передачи в предыдущей передаче ведущего устройства (Master)
 */
bool AS5048A::error() {
  return _errorFlag;
}

/**
 * Вычисляет медианного среднее для заданного массива
 */
word AS5048A::calculateMedian(word *adcValues, int sizeArray) {
  // Сортировка массива
  quickSort(adcValues, 0, sizeArray - 1);

  // Поиск максимального разрыва для коррекции перехода через 0
  word maxGap = 0;
  int maxGapIndex = 0;
  for (int i = 1; i < sizeArray; i++) {
    word gap = adcValues[i] - adcValues[i - 1];
    if (gap > maxGap) { // Поиск начала разрыва
      maxGap = gap;
      maxGapIndex = i;
    }
  }
  // Коррекция перехода через 0
  if (maxGap > AS5048A_ANGLE_HALF) {
    for (int i = 0; i < maxGapIndex; i++) {
      adcValues[i] += AS5048A_ANGLE;
    }
    quickSort(adcValues, 0, sizeArray - 1);
  }

  // Вычисление медианного значения
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
 * Вычисляет круговое среднее для заданного массива
 */
word AS5048A::calculateCircular(word *adcValues, int sizeArray) {
  double sinSum = 0.0;
  double cosSum = 0.0;

  for (int i = 0; i < sizeArray; ++i) {
    // Преобразуем значение АЦП в радианы
    sinSum += sin(static_cast<double>(rotationRawToRadian(adcValues[i])));
    cosSum += cos(static_cast<double>(rotationRawToRadian(adcValues[i])));
  }

  // Вычисляем круговое среднее с использованием arctan2
  // Преобразуем среднее обратно в код АЦП
  return radianToRotationRaw(atan2(sinSum, cosSum));
}

/**
 * Вычисляет круговое среднее c медианой для заданного массива
 */
word AS5048A::calculateCircularMedian(word *adcValues, int sizeArray){
  double sinSum = 0.0;
  double cosSum = 0.0;

  // Сортировка массива
  quickSort(adcValues, 0, sizeArray - 1);

  // Вычисление медианного значения
  if (sizeArray > 1) {
    if ((sizeArray % 2) > 0) {
      sinSum = ( sin( static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + sin( static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) ) + sin( static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) + 1])) ) ) / 3;
      cosSum = ( cos( static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + cos( static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) ) + cos( static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) + 1])) ) ) / 3;
    } else {
      sinSum = ( sin( static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + sin( static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) )) / 2;
      cosSum = ( cos( static_cast<double>(rotationRawToRadian(adcValues[(sizeArray / 2) - 1])) ) + cos( static_cast<double>(rotationRawToRadian(adcValues[sizeArray / 2])) )) / 2;
    }
  } else {
    sinSum = sin( static_cast<double>(rotationRawToRadian(adcValues[0])) );
    cosSum = cos( static_cast<double>(rotationRawToRadian(adcValues[0])) );
  }

  // Вычисляем круговое среднее с использованием arctan2
  // Преобразуем среднее обратно в код АЦП
  return radianToRotationRaw( atan2(sinSum,cosSum) );
}

/**
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 * Отправка команды на чтения регистров сенсора AS5048A
 * median включить режим медианного среднего
 * sizeValues количество измерений для режима медианного среднего
 */
word AS5048A::readAveragingMedian(word registerAddress, byte sizeValues, bool median) {
  word readData;
  if (sizeValues <= 0) {
    sizeValues = 1;
  }
  word adcValues[sizeValues];
  word command = 0b0100000000000000; // PAR=0 R/W=R

  command |= registerAddress;

  // Add a parity bit on the the MSB
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
    // Если в 15 бите установлена 1 (ошибка передачи в предыдущей передаче ведущего устройства) _errorFlag установить 1 иначе 0
    if (bitRead(readData, ERROR_FLAG_BIT)) {
#ifdef AS5048A_DEBUG
    Serial.println("Setting error bit");
    Serial.println("Установлен бит ошибки");
#endif
    _errorFlag = true; // Устанавливаем флаг ошибки
  } else {
    _errorFlag = false; // Сбрасываем флаг ошибки
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
 * Отправка команды на чтения регистров сенсора AS5048A
 * median включить режим медианного среднего или
 * circular включить режим кругового среднего
 * если будет включено оба режима тогда просто опрашиваеться датчик без усреднения.
 * sizeValues количество измерений для режима медианного среднего
 */
word AS5048A::readAveragingUniversal(word registerAddress, byte sizeValues, bool median, bool circular) {

  if ((sizeValues <= 0) || (!median && !circular)) {
    sizeValues = 1;
  }

  byte errorCount = 0;

  // Выделяем память для массива
  word adcValues[sizeValues];

  for (byte i = 0; i < sizeValues;) {
    // Чтение данных из регистра
    word readData = read(registerAddress);
    // Проверка флага ошибки
    if (_errorFlag) {
      if (errorCount >= sizeValues) {
        // Если количество ошибок чтения превышает размер массива выходим из цикла и возвращаем 0 из функции
#ifdef AS5048A_DEBUG
        Serial.println("Ошибка при чтении, значения угла не достоверны...");
#endif
        return 0;
      }
       // Если флаг ошибки установлен, повторяем чтение
#ifdef AS5048A_DEBUG
      Serial.println("Ошибка при чтении, повторяем...");
#endif
      errorCount++;
      continue; // Продолжаем цикл, не увеличивая индекс
    }
    // Если ошибок нет, сохраняем данные
    adcValues[i] = readData;
    i++; // Увеличиваем индекс только если чтение прошло успешно
  }

  if (median && !circular)
    return calculateMedian(adcValues, sizeValues);

  if (circular && !median)
    return calculateCircular(adcValues, sizeValues);

  if (circular && !median)
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

  // Add a parity bit on the the MSB
  // Добавляет бит четности в старший значащий бит (МСБ).
  command |= ((word)spiCalcEvenParity(command) << 15);

#ifdef AS5048A_DEBUG
  Serial.print("Read address (0x");
  Serial.print(registerAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif

  // SPI - begin transaction
  // SPI - начать транзакцию
  SPI.beginTransaction(settings);

  // Send the command target address
  // Отправить команду чтения
  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);

  // Send a NOP to get the new data in the register
  // Отправьте команду NOP, чтобы получить новые данные в регистре.
  digitalWrite(_cs, LOW);
  readData = SPI.transfer16(AS5048A_NOP); // Отправляем пустой байт для получения данных
  digitalWrite(_cs, HIGH);

  // SPI - end transaction
  // SPI - завершить транзакцию
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
  // Выполняет проверку бита ошибки, установлен ли 14-й бит в переменной readData,
  // и в зависимости от этого устанавливает флаг ошибки _errorFlag
  if (bitRead(readData, ERROR_FLAG_BIT)) {
#ifdef AS5048A_DEBUG
    Serial.println("Setting error bit");
    Serial.println("Установлен бит ошибки");
#endif
    _errorFlag = true; // Устанавливаем флаг ошибки
  } else {
    _errorFlag = false; // Сбрасываем флаг ошибки
  }

  // Return the data, stripping the parity and error bits
  // Возвращает данные, удалив биты четности и ошибки
  return readData & ~0xC000;
}

/**
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of writeData
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
word AS5048A::write(word registerAddress, word writeData) {
  word command = 0b0000000000000000; // PAR=0 R/W=W
  word dataToSend = 0b0000000000000000;

  command |= registerAddress;
  dataToSend |= writeData;

  // Add a parity bit on the the MSB
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
  dataToSend = SPI.transfer16(AS5048A_NOP); // Отправляем пустой байт для получения данных
  digitalWrite(_cs, HIGH);

  // SPI - end transaction
  SPI.endTransaction();

  // Return the data, stripping the parity and error bits
  return dataToSend & ~0xC000;
}
