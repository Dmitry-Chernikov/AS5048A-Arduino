#include "AS5048A.h"

//#define AS5048A_DEBUG

const int AS5048A_NOP                           = 0x0000; // Фиктивная операция, нет информации.
const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001; //Регистр ошибок. Все ошибки очищаются путем доступа.
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003; //Регистр управления программированием. Программирование должно быть включено до прожига памяти. Перед программированием проверка является обязательной. См. Процедуру программирования.
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016; //Нулевое значение 8 бит старших
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017; //Нулевое значение 6 бит младших
const int AS5048A_DIAG_AGC                      = 0x3FFD; //(0-7)Значение автоматического регулирования усиления. 0 десятичной представляет высокое магнитное поле, 255 десятичных представляет низкое магнитное поле. (8-13)Флаги диагностики
const int AS5048A_MAGNITUDE                     = 0x3FFE; //Значение выходной мощности CORDIC
const int AS5048A_ANGLE                         = 0x3FFF; //Угловое выходное значение, включая коррекцию нулевой позиции Resolution_ADC 14-bit resolution (0.0219°/LSB)

/**
 * Constructor
 * Инициализация библиотеки AS5048A
 */
AS5048A::AS5048A(byte Arg_Cs){
  _cs = Arg_Cs;
  _errorFlag = false;
  _position = 0;
}


/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::init(){
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
  //инициализация пина Slave Select если LOW ведомый взаимодействует с ведущим если HIGH ведомый игнорирует сигналы от ведущего
  pinMode(_cs, OUTPUT);
  //SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
  SPI.begin();
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void AS5048A::close(){
  SPI.end();
}

/**
 * Utility function used to calculate even parity of word
 * Вычисление бита чётности 14 битного адресса и запись в 15-й бит возвращаемого 16 битного слова
 */
byte AS5048A::spiCalcEvenParity(word Value){
  byte cnt = 0;
  byte i;
  for (i = 0; i < 15; i++)
  {
     if (Value & 0x1)
    {
      cnt++;
    }
    Value >>= 1;
  }
  return cnt & 0x1;

  //byte operand_compare =  bitRead(Value,0);
  //byte i = 1;
  //do{
  //  operand_compare ^= bitRead(Value,i);
  //} while ((i++) < 14);
  //return operand_compare & 0x1;
}



/**
 * Get the rotation of the sensor relative to the zero _position.
 *
 * @return {int} between -2^13 and 2^13
 */
int AS5048A::getRotation(){
  word data;
  int rotation;
  data = AS5048A::getRawRotation();
  rotation = (int)data - (int)_position;
  if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
  //if(rotation < -0x1FFF) rotation = rotation+0x3FFF;
  return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 * Возвращает угловое двоичное 14 битное угловое значение (DEC 16383)
 * Угловое выходное значение, включая коррекцию нулевой позиции.
 */
word AS5048A::getRawRotation(bool EnableMedianValue, byte NumberFunctionValues){
  return AS5048A::read(AS5048A_ANGLE, EnableMedianValue, NumberFunctionValues);
}

/**
 *Возвращает физическую величину в угловых градусах, полученное из двоичного 14 битного числа АЦП
 */
float AS5048A::RotationRawToAngle(word DiscreteCode){
  return DiscreteCode * (360.0 / float(AS5048A_ANGLE));
}

/**
 *Возвращает физическую величину в угловых радианах, полученное из двоичного 14 битного числа АЦП
 */
float AS5048A::RotationRawToRadian(word DiscreteCode){
  return DiscreteCode * ((2 * PI) / float(AS5048A_ANGLE));
}

/**
 * Возвращает инкрементный и декрементный угол поворота в переменную RotationAngle в процедуру прередаються адреса переменных
 */
void AS5048A::AbsoluteAngleRotation (float *RotationAngle, float *AngleCurrent, float *AnglePrevious){

  if (*AngleCurrent != *AnglePrevious){
    //сделан круг на возрастание с 360 на 1
    if ((*AngleCurrent < 90) && (*AnglePrevious > 270) /*|| (*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193) */){
        *RotationAngle += abs(360 - abs(*AngleCurrent - *AnglePrevious));
        _reverse = true;
    }

    //сделан круг на убывание с 1 на 360
    if ((*AnglePrevious < 90) && (*AngleCurrent > 270) /*|| (*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193) */){
        *RotationAngle -= abs(360 - abs(*AngleCurrent - *AnglePrevious));
        _reverse = false;
    }

    //ход по кругу на возрастание
    if (*AngleCurrent > *AnglePrevious && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true /*|| *AngleCurrent > *AnglePrevious && ((*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193))!=true && ((*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193))!=true*/){
        *RotationAngle += abs(*AngleCurrent - *AnglePrevious);
        _reverse = true;
    }

    //ход по кругу на убывание
    if (*AnglePrevious > *AngleCurrent && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true /*|| *AnglePrevious > *AngleCurrent && ((*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193))!=true && ((*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193))!=true*/){
        *RotationAngle -= abs(*AnglePrevious - *AngleCurrent);
        _reverse = false;
    }
  }

    *AnglePrevious = *AngleCurrent;
}

float AS5048A::AbsoluteAngleRotation (float *RotationAngle, float AngleCurrent, float *AnglePrevious){

  if (AngleCurrent != *AnglePrevious){
    //сделан круг на возрастание с 360 на 1
    if ((AngleCurrent < 90) && (*AnglePrevious > 270)){
        *RotationAngle += abs(360 - abs(AngleCurrent - *AnglePrevious));
    }

    //сделан круг на убывание с 1 на 360
    if ((*AnglePrevious < 90) && (AngleCurrent > 270)){
        *RotationAngle -= abs(360 - abs(AngleCurrent - *AnglePrevious));
    }

    //ход по кругу на возрастание
    if (AngleCurrent > *AnglePrevious && ((AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (AngleCurrent > 270))!=true){
        *RotationAngle += abs(AngleCurrent - *AnglePrevious);
    }

    //ход по кругу на убывание
    if (*AnglePrevious > AngleCurrent && ((AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (AngleCurrent > 270))!=true){
        *RotationAngle -= abs(*AnglePrevious - AngleCurrent);
    }
  }

    *AnglePrevious = AngleCurrent;

    return *RotationAngle;
}

/**
*возвращает минуты угла
*/
float AS5048A::GetAngularMinutes (float AngleAbsolute){
  return ( AngleAbsolute - int(AngleAbsolute) ) * 60;

}

/**
*возвращает секунды угла
*/
float AS5048A::GetAngularSeconds (float AngleAbsolute){
  return (AS5048A::GetAngularMinutes(AngleAbsolute) - int(AS5048A::GetAngularMinutes(AngleAbsolute)) ) * 60;
}

/**
*возвращает перемещение прямозубой зубчатой рейки в мм
*WheelRotationAngle - Угол поворота колеса
*NormalModule - Модуль нормальный
*NumberGearTeeth - Число зубьев колеса или число заходов червяка
*(PI * NormalModule) - Шаг торцовый
*AngleTiltTooth Угол наклона зуба, аргументы по умолчанию 20
*/
float AS5048A::LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth){
  return WheelRotationAngle * ( ((PI * NormalModule) * NumberGearTeeth) / 360);
}

float AS5048A::LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth, float AngleTiltTooth = 20){
  return WheelRotationAngle * (( ( (PI * NormalModule) / cos(radians(AngleTiltTooth)) ) * NumberGearTeeth) / 360);
}

/**
*возвращает перемещение винтовой передачи в мм
*StepGroove - шаг резьбы винта
*ScrewRotationAngle - угол поворота винта
*/
float AS5048A::LinearMotionHelicalGear ( float ScrewRotationAngle, float StepGroove){
  return (ScrewRotationAngle * (StepGroove / 360));
}

/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 * Возвращает значение диагностического регистра датчика
 * размером 16 бит из них 13 значащих (пример 1101100110101)
 */
word AS5048A::getState(){
  return read(AS5048A_DIAG_AGC,false) & ~0xC000;
}

/**
 * Print the diagnostic register of the sensor
 * Вывести в порт Serial значение диагностического регистра датчика
 */
void AS5048A::printState(){
  word data;
  data = AS5048A::getState();
  if(AS5048A::error()){
    Serial.println("Error bit was set! (function printState register Diagnostics + Automatic Gain Control (AGC) )");
  }
  Serial.println(" ");
  Serial.println("Значение автоматического регулирования усиления магнитного поля");
  Serial.println("255 представляет собой низкое магнитное поле");
  Serial.println("0 представляет собой высокое магнитное поле");
  //Serial.println(lowByte(data), BIN);
  Serial.println(lowByte(data), DEC);

/**Диагностические функции AS5048
* AS5048 обеспечивает диагностические функции ИС, а также диагностические функции магнитного поля ввода. Доступны следующие диагностические флаги: см. Рис. 22 адрес регистра x3FFD (AS5048A) или адрес 31 адреса адреса 251 деци (AS5048B)
  * • OCF (Компенсация смещения завершена), логический максимум указывает законченный алгоритм компенсации смещения. После включения флаг всегда остается логически высоким.
  * • COF (CORDIC Overflow), логический максимум указывает на ошибку вне диапазона в части CORDIC. Когда этот бит установлен, данные угла и величины недействительны. Абсолютный выход сохраняет последнее действительное угловое значение.
  * • COMP low, указывает на высокое магнитное поле. Рекомендуется дополнительно контролировать величину величины.
  * • COMP высокий, указывает на слабое магнитное поле. Рекомендуется контролировать величину величины.
 */
  Serial.print("Флаги диагностики");
  Serial.print(" OCF-");
  Serial.print(bitRead(data,8), DEC);
  Serial.print(" COF-");
  Serial.print(bitRead(data,9), DEC);
  Serial.print(" Comp Low-");
  Serial.print(bitRead(data,10), DEC);
  Serial.print(" Comp High-");
  Serial.println(bitRead(data,11), DEC);
  Serial.println(" ");
  //Serial.println(data, BIN);
}


/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 * Возвращает значение Автоматического контроля усиления диагностического регистра
 */
byte AS5048A::getGain(){
  word data = AS5048A::getState();
  if(AS5048A::error()){
    Serial.print("Error bit was set! (function getGain register Diagnostics + Automatic Gain Control (AGC) )");
  }
  return (byte) data & 0xFF;
}

/**
 * Get and clear the error register by reading it
 * Очистить флаг ошибки и возврат три бита (0бит Framing Error, 1бит Command Invalid, 2бит Parity Error)
 * Регистр ошибок. Все ошибки очищаются путем доступа
  Возможные условия, которые заставляют установить ERROR FLAG:
  • Неверный паритет
  • Неправильное количество часов (без полного цикла передачи или слишком много часов)
  • Недействительная команда
  • Ошибка кадра
  Примечание (ы): Если флаг ошибки установлен на высокий из-за
  проблема связи, флаг остается установленным до тех пор, пока он не будет
  очищается командой CLEAR ERROR FLAG.
 */
word AS5048A::getErrors(){
  return AS5048A::read(AS5048A_CLEAR_ERROR_FLAG,false) & ~0xC000;
}

/**
 * Получить и очистка регистра ошибок и вывести значение регистра в Serial порт
 */
void AS5048A::printErrors(){
  word data;
  data = AS5048A::getErrors();
  if(AS5048A::error()){
    Serial.println("Error bit was set! (function printErrors register Clear Error Flag)");
  }
  Serial.println("Регистр ошибок");
  Serial.print("Ошибка кадра (пакета) команды ");
  Serial.println(bitRead(data,0), DEC);
  Serial.print("Неверная команда ");
  Serial.println(bitRead(data,1), DEC);
  Serial.print("Ошибка бита четности ");
  Serial.println(bitRead(data,2), DEC);
  Serial.println(" ");
  //Serial.println(data, BIN);
}

/**
 *Функция посылает команда NOP и возвращает содержимое регистра. Команда NOP представляет собой фиктивную
 *запись в регистр x0000 сенсора AS5048
 */
word AS5048A::DummyOperNoInf(){
  return AS5048A::read(AS5048A_NOP,false);
}

/**
 *Процедура записывает абсолютное значен измеренное сенсором AS5048, случайно расположенного магнита на оси вращения,
 *как нулевую позицию угла

Программирование AS5048
Программирование нулевого положения: абсолютное положение угла может быть запрограммировано по интерфейсу. Это может быть полезно для случайного размещения магнита на оси вращения. Считывание в механическом нулевом положении может быть выполнено и записано обратно в ИС. При постоянном программировании позиция не обратима, хранящаяся в ИС. Это программирование может выполняться только один раз. Чтобы упростить вычисление нулевой позиции, необходимо только записать значение в ИС, которое было зачитано ранее из регистра угла.

Последовательность программирования с проверкой: для программирования нулевой позиции необходимо выполнить следующую последовательность:
1. Запишите 0 в регистр нулевой позиции OTP, чтобы очистить.
2. Считать информацию текущего угла
3. Запишите считанное положение угла в регистр нулевой позиции OTP.

Теперь запись нулевого положение. Если вы хотите записать значение регистра OTP, отправьте:

4. Установите бит программирования (Programming Enable) чтобы записать значение регистра управления OTP.
5. Установите бит записи (Burn), чтобы запустить процедуру автоматического программирования.
6. Считайте информацию текущего угла если (равно 0) то.
7. Установите бит Verify для повторной загрузки OTP данных во внутренние регистры.
8. Считайте информацию текущего угла для проверки (равно 0).

Программирование может быть выполнено в режиме 5 В с использованием внутреннего LDO или 3V, но с минимальным напряжением питания 3,3 В. В случае работы 3 В также требуется конденсатор 10 мкФ на выводе VDD3.
 */
void AS5048A::ProgAbsolAngleZeroPosit(){
  word rotationzero = 0b0000000000000000;
  word programcontrol = 0b00000000000000;

  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, AS5048A_NOP & ~0xFF00);
  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, AS5048A_NOP & ~0xFFC0);

  rotationzero |= AS5048A::getRawRotation();

  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, (rotationzero >> 6) & 0xFF);
  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, rotationzero & 0x3F);

  AS5048A::write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol,0));
  AS5048A::write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol,3));

  if (1 < AS5048A::getRawRotation() < -1){
    AS5048A::write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol,6));
  }

  Serial.println(AS5048A::getRawRotation(), DEC);
}

/**
 * Set the zero _position
 */
void AS5048A::setZeroPosition(word Arg_Position){
  _position = Arg_Position % 0x3FFF;
}

/**
 * Returns the current zero _position
 */
word AS5048A::getZeroPosition(){
  return _position;
}

/**
 * функция для сортировки по возрастанию
 */
void AS5048A::quickSort(word *Arr, int Left, int Right) {
  int i = Left, j = Right;
  int tmp;
  word pivot = Arr[(Left + Right) / 2];

  /* partition */
  while (i <= j) {
    while (Arr[i] < pivot)
      i++;
    while (Arr[j] > pivot)
      j--;
    if (i <= j) {
      tmp = Arr[i];
      Arr[i] = Arr[j];
      Arr[j] = tmp;
      i++;
      j--;
    }
  }

  /* recursion */
  if (Left < j)
    quickSort(Arr, Left, j);
  if (i < Right)
    quickSort(Arr, i, Right);
}

/**
 * Check if an error has been encountered.
 * Флаг ошибки, указывающий на ошибку передачи в предыдущей передаче ведущего устройства (Master)
 */
bool AS5048A::error(){
  return _errorFlag;
}

/**
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 * Отправка команды на чтения регистров сенсора AS5048A
 * MeanValueMedian включить режим медианного среднего
 * NumberFunctionValues количество измерений для режима медианного среднего
 */
word AS5048A::read(word RegisterAddress, bool MeanValueMedian, byte NumberFunctionValues){
  word readdata;
  if (NumberFunctionValues <= 0){
    NumberFunctionValues = 1;
  }
  word array_data[NumberFunctionValues];
  word command = 0b0100000000000000; // PAR=0 R/W=R

  command |= RegisterAddress;

  //Add a parity bit on the the MSB
  command |= ((word)spiCalcEvenParity(command)<<15);

  //SPI - begin transaction
  SPI.beginTransaction(settings);

  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  #ifdef AS5048A_DEBUG
    Serial.print("Read (0x");
    Serial.print(RegisterAddress, HEX);
    Serial.print(") with command: 0b");
    Serial.println(command, BIN);
  #endif

  //Send the command and Now read the response
  if (MeanValueMedian == true){

    for ( byte i = 0; i < NumberFunctionValues-1; i++){ //(sizeof(array_data) / sizeof(array_data[0]))
      digitalWrite(_cs, LOW);
      array_data[i] = SPI.transfer16(command) & ~0xC000;
      digitalWrite(_cs, HIGH);
      //Serial.println(array_data[i], BIN);
    }

    SPI.endTransaction();
    //SPI - end transaction

    quickSort(array_data, 0, NumberFunctionValues-1 );
    if (NumberFunctionValues > 1){
      if((NumberFunctionValues % 2) > 0 ){
        readdata = (array_data[(NumberFunctionValues / 2)-1] + array_data[NumberFunctionValues / 2]  + array_data[(NumberFunctionValues / 2)+1]  ) / 3 ;
      }else{
        readdata = ( array_data[(NumberFunctionValues / 2)-1]  + array_data[NumberFunctionValues / 2] ) / 2 ;
      }
    }else{
      readdata = array_data[0] ;
    }


    //Return the data, stripping the parity and error bits
    return readdata;
  }else{
    digitalWrite(_cs, LOW);
    readdata = SPI.transfer16(command);
    digitalWrite(_cs, HIGH);

    #ifdef AS5048A_DEBUG
      Serial.print("Read returned: ");
      Serial.print(highByte(readdata), BIN);
      Serial.print(lowByte(readdata), BIN);
    #endif

    //Check if the error bit is set
    //Если в 15 бите установлена 1 (ошибка передачи в предыдущей передаче ведущего устройства) _errorFlag установить 1 иначе 0
    if (bitRead(readdata,14)) {
      #ifdef AS5048A_DEBUG
        Serial.println("Setting error bit");
      #endif
      _errorFlag = true;
    }else {
      _errorFlag = false;
    }

    SPI.endTransaction();
    //SPI - end transaction

    //Return the data, stripping the parity and error bits
    return readdata & ~0xC000;
  }

}


/**
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of WriteData
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
word AS5048A::write(word RegisterAddress, word WriteData) {
  word command = 0b0000000000000000; // PAR=0 R/W=W
  word dataToSend = 0b0000000000000000;

  command |= RegisterAddress;
  dataToSend |= WriteData;

  //Add a parity bit on the the MSB
  command |= ((word)spiCalcEvenParity(command) << 15);

  //Craft another packet including the data and parity
  dataToSend |= ((word)spiCalcEvenParity(dataToSend) << 15);

#ifdef AS5048A_DEBUG
  Serial.print("Write (0x");
  Serial.print(RegisterAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif

  //SPI - begin transaction
  SPI.beginTransaction(settings);

  //Start the write command with the target address
  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);

#ifdef AS5048A_DEBUG
  Serial.print("Sending data to write: ");
  Serial.println(dataToSend, BIN);
#endif

  //Now send the data packet
  digitalWrite(_cs, LOW);
  SPI.transfer16(dataToSend);
  digitalWrite(_cs, HIGH);

  //Send a NOP to get the new data in the register
  digitalWrite(_cs, LOW);
  dataToSend = SPI.transfer16(AS5048A_NOP);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  //SPI - end transaction
  SPI.endTransaction();

  //Return the data, stripping the parity and error bits
  return dataToSend & ~0xC000;
}
