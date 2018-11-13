#include "Arduino.h"

#include <AS5048A.h> 

//#define AS5048A_DEBUG

const int AS5048A_NOP             				= 0x0000; // Фиктивная операция, нет информации.
const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001; //Регистр ошибок. Все ошибки очищаются путем доступа.
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003; //Регистр управления программированием. Программирование должно быть включено до прожига памяти. Перед программированием проверка является обязательной. См. Процедуру программирования.
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016; //Нулевое значение с высоким байтом
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017; //Нулевая позиция остается 6 младших младших разрядов
const int AS5048A_DIAG_AGC                      = 0x3FFD; //(0-7)Значение автоматического регулирования усиления. 0 десятичной представляет высокое магнитное поле, 255 десятичных представляет низкое магнитное поле. (8-13)Флаги диагностики
const int AS5048A_MAGNITUDE                     = 0x3FFE; //Значение выходной мощности CORDIC 
const int AS5048A_ANGLE                         = 0x3FFF; //Угловое выходное значение, включая коррекцию нулевой позиции Resolution_ADC 14-bit resolution (0.0219°/LSB)

/**
 * Constructor
 * Иницмализация библиотеки AS5048A
 */
AS5048A::AS5048A(byte arg_cs){
	_cs = arg_cs;
	errorFlag = false; 
	position = 0;
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
	* dataOrder - порядок вывода даннах в/из шины SPI,  может быть LSBFIRST (наименьший разряд(бит) первый) или MSBFIRST (старший разряд первый)
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
byte AS5048A::spiCalcEvenParity(word value){
	byte cnt = 0;
	byte i;
	for (i = 0; i < 15; i++)
	{
	   if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
	
	//byte operand_compare =  bitRead(value,0);
	//byte i = 1;
	//do{
	//	operand_compare ^= bitRead(value,i);
	//} while ((i++) < 14);
	//return operand_compare & 0x1;
}



/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */
int AS5048A::getRotation(){
	word data;
	int rotation;
	data = AS5048A::getRawRotation();
	rotation = (int)data - (int)position;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
	//if(rotation < -0x1FFF) rotation = rotation+0x3FFF;
	return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 * Возвращает уголовое двоичное 14 битное угловое значение (DEC 16383)
 * Угловое выходное значение, включая коррекцию нулевой позиции.
 */
word AS5048A::getRawRotation(bool EnableMedianValue){
	return AS5048A::read(AS5048A_ANGLE, EnableMedianValue);
}

/**
 *Возвращает физическую величину в угловых градусах, полученное из двоичного 14 битного числа АЦП
 */
float AS5048A::RotationRawToAngle (word DiscreteCode){
	return DiscreteCode * (360.0 / float(AS5048A_ANGLE));
}

/**
 * Возвращает инкрементный и декрементный угол поворота в переменную RotationAngle в процедуру прередаються адреса переменных 
 */
void AS5048A::AbsoluteAngleRotation (float *RotationAngle, float *AngleCurrent, float *AnglePrevious){

	if (*AngleCurrent != *AnglePrevious){
		//сделан круг на возростание с 360 на 1
        if ((*AngleCurrent < 90) && (*AnglePrevious > 270) || 
		(*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193) ){
            *RotationAngle += abs(360 - abs(*AngleCurrent - *AnglePrevious));
			reverse = true;
		}  
		//сделан круг на убывание с 1 на 360
        if ((*AnglePrevious < 90) && (*AngleCurrent > 270) || 
		(*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193) ){
            *RotationAngle -= abs(360 - abs(*AngleCurrent - *AnglePrevious));
			reverse = false;
		}
        //ход по кругу на возростание
        if (*AngleCurrent > *AnglePrevious && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true ||
		*AngleCurrent > *AnglePrevious && ((*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193))!=true && ((*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193))!=true){
            *RotationAngle += abs(*AngleCurrent - *AnglePrevious);
			reverse = true;
		} 
        //ход по кругу на убывание
        if (*AnglePrevious > *AngleCurrent && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true ||
		*AnglePrevious > *AngleCurrent && ((*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193))!=true && ((*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193))!=true){
            *RotationAngle -= abs(*AnglePrevious - *AngleCurrent);
			reverse = false;
		}		
	}

    *AnglePrevious = *AngleCurrent;		
}

/**
*возвращает минуты угла
*/
float AS5048A::GetAngularMinutes (float AngleAbsolute){
	return ( AngleAbsolute - int(AngleAbsolute) ) *60;
}

/**
*возвращает секунды угла
*/
float AS5048A::GetAngularSeconds (float AngleAbsolute){
	return (AS5048A::GetAngularMinutes(AngleAbsolute) - int(AS5048A::GetAngularMinutes(AngleAbsolute)) ) * 60;
}

/**
*возвращает перемещение прямозубой зубчатой рекйки в мм
*WheelRotationAngle - Угол поворота колеса
*NormalModule - Модуль нормальный
*NumberGearTeeth - Число зубьев колеса или число заходов червяка
*(PI * NormalModule) - Шаг торцовый
*20 - Угол наклона зуба
*/ 
float AS5048A::LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth){	 
	return (WheelRotationAngle * ( (PI * NormalModule) / cos(20) ) * NumberGearTeeth) / 360;
}

/**
*возвращает перемещение винтовой предачи в мм
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
 * размером 16 бит из них 13 значищих (пример 1101100110101)
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
	Serial.println("Значение автоматического регулирования усиления манитного поля");
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
 * Возвращает значение Автоматического контроля усления диагностического регистра
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
	очищается командой CLERAR ERROR FLAG.
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
	Serial.print("Ошибка кадра (пакета) комманды ");
	Serial.println(bitRead(data,0), DEC);
	Serial.print("Неверная команда ");
	Serial.println(bitRead(data,1), DEC);
	Serial.print("Ошибка бита четности ");
	Serial.println(bitRead(data,2), DEC);
	Serial.println(" ");
	//Serial.println(data, BIN);
}	

/**
 * Процидура посылает команда NOP. Команда NOP представляет собой фиктивную запись в регитр x0000 сенсора AS5048
 */ 
void AS5048A::DummyOperNoInf(){
	Serial.println(read(AS5048A_NOP,false), DEC); 	
}


Программирование AS5048
Программирование нулевого положения: абсолютное положение угла может быть запрограммировано по интерфейсу. Это может быть полезно для случайного размещения магнита на оси вращения. Считывание в механическом нулевом положении может быть выполнено и записано обратно в ИС. При постоянном программировании позиция не обратима, хранящаяся в ИС. Это программирование может выполняться только один раз.

Чтобы упростить вычисление нулевой позиции, необходимо только записать значение в ИС, которое было зачитано ранее из регистра угла.

Последовательность программирования с проверкой: для программирования нулевой позиции необходимо выполнить следующую последовательность:
1. Запишите 0 в регистр нулевой позиции OTP, чтобы очистить;
2. Информация о считываемом углу
3. Запишите предыдущее положение угла считывания в ноль OTP
регистр позиции
Теперь устанавливается нулевое положение.
Если вы хотите записать его в регистр OTP, отправьте:
4. Установите бит «Включить программирование» в элементе управления OTP
регистр
5. Установите бит записи, чтобы начать автоматическое программирование.
процедура
6. Информация о ракурсе считывания (равна 0)
7. Установите бит Verify для повторной загрузки данных OTP в
внутренние регистры
8. Информация о углу обзора (равна 0)

Программирование может быть выполнено в режиме 5 В с использованием внутреннего LDO или 3V, но с минимальным напряжением питания 3,3 В. В случае работы 3 В также требуется конденсатор 10 мкФ на выводе VDD3.
Программирование адреса ведомого устройства I?C. Информацию о программировании адреса подчиненного устройства I?C см. В нашем приложении, посвященном этой теме.

/**
 * Set the zero position
 */
void AS5048A::setZeroPosition(word arg_position){
	position = arg_position % 0x3FFF;
}

/**
 * Returns the current zero position
 */
word AS5048A::getZeroPosition(){
	return position;
}

/**
 * функция для сортировки по возрастанию
 */
void AS5048A::quickSort(word *arr, int left, int right) { 
	int i = left, j = right; 
	int tmp; 
	word pivot = arr[(left + right) / 2]; 

	/* partition */ 
	while (i <= j) { 
		while (arr[i] < pivot) 
			i++; 
		while (arr[j] > pivot) 
			j--; 
		if (i <= j) { 
			tmp = arr[i]; 
			arr[i] = arr[j]; 
			arr[j] = tmp; 
			i++; 
			j--; 
		} 
	} 

	/* recursion */ 
	if (left < j) 
		quickSort(arr, left, j); 
	if (i < right) 
		quickSort(arr, i, right); 
}

/**
 * Check if an error has been encountered.
 * Флаг ошибки, указывающий на ошибку передачи в предыдущей передаче ведущего устройства (Master)
 */
bool AS5048A::error(){
	return errorFlag;
}

/**
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 * Отправка комманды на чтения регистров датчика
 */
word AS5048A::read(word registerAddress, bool MeaValueMedian){
	word buffer;
	word array_buffer[16];
	word command = 0b0100000000000000; // PAR=0 R/W=R
	
	command = command | registerAddress;
	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);
	
	SPI.beginTransaction(settings);
	digitalWrite(_cs, LOW);
	SPI.transfer16(command);
	digitalWrite(_cs, HIGH);
	SPI.endTransaction();

#ifdef AS5048A_DEBUG
	Serial.print("Read (0x");
	Serial.print(registerAddress, HEX);
	Serial.print(") with command: 0b");
	Serial.println(command, BIN);
#endif
	
	//Send the command and Now read the response
	if (MeaValueMedian == true){
	
		//SPI - begin transaction
		SPI.beginTransaction(settings);
		for ( byte i = 0; i < 16; i++){
			digitalWrite(_cs, LOW);
			array_buffer[i] = SPI.transfer16(command) & ~0xC000;
			digitalWrite(_cs, HIGH);
			//Serial.println(array_buffer[i], BIN);		
		}
		SPI.endTransaction();
		//SPI - end transaction

		quickSort(array_buffer, 0, 15);
		buffer = ( array_buffer[8]  + array_buffer[9]  ) / 2 ;	
		
		//Return the data, stripping the parity and error bits
		return buffer;	
	}else{
		//SPI - begin transaction
		SPI.beginTransaction(settings);
		digitalWrite(_cs, LOW);
		buffer = SPI.transfer16(command);
		digitalWrite(_cs, HIGH);
		SPI.endTransaction();
		//SPI - end transaction
		
#ifdef AS5048A_DEBUG
	Serial.print("Read returned: ");
	Serial.print(highByte(buffer), BIN);
	Serial.print(lowByte(buffer), BIN);
#endif

		//Check if the error bit is set
		//Если в 15 бите установлена 1 (ошибка передачи в предыдущей передаче ведущего устройства) errorFlag установить 1 иначе 0
		if (bitRead(buffer,14)) {
#ifdef AS5048A_DEBUG
	Serial.println("Setting error bit");
#endif
			errorFlag = true;
		}else {
			errorFlag = false;
		}

	//Return the data, stripping the parity and error bits
	return buffer & ~0xC000;
	}
}


/**
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
word AS5048A::write(word registerAddress, word data) {

	word command = 0b0000000000000000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	byte right_byte = command & 0xFF;
	byte left_byte = ( command >> 8 ) & 0xFF;

#ifdef AS5048A_DEBUG
	Serial.print("Write (0x");
	Serial.print(registerAddress, HEX);
	Serial.print(") with command: 0b");
	Serial.println(command, BIN);
#endif

	//SPI - begin transaction
	SPI.beginTransaction(settings);

	//Start the write command with the target address
	digitalWrite(_cs, LOW);
	SPI.transfer(left_byte);
	SPI.transfer(right_byte);
	digitalWrite(_cs,HIGH);
	
	word dataToSend = 0b0000000000000000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= ((word)spiCalcEvenParity(dataToSend)<<15);
	right_byte = dataToSend & 0xFF;
	left_byte = ( dataToSend >> 8 ) & 0xFF;

#ifdef AS5048A_DEBUG
	Serial.print("Sending data to write: ");
	Serial.println(dataToSend, BIN);
#endif

	//Now send the data packet
	digitalWrite(_cs,LOW);
	SPI.transfer(left_byte);
	SPI.transfer(right_byte);
	digitalWrite(_cs,HIGH);
	
	//Send a NOP to get the new data in the register
	digitalWrite(_cs, LOW);
	left_byte =-SPI.transfer(0x00); // - ?
	right_byte = SPI.transfer(0x00);
	digitalWrite(_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	//Return the data, stripping the parity and error bits
	return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
	}
