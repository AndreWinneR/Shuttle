
/* сделать
 *  
 * кнопка включения вентияора, кнопка отключения автоматического управления (к4)?
 * переключение  на новую печку: сменить пи хай-лоу
 * проверка десятикратного уменьшения времени сушки при чтении параметров
 * проверка работы таймера при r2
 * интерфейс взаимодействия с ПК (через монитор порта)
 * 
 * 
*/

/*оглавление кода

1 введение
2 создание объектов и констант
 2.1...
3 исполняемый код
4 процедуры и функции
  4.1 процедуры движения
  4.2 описание методов класса кнопка
  4.3 функции общения с термодатом
  4.4 внутренние функции
  4.5 функции работы с картой


параметры:
0 (Td)    Температура сушки   '
1 (Dry)   Время сушки         ''
2 (Ta)    Температура отжига  -_-
3 (Annel) Время отжига        ||
4 (Vd)    Скорость сушки      U
5 (Va)    Скорость отжига     U-
6 (C)     Количество циклов   C


*/

// библиотеки
#include <Arduino.h>            // библиотека Ардуино
#include <RBD_Timer.h>          // библиотека таймера
#include <Keypad.h>             // библиотека клавиатуры
#include <SD.h>                 // библиотека картридера
#include <SPI.h>                // библиотека интерфейса
#include <TM1637Display.h>      // библиотека дисплея
#include <Servo.h>              // библиотека сервопривода
Servo revolver;                 // сервопривод
File dataFile;                  // файл лога
RBD::Timer timer;               // таймер нанесения
RBD::Timer timer2;              // таймер сушки
RBD::Timer timer3;              // таймер отжига

// подключение клавиатуры, создание объекта класса
const byte ROWS = 4;            // строки
const byte COLS = 4;            // столбцы
char hexaKeys[ROWS][COLS] = {   // значения
  {'1','2','3','a'},
  {'4','5','6','b'},
  {'7','8','9','c'},
  {'f','0','e','d'}
};
byte rowPins[ROWS] = {24, 25, 26, 27};        // первые 4 пина клавы
byte colPins[COLS] = {28, 29, 30, 31};        // вторые 4 пина клавы
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 


// создание массива параметров
char l;
int param[9][5]={{1,0,0,0,0},{2,0,0,0,0},{3,0,0,0,0},{4,0,0,0,0},{5,0,0,0,0},{6,0,0,0,0},{7,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
int getparam[7];
byte flag[]={0,0,0,0,0,0,0};
byte flagstch=0;      // флаг записи уровней
byte flagsd=0;          // флаг картридера
byte antipar=0;         // флаг считывания условий
uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };    // массив яркости сегментов

// пины
byte pinpul=22;       // контакт частотной модулции
byte pindir=23;       // контакт направления
byte pinskup=33;      // сервисная кнопка вверх
byte pinskdown=32;    // сервисная кнопка вниз
byte pinskdo=34;      // сервисная кнопка действия  
byte pinreset=35;     // пин перезагрузки
byte pinclk=36;       // жёлтый провод дисплея
byte pindio=37;       // оранжевый провод дисплея
byte pinskr=38;       // кнопка выбора режима
byte pinmax=39;       // переходник на RS485
byte pinrele1=40;     // пин реле 1 - вентилятор
byte pinrele2=41;     // пин реле 2
byte pinrele3=42;     // пин реле 3
byte pinrele4=43;     // пин реле 4
byte pinservo=44;     // сервопривод
byte pinsd=53;        // картридер
                      // 19 и 18 пины зарезервированы под serial1 как rx и tx
                      // 21 и 20 пины зарезервированы для прерываний 
                      // 50 - MISO, 51 - MOSI, SCK - 52


// числовые
int tvh=800;          // время импульса для максимальной скорости 
byte Vd;              // заданный делитель скорости сушки 
byte Va;              // заданный делитель скорости отжига
int tv=tvh;           // расчитанное время импульса для максимальной скорости
int tv_d=tvh;         // расчитанное время импульса для скорости сушки
int tv_a=tvh;         // расчитанное время импульса для скорости отжига
int i=0;              // счётчик шагов
int j=0;              // счётчик циклов
int Td;               // уставка температуры сушки 
int Ta;               // уставка температуры отжига
int dT=20;            // отклонение уставки
int P[]={1000,0,0,0}; // уровни
int UUU[]={0,0,0,0,0,0,0,0}; // условия
byte Pi_Hi;           // мощность макс.
byte Pi_Low;          // мощность мин.
byte C;               // количество циклов
int lognumber;        // номер лога
long Dry;             // время сушки 
long Annel;           // время отжига
unsigned long Mil;    // таймер

// текстовые
String datacarry;     // буфер чтения файлов
String logname;       // имя файла данных
String stp;           // буфер шагов

// пртокол MODBUS
String adres = "01";        // адрес термодата
char answer[100];           // буфер ответа термодата
long temp1;                 // буфер температуры 1 канала
long temp2;                 // буфер температуры 2 канала

// пороговые параметры
// Pi_Hi от 5 до 20 %
const int Tmin=50;
const int Tmax=900;
const byte Dry_min=0;
const byte Dry_max=600;
const byte Annel_min=0;
const byte Annel_max=600;
const byte Cmin=1;
const byte Cmax=20;
const byte Vmin=1;
const byte Vmax=100;

// логические
const boolean motorup = LOW;
const boolean motordown = HIGH;             // направление (LOW - вверх, HIGH - вниз)
volatile boolean dir;                       // прерывания типа falling

// подключение дисплея
TM1637Display display(pinclk,pindio);   

// подключение кнопок
class Button {                              // класс - кнопка
  public:
    Button(byte pin, byte tau);             // пин и время стабилизации
    boolean ScanChange ();
    boolean Scan ();
  private:
    byte _pin;
    byte _tau;
    boolean _state;
    boolean _laststate;
};
Button skdo (pinskdo,5);                    // создание объектов класса кнопка
Button skup (pinskup,5);
Button skdown (pinskdown,5);
Button skr (pinskr,5);


void setup() {
if (1){                                     // инициализация
  digitalWrite(pinreset, HIGH);             // определение пинов
  pinMode(pinpul,OUTPUT);
  pinMode(pindir,OUTPUT);
  pinMode(pinreset,OUTPUT);
  pinMode(pinsd, OUTPUT);
  pinMode(pinmax, OUTPUT);
  pinMode(pinrele1, OUTPUT);
  pinMode(pinrele2, OUTPUT);
  pinMode(pinrele3, OUTPUT);
  pinMode(pinrele4, OUTPUT);
  digitalWrite(pinmax,LOW);
  digitalWrite(pinrele1,HIGH);
  digitalWrite(pinrele2,HIGH);
  digitalWrite(pinrele3,HIGH);
  digitalWrite(pinrele4,HIGH);
  revolver.attach(pinservo);               // подклчение сервопривода(revolver.write(random(0,180));)
  Serial1.begin(115200);                   // открытие порта термодата, скорость 115200 бод
  Serial.begin(9600);                      // открытие последовательного соединения
  delay(100);
  Serial.println("START DEBUGGING");       // начало отладки
  cardbegin();
  delay(100);
  // обнуление внутренних переменных объектов класса кнопка                                  
  skdo.Scan();                             
  skdo.Scan();
  skup.Scan();
  skup.Scan();
  skdown.Scan();
  skdown.Scan();
  skr.Scan();
  skr.Scan();
  display.setBrightness(0x0f);             // яркость от 0 до 7  
  display.setSegments(data);               // включить дисплей
  // начало управляемого цикла
  data[0] = 0b01101101;                    
  data[1] = 0b00111111;
  data[2] = 0b01101101;
  data[3] = 0b00000000;
  while ((skup.Scan())||(skdown.Scan())) {  // проверка начального положения переключателей
  
    display.setSegments(data);
    delay(500);
  }
}
if (1){                                     // выбор режима, работа в режиме 1
  data[0] = 0b01010000;              
  data[2] = 0b00000000;
  data[3] = 0b00000000;
  if (skr.Scan()) data[1] = 0b01011011;
  else data[1] = 0b00000110;
  display.setSegments(data);               // дисплей: r1/r2
  delay(500);
  while (!skdo.ScanChange()) {             
    if (skr.Scan()) data[1] = 0b01011011;
    else data[1] = 0b00000110;
    display.setSegments(data);
    delay(500);
  }
  if (skr.Scan()&&(flagsd==1)) goto regime2; // переход на второй режим
  
  data[2]=0b00001000;
  data[3]=0b00000110;      
  display.setSegments(data);               // дисплей: r1_1
  startreturn();                           // выведение в начальное положение
  data[2]=0b00011100;
  for (int t=1; t<4; t++) {                // установка уровней
    data[3]=display.encodeDigit(t);         
    display.setSegments(data);             // дисплей r1ut
    while (!skdo.ScanChange()) {
      if ((skup.Scan())&&(digitalRead(20))) {
        dir=motorup;
        digitalWrite (pindir, dir);
        onestep(tvh);
      }
      if ((skdown.Scan())&&(digitalRead(21))) {
        dir = motordown;
        digitalWrite (pindir, dir);
        onestep(tvh);
      }
    }
    P[t] = i;                              // запись уровня
    dir = motorup;
    digitalWrite (pindir, dir);
  }
  data[2]=0b00001000;
  data[3]=0b01011011;      
  display.setSegments(data);               // дисплей: r1_2
  swmaster();                              // запись на карту
  flagstch=1;
  while (!skdo.ScanChange()) delay(300);   // ожидание
}  
if (1){                                     // начало второго режима
  regime2:                                 
  data[1] = 0b01011011;
  data[2] = 0b00000000;                     // выбор ввода данных
  if (skr.Scan()) data[3] = 0b01101101;
  else data[3] = 0b01010100;
  display.setSegments(data);                // дисплей: r2 S/r2 n
  delay(500);
  while (!skdo.ScanChange()) {             
    if (skr.Scan()) data[3] = 0b01101101;
    else data[3] = 0b01010100;
    display.setSegments(data);
    delay(500);
  }
  if (skr.Scan()&&(flagsd==1)) {            // считывание условий
    urmaster();
    antipar=1;
  }
  //for (int i=1; i<8;i++) Serial.println(UUU[i]); // проверка считанных параметров
  data[1]=0b01011011;
  data[2]=0b00001000;
  data[3]=0b00000110;
  display.setSegments(data);               // дисплей: r2_1
  srmaster();                              // считывание уровней
  logmaster();                             // имя лога 
  if (flagstch==0) startreturn();          // возврат в начало если не было задания уровней
  delay(100);
  dir=motorup;
  digitalWrite(pindir,dir);
  //attachInterrupt(2,reverse,FALLING);      
  //attachInterrupt(3,reverse,FALLING);    // включение разворота при прерываниях
  dir=motorup;
  digitalWrite(pindir,dir);
  /*Serial.println(i);
  Serial.println(P[1]);
  Serial.println(P[2]);
  Serial.println(P[3]);*/
  while (i<P[2]) onestep(tvh);             // проход до уровня 2 для закрепления подложки
    
  // регулировка (только в режиме r2r2)
  data[2]=0b01010000;
  if (skr.Scan()) data[3]=0b01011011;
  else data[3]=0b00000110;
  display.setSegments(data);                             // дисплей: r2(r1/r2)
  while (!skdo.ScanChange()) {                           // выбор режима 
    if (skr.Scan()) data[3] = 0b01011011;
    else data[3] = 0b00000110;
    display.setSegments(data);
    delay(500);
  }
  if (skr.Scan()) {                                      // проверка на вхождение в кювету
    while (!skdo.ScanChange()) {
        if ((skup.Scan())&&(digitalRead(20))) {
          dir=motorup;
          digitalWrite (pindir, dir);
          onestep(tvh);
        }
        if ((skdown.Scan())&&(digitalRead(21))) {
          dir = motordown;
          digitalWrite (pindir, dir);
          onestep(tvh);
        }
    }
    
  }
  dir=motordown;                                       // возвращение на 2 уровень
  digitalWrite (pindir, dir);
  while (i>P[2]) onestep(tvh);    
  dir=motorup;
  digitalWrite (pindir, dir);
  while (i<P[2]) onestep(tvh);                        // направление - вверх                                                       
}
if (1){                                     // начало работы с параметрами синтеза
  if (antipar==0) {                         // в случае введения новых параметров 
    data[2]=0b00010000;
    data[3]=0b01010100;
    display.setSegments(data);                           // дисплей: r2in
    parins();                                            // введение параметров с клавиатуры
    dopus();                                             // проверка параметров на допустимость
  }
  uwmaster();
  data[0]=0b01010000;                                    // дисплей r2_2
  data[1]=0b01011011;
  data[2]=0b00001000;
  data[3]=0b01011011;
  display.setSegments(data);
  while (!skdo.ScanChange()) delay(300);                 // ожидание
  startlog();                                            // шапка лога
  timer.setTimeout(5000);                                // таймеры
  timer.restart();
  timer2.setTimeout(Dry);
  timer2.restart();
  timer3.setTimeout(Annel);
  timer3.restart();
  Mil=millis();
}  
if (1){                                     // подготовка и нанесение
  delay(500);
  tempread();
  data[3]=0b01110011;
  display.setSegments(data);                // дисплей: r2_p
  stagechange("Heating");
  termodat(Td);
  data[1]=0b01111001;                       
  data[2]=0b01011110;
  data[3]=0b00000000;
  display.setSegments(data);                // дисплей: rEd
  while (!skdo.ScanChange()) {              // начало нанесения
    interr();
    delay(5000);
  }
  cy();
}
if (1){                                     // выключение 
  stagechange("Ending");
  termodat(50);
  while (!skdo.ScanChange()) delay(500);
  startreturn();
  data[0]=0b01111001;
  data[1]=0b01010100;                       
  data[2]=0b01011110;
  data[3]=0b00000000;
  display.setSegments(data);                // дисплей: End 
  detachInterrupt(2);
  detachInterrupt(3);                                
  
  // перезагрузка?
}
}
void loop(){}                               // пустой повторяющийся цикл
// процедуры движения
////////////////////////////////////////////////////////////////////
void onestep (int tv) {                     // подпрограмма одного шага
  digitalWrite(pinpul, HIGH);  
  delayMicroseconds(tv);                  
  digitalWrite(pinpul, LOW);    
  delayMicroseconds(tv);
  if (!dir) i++;
  else i--;
}              
/*void onestep (int tv) {                   // альтернативная подпрограмма одного шага
  digitalWrite(pinpul, HIGH);  
  delayMicroseconds(tv);
  if (!dir) i++;                  
  digitalWrite(pinpul, LOW);    
  delayMicroseconds(tv);
  if (dir) i-;
}   */           
void startreturn(){                         // выведение в начальное положение
  dir = motordown;                         
  digitalWrite (pindir, dir);
  int Pol=digitalRead(21);
  while(Pol) {
    onestep(tvh);
    Pol=digitalRead(21);
  }
  dir = motorup;                           
  digitalWrite (pindir, dir);
  i=0;
  delay(100);
  while (i<P[0]) onestep(tvh) ;
  //for (i=0; i<P[0];) onestep(tvh);
}
void cy() {                                 // полное нанесение
  data[0]=0b00111001;
  for (int k=1;k<C+1;k++) {                 // цикл нанесения 
      int pt1=k%10;
      int pt2=int((k-pt1)/10);
      data[1]=0b01011110;
      data[2]=display.encodeDigit(pt2);
      data[3]=display.encodeDigit(pt1);
      display.setSegments(data);            // дисплей: CDNN 
      String comaa = "Cycle ";
      comaa+=String(k);
      stagechange(comaa);
      stagechange("Drying");
      while (i<P[3]) {                      // поднять до уровня 3
        // отмена записи времени - окунание //if(timer.onRestart()) interr();
        onestep(tv_d);                        
      }
      reverse();                            // разворот 
      while (i>P[2]) {                      // опустить до уровня 2 (вытянуть медленно)
        //отмена записи времени - вытягивание //if(timer.onRestart()) interr();
        onestep(tv_d);                        
      }
      while (i>P[1]) {                      // опустить до уровня 1 (сушка)
        if(timer.onRestart()) interr();
        onestep(tvh*2);                        
      }
      timer2.restart();
      while (!timer2.onRestart()) {         // таймер сушки
        interr();
        delay(5000);
      }
      reverse();                            // разворот 
      while (i<P[2]) {                      // поднять до уровня 2
        if(timer.onRestart()) interr();
        onestep(tvh*2);                        
      }
      data[1]=0b01110111;
      display.setSegments(data);            // дисплей: CANN 
      stagechange("Heating");
      termodat(Ta);                         // нагрев для отжига
      stagechange("Anneling");
      reverse();                            // разворот 
      while (i>P[1]) {                      // опустить до уровня 1 (отжиг)
        if(timer.onRestart()) interr();
        onestep(tv_a);                        
      }
      timer3.restart();
      while (!timer3.onRestart()) {         // таймер отжига
        interr();
        delay(5000);
      }
      reverse();                            // разворот 
      while (i<P[2]) {                      // поднять до уровня 2
        if(timer.onRestart()) interr();
        onestep(tvh*2);                        
      }
      stagechange("Cooling");
      cooler("on");
      termodat(Td);                         // охлаждение для сушки
      cooler("off");
    }
  data[0]=0b00110000;
  data[1]=0b01011110;                       
  data[2]=0b00000100;
  data[3]=0b01011110;
  display.setSegments(data);                // дисплей: I did    
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
// описание методов класса кнопки
////////////////////////////////////////////////////////////////////           
Button::Button(byte pin, byte tau){          // конструктор
  _pin=pin;
  _tau=tau;
  pinMode(_pin,INPUT);
}
boolean Button::ScanChange(){                // сканирование изменения нажатия
  _state=digitalRead(_pin);
  if ((_laststate)!=(_state)) {
    _laststate=_state;
    return true;
  }
  else return false;
}
boolean Button::Scan(){                      // сканирование состояния кнопки
  _laststate=_state;
  _state=digitalRead(_pin);
  return _state;
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
// функции общения с термодатом
////////////////////////////////////////////////////////////////////
unsigned char LRC (String command) {      // подсчёт контрольной суммы
  int len=command.length()/2;
  String buf;
  unsigned char x;
  unsigned char sum=0;
  for (int i=0;i<len;i++) {
    buf=command.substring(i*2,i*2+2);
    x = strtoul(buf.c_str(), NULL, 16);
    sum+=x;
  }
  sum=0xFF-sum+1;
  return sum;
}
void transmit (String text) {             // передача данных на термодат
  digitalWrite(pinmax, HIGH);
  delayMicroseconds(500);
  text=adres+text;
  int len=text.length();
  String buf;
  unsigned char x[100];                    
  String control = String(LRC(text), HEX);
  if (LRC(text)<0xF) control="0"+control;
  text+=control;
  for (int i=0; i<len+2; i++){
    buf=text.substring(i,i+1);
    if (buf == "0") x[i+1]=48;
    if (buf == "1") x[i+1]=49;
    if (buf == "2") x[i+1]=50;
    if (buf == "3") x[i+1]=51;
    if (buf == "4") x[i+1]=52;
    if (buf == "5") x[i+1]=53;
    if (buf == "6") x[i+1]=54;
    if (buf == "7") x[i+1]=55;
    if (buf == "8") x[i+1]=56;
    if (buf == "9") x[i+1]=57;
    if ((buf == "A")||(buf == "a")) x[i+1]=65;
    if ((buf == "B")||(buf == "b")) x[i+1]=66;
    if ((buf == "C")||(buf == "c")) x[i+1]=67;
    if ((buf == "D")||(buf == "d")) x[i+1]=68;
    if ((buf == "E")||(buf == "e")) x[i+1]=69;
    if ((buf == "F")||(buf == "f")) x[i+1]=70;
  }
  x[0]=58;
  x[len+3]=13;
  x[len+4]=10;
  for (int i=0; i<len+5; i++) {
    Serial1.write(x[i]);
    delayMicroseconds(100);
  }
  digitalWrite(pinmax, LOW);
  delayMicroseconds(20);
}
void recieve () {                         // приём данных
  int t=0;
  delayMicroseconds(20);
  while (Serial1.available()) {
    answer[t]=Serial1.read();
    delayMicroseconds(100);
    t++;
  }
  delayMicroseconds(50);
  for (int i=0; i<t; i++) Serial.write(answer[i]);
}
void tempread () {                        // считывание температуры
  volatile int j=0;
  volatile String tbuf1,tbuf2;
  String command = "0300000002";
  for (int i=0; i<80; i++) answer[i]="";
  transmit(command);                      
  recieve();
  while (answer[j]!=58) j++;              // поиск нужного фрагмента отчёта
  while (answer[j+4]!=51){
    while (answer[j]!=58) j++;
    answer[j]="";
  }
  for (int i=j+7; i<j+11; i++) tbuf1+=String(answer[i]);
  for (int i=j+11; i<j+15; i++) tbuf2+=String(answer[i]);
  
  temp1 = strtoul(tbuf1.c_str(), NULL, 16);
  if (temp1>32767) temp1= (-(temp1-32768));
  temp2 = strtoul(tbuf2.c_str(), NULL, 16);
  if (temp2>32767) temp2= (-(temp2-32768)); 
}
void ustav (int temper) {                 // изменение уставки
  long tbuf;
  String te;
  tbuf=temper*10;
  if (tbuf<0) tbuf=-tbuf+32768;
  String command = "0601730000";
  te=String(tbuf,HEX);
  for (int i=0; i<te.length();i++) command[9-i]=te[te.length()-i-1];
  transmit(command);                      
  recieve();
}
void pih (byte proc) {                    // изменение максимальной мощности
  String pr;
  String command = "0601870000";
  pr=String(proc,HEX);
  for (int i=0; i<pr.length();i++) command[9-i]=pr[pr.length()-i-1];
  transmit(command);                      
  recieve();
}
void pil (byte proc) {                    // изменение минимальной мощности
  String pr;
  String command = "0601880000";
  pr=String(proc,HEX);
  for (int i=0; i<pr.length();i++) command[9-i]=pr[pr.length()-i-1];
  transmit(command);                      
  recieve();
}
void termodat(int zna){                   // отправка данных на термодат
  ustav (zna);
  delay(500);                                                  
  if (zna<349) {                          // выбор мощности
    Pi_Hi=40;                                        
    Pi_Low=0;
  }
  else if ((zna<499)and(zna>350)){  
    Pi_Hi=80;
    Pi_Low=30;
  }
  else {
    Pi_Hi=100;
    Pi_Low=80;
  }
  pih(Pi_Hi);
  delay(500);
  pil(Pi_Low);
  
  int U=0;
  delay(500); 
  while ((abs(temp1/10-zna)>dT)||(U<6)) {                       // ожидание установления температуры
    interr();
    if ((abs(temp1/10-zna)<dT)) U++;
    delay(5000);
  }
} 
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
// внутренние функции
////////////////////////////////////////////////////////////////////
void reset() {                            // подпрограмма перезагрузки
  digitalWrite (pinreset, LOW);
}    
void reverse() {                          // исполнительная функция прерываний - смена направления
  dir=!dir;
  digitalWrite(pindir,dir);
}
void parins() {                           // введение параметров
  for (byte curpar=0; (flag[0]==0)||(flag[1]==0)||(flag[2]==0)||(flag[3]==0)||(flag[4]==0)||(flag[5]==0)||(flag[6]==0); ) {     // считывание значений с клавиатуры
    l=customKeypad.getKey();
    if (l){
      int p = int(l);
      switch (p) {
        case 'a':
          if (flag[curpar]==0){
            param[curpar][1]=0;
            param[curpar][2]=0;
            param[curpar][3]=0;
            param[curpar][4]=0;
          }
          curpar=0;
          break; 
        case 'b':
          if (flag[curpar]==0){
            param[curpar][1]=0;
            param[curpar][2]=0;
            param[curpar][3]=0;
            param[curpar][4]=0;
          }
          if (curpar!=0){
            curpar-=1;
            curpar=constrain(curpar,0,6);
          }
          break;
        case 'c':
          if (flag[curpar]==0){
            param[curpar][1]=0;
            param[curpar][2]=0;
            param[curpar][3]=0;
            param[curpar][4]=0;
          }
          curpar+=1;
          curpar=constrain(curpar,0,6);
          break;
        case 'd':
          if (flag[curpar]==0){
            param[curpar][1]=0;
            param[curpar][2]=0;
            param[curpar][3]=0;
            param[curpar][4]=0;
          }
          curpar=6;
          break;
        case 'e':        // решётка
          param[curpar][1]=0;
          param[curpar][2]=0;
          param[curpar][3]=0;
          param[curpar][4]=0;
          flag[curpar]=0;
          break;
        case 'f':        // звёздочка
          flag[curpar]=1;
          curpar++;
          break;
        default:
          int u =(p-'0');
          if ((flag[curpar]==0)&&(param[curpar][4]!=3)&&(u>=0)&&(u<=9)){
            switch (param[curpar][4]){
              case 0:
                param[curpar][3]=u;
                param[curpar][4]++;
                break;
              case 1:
                param[curpar][2]=param[curpar][3];
                param[curpar][3]=u;
                param[curpar][4]++;
                break;
              case 2:
                param[curpar][1]=param[curpar][2];
                param[curpar][2]=param[curpar][3];
                param[curpar][3]=u;
                param[curpar][4]++;
                break;
            }
          }
          break;
      }
      /*for (int i=0;i<7;i++){
        for (int j=0; j<5; j++) {
          Serial.print(param[i][j]);
          Serial.print(" ");
        }
      Serial.println();  
      }
      for(int i=0; i<7; i++){
        Serial.print(flag[i]);
        Serial.print(" ");
      }
      Serial.println();*/
                        
      switch(param[curpar][0]){
        case 1:
        data[0]=0b00100000;
        break;
        case 2:
        data[0]=0b00100010;
        break;
        case 3:
        data[0]=0b01001001;
        break;
        case 4:
        data[0]=0b00110110;
        break;
        case 5:
        data[0]=0b00111110;
        break;
        case 6:
        data[0]=0b01111110;
        break;
        case 7:
        data[0]=0b00111001;
        break;
      }                    
      data[1]=display.encodeDigit(param[curpar][1]);
      data[2]=display.encodeDigit(param[curpar][2]);
      data[3]=display.encodeDigit(param[curpar][3]);
      display.setSegments(data);
      }
  }
  int mnoj2=1;
  for (int i=3; i>0; i--) {
    for (int j=0; j<7; j++) getparam[j]+=param[j][i]*mnoj2;
    mnoj2*=10;
  }
}
void dopus() {                            // проверка параметров на допустимость    
  Td=constrain(getparam[0], Tmin, Tmax);                    
  Dry=constrain(getparam[1], Dry_min, Dry_max); 
  Ta=constrain(getparam[2], Tmin, Tmax);
  Annel=constrain(getparam[3], Annel_min, Annel_max);
  Vd=constrain(getparam[4], Vmin, Vmax);                    
  Va=constrain(getparam[5], Vmin, Vmax); 
  C=constrain(getparam[6], Cmin, Cmax);
  Dry*=10000;                               // время сушки в милисекундах
  Annel*=10000;                             // время отжига в милисекундах
  tv_d=tvh*Vd;                              // установка скорости отжига
  tv_a=tvh*Vd;                              // установка скорости сушки
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
// функции работы с картой
////////////////////////////////////////////////////////////////////
void cardbegin() {                        // подключение SD карты
  if (SD.begin(pinsd)) {                   
    flagsd=1;                   
    Serial.println("Card was found");
  }  
  else Serial.println("Card was NOT found");
}
void swmaster() {                         // удаление и перезапись файла уровней
  if (flagsd==1) {              
    if (SD.exists("Steps.txt")) SD.remove("Steps.txt"); // удаление устаревшего файла
    File stepFile = SD.open("Steps.txt", FILE_WRITE);
    stepFile.println(P[1]);              // запись данных на карту
    stepFile.println(P[2]);
    stepFile.println(P[3]);
    stepFile.close();
  }
}
void srmaster() {                         // считывание уровней из файла
  if (flagsd==1) {             
    File readFile = SD.open("Steps.txt");
    datacarry="";
    P[1]=0;
    P[2]=0;
    P[3]=0;
    while (readFile.available()) datacarry+=readFile.read();
    readFile.close();
    for (int j=1; j<4; j++) {             // обработка буфера шагов
      String stp;  
      stp=datacarry.substring(0,datacarry.indexOf("13"));
      int slen=stp.length()/2;
      int mnoj=1;
      for (int i=slen;i>0;i--) {
        P[j]+=mnoj*(stp.substring(2*i-2,2*i).toInt()-48);
        mnoj*=10;
      }
    datacarry.remove(0,datacarry.indexOf("13")+4);
    }
    
  }
}    
void uwmaster() {                         // удаление и перезапись файла условий
  if (flagsd==1) {
    if (SD.exists("Usl.txt")) SD.remove("Usl.txt"); // удаление устаревшего файла
    File uslFile = SD.open("Usl.txt", FILE_WRITE);
    uslFile.println(Td);                  // запись данных на карту
    uslFile.println(Dry);                 
    uslFile.println(Ta);                  
    uslFile.println(Annel);              
    uslFile.println(tv_d);              
    uslFile.println(tv_a);               
    uslFile.println(C);
    uslFile.close();                 
  }
}
void urmaster() {                         // считывание условий из файла
  if (flagsd==1) {
    File taskFile = SD.open("Usl.txt");
    datacarry="";
    Td=0;
    Dry=0;          
    Ta=0;                 
    Annel=0;       
    tv_d=0;           
    tv_a=0;             
    C=0;
    while (taskFile.available()) datacarry+=taskFile.read();
    taskFile.close();
    for (int j=1; j<8; j++) {             // обработка буфера условий
      String stp3;  
      stp3=datacarry.substring(0,datacarry.indexOf("13"));
      int slen3=stp3.length()/2;
      int mnoj3=1;
      for (int i=slen3;i>0;i--) {
        UUU[j]+=mnoj3*(stp3.substring(2*i-2,2*i).toInt()-48);
        mnoj3*=10;
      }
    datacarry.remove(0,datacarry.indexOf("13")+4);
    }
    Td=UUU[1];
    Dry=UUU[2];          
    Ta=UUU[3];                 
    Annel=UUU[4];       
    tv_d=UUU[5];           
    tv_a=UUU[6];             
    C=UUU[7];
  }
}
void logmaster() {                        // создание имени файла лога
  if (flagsd==1) {
    datacarry="";
    if (SD.exists("Counter.txt")) {       // считывание номера лога
      File countFile = SD.open("Counter.txt");
      while (countFile.available()) datacarry+=countFile.read();
      int slen=(datacarry.length()-4)/2;
      int mnoj=1;
      for (int i=slen;i>0;i--) {
        lognumber+=mnoj*(datacarry.substring(2*i-2,2*i).toInt()-48);
        mnoj*=10;
      }
      lognumber++;                          // увеличение на 1
      countFile.close();
      SD.remove("Counter.txt");             // удаление старого файла
    }
    else lognumber=0;
    File newcountFile = SD.open("Counter.txt", FILE_WRITE);// перезапись номера лога
    newcountFile.println(lognumber);
    newcountFile.close();
    logname="Log"+String(lognumber)+".txt";        // получение имени лога
    dataFile = SD.open(logname, FILE_WRITE);       // создание лога
  }
} 
void startlog(){                          // шапка лога
  if (flagsd==1) {
    dataFile.print("Experiment ");            
    dataFile.println(lognumber);
    dataFile.print("Drying temperature = ");
    dataFile.print(Td);
    dataFile.println(" C");
    dataFile.print("Anneling temperature = ");
    dataFile.print(Ta);
    dataFile.println(" C");
    dataFile.print("Drying time = ");
    dataFile.print(Dry/1000);
    dataFile.println(" sec");
    dataFile.print("Annelling time = ");
    dataFile.print(Annel/1000);
    dataFile.println(" sec");
    dataFile.print("Number of cycles = ");
    dataFile.println(C);
    dataFile.print("Drying velocity divider = ");
    dataFile.println(Vd);
    dataFile.print("Anneling velocity divider = ");
    dataFile.println(Va);
    dataFile.println();
    dataFile.println();
    dataFile.println("time     |  T (channel 1)  |  T (channel 2)");
    dataFile.close();
  }
}
void interr(){                            // запись температуры
  tempread();
  dataFile = SD.open(logname, FILE_WRITE);
  dataFile.print((millis()-Mil)/1);
  dataFile.print(" sec      ");
  dataFile.print(temp1/10);
  dataFile.print(" C      ");
  dataFile.print(temp2/10);
  dataFile.println(" C");
  dataFile.close();
  Serial.print((millis()-Mil)/1);
  Mil=millis();
  Serial.print("   ");
  Serial.print(temp1/10);
  Serial.print("   ");
  Serial.println(temp2/10);
}
void stagechange(String tage){            // запись состояния
  dataFile = SD.open(logname, FILE_WRITE);
  dataFile.println(tage);
  dataFile.close();
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
// функции работы с дополнительным оборудованием
////////////////////////////////////////////////////////////////////
void cooler(String sost) {                // вентилятор
  if (sost=="on") {
    digitalWrite (pinrele1, LOW);
  }
  else if (sost=="off") {
    digitalWrite (pinrele1, HIGH);
  }
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
