//ПИД-регулирование оборотов двигателя 

/*a) Настройка Kp: диапазон 20 - 50   
  б) Настройка Ki: диапазон 10 - 120
  в) Настройка Kd: диапазон 0.1 - 0.2
*/

#define max_Obor 100         // максимальные обороты двигателя (обор./c)

#define ZERO_PIN 2           // пин детектора нуля
#define HOLL_PIN 3           // пин для датчика Холла
#define DIMMER_PIN 4         // управляющий пин симистора

#define POTEN_PIN A6         // пин регулятора: выставление нужных оборотов в секунду

#define period 1                 // период расчетов и изменений (мс) в ПИД-регуляторе

#include "GyverPID.h"            // библиотека ПИД-регулятора      // подробнее о библиотеке https://alexgyver.ru/gyverpid/  
GyverPID pid (30, 45, 0.1);      // назначение коэфов Kp, Ki, Kd   (25, 100, 0.15)      (25, 50, 0.1)    (30 50 0.1)   (30 45 0.1)   https://alexgyver.ru/lessons/pid/

#include <FastDefFunc.h>                // библиотека для убыстрения функций: pinMode, digitalWrite, ...Read, analogWrite, ...Read

#include <GyverTimers.h>                // библиотека таймера         // https://alexgyver.ru/gyvertimers/

#include "GyverFilters.h"               // библиотека  фильтров       // https://alexgyver.ru/gyverfilters/
GFilterRA filt_pot;                     // создание объекта класса фильтра данных с потенциометра (регулятора)  // https://alexgyver.ru/lessons/filters/

volatile unsigned int dimmer;           // переменная диммера текущая(мкс)
volatile unsigned int lastDim;          //                ... предыдущая (мкс)
 
volatile unsigned int HOLL_N;           // для подсчёта кол-ва импульсов с датчика Холла

unsigned long tmr_pid;                  // для отсчёта времени при ПИД-регулировании

volatile unsigned int odo;              // одометр -- кол-во оборотов в cекунду  
unsigned int odo_val;                   // "обычная" переменная, для переписывания данных с odo

void setup() {
  Serial.begin(115200);                   // скорость взаимодействи в мониторе порта, 9600 -- слишком мало
  Serial.flush();                         // ждём окончания передачи предыдущих данных

  //Serial.println("in, set, dim");
  
  pinModeFast (HOLL_PIN, INPUT_PULLUP);   // INPUT_PULLUP -- один контакт на S другой на GND, в выключенном состоянии на S сигнал -- 1

  pid.setLimits(500, 9500);               // ограничение выходного сигнала (по умолчанию 0-255)
  pid.setDirection(REVERSE);              // обратное воздействие: увеличению соот. уменьшение (из-за того, что 9500 соответсвует минимуму, 500 - макс. открытия симмистора)
  pid.setDt(period);                      // временной шаг расчёта функции ПИД-регулятора 

  pid.integral = 9500;                    // ввиду REVERSE минимальное значение 9500
     
  filt_pot.setCoef(0.1);                  // фильтр для потенциометра //резкость фильтрации (0.00 -- 1.00), чем выше, тем больше скачков      

  Timer1.setPeriod(125000);               // прерывание по таймеру (мкс), период вызова  125 мс: 1 000 000 / 8 импульсов за 1 оборот вала, отсюда 125 мс
  
  Timer1.enableISR(CHANNEL_B);            // подкл-но стандартное прерывание, канал B, без сдига фаз, частота ШИМ изменена на D10
  Timer2.enableISR(CHANNEL_A);            //                                                                              ... D11

  attachInterrupt(digitalPinToInterrupt(ZERO_PIN), isr, RISING);    // функция прерывания вызывается по смене сигнала с 0 на 1
  attachInterrupt(digitalPinToInterrupt(HOLL_PIN), holl, FALLING);  //                                              ... 1 на 0

  delay(5);                             // БЕЗ НЕГО ПРИ ПОДКЛЮЧЕННОМ К ДИММЕРУ 230В В МОМЕНТ ВКЛ ПИТАНИЯ НА МК НА ВЫХОДАХ ЕГО УПРАВЛЯЮЩИХ ПИНОВ ПРОСКАКИВАЕТ ИНОГДА HIGH
  pinModeFast(DIMMER_PIN, OUTPUT);      // при OUTPUT на пине по умолчанию 0
}

void loop() {
  while (int(filt_pot.filteredTime(analogReadFast(POTEN_PIN))) > 2){
    PID();             // ПИД-регулирование согласно данным с потенциометра 
    //serial_print();    // вывод желаемых данных в Serial_plotter
  }
  pid.integral = 9500;  // для плавного начала после предыдущего цикла
  dimmer = 9500;        // чтобы выключить симистор
}

//функция ПИД-регулятора
void PID(){
  odo_val = odo;                    //переписываем в "обычную" переменную количество оборотов в секунду
  if(millis() - tmr_pid >= period){     //производим ПИД-регулирование каждую 1мс
    pid.setpoint = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1024, 0, max_Obor);      //берутся с пот-ра выставленное значение оборотов двигателя, сразу ф-ые
    pid.input = odo_val;                                                  //берутся входящее значение оборотов двигателя                                      
    pid.getResult();                             //производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра
    dimmer = int(expRAA(pid.output));            //на управляющее устройство даётся расчитанный сигнал 
    tmr_pid = millis(); 
  }                             
}

//вывод данных в Serial_plotter
void serial_print(){
      Serial.print(pid.input); Serial.print(',');           // вывод текущих значений кол-ва оборотов в секунду с датчика Холла
      //Serial.print(pid.output); Serial.print(',');
      Serial.println(pid.setpoint);// Serial.print(',');    //   ... выставленных потенциометром
      //Serial.println(dimmer); 
}

//функция прерывания для подсчёта кол-ва импульсов от датчика Холла
void holl(){
    HOLL_N++;
}

//функция прерывания детектора нуля
void isr() {
  digitalWriteFast(DIMMER_PIN, 0);      // выключаем симистор
  if (lastDim != dimmer)     Timer2.setPeriod(lastDim = dimmer);    // если значение изменилось, устанавливаем новый период
  else    Timer2.restart();                                      // если нет, то просто перезапускаем со старым//перезапустить таймер (сбросить счётчик) с новым периодом                    
}

// прерывание таймера диммера
ISR(TIMER2_A) {
  digitalWriteFast(DIMMER_PIN, 1);          // включаем симистор
  Timer2.stop();                        // останавливаем таймер
}

// прерывание таймера подсчёта количества импульсов с датчика Холла, вызывается каждые 125мс
ISR(TIMER1_B) {
  odo = HOLL_N;                       
  HOLL_N = 0; 
}

// бегущее среднее с адаптивным коэффициентом
float expRAA(float newVal) {
  static float filVal = 0;
  float k;
  if (abs(newVal - filVal) > 9000) k = 1;    // резкость фильтра зависит от модуля разности значений
  else k = 0.1;
  
  filVal += (newVal - filVal) * k;
  return filVal;
}
