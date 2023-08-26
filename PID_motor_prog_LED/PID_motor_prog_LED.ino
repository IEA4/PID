/*
ПИД-регулирование оборотов двигателя с программной и ручной регулировкой и отображением производимых действий на светодиодах
(Логика работы та же, что и с LCD1602_I2C. Без ЖК-дисплея проще понять код)
*/

#define ZERO_PIN 2          // пин детектора нуля
#define HOLL_PIN 3          // пин для датчика Холла
#define DIMMER_PIN 4        // управляющий пин симистора
#define BUTTON_PIN 5        // пин с кнопкой "ПУСК": запуск выбранного режима, запуск после переворота
#define REGIM_PIN  7        // пин с тублером выбора регулирования: программный (Пр) / ручной (Руч)

#define led_REG 6           // пин светодиода регулятора: повороту регулятора в сторону увеличения соответствует рост яркости светодиода
#define LED_P 8             //            ... режима программного ...
#define LED_R 9             //                   ... ручного регулирования

#define POTEN_PIN A6        // пин регулятора: выставление нужных оборотов в секунду

#define period 1            // период расчетов и изменений (мс) в ПИД-регуляторе

/* при других величинах элементов массива нужен соответсвующий тип переменных,
 а также соответствующия функция чтения переменной https://alexgyver.ru/lessons/progmem/ */
const uint8_t time_stup_arr[3][12] PROGMEM = {         // время работы ступеней  (с)         // 3 программы по 12 ступеней
  {90, 90, 90, 0, 60, 90, 90, 90, 0, 30, 60, 90},   // отсчёт времени работы ступени начинается после того,
  {60, 120, 180, 0, 120, 120, 180, 240, 0, 60,  120, 120}, //   ... как обороты вала выходят на обороты ступени
  {120, 180, 180, 0, 120, 120, 240, 240, 0, 120, 120, 120},
};
const uint8_t stup_arr[3][12] PROGMEM = {              // значения оборотов ступеней (оборотов в секунду вала электродвигателя)
  {20, 25, 35, 0, 20, 35, 45, 50, 0, 25, 40, 50},
  {20, 25, 35, 0, 20, 35, 45, 50, 0, 25, 40, 50},
  {20, 25, 35, 0, 20, 35, 45, 50, 0, 25, 40, 50},
};

#include "GyverPID.h"                   // библиотека ПИД-регулятора      // подробнее о библиотеке https://alexgyver.ru/gyverpid/
GyverPID pid (30, 45, 0.1);             // назначение коэфов Kp, Ki, Kd   // подробнее о коэфах https://alexgyver.ru/lessons/pid/

#include <FastDefFunc.h>                // библиотека для убыстрения функций: pinMode, digitalWrite, ...Read, analogWrite, ...Read

#include <GyverTimers.h>                // библиотека таймера         // https://alexgyver.ru/gyvertimers/

#include "GyverFilters.h"               // библиотека  фильтров       // https://alexgyver.ru/gyverfilters/
GFilterRA filt_pot;                     // создание объекта класса фильтра данных с потенциометра (регулятора)  // https://alexgyver.ru/lessons/filters/

#include <VirtualButton.h>              // библиотека кнопки, устранен дребезг контактов
VButton btn;                            // создание объекта класса для кнопки

boolean f_AR = 1;                       // флажок смены режима: Пр / Руч
boolean f_AuL = 1;                      // флажок разрешения/запрета шаблонной информации

unsigned long tmr_pid;                  // для отсчёта времени при вычислениях в ПИД
unsigned long tmr_mig;                  //                     ... мигании светодиода регулятора

byte j;                                 // для выбора программы
byte i;                                 //        ... ступени

volatile unsigned int dimmer;           // переменная диммера текущая(мкс)
volatile unsigned int lastDim;          //                ... предыдущая (мкс)

volatile unsigned int HOLL_N;           // для подсчёта кол-ва импульсов с датчика Холла

volatile unsigned int odo;              // одометр -- кол-во оборотов в cекунду
unsigned int odo_val;                   // "обычная" переменная, для переписывания данных с odo

void setup() {
    Serial.begin(115200);                // скорость взаимодействи по Serial, 9600 -- слишком мало (в мониторе порта также должна быть выставлена эта скорость взаимодействия)
    Serial.flush();                      // ждём окончания передачи предыдущих данных

    pinModeFast(HOLL_PIN, INPUT_PULLUP);  // INPUT_PULLUP -- один контакт на пин другой на GND, в выключенном состоянии на пине сигнал -- 1
    pinModeFast(BUTTON_PIN, INPUT_PULLUP);
    pinModeFast(REGIM_PIN, INPUT_PULLUP);

    pid.setLimits(500, 9500);             // ограничение выходного сигнала ПИД-регулятора (по умолчанию 0-255)
    pid.setDirection(REVERSE);            // обратное воздействие: увеличению соот. уменьшение (из-за того, что 9500 соответсвует минимуму, 500 - макс. открытия симмистора)
    pid.setDt(period);                    // временной шаг расчёта функции ПИД-регулятора

    filt_pot.setCoef(0.1);                // коэф фильтрации данных с потенциометра // резкость фильтрации (0.00 -- 1.00), чем выше, тем больше скачков

    Timer1.setPeriod(125000);             // прерывание по таймеру, период вызова  125 мс (1000мс / 8 импульсов за 1 оборот вала, получаем 125 мс)

    Timer1.enableISR(CHANNEL_B);          // подкл-но стандартное прерывание, канал B, без сдига фаз, частота ШИМ изменена на D10
    Timer2.enableISR(CHANNEL_A);          //                                                                              ... D11

    attachInterrupt(digitalPinToInterrupt(ZERO_PIN), isr, RISING);    // функция прерывания вызывается по смене сигнала с 0 на 1
    attachInterrupt(digitalPinToInterrupt(HOLL_PIN), holl, FALLING);  // реагирует при смене сигнала c 1 на 0

    delay(5);                             // БЕЗ НЕГО ПРИ ПОДКЛЮЧЕННОМ К ДИММЕРУ 230В В МОМЕНТ ВКЛ ПИТАНИЯ НА МК НА ВЫХОДАХ ЕГО УПРАВЛЯЮЩИХ ПИНОВ ПРОСКАКИВАЕТ ИНОГДА HIGH
    pinModeFast(led_REG, OUTPUT);         // при OUTPUT на пине по умолчанию 0
    pinModeFast(LED_P, OUTPUT);
    pinModeFast(LED_R, OUTPUT);

    pinModeFast(DIMMER_PIN, OUTPUT);
}

void loop(){
    while(true){
        while(digitalReadFast(REGIM_PIN) == 1){           // РЕЖИМ ПРОГРАММНОЙ РЕГУЛИРОВКИ. Откачка согласно программе, по ступенькам по заданным интервалам времени
            digitalWriteFast(LED_P, HIGH);                // сигнализируем включением светодиода-индикатора режима программной регулировки

            while (push_button() == 0){                   // ждать пока не нажмётся кнопка для выбора этого режима
                if(digitalReadFast(REGIM_PIN) == 0){        // если при этом сменится режим работы,
                    f_AR = 0;                                     // кладём флажок режима
                    break;                                            // и выходим из программного режима
                }
                if (analogReadFast(POTEN_PIN) <= 341)     j = 0;   // во время ожидания регулятором РЕГ выбираем программу
                else if (analogReadFast(POTEN_PIN) > 341 && analogReadFast(POTEN_PIN) <= 682)     j = 1;
                else if (analogReadFast(POTEN_PIN) > 682)     j = 2;

                //Serial.println(analogReadFast(POTEN_PIN));         // использовать если нужно уст-ть другие диапазаны типа откачки (смотреть в мониторе порта)

                analogWriteFast(led_REG, analogReadFast(POTEN_PIN) >> 2);  // ШИМирование яркости светодиода регулятора в зависимости от величины его поворота
            }

            if (f_AR == 1)  auto_regim();            // если кнопка нажалась и не было смены режима работы  // производится откачка согласно выбранной программе
            else      f_AR = 1;                      // если же был сменён режим, перед выходом поднимаем флажок режима

            digitalWriteFast(LED_P, LOW);            // выключаем светодиод - индикатор этого режима
            digitalWriteFast(led_REG, LOW);          // выключаем светодиод - индикатор регулировки
        }

        while (digitalReadFast(REGIM_PIN) == 0){          // РЕЖИМ РУЧНОЙ РЕГУЛИРОВКИ. Скорость откачки выставляется потенциометром (РЕГ)
            digitalWriteFast(LED_R, HIGH);                // сигнализируем включением светодиода-индикатора режима ручной регулировки

            while (push_button() == 0) {                  // ждать пока не нажмётся кнопка для выбора этого режима
                if(digitalReadFast(REGIM_PIN) == 1){
                    f_AR = 0;
                    break;
                }
            }

            if (f_AR == 1){
                if(analogReadFast(POTEN_PIN) != 0){                            // если окажется, что потенциометр  выставлен на некоторое значение,
                    while(analogReadFast(POTEN_PIN) != 0){                      // ждать пока не вернут его в положение с минимальным значением
                        if (millis() - tmr_mig >= 250) {                         // частым миганием сигнализируем светодиодом регулировки
                            digitalWriteFast(led_REG, !digitalReadFast(led_REG));      // мигание
                            tmr_mig = millis();
                        }
                    }
                }

                while(digitalReadFast(REGIM_PIN) == 0){         // пока режим откачки не сменится
                    rutsch_regim();                             // откачиваем на ручном режиме
                }
            }
            else     f_AR = 1;

            digitalWriteFast(LED_R, LOW);          // выключаем светодиод - индикатор режима ручной регулировки
        }
    }
}

void auto_regim(){
    boolean f_AbO = 1;                     // флажок начала отсчёта откачки по периоду ступени
    pid.integral = 9500;                   // при таком значении интегральной суммы время диммирования dimmer максимально, что необ. для минимума открытия симистора
    byte typ = analogReadFast(POTEN_PIN) >> 2;    // запоминаем яркость светодиода при выбранной программе
    for (i=0; i<12; i++){                           // 12 ступеней откачки
        if (i == 3 || i == 8){                      // на этих ступенях откачка останавливается для возможности переверота
            pid.integral = 9500;
            while (push_button() == 0){             // как перевернём, нажимаем кнопку и запускаем дальнейшую откачку
                pid_in_auto();                      // функция ПИД-регулирования в программном режиме
                if (millis() - tmr_mig >= 750){                               // таймер мигания на 750 мс
                    digitalWriteFast (led_REG, !digitalReadFast(led_REG));     // мигаем светодиодном регулятора, сигнализируя о необ. действия со стороны человека
                    tmr_mig = millis();
                }
                //serial_print();               // плоттер
            }
            f_AuL = 1;                          // добро на обновление данных для ступени
            analogWriteFast(led_REG, typ);      // вкл светодиод регулятора, яркость в соответствии с режимом
        }

        else if (i != 3 || i != 8){
            uint32_t tmrbO = millis();                                          // таймер начала откачки
            unsigned long tao = 1000 * ((uint32_t)pgm_read_byte(&time_stup_arr[j][i]));  //перевод в миллисекунды
            while(millis() - tmrbO < tao){      // откачка на задданных ступенью оборотах в течении заданного программой времени
                pid_in_auto();
                //serial_print();

                if(f_AbO == 1){                   // отсчёт времени откачки ступени начинаем только после того, как текущие обороты выйдут на выставленные
                  if(odo_val < pid.setpoint){
                      tmrbO = millis();         // занулять разницу времени начала откачки до тех пор, пока текущие обороты не выйдут на выставленные
                  }
                  else   f_AbO = 0;             // если текущие обороты выставленные, то прекращаем зануление разницы
                }
            }
            f_AbO = 1;
            f_AuL = 1;
        }
    }
    odo_to_null();                      // прекращение вращения вала
}

void rutsch_regim(){
    pid.integral = 9500;
    while (int(filt_pot.filteredTime(analogReadFast(POTEN_PIN))) > 5){        // активизация только после того, как начнёт поворачиваться ручка потенциометра (РЕГ)
        odo_val = odo;
        if(millis() - tmr_pid >= period){
            pid.setpoint = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1024, 0, 100); // берутся с РЕГ фильтрованные значения оборотов двигателя (мах 100 об/c)
            pid.input = odo_val;                                 // берутся входящее значение оборотов двигателя
            pid.getResult();                           // производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра
            dimmer = int(expRAA(pid.output));         // на управляющее устройство даётся фильтрованные расчитанный сигнал
            tmr_pid = millis();
        }

        analogWriteFast(led_REG, analogReadFast(POTEN_PIN) >> 2);  // яркость светодиода регулировки в соответсвии с поворотом ручки регулятора оборотов

        //serial_print();               //вывод на экран данных
    }
    digitalWriteFast(led_REG, LOW);
    odo_to_null();
}

//функция пид-регулирования в программном режиме
void pid_in_auto(){
    if (f_AuL == 1){
        pid.setpoint = (int)(pgm_read_byte(&stup_arr[j][i]));   // берём значения необходимых оборотов из массива
        f_AuL = 0;
    }

    odo_val = odo;                            // переписываем в "обычную" переменную количество оборотов в секунду с вала электродвигателя
    if(millis() - tmr_pid >= period){
        pid.input = odo_val;                  // берутся входящее значение оборотов двигателя
        pid.getResult();                      // производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра (РЕГ)
        dimmer = int(expRAA(pid.output));     // на управляющее устройство даётся отфильтрованный расчитанный сигнал
        tmr_pid = millis();
    }
}

// функция прекращающая вращение вала
void odo_to_null(){
    while (pid.input != 0){             // после завершения цикла откачки ждём пока вал не перестанет вращаться
        odo_val = odo;
        if(millis() - tmr_pid >= period){
            pid.setpoint = 0;
            pid.input = odo_val;;
            pid.getResult();
            dimmer = int(expRAA(pid.output));
            tmr_pid = millis();
        }

        if (dimmer == 9500)     break;       // как только симистор закроется выходим
    }
    dimmer = 9500;                               // для уверенности увеличиваем до максимума переменную диммирования
}

// функция клика кнопкой
boolean push_button(){
  btn.poll(!digitalReadFast(BUTTON_PIN));
  if (btn.click()) return 1;             // возвращает единицу при клике кнопкой
  else return 0;
}

// функци вывода данных в Serial_port
void serial_print(){
      Serial.print(100 * pid.input); Serial.print(',');      // входящие обороты, умноженные на 100
      //Serial.print(pid.output); Serial.print(',');
      Serial.print(100 * pid.setpoint); Serial.print(',');   // выставленные обороты, умноженные на 100
      Serial.println(dimmer);
}

// функция прерывания для подсчёта кол-ва импульсов от датчика Холла, вызывается когда на датчик попадает полюс магнита на валу
void holl(){
    HOLL_N++;
}

// функция прерывания детектора ноля, вызывается при прохождении синусоиды сетевого напряжения через ноль
void isr() {
  digitalWriteFast(DIMMER_PIN, 0);      // выключаем симистор
  if (lastDim != dimmer){               // если значение изменилось, устанавливаем новый период
    Timer2.setPeriod(lastDim = dimmer);
  }
  else {                                // если нет, то просто перезапускаем со старым
    Timer2.restart();                   //перезапустить таймер (сбросить счётчик) с новым периодом
  }
}

// прерывание таймера диммера
ISR(TIMER2_A) {
  digitalWrite(DIMMER_PIN, 1);          // включаем симистор
  Timer2.stop();                        // останавливаем таймер
}

// прерывание таймера подсчёта количества импульсов с датчика Холла, вызывается каждые 125мс
ISR(TIMER1_B) {
  odo = HOLL_N;
  HOLL_N = 0;
}

// фильтр выходных данных
float expRAA(float newVal) {
  static float filVal = 0;
  float k;
  // резкость фильтра зависит от модуля разности значений
  if (abs(newVal - filVal) > 9000) k = 1;
  else k = 0.1;

  filVal += (newVal - filVal) * k;
  return filVal;
}
