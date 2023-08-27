/*ПИД-регулирование оборотов двигателя с программной и ручной регулировкой и отображением производимых действий на светодиодах и ЖК-экране

Не забудьте прописать в коде в строке Timer1.setPeriod(125000);  результат деления 1 000 000  на количество полюсов на кольцевом магните  
При необходимости изменить максимальное количество оборотов вала (вместо 100 нужное значение) --> ...
                                           ... строка  pid.setpoint = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1024, 0, 100);
*/

#define ZERO_PIN 2          // пин детектора ноля
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

#include <LiquidCrystal_I2C.h>          //библиотека LCD по I2C
LiquidCrystal_I2C lcd(0x27,16,2);       //указываем адрес дисплея, количество столбцов и строк  // SDA – A4, SCL – A5

#include <VirtualButton.h>              //библиотека кнопки, устранен дребезг контактов в частности
VButton btn;                            //класс для кнопки

boolean f_AR = 1;                       // флажок смены режима: Auto/Rutsch
boolean f_AuL = 1;                      // флажок разрешения/запрета шаблонной информации

unsigned long tmr_pid;                  // для отсчёта времени при вычислениях в ПИД
unsigned long tmr_lcd;                  //                    ... вывода новых данных на LCD
unsigned long tmr_mig;                  //                    ... мигания светодиодом регулятора

byte j;                                 // для выбора программы
byte i;                                 //       ...  ступени

volatile unsigned int dimmer;           // переменная диммера текущая(мкс)
volatile unsigned int lastDim;          //                 ...предыдущая (мкс)

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

    lcd.init();                           //инициализация дисплея
    lcd.backlight();                      //включение фоновой подсветки

    delay(5);                             //БЕЗ НЕГО ПРИ ПОДКЛЮЧЕННОМ К ДИММЕРУ 230В В МОМЕНТ ВКЛ ПИТАНИЯ НА МК НА ВЫХОДЕ ЕГО УПРАВЛЯЮЩЕГО ПИНА ПРОСКАКИВАЕТ ИНОГДА HIGH
    pinModeFast(led_REG, OUTPUT);         // при OUTPUT на пине по умолчанию 0
    pinModeFast(LED_P, OUTPUT);
    pinModeFast(LED_R, OUTPUT);

    pinModeFast(DIMMER_PIN, OUTPUT);
}

void loop(){
    while(true){
        while(digitalReadFast(REGIM_PIN) == 1){           // РЕЖИМ ПРОГРАММНОЙ РЕГУЛИРОВКИ. Откачка согласно программе, по ступенькам по заданным интервалам времени

            lcd.setCursor(0,0);    lcd.print(F("p"));      // Программный
            lcd.setCursor(0,1);    lcd.print(F("Set:"));    // Выставлено
            lcd.setCursor(9,1);    lcd.print(F("Inp:"));    // Текущие

            digitalWriteFast(LED_P, HIGH);          // сигнализируем включением светодиода-индикатора режима программной регулировки

            while (push_button() == 0){                   // ждать пока не нажмётся кнопка для выбора этого режима
                if(digitalReadFast(REGIM_PIN) == 0){        // если при этом сменится режим работы,
                    f_AR = 0;                                     // кладём флажок режима
                    break;                                            // и выходим из программного режима
                }
                if (analogReadFast(POTEN_PIN) <= 341){        // во время ожидания регулятором РЕГ выбираем программу
                    j=0;
                    lcd.setCursor(1,0); lcd.print(F("1"));
                }
                else if (analogReadFast(POTEN_PIN) > 341 && analogReadFast(POTEN_PIN) <= 682){
                    j = 1;
                    lcd.setCursor(1,0); lcd.print(F("2"));
                }
                else if (analogReadFast(POTEN_PIN) > 682){
                    j = 2;
                    lcd.setCursor(1,0); lcd.print(F("3"));
                }
                //Serial.println(analogReadFast(POTEN_PIN));         // использовать если нужно уст-ть другие диапазаны типа откачки (смотреть в мониторе порта)

                analogWriteFast(led_REG, analogReadFast(POTEN_PIN) >> 2);  // ШИМирование яркости светодиода регулятора в зависимости от величины его поворота
            }

            if (f_AR == 1)  auto_regim();            // если кнопка нажалась и не было смены режима работы  // производится откачка согласно выбранной программе
            else      f_AR = 1;                      // если же был сменён режим, перед выходом поднимаем флажок режима

            lcd.clear();                             // очищаем экран
            digitalWriteFast(LED_P, LOW);            // выключаем светодиод - индикатор этого режима
            digitalWriteFast(led_REG, LOW);          // выключаем светодиод - индикатор регулировки
        }

        while (digitalReadFast(REGIM_PIN) == 0){         // РЕЖИМ РУЧНОЙ РЕГУЛИРОВКИ. Скорость откачки выставляется потенциометром (РЕГ)
            lcd.setCursor(0,0);     lcd.print(F("R"));
            lcd.setCursor(0,1);     lcd.print(F("Set:"));
            lcd.setCursor(9,1);     lcd.print(F("Inp:"));

            digitalWriteFast(LED_R, HIGH);           // сигнализируем включением светодиода-индикатора режима ручной регулировки

            while (push_button() == 0) {                  // ждать пока не нажмётся кнопка для выбора этого режима
                if(digitalReadFast(REGIM_PIN) == 1){
                    f_AR = 0;
                    break;
                }
            }

            if (f_AR == 1){
                if(analogReadFast(POTEN_PIN) != 0){                            // если окажется, что потенциометр  выставлен на некоторое значение,
                    lcd.setCursor(5,0);      lcd.print(F("REG TO MIN"));
                    while(analogReadFast(POTEN_PIN) != 0){                     // ждать пока не вернут его в нулевое положение
                        if (millis() - tmr_mig >= 250) {
                            digitalWriteFast(led_REG, !digitalReadFast(led_REG));      //мигаем светодиодом режима ручной регулировки
                            tmr_mig = millis();
                        }
                    }
                    lcd.setCursor(5,0);      lcd.print(F("          "));           // удаление просьбы "Pot to 0"
                }
                lcd.setCursor(4,1);   lcd.print(F("0"));                  // после нажатия на кнопку выводим нули
                lcd.setCursor(13,1);   lcd.print(F("0"));

                while(digitalReadFast(REGIM_PIN) == 0){         // пока режим откачки не сменится
                    rutsch_regim();                             // откачиваем на ручном режиме
                }
            }
            else   f_AR = 1;

            lcd.clear();
            digitalWriteFast(LED_R, LOW);          // выключаем светодиод - индика
        }
    }
}

void auto_regim(){
    boolean f_AbO = 1;                     // флажок начала отсчёта откачки по периоду ступени
    pid.integral = 9500;                   // при таком значении интегральной суммы время диммирования dimmer максимально, что необ. для минимума открытия симистора
    byte typ = analogReadFast(POTEN_PIN) >> 2;    // запоминаем яркость светодиода при выбранной программе
    for (i = 0; i < 12; i++){                                // 12 ступеней откачки
        lcd.setCursor(3,0);     lcd.print(F("12|"));
        lcd.setCursor(6,0);     lcd.print(i+1);                 // выводим на первой строчке номер ступени, начиная от 1
        lcd.setCursor(12,0);    lcd.print(F("|"));              // для таймера отсчёта

        if (i == 3 || i == 8){                      // на этих ступенях откачка останавливается для возможности перевернуть рамки
            pid.integral = 9500;
            lcd.setCursor(9,0);    lcd.print(F("POVOROT"));
            lcd.setCursor(5,1);    lcd.print(F(" "));     // удаляем цифру за нулём
            while (push_button() == 0){             // как перевернём, нажимаем кнопку и запускаем дальнейшую откачку
                pid_in_auto();
                if (millis() - tmr_mig >= 750){                               // вывод новых данных каждые 750мс
                    digitalWriteFast (led_REG, !digitalReadFast(led_REG));     // мигаем светодиодном регулятора, сигнализируя о необ. действия со стороны человека
                    tmr_mig = millis();
                }
                //serial_print();               //плоттер
            }
            f_AuL = 1;               //добро на обновление данных для ступени
            lcd.setCursor(9,0);    lcd.print(F("       ")); // удаляем "POVOROT"
            analogWriteFast(led_REG, typ);      // зажигаем светодиод регулятора
        }

        else if (i != 3 || i != 8){
            uint8_t tmr_delta, tmr_deltaLast;
            unsigned long tmrbO = millis();

            if  (pgm_read_byte(&time_stup_arr[j][i]) < 10){
                lcd.setCursor(11,0);    lcd.print(pgm_read_byte(&time_stup_arr[j][i]));
            }
            else if (pgm_read_byte(&time_stup_arr[j][i]) >= 10 && pgm_read_byte(&time_stup_arr[j][i]) < 100) {
                lcd.setCursor(10,0);    lcd.print(pgm_read_byte(&time_stup_arr[j][i]));
            }
            else if (pgm_read_byte(&time_stup_arr[j][i]) >= 100) {
                lcd.setCursor(9,0);     lcd.print(pgm_read_byte(&time_stup_arr[j][i]));
            }

            lcd.setCursor(13,0);    lcd.print(pgm_read_byte(&time_stup_arr[j][i]));   //обратный отсчёт на дисплее
            //lcd.setCursor(15,0);    lcd.print(F(" "));  //костыль чтоб не выводились левые символы

            unsigned long tao = 1000 * ((uint32_t)pgm_read_byte(&time_stup_arr[j][i]));  //перевод в миллисекунды
            while(millis() - tmrbO < tao){
                pid_in_auto();                //пид-регулирование
                //serial_print();               //плоттер

                if(f_AbO == 1){               //отсчёт времени откачки ступени начинаем только после того, как текущие обороты сравняются с выставленными
                    if(odo_val < pid.setpoint){
                        tmrbO = millis();     // занулять разницу времени начала откачки до тех пор, пока текущие обороты не выйдут на выставленные
                    }
                    else   f_AbO = 0;
                }

                if(f_AbO == 0){         //обратный отсчёт оставшегося времени откачки
                    tmr_delta = (uint8_t)(0.001 * (tao - (millis() - tmrbO)));
                    if (tmr_delta != tmr_deltaLast && tmr_delta >= 1){
                      lcd.setCursor(13,0);    lcd.print(tmr_delta);
                      if (tmr_delta < 100 && tmr_delta >= 10){
                         lcd.setCursor(15,0);    lcd.print(F(" "));
                      }
                      else if (tmr_delta < 10){
                         lcd.setCursor(14,0);    lcd.print(F("  "));
                      }
                      tmr_deltaLast = tmr_delta;
                    }
                }
            }
            f_AbO = 1;
            f_AuL = 1;
        }
    }
    odo_to_null();                      //прекращение вращения вала
}

void rutsch_regim(){
    pid.integral = 9500;
    boolean f_z, f_h;
    while (int(filt_pot.filteredTime(analogReadFast(POTEN_PIN))) > 5){        //активизация только после того, как начнёт поворачиваться ручка потенциометра
        odo_val = odo;                    //переписываем в "обычную" переменную количество оборотов в секунду
        if(millis() - tmr_pid >= period){
            pid.setpoint = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1024, 0, 100);      //берутся с пот-ра выставленное значение оборотов двигателя, сразу ф-ые
            pid.input = odo_val;                                                  //берутся входящее значение оборотов двигателя
            pid.getResult();                                            //производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра
            dimmer = int(expRAA(pid.output));                           //на управляющее устройство даётся расчитанный сигнал
            tmr_pid = millis();
        }

        analogWriteFast(led_REG, analogReadFast(POTEN_PIN) >> 2);  // яркость светодиода регулировки в соответсвии с поворотом ручки регулятора оборотов

        if (millis() - tmr_lcd >= 500){                                  //вывод новых данных каждые 500мс
            lcd.setCursor(4,1);   lcd.print(int(pid.setpoint));         //выставленные на потенциометре обороты
            lcd.setCursor(13,1);   lcd.print(int(pid.input));           //текущие обороты двигателя
            if (int(pid.setpoint) < 10){                                //если выставленные меньше 10, то очищаем на дисплее место второй цифры (для ясности выводимой информации)
                lcd.setCursor(5,1);   lcd.print(F("  "));
            }

            if ( f_z == 0 && int(pid.input) < 10){
                lcd.setCursor(14,1);   lcd.print(F("  "));
                f_z = 1;
            }
            else if (int(pid.input) >= 10 && int(pid.input) < 100) {
              f_z = 0;
              if (f_h == 0 && int(pid.input) < 100){
                lcd.setCursor(15,1);   lcd.print(F(" "));
                f_h = 1;
              }
            }
            else if (int(pid.input) >= 100) f_h = 0;

            tmr_lcd = millis();
        }

        //serial_print();               //вывод на экран данных
    }
    digitalWriteFast(led_REG, LOW);
    odo_to_null();
}

//функция пид-регулирования в программном режиме
void pid_in_auto(){
    if (f_AuL == 1){
        lcd.setCursor(4,1);    lcd.print(pgm_read_byte(&stup_arr[j][i]));
        pid.setpoint = (int)(pgm_read_byte(&stup_arr[j][i]));   // берём значения необходимых оборотов из массива
        f_AuL = 0;
    }

    if (millis() - tmr_lcd >= 500){                               // вывод новых данных каждые 500мс
        lcd.setCursor(13,1);   lcd.print(int(pid.input));         // текущие обороты двигателя
        if (int(pid.input) < 10){
            lcd.setCursor(14,1);   lcd.print(F("  "));
        }
        tmr_lcd = millis();
    }

    odo_val = odo;                            //переписываем в "обычную" переменную количество оборотов в секунду
    if(millis() - tmr_pid >= period){
        pid.input = odo_val;                  //берутся входящее значение оборотов двигателя
        pid.getResult();                      //производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра
        dimmer = int(expRAA(pid.output));     //на управляющее устройство даётся расчитанный сигнал
        tmr_pid = millis();
    }
}

//функция прекращающая вращение вала
void odo_to_null(){
    while (pid.input != 0){             //после завершения откачки ждём пока вал не перестанет вращаться
        odo_val = odo;
        if(millis() - tmr_pid >= period){
            pid.setpoint = 0;
            pid.input = odo_val;;
            pid.getResult();
            dimmer = int(expRAA(pid.output));
            tmr_pid = millis();
        }

        if (millis() - tmr_lcd >= 100){                                 //вывод новых данных каждые 100мс
            lcd.setCursor(4,1);   lcd.print(int(pid.setpoint));         //выставленные на потенциометре обороты
            lcd.setCursor(13,1);   lcd.print(int(pid.input));           //текущие обороты двигателя
            if (int(pid.setpoint) < 10){                                //если выставленные меньше 10, то очищаем на дисплее место второй цифры (для ясности выводимой информации)
                lcd.setCursor(5,1);   lcd.print(F("  "));
            }
            if (int(pid.input) < 10){
                lcd.setCursor(14,1);   lcd.print(F("  "));
            }
            tmr_lcd = millis();
        }

       if (dimmer == 9500)     break;       // как только симистор закроется выходим
    }
    lcd.setCursor(13,1);   lcd.print(F("0"));
    dimmer = 9500;                               // для уверенности увеличиваем до максимума переменную диммирования
}

// функция клика кнопкой
boolean push_button(){
  btn.poll(!digitalReadFast(BUTTON_PIN));
  if (btn.click()) return 1;             //функция возвращает единицу при клике кнопкой
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
  digitalWriteFast(DIMMER_PIN, 1);          // включаем симистор
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
