## ПИД-регулятор на Arduino

### Пропорционально-интегрально-дифференциальный (ПИД) регулятор оборотов для коллекторного двигателя, питание от сети переменного напряжения 230В, 50Гц.

### ___Возможности:___
+ Ручной ПИД-регулятор без индикации: обороты выставляются потенциометром;
+ ПИД-регулятор с возможностью выбора режима __ручной/программный__, есть светодиодная индикация;
+ ПИД-регулятор с возможностью выбора режима __ручной/программный__, есть светодиодная индикация, ___плюс отображение данных на LCD1608: режим, выставленные и текущие обороты, ступень, оставшееся время.___
___

### __Последовательность действий__

1. Предварительно
   1. разобраться с подключением к сетевому напряжению коллекторного дигателя;
   2. установить Arduino IDE. Информация по установке [здесь](https://alexgyver.ru/lessons/before-start/) и [здесь](https://alexgyver.ru/arduino-first/);
   3. содержимое папки bibl (FastDefFunc обязательно) скопировать в папку libraries. Последняя находится в папке с программой Arduino. Подробнее   [здесь](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA).

2. Собрать проект согласно схеме соединений;
3. Определить количество полюсов на магнитном кольце ротора, скетч Polus.ino;
4. Загрузить скетч PID_motor.ino. При неконтролируемом поведении двигателя варьировать коэффициенты ПИД-регулирования;
5. При желании залить PID_motor_prog_LED.ino или PID_motor_prog_LED_LCD1608I2С.ino.

#### ___Примечание:___

В папке bibl находятся используемые в проекте библиотеки, из которых все кроме FastDefFunc можно найти [у Алекса Гайвера](https://github.com/GyverLibs). FastDefFunc содержит несколько "убыстрённых" стандартных функций:  можно их и не использовать (что нежелательно), тогда нужно в имеющихся в коде "какая_то_стандартная_функцияFast" убрать суффикс "Fast".
