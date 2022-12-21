// Определение количества полюсов на кольцевом магните якоря

#define Holl_PIN 3 // сигнальный провод датчика Холла на 3 цифровой пин ардуино

volatile byte HOLL_N;    // переменная для подсчёта кол-ва полюсов кольцевого магнита в функции прерывания
byte holl_N, holl_NLast; // "обычные" для перезаписи переменной из прерывания

void setup()
{
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(Holl_PIN), isr, FALLING); // прерывание на 3 пине по спаду
}

void loop()
{
    holl_N = HOLL_N; // перезапись из переменной прерывания в обычную
    if (holl_NLast != holl_N)
    {
        holl_NLast = holl_N;
        Serial.println(holl_NLast); // вывод на монитор порта сведения о смене полюса
    }
}

void isr()
{ // функция из прерывания, вызвается каждый раз при смене полюса
    HOLL_N++;
}