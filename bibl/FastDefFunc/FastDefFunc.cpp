#include "FastDefFunc.h"

void pinModeFast(uint8_t pin, uint8_t mode) {
  switch (mode) {
    case INPUT:
      if (pin < 8) {
        bitClear(DDRD, pin);    
        bitClear(PORTD, pin);
      } else if (pin < 14) {
        bitClear(DDRB, (pin - 8));
        bitClear(PORTB, (pin - 8));
      } else if (pin < 20) {
        bitClear(DDRC, (pin - 14));   
        bitClear(PORTC, (pin - 8));  
      }
      return;
    case OUTPUT:
      if (pin < 8) {
        bitSet(DDRD, pin);
        bitClear(PORTD, pin);
      } else if (pin < 14) {
        bitSet(DDRB, (pin - 8));
        bitClear(PORTB, (pin - 8));
      } else if (pin < 20) {
        bitSet(DDRC, (pin - 14));  
        bitClear(PORTC, (pin - 8));  
      }
      return;
    case INPUT_PULLUP:
      if (pin < 8) {
        bitClear(DDRD, pin);
        bitSet(PORTD, pin);
      } else if (pin < 14) {
        bitClear(DDRB, (pin - 8));
        bitSet(PORTB, (pin - 8));
      } else if (pin < 20) {
        bitClear(DDRC, (pin - 14));  
        bitSet(PORTC, (pin - 14));  
      }
      return;
  }
}

bool digitalReadFast(uint8_t pin) {
  if (pin < 8) {
    return bitRead(PIND, pin);
  } else if (pin < 14) {
    return bitRead(PINB, pin - 8);
  } else if (pin < 20) {
    return bitRead(PINC, pin - 14);    
  }
}

void digitalWriteFast(uint8_t pin, bool x) {
  switch (pin) {            
    case 3: bitClear(TCCR2A, COM2B1);
      break;
    case 5: bitClear(TCCR0A, COM0B1);
      break;
    case 6: bitClear(TCCR0A, COM0A1);
      break;
    case 9: bitClear(TCCR1A, COM1A1);
      break;
    case 10: bitClear(TCCR1A, COM1B1);
      break;
    case 11: bitClear(TCCR2A, COM2A1);  
      break;
  }
  if (pin < 8) {
    bitWrite(PORTD, pin, x);
  } else if (pin < 14) {
    bitWrite(PORTB, (pin - 8), x);
  } else if (pin < 20) {
    bitWrite(PORTC, (pin - 14), x);    
  }
}

//Нужное опорное установлено DEFAULT, можно изменить на своё
uint16_t analogReadFast(uint8_t pin) {
  pin = ((pin < 8) ? pin : pin - 14);    
  ADMUX = (DEFAULT<< 6) | pin;    
  bitSet(ADCSRA, ADSC);             
  while (ADCSRA & (1 << ADSC));        
  return ADC;                
}

void analogWriteFast(uint8_t pin, uint16_t duty) {
  
  if (!duty) {          
    digitalWrite(pin, LOW);    
    return;            
  }

  switch (pin) {
  case 5:
    bitSet(TCCR0A, COM0B1);    
    OCR0B = duty;        
    return;
  case 6:
    bitSet(TCCR0A, COM0A1);
    OCR0A = duty;
    return;
  case 10:
    bitSet(TCCR1A, COM1B1);
    OCR1B = duty;
    return;
  case 9:
    bitSet(TCCR1A, COM1A1);
    OCR1A = duty;
    return;
  case 3:
    bitSet(TCCR2A, COM2B1);
    OCR2B = duty;
    return;
  case 11:
    bitSet(TCCR2A, COM2A1);
    OCR2A = duty;
    return;
  }
}
