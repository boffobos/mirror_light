#include <Arduino.h>

#define REL_1 A0
#define REL_2 A1
#define BUT_1 12
#define SET_INTERVAL_S 5

int rel_state = 0;
int prev_btn_state;
unsigned long last_changed_btn_state_time;

// put function declarations here:
int myFunction(int, int);
int handle_button(uint8_t pin);

void setup()
{
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  pinMode(REL_1, OUTPUT);
  pinMode(REL_2, OUTPUT);
  pinMode(BUT_1, INPUT_PULLUP);
  digitalWrite(REL_1, HIGH);
  digitalWrite(REL_2, HIGH);
  prev_btn_state = handle_button(BUT_1);
  last_changed_btn_state_time = millis();
  Serial.begin(9600);
}

void loop()
{
  int btn_state = handle_button(BUT_1);
  if (btn_state != prev_btn_state)
  {
    if (millis() - last_changed_btn_state_time < SET_INTERVAL_S * 1000)
    {
      if (rel_state + 1 > 2)
      {
        rel_state = 0;
      }
      else
      {
        rel_state += 1;
      }
    }
    last_changed_btn_state_time = millis();
    prev_btn_state = btn_state;
  }
  Serial.println(btn_state);
  if (btn_state == 0)
  {
    if (rel_state == 0)
    {
      digitalWrite(REL_1, LOW);
      digitalWrite(REL_2, LOW);
    }
    else if (rel_state == 1)
    {
      digitalWrite(REL_1, LOW);
      digitalWrite(REL_2, HIGH);
    }
    else if (rel_state == 2)
    {
      digitalWrite(REL_1, HIGH);
      digitalWrite(REL_2, LOW);
    }
  }
  else
  {
    digitalWrite(REL_1, HIGH);
    digitalWrite(REL_2, HIGH);
  }
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}

int handle_button(uint8_t pin)
{
  unsigned long aprox_time_ms = 50;
  unsigned long start = millis();
  unsigned long current = millis();
  int counter = 0;
  int button_state = 0;
  while (current - start < aprox_time_ms)
  {
    counter++;
    button_state += digitalRead(pin);
    current = millis();
  }
  // Serial.print("button_state: ");
  // Serial.print(button_state);
  // Serial.print(", counter: ");
  // Serial.print(counter);
  // Serial.print(", devide: ");
  // Serial.println(button_state / counter);
  return button_state / counter;
}