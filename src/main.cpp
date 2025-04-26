#include <Arduino.h>
#include <EEPROM.h>

#define REL_1 A0
#define REL_2 A1
#define SET_INTERVAL_S 5

typedef struct
{
  // Light can be turned on or off (1 or 0 respectively)
  uint8_t light_state;
  // Light mode can be cold white, warm white and mixed lamp turned on ("0", "1", "2" respectively)
  char light_mode;
} M_STATE;

typedef struct
{
  uint8_t pin;        // button pin on arduino
  char type;          // 'L' - self locked (maintened)  button type, 'M' - momentary button type
  uint8_t front;      // 0 if pushed button connects to GND, 1 if connect to VCC
  uint8_t state;      // current state of button: 0 - off state, 1 - on state;
  uint8_t prev_state; // previous state of button: 0 - off state, 1 - on state;
} BUTTON;

M_STATE light;
BUTTON b1 = {
    12,
    127,
    255,
    255,
    255};
int rel_state = 0;
int prev_btn_state;
unsigned long last_changed_btn_state_time;

// put function declarations here:
int handle_button(uint8_t pin);

void setup()
{
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  pinMode(REL_1, OUTPUT);
  pinMode(REL_2, OUTPUT);
  pinMode(b1.pin, INPUT_PULLUP);
  digitalWrite(REL_1, HIGH);
  digitalWrite(REL_2, HIGH);
  prev_btn_state = handle_button(b1.pin);
  last_changed_btn_state_time = millis();
  Serial.begin(9600);
}

void loop()
{
  int btn_state = handle_button(b1.pin);
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

void load_m_state(M_STATE *id)
{
  int address = 0;
  EEPROM.put(address, *id);
  if (id->light_state > 1)
  {
    id->light_state = 0;
  }
  if (id->light_mode != '0' || id->light_mode != '1' || id->light_mode != '2')
  {
    id->light_mode = '2';
  }

  return;
}

void load_button(BUTTON *btn, uint8_t btn_number)
{
  int address = sizeof(M_STATE) + sizeof(BUTTON) * (btn_number - 1);
  EEPROM.get(address, *btn);
  // check if loaded values is in valid range and handle invalid values
}