#include <Arduino.h>
#include <EEPROM.h>

#define REL_1 A0
#define REL_2 A1

typedef struct
{
  // Light can be turned on or off (1 or 0 respectively)
  uint8_t light_state;
  // Light mode can be cold white, warm white and mixed lamp turned on (1, 2, 3 respectively - like a byte [10000000], [01000000], [11000000])
  uint8_t light_mode;
} M_STATE;

typedef struct
{
  uint8_t is_defined;          // levels of definition: 0 - not defined, 3 - well defined
  uint8_t pin;                 // button pin on arduino
  char type;                   // 'L' - self locked (maintained)  button type, 'M' - momentary button type
  uint8_t front;               // 0 if pushed button connects to GND, 1 if connect to VCC
  uint8_t state;               // current_time state of button: 0 - off state, 1 - on state
  uint8_t last_pin_state;      // previous state of button: 0 - off state, 1 - on state
  uint32_t current_state_time; // time when current state was changed to ON state
  uint32_t last_state_time;    // last time when state was changed to ON state
} BUTTON;

M_STATE light = {0, 3};
BUTTON b1 = {
    0,
    12,
    127,
    255,
    255,
    255,
    0,
    0};
uint8_t prev_btn_state;
uint32_t last_changed_btn_state_time;

// put function declarations here:
int digitalReadDebounce(int pin);
void define_new_button(BUTTON *btn);
int handle_press_button(BUTTON *btn);
void load_m_state(M_STATE *id);
void load_button(BUTTON *btn, uint8_t btn_number);
void handle_light(M_STATE *light);
void change_light_mode(BUTTON *btn, M_STATE *light);

void setup()
{
  // put your setup code here, to run once:
  pinMode(REL_1, OUTPUT);
  pinMode(REL_2, OUTPUT);
  pinMode(b1.pin, INPUT_PULLUP);
  digitalWrite(REL_1, HIGH);
  digitalWrite(REL_2, HIGH);
  define_new_button(&b1);
  prev_btn_state = handle_press_button(&b1);
  last_changed_btn_state_time = millis();
  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:

  handle_press_button(&b1);
  change_light_mode(&b1, &light);

  // debug messages
  if (millis() % 200 == 0)
  {
    Serial.print("State: ");
    Serial.print(b1.state);
    Serial.print(" Type: ");
    Serial.print(b1.type);
    Serial.print(" Cur_st_time: ");
    Serial.print(b1.current_state_time / 1000);
    Serial.print(" Lst_st_time: ");
    Serial.print(b1.last_state_time / 1000);
    Serial.print("\tLight_mode: ");
    Serial.println(light.light_mode);
  }
  if (b1.state)
  {
    light.light_state = 1;
    handle_light(&light);
  }
  else
  {
    light.light_state = 0;
    handle_light(&light);
  }
}

// put function definitions here:
void define_new_button(BUTTON *btn)
{
  // on the first power on this function should recognize button type: e.g. maintained or momentary, high or low level
  unsigned long max_depress_time_ms = 300; // time need to depress and release momentary button
  int current_signal_state = digitalReadDebounce(btn->pin);
  unsigned long current_time = millis();
  unsigned long signal_changed_time = 0;
  int btn_prev_state = current_signal_state;
  while (!btn->is_defined)
  {
    // need insert periodicaly changing pinMode between INPUT and INPUT_PULLUP to recognize LOW and HIGH buttons
    current_signal_state = digitalReadDebounce(btn->pin);
    current_time = millis();
    if (current_signal_state != btn_prev_state)
    {
      if (signal_changed_time == 0)
      {
        signal_changed_time = current_time;
        btn_prev_state = current_signal_state;
      }
      else if (current_time - signal_changed_time <= max_depress_time_ms)
      {
        btn->type = 'M';
        btn->front = btn_prev_state;
        btn->is_defined = 1;
        btn->state = 0;
        btn->last_pin_state = current_signal_state;
        // Write to EEPROM button data?
      }
      else if (current_time - signal_changed_time > max_depress_time_ms)
      {
        btn->type = 'L';
        btn->front = btn_prev_state;
        btn->is_defined = 1;
        btn->state = 0;
        // Write to EEPROM button data?
      }
    }
  }
}

int handle_press_button(BUTTON *btn)
{
  // this function handle presses on buttons and write this data to button struct
  int current_signal_state = digitalReadDebounce(btn->pin);
  unsigned long current_time = millis();

  if (btn->type == 'L')
  {
    if (current_signal_state == btn->front)
    {
      btn->state = 1;
      btn->last_state_time = btn->current_state_time;
      btn->current_state_time = current_time;
    }
    else
    {
      btn->state = 0;
      btn->last_state_time = btn->current_state_time;
      btn->current_state_time = current_time;
    }
  }
  else if (btn->type == 'M')
  {
    if (current_signal_state != btn->last_pin_state)
    {
      if (current_signal_state == btn->front)
      {
        btn->state = btn->state;
      }
      else
      {
        btn->state = !btn->state;
        btn->last_state_time = btn->current_state_time;
        btn->current_state_time = current_time;
      }
    }
    btn->last_pin_state = current_signal_state;
  }

  return btn->state;
}

int digitalReadDebounce(int pin)
{
  uint8_t bounce_time = 5;
  unsigned long start = millis();
  unsigned long current_time = start;
  int counter = 0;
  int pin_state_accumulator = digitalRead(pin); // pin is pulled up;
  while (current_time - start < bounce_time)
  {
    counter++;
    pin_state_accumulator += digitalRead(pin);
    current_time = millis();
  }

  return pin_state_accumulator / counter;
}

void handle_light(M_STATE *light)
{
  if (light->light_state == 1)
  {
    if (light->light_mode == 1)
    {
      digitalWrite(REL_1, LOW);
      digitalWrite(REL_2, HIGH);
    }
    else if (light->light_mode == 2)
    {
      digitalWrite(REL_1, HIGH);
      digitalWrite(REL_2, LOW);
    }
    else if (light->light_mode == 3)
    {
      digitalWrite(REL_1, LOW);
      digitalWrite(REL_2, LOW);
    }
  }
  else
  {
    digitalWrite(REL_1, HIGH);
    digitalWrite(REL_2, HIGH);
  }
  // whire to EEPROM M_STATE ?
}

void change_light_mode(BUTTON *btn, M_STATE *light)
{
  unsigned int pressing_interval_for_setting_ms = 1000;
  int max_light_mode = 3; // it can be retrive from count of relays if they control same group of devices. For future development
  uint32_t t2 = millis();
  uint32_t t1 = btn->last_state_time;
  if (t2 - t1 <= pressing_interval_for_setting_ms && btn->state == 1)
  {
    light->light_mode = light->light_mode > 1 ? light->light_mode - 1 : max_light_mode;
  }
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
