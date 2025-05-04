#include <Arduino.h>
#include <EEPROM.h>

// Todo:
// - Timeout when light should turn off automaticaly
// Working with few buttons
// Posibility to load pins number and working mode from external source then initialize it programmaticaly
// Improvments:
// in function define new button need to change periodicaly pinMode to detect pressing button with different front

#define member_size(type, member) (sizeof(((type *)0)->member))
#define REL_1 A0
#define REL_2 A1
#define TIMEOUT 5

typedef struct
{
  // Light can be turned on or off (1 or 0 respectively)
  uint8_t light_state;
  // Light mode can be cold white, warm white and mixed lamp turned on (1, 2, 3 respectively - like a byte [10000000], [01000000], [11000000])
  uint8_t light_mode;
  uint32_t timestamp;
} M_STATE;

typedef struct
{
  uint8_t is_defined;          // levels of definition: 0 - not defined, 1 - defined
  uint8_t pin;                 // button pin on arduino
  char type;                   // 'L' - self locked (maintained)  button type, 'M' - momentary button type
  uint8_t front;               // 0 if pushed button connects to GND, 1 if connect to VCC
  uint8_t state;               // current_time state of button: 0 - off state, 1 - on state
  uint8_t last_state;          // previous cycle button state
  uint8_t last_pin_state;      // previous cycle state on button pin: 0 - off state, 1 - on state
  uint32_t current_state_time; // time when current state was changed to ON state
  uint32_t last_state_time;    // last time when state was changed to ON state
} BUTTON;

M_STATE light = {0, 3, 0};
BUTTON b1 = {
    0,
    12,
    127,
    255,
    255,
    255,
    0,
    0};

// put function declarations here:
int digitalReadDebounce(int pin);
void define_new_button(BUTTON *btn);
int handle_press_button(BUTTON *btn);
void m_state_rom(M_STATE *id, char action);
int button_state_rom(BUTTON *btn, uint8_t btn_number, char action);
void watching_buttons_state_changes(M_STATE *light, BUTTON *btns[], int btn_count);
void change_light_mode(M_STATE *light, BUTTON *btn);
void handle_switching_light(M_STATE *id);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    ;
  pinMode(REL_1, OUTPUT);
  pinMode(REL_2, OUTPUT);
  pinMode(b1.pin, INPUT_PULLUP);
  digitalWrite(REL_1, HIGH);
  digitalWrite(REL_2, HIGH);
  uint8_t is_loaded = button_state_rom(&b1, 1, 'L');
  if (!is_loaded)
  {
    define_new_button(&b1);
    button_state_rom(&b1, 1, 'S');
  }
  m_state_rom(&light, 'L');
}

void loop()
{
  // put your main code here, to run repeatedly:
  BUTTON *arr[] = {&b1};
  handle_press_button(&b1);
  watching_buttons_state_changes(&light, arr, 1);
  handle_switching_light(&light);
  // change_light_mode(&light, &b1);
  // handle_switching_light(&light, &b1);

  // debug messages
  // if (millis() % 100 == 0)
  // {
  //   Serial.print("State: ");
  //   Serial.print(b1.state);
  //   Serial.print(" Last state: ");
  //   Serial.print(b1.last_state);
  //   Serial.print(" Light_state: ");
  //   Serial.print(light.light_state / 1000);
  //   Serial.print(" Light_mode: ");
  //   Serial.println(light.light_mode);
  // }
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
      }
      else if (current_time - signal_changed_time > max_depress_time_ms)
      {
        btn->type = 'L';
        btn->front = btn_prev_state;
        btn->is_defined = 1;
        btn->state = 0;
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
    if (current_signal_state != btn->last_pin_state)
    {
      // set button state dependig of button front
      btn->last_state_time = btn->current_state_time;
      btn->state = current_signal_state == btn->front ? 1 : 0;
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
        btn->last_state_time = btn->current_state_time;
        btn->state = !btn->state;
        btn->current_state_time = current_time;
      }
    }
  }
  btn->last_pin_state = current_signal_state;

  return btn->state;
}

void watching_buttons_state_changes(M_STATE *light, BUTTON *btns[], int btn_count)
{
  // This function watching for changing states of buttons and triggers nessesary functions or states in structs
  const int max_btn_count = 10;
  uint32_t current_time = millis();
  struct b_state
  {
    uint8_t b_pin;
    uint32_t last_b_state;
  };

  static struct b_state b_time[max_btn_count];

  for (int i = 0; i < btn_count; i++)
  {
    uint8_t matched = 0;
    for (int j = 0; j < btn_count; j++)
    {
      if (btns[i]->pin == b_time[j].b_pin)
      {
        if (btns[i]->state != b_time[j].last_b_state)
        {
          light->light_state = btns[i]->state;
          light->timestamp = current_time;
          change_light_mode(light, btns[i]);
        }
        b_time[j].last_b_state = btns[i]->state;
        matched = 1;
      }
    }
    if (!matched)
    {
      b_time[i].b_pin = btns[i]->pin;
      b_time[i].last_b_state = btns[i]->state;
    }
    matched = 0;
  }
}

void handle_switching_light(M_STATE *id)
{

  uint32_t current_time = millis();
  uint32_t timeout_ms = TIMEOUT * 1000;
  // handling timeout
  if (current_time - id->timestamp > timeout_ms)
  {
    id->light_state = 0;
  }

  if (id->light_state == 0)
  {
    digitalWrite(REL_1, HIGH);
    digitalWrite(REL_2, HIGH);
    return;
  }

  if (id->light_mode == 1)
  {
    digitalWrite(REL_1, LOW);
    digitalWrite(REL_2, HIGH);
  }
  else if (id->light_mode == 2)
  {
    digitalWrite(REL_1, HIGH);
    digitalWrite(REL_2, LOW);
  }
  else if (id->light_mode == 3)
  {
    digitalWrite(REL_1, LOW);
    digitalWrite(REL_2, LOW);
  }
  return;
}

void change_light_mode(M_STATE *light, BUTTON *btn)
{
  unsigned int double_click_interval_ms = 200;
  int max_light_mode = 3; // it can be retrive from count of relays if they control same group of devices. For future development
  uint32_t t2 = btn->current_state_time;
  uint32_t t1 = btn->last_state_time;

  if (t2 - t1 <= double_click_interval_ms && t1 != 0)
  {
    light->light_mode = light->light_mode > 1 ? light->light_mode - 1 : max_light_mode;
    m_state_rom(light, 'S');
  }
  return;
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

void m_state_rom(M_STATE *id, char action)
{
  // Need to verify saving and loaded data
  int address_offset_state = 0;
  int address_offset_mode = address_offset_state + sizeof(id->light_state);
  if (action == 'S')
  {
    uint8_t first_byte = EEPROM.put(address_offset_state, id->light_state);
    uint8_t second_byte = EEPROM.put(address_offset_mode, id->light_mode);
    Serial.println(first_byte);
    Serial.println(second_byte);
  }
  else if (action == 'L')
  {
    uint8_t first_byte = EEPROM.get(address_offset_state, id->light_state);
    uint8_t second_byte = EEPROM.get(address_offset_mode, id->light_mode);
    Serial.println(first_byte);
    Serial.println(second_byte);
  }
  return;
}

int button_state_rom(BUTTON *btn, uint8_t btn_number, char action)
{
  // Function save or load button state from EEPROM. action has 2 options: 'S' - save, 'L' - load
  // memory size occupied by M_STATE properties saved in EEPROM
  uint8_t m_state_address_offset = member_size(M_STATE, light_state) + member_size(M_STATE, light_mode);
  // memory size occupied by BUTTON properties saved in EEPROM
  uint8_t button_address_offset = member_size(BUTTON, pin) + member_size(BUTTON, type) + member_size(BUTTON, front);
  // address offset depending of number of button
  uint8_t address_offset = m_state_address_offset + button_address_offset * (btn_number - 1);
  uint8_t pin_offset = address_offset + sizeof(btn->pin);
  uint8_t type_offset = pin_offset + sizeof(btn->type);
  BUTTON backup = *btn;
  if (action == 'S')
  {
    Serial.print("Saving....");
    if (!btn->is_defined)
    {
      return 0;
    }

    if (btn->pin != EEPROM.put(address_offset, btn->pin))
    {
      return 0;
    }
    if (btn->type != EEPROM.put(pin_offset, btn->type))
    {
      return 0;
    }
    if (btn->front != EEPROM.put(type_offset, btn->front))
    {
      return 0;
    }
    return 1;
  }
  else if (action == 'L')
  {
    EEPROM.get(address_offset, backup.pin);
    EEPROM.get(pin_offset, backup.type);
    EEPROM.get(type_offset, backup.front);
    if (backup.pin > 13 || backup.pin < 2)
    {
      return 0;
    }
    if (backup.type != 'L' && backup.type != 'M')
    {
      return 0;
    }
    if (backup.front > 1)
    {
      return 0;
    }
    backup.state = 0;
    backup.last_state = 0;
    backup.last_pin_state = !backup.front;
    backup.current_state_time = millis();
    backup.last_state_time = 0;
    *btn = backup;
    return 1;
  }
  return 0;
}
