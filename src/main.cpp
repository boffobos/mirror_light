#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

// Todo:
// + Timeout when light should turn off automaticaly
// + Working with few buttons
// +/- Posibility to load pins number and working mode from external source then initialize it programmaticaly
//  Adaptive timeout (when after timeout event light is turned on in short period of time need to increase timeout, if not - decrease timeout)
/* JSON example for buttons
  {
    "pin":10,
    "device":"B",
  }
   JSON example for relay
  {
    "pin":"A1",
    "device":"R",
    "type":"L"
  }
*/
// If button type M when timeout happens may should reset button state to 0
// Relay struct and managing independent relay type (high or low signal triggered) by sending array 1 and 0 [1,0] [1,1] [0,1] [0,0] etc.
// Improvments:
// in function define new button need to change periodicaly pinMode to detect pressing button with different front
// Need to check loaded data for M_STATE from EEPROM

#define member_size(type, member) (sizeof(((type *)0)->member))
#define REL_1 A0
#define REL_2 A1
#define TIMEOUT 5L
#define MAX_BUTTONS 5
#define MAX_RELAYS 5
#define START_BTN_PIN 2  // Define first GPIO in the row for buttons
#define END_BTN_PIN 13   // Define last GPIO in the row for buttons
#define START_REL_PIN A0 // Define first GPIO in the row for relays
#define END_REL_PIN A7   // Define first GPIO in the row for relays

// Type and struct definitions:
struct M_STATE
{
  // Light can be turned on or off (1 or 0 respectively)
  uint8_t light_state;
  // Light mode can be cold white, warm white and mixed lamp turned on (1, 2, 3 respectively - like a byte [10000000], [01000000], [11000000])
  uint8_t light_mode;
  uint32_t timestamp;
};

struct BUTTON
{
  uint8_t is_defined;          // levels of definition: 0 - not defined, 1 - defined
  uint8_t pin;                 // button pin on arduino
  char type;                   // 'L' - self locked (maintained)  button type, 'M' - momentary button type
  uint8_t front;               // 0 if pushed button connects to GND, 1 if connect to VCC
  uint8_t state;               // current state of button: 0 - off state, 1 - on state
  uint8_t last_state;          // previous cycle button state
  uint8_t last_pin_state;      // previous cycle state on button pin: 0 - off state, 1 - on state
  uint32_t current_state_time; // time when current state was changed to ON state
  uint32_t last_state_time;    // last time when state was changed to ON state
};

struct RELAY
{
  uint8_t pin;   // that is pin relay connected to
  char type;     // 'L' - low state relay, 'H' - high state relay (triggered by LOW of HIGH signar respectively)
  uint8_t state; // 0 - relay is turned off, 1 - turned on
};

struct PERIPHERALS
{
  struct BUTTON *button;
  uint8_t is_button; // 1 - new button arriver, 0 - no
  struct RELAY *relay;
  uint8_t is_relay; // 1 - new relay arrived, 0 - no
};

// Global variables:

struct BUTTON buttons[MAX_BUTTONS];
uint8_t buttons_count = 0;

struct RELAY relays[MAX_RELAYS];
uint8_t relays_count = 0;

M_STATE light = {0, 3, 0};
// BUTTON b1 = {
//     0,
//     12,
//     127,
//     255,
//     255,
//     255,
//     0,
//     0};
// BUTTON b2 = {
//     0,
//     11,
//     127,
//     255,
//     255,
//     255,
//     0,
//     0};
uint8_t const buf_len = 64;
char input_buffer[buf_len];

JsonDocument json;

// put function declarations here:
int digitalReadDebounce(int pin);
void define_new_button(BUTTON *btn);
int handle_press_button(BUTTON *btn);
void m_state_rom(M_STATE *id, char action);
int button_state_rom(BUTTON *btn, uint8_t btn_number, char action);
void watching_buttons_state_changes(M_STATE *light, BUTTON btns[], int btn_count);
void change_light_mode(M_STATE *light, BUTTON *btn);
void handle_switching_light(M_STATE *id);
uint8_t read_input(char *buf, int len);
PERIPHERALS handle_input(char *input);

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(REL_1, OUTPUT);
  pinMode(REL_2, OUTPUT);
  digitalWrite(REL_1, HIGH);
  digitalWrite(REL_2, HIGH);

  // pinMode(b1.pin, INPUT_PULLUP);
  // pinMode(b2.pin, INPUT_PULLUP);
  // Serial.print("Is loaded: ");
  // Serial.println(is_loaded);
  for (int i = 0; i < MAX_BUTTONS; i++)
  {
    uint8_t is_loaded = button_state_rom(&buttons[i], i, 'L');
    if (is_loaded)
    {
      pinMode(buttons[i].pin, INPUT_PULLUP);
      buttons_count++;
    }
  }
  Serial.print("Number of buttons: ");
  Serial.println(buttons_count);
  // else if (!is_loaded2)
  // {
  //   define_new_button(&b2);
  //   button_state_rom(&b2, 2, 'S');
  // }
  m_state_rom(&light, 'L');
}

void loop()
{
  // put your main code here, to run repeatedly:
  // BUTTON *arr[] = {&b1, &b2};
  for (int i = 0; i < buttons_count; i++)
  {
    handle_press_button(&buttons[i]);
  }
  watching_buttons_state_changes(&light, buttons, buttons_count);
  handle_switching_light(&light);
  uint8_t new_data = read_input(input_buffer, buf_len);
  if (new_data)
  {
    PERIPHERALS new_devices = handle_input(input_buffer);
    Serial.println("New device: ");
    Serial.print("is_button: ");
    Serial.println(new_devices.is_button);
    Serial.print("is_relay: ");
    Serial.println(new_devices.is_relay);
    if (new_devices.is_button)
    {
      define_new_button(new_devices.button);
      int saved = button_state_rom(&buttons[buttons_count], buttons_count, 'S');
      if (saved)
        Serial.println("Button saved to ROM");
      else
        Serial.println("Button saving failed");
      // buttons[buttons_count] = *new_devices.button;
      buttons_count = buttons_count + 1;
      Serial.print(" Buttons count: ");
      Serial.println(buttons_count);
      Serial.print("New button pin: ");
      Serial.println(buttons[buttons_count - 1].pin);
      Serial.print("New button def: ");
      Serial.println(buttons[buttons_count - 1].is_defined);
      Serial.print("New button type: ");
      Serial.println(buttons[buttons_count - 1].type);
      Serial.print("New button front: ");
      Serial.println(buttons[buttons_count - 1].front);
    }
    else if (new_devices.is_relay)
    {
      Serial.println("Relay");
    }
    Serial.println("Finishing device definition");
    // Serial.println(input_buffer);
    // Serial.println(pin);
  }
  // debug messages
  // if (millis() % 100 == 0)
  // {
  //   Serial.print("B1 State: ");
  //   Serial.print(b1.state);
  //   Serial.print(" B1 Last state: ");
  //   Serial.print(b1.last_state);
  //   Serial.print(" B2 State: ");
  //   Serial.print(b2.state);
  //   Serial.print(" B2 Last state: ");
  //   Serial.print(b2.last_state);
  //   Serial.print(" Light_state: ");
  //   Serial.print(light.light_state);
  //   Serial.print(" Light_mode: ");
  //   Serial.println(light.light_mode);
  // }
}
// put function definitions here:
void define_new_button(BUTTON *btn)
{
  // on the first power on this function should recognize button type: e.g. maintained or momentary, high or low level
  pinMode(btn->pin, INPUT_PULLUP);

  unsigned long max_depress_time_ms = 300; // time need to depress and release momentary button
  int current_signal_state = digitalReadDebounce(btn->pin);
  unsigned long current_time = millis();
  unsigned long signal_changed_time = 0;
  int btn_prev_state = current_signal_state;

  Serial.println("From define_new_button");
  Serial.print("Button pin: ");
  Serial.print(btn->pin);
  Serial.print(" Button def: ");
  Serial.println(btn->is_defined);

  while (btn->is_defined != 1)
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

void watching_buttons_state_changes(M_STATE *light, BUTTON btns[], int btn_count)
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
      if (btns[i].pin == b_time[j].b_pin)
      {
        if (btns[i].state != b_time[j].last_b_state)
        {
          light->light_state = btns[i].state;
          light->timestamp = current_time;
          change_light_mode(light, &btns[i]);
        }
        b_time[j].last_b_state = btns[i].state;
        matched = 1;
      }
    }
    if (!matched)
    {
      b_time[i].b_pin = btns[i].pin;
      b_time[i].last_b_state = btns[i].state;
    }
    matched = 0;
  }
}

void handle_switching_light(M_STATE *id)
{

  uint32_t current_time = millis();
  uint32_t timeout_ms = TIMEOUT * 1000 * 60;
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
    /* Need to check loaded data*/
    EEPROM.put(address_offset_state, 0); // always boot with turned off light
    EEPROM.put(address_offset_mode, id->light_mode);
    // uint8_t first_byte = EEPROM.put(address_offset_state, 0);
    // uint8_t second_byte = EEPROM.put(address_offset_mode, id->light_mode);
    //   Serial.print("Saved: light state - ");
    //   Serial.print(first_byte);
    //   Serial.print(" light mode - ");
    //   Serial.println(second_byte);
  }
  else if (action == 'L')
  {
    EEPROM.get(address_offset_state, id->light_state);
    EEPROM.get(address_offset_mode, id->light_mode);
    // uint8_t first_byte = EEPROM.get(address_offset_state, id->light_state);
    // uint8_t second_byte = EEPROM.get(address_offset_mode, id->light_mode);
    // Serial.print("Loaded: light state - ");
    // Serial.print(first_byte);
    // Serial.print(" light mode - ");
    // Serial.println(second_byte);
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
  uint8_t pin_offset = m_state_address_offset + button_address_offset * btn_number;
  uint8_t type_offset = pin_offset + member_size(BUTTON, pin);
  uint8_t front_offset = type_offset + member_size(BUTTON, type);
  BUTTON backup;
  if (action == 'S')
  {
    Serial.println("Saving....");
    if (!btn->is_defined)
      return 0;

    if (btn->pin < START_BTN_PIN || btn->pin > END_BTN_PIN)
      return 0;

    if (btn->front != 0 && btn->front != 1)
      return 0;

    if (btn->type != 'L' && btn->type != 'M')
      return 0;

    if (btn->pin != EEPROM.put(pin_offset, btn->pin))
    {
      return 0;
    }
    if (btn->type != EEPROM.put(type_offset, btn->type))
    {
      return 0;
    }
    if (btn->front != EEPROM.put(front_offset, btn->front))
    {
      return 0;
    }
    return 1;
  }
  else if (action == 'L')
  {
    EEPROM.get(pin_offset, backup.pin);
    EEPROM.get(type_offset, backup.type);
    EEPROM.get(front_offset, backup.front);
    if (backup.pin > END_BTN_PIN || backup.pin < START_BTN_PIN)
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

uint8_t read_input(char *buf, int len)
{
  // Function read data from serial to array and emits 0 if data still not finish or nothing and 1 when receiving data finish
  static uint8_t ndx = 0;
  static uint8_t receiving = 0;
  char starting_byte = '{'; // receiving only json formated string started from { and ending with \n
  char finish_byte = '\n';
  char byte;

  while (Serial.available() > 0)
  {
    byte = Serial.read();
    if (ndx >= len)
    {
      ndx = len - 1;
    }
    if (receiving)
    {
      if (byte != finish_byte)
      {
        buf[ndx] = byte;
        ndx++;
      }
      else if (byte == finish_byte)
      {
        buf[ndx] = '\0';
        receiving = 0;
        ndx = 0;
        return 1;
      }
    }
    else if (byte == starting_byte)
    {
      receiving = 1;
      buf[ndx] = byte;
      ndx++;
    }
  }
  return 0;
}

PERIPHERALS handle_input(char *input)
{
  // It is possible return pointer or struct it self. Whan is the best from performans POV?
  PERIPHERALS dev;
  JsonDocument json;
  uint8_t invalid_param = 127;
  BUTTON *btn = &buttons[buttons_count];
  RELAY *relay = &relays[relays_count];

  Serial.println(input);
  DeserializationError error = deserializeJson(json, input);
  Serial.println(error.c_str());
  if (error)
  {
    dev.is_button = 0;
    dev.button = 0;
    dev.is_relay = 0;
    dev.relay = 0;
    return dev;
  }

  serializeJson(json, Serial);
  int pin = json["pin"];
  const char *device_type = json["device"];
  Serial.print("Pin: ");
  Serial.println(pin);
  Serial.print("device_type: ");
  Serial.println(device_type[0]);
  if (device_type[0] == 'B')
  {
    dev.is_relay = 0;
    dev.relay = 0;
    btn->pin = (pin >= START_BTN_PIN ? (pin <= END_BTN_PIN ? pin : invalid_param) : invalid_param);
    if (btn->pin == invalid_param)
    {
      dev.is_button = 0;
      dev.button = 0;
    }
    btn->is_defined = 0;
    dev.is_button = 1;
    dev.button = btn;
  }
  else if (device_type[0] == 'R')
  {
    dev.is_button = 0;
    dev.button = 0;

    const char *rel_type = json["type"];
    Serial.print("rel_type: ");
    Serial.println(rel_type[0]);
    relay->pin = (pin >= START_REL_PIN ? (pin <= END_REL_PIN ? pin : invalid_param) : invalid_param); // Check if pin in right range
    relay->type = ((rel_type[0] == 'L') || (rel_type[0] == 'H') ? rel_type[0] : invalid_param);       // Check if json have only H of L for relay type
    if (relay->pin == invalid_param || relay->type == invalid_param)
    {
      dev.is_relay = 0;
      dev.relay = 0;
      return dev;
    }
    dev.is_relay = 1;
    dev.relay = relay;
  }
  else
  {
    dev.is_button = 0;
    dev.button = 0;
    dev.is_relay = 0;
    dev.relay = 0;
    return dev;
  }
  return dev;
}