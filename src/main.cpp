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
// At startup send to serial json exaples and description of work. It helps remember in future how to redefine settings etc.
// If button type M when timeout happens may should reset button state to 0
// Posibility to remove devices
// Relay struct and managing independent relay type (high or low signal triggered) by sending array 1 and 0 [1,0] [1,1] [0,1] [0,0] etc.
// Improvments:
// in function define new button need to change periodicaly pinMode to detect pressing button with different front
// Need to check loaded data for M_STATE from EEPROM
// momentary button may should't have defined state. It should emits only changing signal levels.

#define member_size(type, member) (sizeof(((type *)0)->member))
#define TIMEOUT 5L
#define MAX_BUTTONS 5
#define MAX_RELAYS 5
#define START_BTN_PIN 2  // Define first GPIO in the row for buttons
#define END_BTN_PIN 13   // Define last GPIO in the row for buttons
#define START_REL_PIN A0 // Define first GPIO in the row for relays
#define END_REL_PIN A7   // Define first GPIO in the row for relays
#define JSON_BUFFER 64   // Buffer for incoming strings from Serial or other external sources
#define DEBUG 0          // Switch some serial ouput

// Type and struct definitions:

struct M_STATE
{
  // Light can be turned on or off (1 or 0 respectively)
  uint8_t light_state;
  // Light mode can be cold white, warm white and mixed lamp turned on (1, 2, 3 respectively - like a byte [10000000], [01000000], [11000000])
  uint8_t light_mode;
  uint32_t timestamp;
  uint8_t max_light_mode;
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

M_STATE light = {0, 3, 0, 0};

char input_buffer[JSON_BUFFER];

// put function declarations here:
int digitalReadDebounce(int pin);
void define_new_button(BUTTON *btn);
int handle_press_button(BUTTON *btn);
uint8_t m_state_rom(M_STATE *id, char action);
int button_rom(BUTTON *btn, uint8_t btn_number, char action);
int relay_rom(RELAY *relay, uint8_t relay_number, char action);
void watching_buttons_state_changes(M_STATE *light, BUTTON btns[], int btn_count);
uint8_t set_relay_state(RELAY *relay, uint8_t to_state);
void handle_relays_switching(RELAY relays[], uint8_t control[]);
uint8_t dec_to_bin_arr(uint8_t number, uint8_t arr, uint8_t arr_size);
void change_light_mode(M_STATE *light, BUTTON *btn);
void handle_switching_light(M_STATE *id);
uint8_t read_input(char *buf, int len);
PERIPHERALS handle_input(char *input);
int pin_to_int(const char *pin);

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println(F("Loading relays..."));
  for (int i = 0; i < MAX_RELAYS; i++)
  {
    uint8_t is_loaded = relay_rom(&relays[i], i, 'L');
    if (is_loaded)
    {
      // Serial.println(F("Relay loaded succesfully"));
      pinMode(relays[i].pin, OUTPUT);
      set_relay_state(&relays[i], 0);
      relays_count++;
    }
    else
    {
      // Serial.println(F("Failed to load relay config from ROM"));
    }
  }
  Serial.print(F("Relays count: "));
  Serial.println(relays_count);

  Serial.println(F("Loading buttons..."));
  for (int i = 0; i < MAX_BUTTONS; i++)
  {
    uint8_t is_loaded = button_rom(&buttons[i], i, 'L');
    if (is_loaded)
    {
      pinMode(buttons[i].pin, INPUT_PULLUP);
      buttons_count++;
    }
    else
    {
      // Serial.println(F("Faled to load button config from ROM"));
    }
  }
  Serial.print(F("Buttons count: "));
  Serial.println(buttons_count);

  uint8_t is_loaded = m_state_rom(&light, 'L');
  if (!is_loaded)
  {
    Serial.println(F("Light config failed to load from ROM"));
  }
  else
  {
    uint8_t max_mode = 1;
    for (int i = 0; i < relays_count; i++)
      max_mode *= 2;

    light.max_light_mode = max_mode - 1;
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  for (int i = 0; i < buttons_count; i++)
  {
    handle_press_button(&buttons[i]);
    // debug messages
    if (DEBUG)
    {

      if (millis() % 1000 == 0)
      {
        Serial.print("B");
        Serial.print(i);
        Serial.print(" State: ");
        Serial.print(buttons[i].state);
        Serial.print(" Light_state: ");
        Serial.print(light.light_state);
        Serial.print(" Light_mode: ");
        Serial.print(light.light_mode);
        for (int j = 0; j < relays_count; j++)
        {
          Serial.print(" Rel_");
          Serial.print(j);
          Serial.print(" ");
          Serial.print(relays[j].state);
        }
        Serial.println();
      }
    }
    //
  }
  watching_buttons_state_changes(&light, buttons, buttons_count);
  handle_switching_light(&light);
  uint8_t new_data = read_input(input_buffer, JSON_BUFFER);
  if (new_data)
  {
    PERIPHERALS new_devices = handle_input(input_buffer);
    Serial.println("New device: ");
    Serial.print(F("is_button: "));
    Serial.println(new_devices.is_button);
    Serial.print(F("is_relay: "));
    Serial.println(new_devices.is_relay);
    if (new_devices.is_button)
    {
      // Check if provided pin not already used if it is re-define that button
      // If it new button and pin not used check if array of buttons not full
      define_new_button(new_devices.button);
      if (new_devices.button->is_defined)
      {
        int saved = button_rom(new_devices.button, buttons_count, 'S');
        if (saved)
        {
          buttons[buttons_count] = *new_devices.button;
          Serial.println("Button saved to ROM");
        }
        else
        {
          Serial.println("Button saving failed");
        }
        // debug purpose
        if (DEBUG)
        {
          Serial.print(F(" Buttons count: "));
          Serial.println(buttons_count);
          Serial.print(F("New button pin: "));
          Serial.println(buttons[buttons_count].pin);
          Serial.print(F("New button def: "));
          Serial.println(buttons[buttons_count].is_defined);
          Serial.print(F("New button type: "));
          Serial.println(buttons[buttons_count].type);
          Serial.print(F("New button front: "));
          Serial.println(buttons[buttons_count].front);
        }
        //
        buttons_count = buttons_count + 1;
        free(new_devices.button);
      }
    }
    else if (new_devices.is_relay)
    {
      if (relays_count < MAX_RELAYS)
      {
        relays[relays_count] = *new_devices.relay;
        int is_saved = relay_rom(&relays[relays_count], relays_count, 'S');
        if (is_saved)
          Serial.println("Relay saved to ROM");

        pinMode(relays[relays_count].pin, OUTPUT);
        relays_count++;
      }
      free(new_devices.relay);
    }
    // Serial.println("Finishing device definition");
    // Serial.println(input_buffer);
    // Serial.println(pin);
  }
}
// put function definitions here:
void define_new_button(BUTTON *btn)
{
  // on the first power on this function should recognize button type: e.g. maintained or momentary, high or low level
  if (btn->pin < START_BTN_PIN || btn->pin > END_BTN_PIN)
    return;

  pinMode(btn->pin, INPUT_PULLUP);

  unsigned long max_depress_time_ms = 300; // time need to depress and release momentary button
  int current_signal_state = digitalReadDebounce(btn->pin);
  unsigned long current_time = millis();
  unsigned long signal_changed_time = 0;
  int btn_prev_state = current_signal_state;

  // Debug purpose
  if (DEBUG)
  {
    Serial.println("From define_new_button");
    Serial.print("Button pin: ");
    Serial.print(btn->pin);
    Serial.print(" Button def: ");
    Serial.println(btn->is_defined);
  }
  //

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
  const int max_btn_count = MAX_BUTTONS;
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
          light->light_state = btns[i].state; // change avoid dependecy of light_state on button_state. Changing button states shoul !switch light state
          light->timestamp = current_time;
          change_light_mode(light, &btns[i]); // to check if pressing was double or not
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

uint8_t set_relay_state(RELAY *relay, uint8_t to_state)
{
  // Function take relay pointer and change state to on(1) or off(0)
  if (relay->type == 'H')
  {
    if (to_state == 1)
    {
      digitalWrite(relay->pin, HIGH);
      relay->state = to_state;
    }
    else if (to_state == 0)
    {
      digitalWrite(relay->pin, LOW);
      relay->state = to_state;
    }
  }
  else if (relay->type == 'L')
  {
    if (to_state == 1)
    {
      digitalWrite(relay->pin, LOW);
      relay->state = to_state;
    }
    else if (to_state == 0)
    {
      digitalWrite(relay->pin, HIGH);
      relay->state = to_state;
    }
  }
  else
  {
    relay->state = 0;
  }
  return relay->state;
}

void handle_relays_switching(RELAY relays[], uint8_t control[])
{
  // Function iterates throug relays array and swiched state according control byte

  // check if control have valid values (0 or 1)

  for (int i = 0; i < relays_count; i++)
  {
    if (control[i] > 1)
      continue;
    set_relay_state(&relays[i], control[i]);
  }
}

uint8_t dec_to_bin_arr(uint8_t number, uint8_t *arr, uint8_t arr_size)
{
  int dividend = number;
  for (int i = 0; i < arr_size; i++)
  {
    arr[i] = dividend % 2;
    dividend = dividend / 2;
  }
  return 1;
}

void handle_switching_light(M_STATE *id)
{

  uint32_t current_time = millis();
  uint32_t timeout_ms = TIMEOUT * 1000 * 60;
  uint8_t arr[relays_count];
  // handling timeout
  if (current_time - id->timestamp > timeout_ms)
  {
    id->light_state = 0;
  }

  if (id->light_state == 0)
  {
    uint8_t result = dec_to_bin_arr(0, arr, relays_count);
    if (result)
      handle_relays_switching(relays, arr);
    // set_relay_state(&relays[0], 0);
    // set_relay_state(&relays[1], 0);
    return;
  }
  else if (id->light_state == 1)
  {
    uint8_t result = dec_to_bin_arr(id->light_mode, arr, relays_count);
    if (result)
      handle_relays_switching(relays, arr);
    return;
  }

  if (id->light_mode == 1)
  {
    set_relay_state(&relays[0], 1);
    set_relay_state(&relays[1], 0);
  }
  else if (id->light_mode == 2)
  {
    set_relay_state(&relays[0], 0);
    set_relay_state(&relays[1], 1);
  }
  else if (id->light_mode == 3)
  {
    set_relay_state(&relays[0], 1);
    set_relay_state(&relays[1], 1);
  }
  return;
}

void change_light_mode(M_STATE *light, BUTTON *btn)
{
  unsigned int double_click_interval_ms = 200;
  uint32_t t2 = btn->current_state_time;
  uint32_t t1 = btn->last_state_time;

  uint8_t max_light_mode = light->max_light_mode;
  if (max_light_mode == 0 && relays_count != 0)
  {
    max_light_mode = 1;
    for (int i = 0; i < relays_count; i++)
      max_light_mode *= 2;
    max_light_mode = max_light_mode - 1;
  }
  // may need to change this code to not use btn
  light->light_mode = light->light_mode > max_light_mode ? max_light_mode : light->light_mode;
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

uint8_t m_state_rom(M_STATE *id, char action)
{
  // Need to verify saving and loaded data
  int address_offset_state = 0;
  int address_offset_mode = address_offset_state + sizeof(id->light_state);
  uint8_t max_mode = 3;
  if (action == 'S')
  {
    /* Need to check loaded data*/
    if (id->light_mode != EEPROM.put(address_offset_mode, id->light_mode))
      return 0;

    return 1;
  }
  else if (action == 'L')
  {
    // EEPROM.get(address_offset_state, id->light_state);
    id->light_state = 0; // set light off when boot
    EEPROM.get(address_offset_mode, id->light_mode);
    if (id->light_mode < 1 || id->light_mode > max_mode)
      id->light_mode = max_mode;

    return 1;
  }
  return 0;
}

int button_rom(BUTTON *btn, uint8_t btn_number, char action)
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
  if (action == 'S')
  {
    // Serial.println(F("Button saving..."));
    if (!btn->is_defined)
      return 0;

    if (btn->pin < START_BTN_PIN || btn->pin > END_BTN_PIN)
      return 0;

    if (btn->front != 0 && btn->front != 1)
      return 0;

    if (btn->type != 'L' && btn->type != 'M')
      return 0;

    if (btn->pin != EEPROM.put(pin_offset, btn->pin))
      return 0;

    if (btn->type != EEPROM.put(type_offset, btn->type))
      return 0;

    if (btn->front != EEPROM.put(front_offset, btn->front))
      return 0;

    return 1;
  }
  else if (action == 'L')
  {
    // Serial.println(F("Button loading..."));
    BUTTON backup;
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

int relay_rom(RELAY *relay, uint8_t relay_number, char action) // Function save or load relay data from/to EEPROM
{
  // Function save or load relay data from/to EEPROM
  uint8_t m_state_address_offset = member_size(M_STATE, light_state) + member_size(M_STATE, light_mode);
  // memory size occupied by BUTTON properties saved in EEPROM
  uint8_t button_address_offset = member_size(BUTTON, pin) + member_size(BUTTON, type) + member_size(BUTTON, front);
  // memory size occupied by RELAY properties saved in EEPROM
  uint8_t relay_address_offset = member_size(RELAY, pin) + member_size(RELAY, type);
  uint8_t pin_offset = m_state_address_offset + button_address_offset * MAX_BUTTONS + relay_address_offset * relay_number;
  uint8_t type_offset = pin_offset + member_size(RELAY, pin);

  if (action == 'S')
  {
    // Serial.println(F("Saving relay..."));
    if (relay->pin < START_REL_PIN || relay->pin > END_REL_PIN)
      return 0;
    if (relay->type != 'H' && relay->type != 'L')
      return 0;
    if (relay->pin != EEPROM.put(pin_offset, relay->pin))
      return 0;
    if (relay->type != EEPROM.put(type_offset, relay->type))
      return 0;

    return 1;
  }
  else if (action == 'L')
  {
    // Serial.println(F("Loading relay..."));
    RELAY temp_rel;
    EEPROM.get(pin_offset, temp_rel.pin);
    EEPROM.get(type_offset, temp_rel.type);

    temp_rel.state = 0;
    if (temp_rel.pin < START_REL_PIN || temp_rel.pin > END_REL_PIN)
      return 0;
    if (temp_rel.type != 'H' && temp_rel.type != 'L')
      return 0;
    *relay = temp_rel;

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
  // This functions get received string from serial and tranform to json and analize what device is being added and return struct with pointer to cell in global arrays of devices and what kind of device it is being added
  PERIPHERALS dev;
  JsonDocument json;
  uint8_t invalid_param = 127;
  // need to handle case when array reaches maximum buttons
  // BUTTON *btn = &buttons[buttons_count]; // get not accupied array member with buttons
  // RELAY *relay = &relays[relays_count]; // get not occupied array member with relays

  DeserializationError error = deserializeJson(json, input);

  if (error)
  {
    dev.is_button = 0;
    dev.button = 0;
    dev.is_relay = 0;
    dev.relay = 0;
    return dev;
  }

  int pin = pin_to_int(json["pin"].as<const char *>());
  pin = pin <= 0 ? json["pin"] : pin; // It need cause json["pin"].as<const char *> can't convert number from json to string
  const char *device_type = json["device"];

  if (device_type[0] == 'B')
  {
    BUTTON *btn = (BUTTON *)malloc(sizeof(BUTTON));
    if (!btn)
    {
      dev.is_button = 0;
      dev.button = 0;
      dev.is_relay = 0;
      dev.relay = 0;
      return dev;
    }

    dev.is_relay = 0;
    dev.relay = 0;

    btn->pin = (pin >= START_BTN_PIN ? (pin <= END_BTN_PIN ? pin : invalid_param) : invalid_param);
    if (btn->pin == invalid_param)
    {
      dev.is_button = 0;
      dev.button = 0;
    }
    else
    {
      btn->is_defined = 0;
      dev.is_button = 1;
      dev.button = btn;
    }
  }
  else if (device_type[0] == 'R')
  {
    RELAY *relay = (RELAY *)malloc(sizeof(RELAY));
    if (!relay)
    {
      dev.is_button = 0;
      dev.button = 0;
      dev.is_relay = 0;
      dev.relay = 0;
      return dev;
    }
    const char *rel_type = json["type"];

    dev.is_button = 0;
    dev.button = 0;
    relay->pin = (pin >= START_REL_PIN ? (pin <= END_REL_PIN ? pin : invalid_param) : invalid_param); // Check if pin in right range
    relay->type = ((rel_type[0] == 'L') || (rel_type[0] == 'H') ? rel_type[0] : invalid_param);       // Check if json have only H of L for relay type
    if (relay->pin == invalid_param || relay->type == invalid_param)
    {
      dev.is_relay = 0;
      dev.relay = 0;
    }
    else
    {
      dev.is_relay = 1;
      dev.relay = relay;
    }
  }
  else
  {
    dev.is_button = 0;
    dev.button = 0;
    dev.is_relay = 0;
    dev.relay = 0;
  }

  return dev;
}

int pin_to_int(const char *pin)
{
  // Function convert analog pin number in string form from input to digital pin number
  // e.g. A0 -> 14, A1 -> 15, A2 -> 16, A3 -> 17, A4 -> 18, A5 -> 19
  Serial.print("Inside pin_to_int pin: ");
  Serial.println(pin);
  if (pin[0] == 'A')
  {
    int pin_number = atoi(pin + 1);
    if (pin_number + 14 >= A0 && pin_number + 14 <= A7)
    {
      return pin_number + 14;
    }
  }
  else if (pin[0] == 'D')
  {
    int pin_number = atoi(pin + 1);
    if (pin_number >= START_BTN_PIN && pin_number <= END_BTN_PIN)
    {
      return pin_number;
    }
  }
  else
  {
    int pin_number = atoi(pin);
    if ((pin_number >= START_BTN_PIN && pin_number <= END_BTN_PIN) ||
        (pin_number >= START_REL_PIN && pin_number <= END_REL_PIN))
    {
      return pin_number;
    }
  }
  // If pin is not in range return -1
  return -1;
}