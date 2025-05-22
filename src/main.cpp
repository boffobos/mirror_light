#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

// Todo:
// + Timeout when light should turn off automaticaly
// + Working with few buttons
// + Posibility to load pins number and working mode from external source then initialize it programmaticaly
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
// + Relay struct and managing independent on relay type (high or low signal triggered) by sending array 1 and 0 [1,0] [1,1] [0,1] [0,0] etc.
// + Adaptive timeout (when after timeout event light is turned on in short period of time need to increase timeout, if not - decrease timeout)
// + If button type M when timeout happens may should reset button state to 0
// + Momentary button may should't have defined state. It should emits only changing signal levels.
// {
// Posibility to remove devices or reinitialize if device exists
// Additional to define new devices need to receive JSON with commands, that could change eg. light_state, light_mode, timeout etc.
// }
// At startup send to serial json exaples and description of work. It helps remember in future how to redefine settings etc.
/* extended JSON for commands and other options
  {
    "class": "D" | "C",                    // "D" - for device adding, "C" - for command
    ["action":"command"],                  // optional command text
    ["options": [1, 2, 3, "a", "b", "c"]], // optional additional options for command
    ["device": { }],                       // optional object with device configuration described above
  }
*/
////////////////////////////////////////
// Improvments:
// in function define new button need to change periodicaly pinMode to detect pressing button with different front
// Need to check loaded data for M_STATE from EEPROM
// May should blink with light when button is defined

#define member_size(type, member) (sizeof(((type *)0)->member))
#define INITIAL_TIMEOUT 20L
#define MAX_BUTTONS 5
#define MAX_RELAYS 5
#define START_BTN_PIN 2  // Define first GPIO in the row for buttons
#define END_BTN_PIN 13   // Define last GPIO in the row for buttons
#define START_REL_PIN A0 // Define first GPIO in the row for relays
#define END_REL_PIN A7   // Define first GPIO in the row for relays
#define JSON_BUFFER 64   // Buffer for incoming strings from Serial or other external sources
#define DEBUGING 0       // Switch some serial ouput for debuging purpose
#define CLEAN_ROM 1      // Erase EEPRON during setup(). For debuging
#define DOUBLE_CLICK_TIME 500

// Type and struct definitions:

struct M_STATE
{
  uint8_t light_state;      // Light can be turned on or off (1 or 0 respectively)
  uint8_t light_mode;       // Light mode can be cold white, warm white and mixed lamp turned on (1, 2, 3 respectively - like a byte [10000000], [01000000], [11000000])
  uint32_t timestamp;       // last system time light_mode has been changed
  uint8_t max_light_mode;   // depends on
  uint8_t avg_on_duration;  // part of timeout duration - avg_on_duration + delta = timeout
  uint8_t timeout_cooldown; // period of time in seconds after timeout is griggered it is possible to turn device on to increase timeout delay else it is derceased
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
  char type;     // 'L' - low triggered relay, 'H' - high triggered relay
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

M_STATE light; // need to initialize in runtime

char input_buffer[JSON_BUFFER];

// put function declarations here:
int digitalReadDebounce(int pin);
void define_new_button(BUTTON *btn);
int handle_press_button(BUTTON *btn);
uint8_t m_state_rom(M_STATE *id, char action);
int button_rom(BUTTON *btn, uint8_t btn_number, char action);
int relay_rom(RELAY *relay, uint8_t relay_number, char action);
void watching_buttons_state_changes(M_STATE *light, BUTTON *btns, int btn_count);
uint8_t set_relay_state(RELAY *relay, uint8_t to_state);
void handle_relays_switching(RELAY relays[], uint8_t control[]);
uint8_t dec_to_bin_arr(uint8_t number, uint8_t *arr, uint8_t arr_size);
void change_light_mode(M_STATE *light);
int toggle_light(M_STATE *light, uint8_t state);
void handle_switching_light(M_STATE *id);
uint8_t read_input(char *buf, int len);
PERIPHERALS handle_input(char *input);
int pin_to_int(const char *pin);
int power(int x, int y);
void handle_input_commands(char *input);

void setup()
{
  // put your setup code here, to run once:
  // Clean ROM before start
  if (CLEAN_ROM)
  {
    for (uint16_t i = 0; i < EEPROM.length(); i++)
    {
      EEPROM.put(i, 0);
    }
  }
  // debug_init();
  //////////////////
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
    light.light_state = 0;
    light.max_light_mode = power(2, relays_count) - 1;
    light.light_mode = light.max_light_mode;
    light.avg_on_duration = 0;
  }
  else
  {
    light.max_light_mode = power(2, relays_count) - 1;
    light.timeout_cooldown = 60;
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  for (int i = 0; i < buttons_count; i++)
  {
    handle_press_button(&buttons[i]);
    // debug messages
    if (DEBUGING)
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
    handle_input_commands(input_buffer);
    new_data = 0;
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
  if (DEBUGING)
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
  uint8_t current_signal = digitalReadDebounce(btn->pin);
  uint8_t last_signal = btn->last_pin_state;
  uint8_t front = btn->front;
  int8_t state = 0;
  unsigned long current_time = millis();

  if (current_signal != last_signal)
  {
    if (current_signal == HIGH)
    {
      if (front == 0)
      {
        state = -1;
      }
      else if (front == 1)
      {
        state = 1;
        // This is mean that double click counts between two fronts of signal of high-level button
        btn->last_state_time = btn->current_state_time;
        btn->current_state_time = current_time;
      }
    }
    else
    {
      if (front == 0)
      {
        state = 1;
        // This is mean that double click counts between two fronts of signal of low-level button
        btn->last_state_time = btn->current_state_time;
        btn->current_state_time = current_time;
      }
      else if (front == 1)
      {
        state = -1;
      }
    }
  }
  else
  {
    state = 0;
  }
  btn->last_pin_state = current_signal;
  btn->state = state;
  return btn->state;
}

void watching_buttons_state_changes(M_STATE *light, BUTTON *btns, int btn_count)
{
  // This function watching for changing states of buttons and triggers nessesary functions or states of objects
  uint32_t current_time = millis();
  static uint8_t is_double_click_waiting = 0;
  static uint32_t timestamp;

  for (int i = 0; i < btn_count; i++)
  {
    int8_t state = btns[i].state;
    int8_t type = btns[i].type;

    if (is_double_click_waiting == 1 && current_time - timestamp > DOUBLE_CLICK_TIME)
    {
      is_double_click_waiting = 0;
      timestamp = 0;
      toggle_light(light, !light->light_state);
    }

    if (state == 1)
    {
      if (is_double_click_waiting == 1)
      {
        is_double_click_waiting = 0;
        timestamp = 0;
        change_light_mode(light);

        // when light_mode changed during light is turned off need to light turn on
        if (light->light_state == 0)
          toggle_light(light, !light->light_state);
      }
      else
      {
        is_double_click_waiting = 1;
        timestamp = current_time;
      }
    }
    else if (state == -1)
    {
      if (type == 'L' && light->light_state == 1)
        toggle_light(light, !light->light_state);
    }
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
  // function convert number (represent light_mode) to array of 0 and 1 to control array of relays with this array. Nuber represent byte, every bit in this byte meaning relay state need to set up so it convert dec number to binary representation
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
  struct light_state
  {
    uint8_t prev_light_state : 1;
    uint8_t is_timeout : 1;   // if state was changed by timeout
    int8_t timeout_delay : 6; // additional time in minutes for id->average_on_duration to produce appropriate timeout time
  };
  uint32_t current_time = millis();
  static uint32_t timeout_timestamp = 0;
  static struct light_state conf = {0, 0, 0};
  uint32_t timeout_ms = ((uint32_t)(id->avg_on_duration + conf.timeout_delay)) * 60U * 1000U;
  uint8_t arr[relays_count];

  // handling timeout
  if (current_time - id->timestamp > timeout_ms && id->light_state == 1 && id->avg_on_duration != 0)
  {
    toggle_light(id, 0);
    conf.is_timeout = 1;
    timeout_timestamp = current_time;
  }

  if (id->light_state == 0)
  {
    uint8_t result = dec_to_bin_arr(0, arr, relays_count);
    if (result)
      handle_relays_switching(relays, arr);
  }
  else if (id->light_state == 1)
  {

    uint8_t result = dec_to_bin_arr(id->light_mode, arr, relays_count);
    if (result)
      handle_relays_switching(relays, arr);

    if (conf.is_timeout)
    {
      uint32_t t = (uint32_t)id->timeout_cooldown * 1000U;
      if (current_time - timeout_timestamp < t)
      {
        if (conf.timeout_delay < 31)
          conf.timeout_delay += 1;
        conf.is_timeout = 0;
      }
      else
      {
        if (conf.timeout_delay > -31)
          conf.timeout_delay -= 1;
        conf.is_timeout = 0;
      }
    }
  }
  conf.prev_light_state = id->light_state;

  return;
}

void change_light_mode(M_STATE *light)
{

  uint8_t max_light_mode = light->max_light_mode;

  if (max_light_mode == 0 && relays_count != 0)
    max_light_mode = power(2, relays_count) - 1;

  light->light_mode = light->light_mode > 1 ? light->light_mode - 1 : max_light_mode;
  light->light_mode = light->light_mode > max_light_mode ? max_light_mode : light->light_mode;
  m_state_rom(light, 'S');

  return;
}

int toggle_light(M_STATE *light, uint8_t state)
{
  uint32_t current_time = millis();
  const uint8_t iterations = 20;
  static uint8_t durations[iterations];
  static uint8_t index = 0;

  if (light->avg_on_duration == 0 && light->timestamp != 0)
  {
    if (state == 0)
    {
      uint8_t duration = (current_time - light->timestamp) / (1000U * 60U);
      light->avg_on_duration = duration;
    }
  }

  if (state == 0)
  {
    // after every 20th turn off it is calculated average duration device has been in ON state last 20 turns on. This average partialy defines timeout
    uint8_t duration = (current_time - light->timestamp) / (1000U * 60U);
    durations[index] = duration;
    index++;
    if (index >= iterations)
    {
      uint16_t summ = 0;
      for (int i = 0; i < iterations; i++)
        summ += durations[i];
      light->avg_on_duration = (uint8_t)((uint16_t)(light->avg_on_duration + summ) / (uint16_t)(iterations + 1));
      index = 0;
    }

    light->light_state = state;
    light->timestamp = current_time;
    m_state_rom(light, 'S');

    return 0;
  }
  else if (state == 1)
  {
    light->light_state = state;
    light->timestamp = current_time;
    return 1;
  }

  return 255;
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
  int address_offset_duration = 0;
  int address_offset_mode = address_offset_duration + sizeof(id->light_state);
  uint8_t max_mode = power(2, relays_count) - 1;
  if (action == 'S')
  {
    if (id->light_mode != EEPROM.put(address_offset_mode, id->light_mode))
      return 0;
    if (id->avg_on_duration != EEPROM.put(address_offset_duration, id->avg_on_duration))
      return 0;

    return 1;
  }
  else if (action == 'L')
  /* Need to check loaded data*/
  {
    // EEPROM.get(address_offset_state, id->light_state);
    id->light_state = 0; // set light off when boot
    EEPROM.get(address_offset_mode, id->light_mode);
    EEPROM.get(address_offset_duration, id->avg_on_duration);
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
  pin = pin <= 0 ? json["pin"] : pin; // It need cause json["pin"].as<const char *> can't convert number (without quotes) from json to string
  // Serial.print("Inside handle_input() pin: ");
  // Serial.println(pin);
  const char *device_type = json["device"];

  if (device_type[0] == 'B') // May use strcmp() instead
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
  // e.g. A0 -> 14, A1 -> 15, A2 -> 16, A3 -> 17, A4 -> 18, A5 -> 19 for arduino nano boards
  // Serial.print("Inside pin_to_int pin: ");
  // Serial.println(*pin);
  if (pin[0] == 'A')
  {
    int pin_number = atoi(pin + 1);
    Serial.println(pin_number);
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

int power(int x, int y)
{
  int res = 1;

  for (int i = 0; i < y; i++)
    res *= x;

  return res;
}

void handle_input_commands(char *input)
{
  const uint8_t device_max_chars = 64;
  JsonDocument json;
  DeserializationError err = deserializeJson(json, input);

  if (err)
    return;

  uint8_t is_pin = json["pin"].is<JsonVariant>();

  if (!json["class"].is<JsonVariant>() && !is_pin)
    return;

  if (strcmp(json["class"], "D") == 0 || is_pin) // hande new and old style json in the same time
  {
    if (!json["device"].is<JsonVariant>() && !is_pin)
      return;

    char device[device_max_chars];
    if (is_pin)
      serializeJson(json.as<JsonObject>(), device);
    else
      serializeJson(json["device"].as<JsonObject>(), device);

    Serial.println(device);

    PERIPHERALS new_dev = handle_input(device);
    if (new_dev.is_button || new_dev.is_relay)
    {
      Serial.print(F("New device - "));
      if (new_dev.is_button)
        Serial.println(F("button"));
      else if (new_dev.is_relay)
        Serial.println(F("relay"));
    }
    else
    {
      Serial.println("No new device received. Check sent data!");
    }

    if (new_dev.is_button)
    {
      // Check if provided pin not already used if it is re-define that button
      // If it new button and pin not used check if array of buttons not full
      define_new_button(new_dev.button);
      if (new_dev.button->is_defined)
      {
        // should check if there is button on this pin and rewrite this button is buttons array
        int saved = button_rom(new_dev.button, buttons_count, 'S');
        if (saved)
        {
          buttons[buttons_count] = *new_dev.button;
          Serial.println(F("Button saved to ROM"));
        }
        else
        {
          Serial.println(F("Button saving failed"));
        }
        // debug purpose
        if (DEBUGING)
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
        free(new_dev.button);
      }
    }
    else if (new_dev.is_relay)
    {
      if (relays_count < MAX_RELAYS)
      {
        relays[relays_count] = *new_dev.relay;
        int is_saved = relay_rom(&relays[relays_count], relays_count, 'S');
        if (is_saved)
          Serial.println("Relay saved to ROM");

        pinMode(relays[relays_count].pin, OUTPUT);
        relays_count++;
      }
      free(new_dev.relay);
    }
  }
  else if (strcmp(json["class"], "C") == 0)
  {
    Serial.println("Command");
  }
}