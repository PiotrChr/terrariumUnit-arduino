#include <pnew.cpp>
#include <iterator>
#include <map>
#include <string>

#include <OneWire.h> 
#include <DallasTemperature.h>
#include <CircularBuffer.h>
#include <Wire.h>

const int SECTIONS = 2;

// Buttons 
const int THERMOSTATE_BUTTON_PIN_1 = 10;
const int THERMOSTATE_BUTTON_PIN_2 = 11;
const int buttons[SECTIONS] = {THERMOSTATE_BUTTON_PIN_1, THERMOSTATE_BUTTON_PIN_2};
int buttonStates[SECTIONS] = {LOW, LOW};

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 150;
unsigned long lastButtonLockTime = millis();
const unsigned long buttonLockTime = 3000;
bool buttonLocked = false;

// Leds
const int THERMOSTATE_LED_PIN_1 = 12;
const int THERMOSTATE_LED_PIN_2 = 5;

const int leds[SECTIONS] = {THERMOSTATE_LED_PIN_1, THERMOSTATE_LED_PIN_2};
int ledStates[SECTIONS] = {HIGH, HIGH};

// Relays
const int RELAY_PIN_1 = 6;
const int RELAY_PIN_2 = 7;
const int RELAY_PIN_3 = 8;
const int RELAY_PIN_4 = 9;

// Thermometer
const int THERMOMETR_PIN_1 = 4;
const int THERMOMETR_PIN_2 = 3;

// I2C 
const int I2C_SLAVE_ADDRESS = 11;
const int PI_MESSAGE_SIZE = 2;

bool debug = true;

/**
 * Instructions:
 * 10.x.xx - (InstructionNumber.Params.Value)
 * 
 * 10xxx       -    ChangeTargetTemp
 * 110xx       -    ChangeTempOffset
 * 12xxx       -    ChangeTempLock
 * 13x02/13x01 -    Toggle (Disable/Enable) Thermostat
 * 14x00       -    ThermostatStatus
 * 
 */
 
const int PI_MESSAGE_SET_TARGET_TEMP = 10;
const std::string PI_MESSAGE_SET_TARGET_TEMP_REF = "setTargetTemp";

const int PI_MESSAGE_SET_TEMP_OFFSET = 11;
const std::string PI_MESSAGE_SET_TEMP_OFFSET_REF = "setTempOffset";

const int PI_MESSAGE_SET_TEMP_LOCK = 12;
const std::string PI_MESSAGE_SET_TEMP_LOCK_REF = "setTempLock";

const int PI_MESSAGE_TOGGLE_THERMOSTAT = 13;
const std::string PI_MESSAGE_TOGGLE_THERMOSTAT_REF = "toggleThermostat";

const int PI_MESSAGE_THERMOSTAT_STATUS = 14;
const std::string PI_MESSAGE_THERMOSTAT_STATUS_REF = "thermostatStatus";

const int DEFAULT_TARGET_TEMP = 25;
const int DEFAULT_TEMP_OFFSET = 5;
const int DEFAULT_INIT_TEMP = 0;

OneWire bus1(THERMOMETR_PIN_1);
OneWire bus2(THERMOMETR_PIN_2);

DallasTemperature dtemp1(&bus1);
DallasTemperature dtemp2(&bus2);

unsigned long tempLockTime = 3000;
unsigned long lastTempLockTime = millis();

bool thermostateOn = true;
int disabled[SECTIONS] = {0,0};


// TODO: Create a struct for those values
int targetTemp[SECTIONS] = {DEFAULT_TARGET_TEMP, DEFAULT_TARGET_TEMP};
int tempOffset[SECTIONS] = {DEFAULT_TEMP_OFFSET, DEFAULT_TEMP_OFFSET};
int currTemp[SECTIONS] = {DEFAULT_INIT_TEMP, DEFAULT_INIT_TEMP};

std::map<int, std::string> instructionMap;

typedef void (*Handler)(int param, int value);
std::map<std::string, Handler> handlers;


struct Instruction {
  int id;
  std::string name;
  int param;
  int value;
};

typedef void (*Job)();
struct Task {
  std::string name;
  unsigned long dueDate;
  Job job;
};

const int taskLimit = 20;
CircularBuffer <Task, taskLimit> tasks;

void setup() {
  Serial.begin(9600);
  
  setupInstructionMap();
  setupI2c();
  setupRelay();
  setupSensors();
  setupButtons();
  setupLeds();
}

void setupButtons() {
  pinMode(THERMOSTATE_BUTTON_PIN_1, INPUT);
  pinMode(THERMOSTATE_BUTTON_PIN_2, INPUT);
}

void setupLeds() {
  pinMode(THERMOSTATE_LED_PIN_1, OUTPUT);
  pinMode(THERMOSTATE_LED_PIN_2, OUTPUT);
}

void setupSensors() {
  dtemp1.begin();
  dtemp2.begin();

  delay(500);
  
  if (debug) {
    Serial.println("Setting up thermal sensors: Finished");
  }
}

void setupI2c() {
  if (debug) {
    Serial.println("Setting up I2c interface");
  }
  
  Wire.begin(I2C_SLAVE_ADDRESS);
  delay(1000);
  //  Wire.onRequest(i2cRequest);
  Wire.onReceive(i2cReceive);

  if (debug) {
    Serial.println("Setting up I2c interface: Finished");
  }
}

void setupInstructionMap() {
  if (debug) {
    Serial.println("Setting up InstructionMap");
  }
  
  instructionMap.insert(std::pair<int, std::string>(PI_MESSAGE_SET_TARGET_TEMP, PI_MESSAGE_SET_TARGET_TEMP_REF));
  instructionMap.insert(std::pair<int, std::string>(PI_MESSAGE_SET_TEMP_OFFSET, PI_MESSAGE_SET_TEMP_OFFSET_REF));
  instructionMap.insert(std::pair<int, std::string>(PI_MESSAGE_SET_TEMP_LOCK, PI_MESSAGE_SET_TEMP_LOCK_REF));
  instructionMap.insert(std::pair<int, std::string>(PI_MESSAGE_TOGGLE_THERMOSTAT, PI_MESSAGE_TOGGLE_THERMOSTAT_REF));
  instructionMap.insert(std::pair<int, std::string>(PI_MESSAGE_THERMOSTAT_STATUS, PI_MESSAGE_THERMOSTAT_STATUS_REF));

  if (debug) {
    Serial.println("Setting up InstructionMap: Finished");
  }
}

void setupRelay() {
  if (debug) {
    Serial.println("Setting up Relay module");
  }

  
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  pinMode(RELAY_PIN_4, OUTPUT);

  delay(500);
  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);
  digitalWrite(RELAY_PIN_3, HIGH);
  digitalWrite(RELAY_PIN_4, HIGH);
  delay(1000);

  if (debug) {
    Serial.println("Setting up Relay module: Finished");
  }
}


void setupHandlers() {
  addMessageHandler(PI_MESSAGE_SET_TARGET_TEMP_REF, [](int section, int temp) {
    targetTemp[section] = temp;
  });

  addMessageHandler(PI_MESSAGE_SET_TEMP_OFFSET_REF, [](int section, int offset) {
    tempOffset[section] = offset;
  });

  addMessageHandler(PI_MESSAGE_SET_TEMP_LOCK_REF, [](int _, int tempLock) {
    tempLockTime = tempLock * 1000;
  });

  addMessageHandler(PI_MESSAGE_TOGGLE_THERMOSTAT_REF, [](int section, int thermostateState) {
    toggleThermostat(section, thermostateState);
  });

  addMessageHandler(PI_MESSAGE_THERMOSTAT_STATUS_REF, [](int section, int _) {
    // TODO: Return some status
  });

};

void i2cReceive(int howMany) {
  if (howMany != PI_MESSAGE_SIZE) {
    return;
  }

  for (int i = 0; i < howMany; i++) {
    byte c = Wire.read();
    if (i == PI_MESSAGE_SIZE - 1) {
      handleI2cMessage(c);
    }
  }
}

void i2cRequest() {

}

void toggleLed(int section, int state) {
  ledStates[section] = state;
}

void checkLeds() {
  for (int section = 0; section < SECTIONS; section++) {
    digitalWrite(leds[section], ledStates[section]);
  }
}


void checkButtons() {
  for (int button = 0; button < SECTIONS; button++) {
    int reading = digitalRead(buttons[button]);
    
    unsigned long now = millis();

    if ((now - lastButtonLockTime) < buttonLockTime) {
      return;
    }

    if ((now - lastDebounceTime) > debounceDelay) {
      if (reading != buttonStates[button]) {
        buttonStates[button] = reading;
        lastButtonLockTime = now;
        if (buttonStates[button] == HIGH) {
          buttonPressed(button);
        }
      }
    }
    
  }
}

void buttonPressed(int section) {
  int state = disabled[section] == LOW ? HIGH : LOW;
  toggleThermostat(section, state);
}

Instruction messageToInstruction(int message) {
  std::string name = "";

  for (auto const &instruction : instructionMap) {
    if (instruction.first == (int)(message / 1000)) {
      name = instruction.second;
    }
  }

  int param = (int)((message % 10000) / 100);
  int value = message % 100;

  struct Instruction instruction = {
    message,
    name,
    param,
    value
  };

  return instruction;
}

void handleI2cMessage(int message) {
  struct Instruction instruction = messageToInstruction(message);

  if (instruction.name != "") {
    handlers[instruction.name](instruction.param, instruction.value);
  }
}

void toggleRelay(int section, int state) {
  int pin = section == 0 ? RELAY_PIN_1 : RELAY_PIN_2;

  digitalWrite(pin, state);
}

void addMessageHandler(std::string name, Handler handler) {
  handlers.insert(
    std::pair<std::string, Handler>(name, handler)
  );
}

void toggleThermostat(int section, int state) {
  disabled[section] = state == LOW ? HIGH : LOW;
  toggleRelay(section, state);
  toggleLed(section, state == LOW ? HIGH : LOW);
  
}

int intToTemp(int intTemp) {
  return intTemp;
}

int readTemp(int section) {
  DallasTemperature sensor = section == 0 ? dtemp1 : dtemp2;
  sensor.requestTemperatures();
  delay(500);
  int temp = sensor.getTempCByIndex(0);

  return temp;
}

void addTask(std::string name, unsigned long dueDate, void job ()) {
  struct Task task = {name, dueDate, job};
  tasks.push(task);
}

void runThermostat() {
  for (int section=0; section < SECTIONS; section++) {
    if (disabled[section] == HIGH) {
      toggleRelay(section, HIGH);
      continue;  
    }
    Serial.println(currTemp[section]);
    if (currTemp[section] >= targetTemp[section] + tempOffset[section]) {
      toggleRelay(section, HIGH);
    } else if (currTemp[section] < targetTemp[section] - tempOffset[section]) {
      toggleRelay(section, LOW);
    }
  }
}

void performTasks() {
  for (int i = 0; i < tasks.size(); i++) {
    if (tasks[i].dueDate < millis()) {
      struct Task task = tasks.shift();
      task.job();
    } else {
      return;
    }
  }
}

void debugTemp() {
//  Serial.println(currTemp[0]);
}

void updateTemp() {
  if (thermostateOn == false || millis() < (lastTempLockTime + tempLockTime)) {
    return;
  }

  lastTempLockTime = millis();
  
  currTemp[0] = readTemp(0);
  currTemp[1] = readTemp(1);
}

void loop() {
  updateTemp();
  runThermostat();
  checkLeds();
  checkButtons();
  performTasks();

  debugTemp();
}
