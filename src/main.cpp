#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <functional>
#include <map>
#include <vector>


String splitString(String message, char delimiter, int index) {
  int currentIndex = 0;
  int count = 0;
  int startIndex = 0;

  while (count < index) {
    currentIndex = message.indexOf(delimiter, startIndex);
    if (currentIndex == -1) {
      return ""; // Index out of bounds
    }
    startIndex = currentIndex + 1;
    count++;
  }
  
  currentIndex = message.indexOf(delimiter, startIndex);
  
  if (currentIndex == -1) {
    // Last item
    return message.substring(startIndex);
  }

  return message.substring(startIndex, currentIndex);
}

// Add CRC32 calculation function at the top level
uint32_t calculateCRC32(const String& data) {
  uint32_t crc = 0xFFFFFFFF;
  for (char c : data) {
    crc ^= c;
    for (int i = 0; i < 8; i++) {
      crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
    }
  }
  return ~crc;
}

// Replace the placeholder CRC with a function
String generateCRC(const String& data) {
  char buf[9];
  sprintf(buf, "%08X", calculateCRC32(data));
  return String(buf);
}

class Message {
  String fromAddr;
  String toAddr;
  String type;
  StaticJsonDocument<512> jsonData;
  String rawData;
  String crc;
  bool isValid;
  String errorMsg;
  
public: 
  Message(String message) {
    isValid = true;
    errorMsg = "";
    
    fromAddr = splitString(message, '|', 0);
    toAddr = splitString(message, '|', 1);
    type = splitString(message, '|', 2);
    rawData = splitString(message, '|', 3);
    crc = splitString(message, '|', 4);

    if (fromAddr.isEmpty() || toAddr.isEmpty() || type.isEmpty() || rawData.isEmpty() || crc.isEmpty()) {
      isValid = false;
      errorMsg = "Invalid message format";
      return;
    }

    // Validate CRC
    String dataToCheck = fromAddr + "|" + toAddr + "|" + type + "|" + rawData;
    String expectedCRC = generateCRC(dataToCheck);
    if (crc != expectedCRC) {
      isValid = false;
      errorMsg = "CRC check failed";
      return;
    }

    DeserializationError error = deserializeJson(jsonData, rawData);
    if (error) {
      isValid = false;
      errorMsg = "Invalid JSON data: " + String(error.c_str());
      return;
    }
  }

  bool valid() const {
    return isValid;
  }

  String getError() const {
    return errorMsg;
  }

  void print() {
    Serial.println("Message Status: " + String(isValid ? "Valid" : "Invalid"));
    if (!isValid) {
      Serial.println("Error: " + errorMsg);
      return;
    }
    
    Serial.println("From: " + fromAddr);
    Serial.println("To: " + toAddr);
    Serial.println("Type: " + type);
    Serial.print("Data: ");
    serializeJson(jsonData, Serial);
    Serial.println();
    Serial.println("CRC: " + crc);
  }

  Message* respond(JsonDocument& responseData) {
    String jsonString;
    serializeJson(responseData, jsonString);
    String messageData = toAddr + "|" + fromAddr + "|" + type + "|" + jsonString;
    String messageCRC = generateCRC(messageData);
    return new Message(messageData + "|" + messageCRC);
  }

  String toString() {
    String jsonString;
    serializeJson(jsonData, jsonString);
    return fromAddr + "|" + toAddr + "|" + type + "|" + jsonString + "|" + crc;
  }

  String getFromAddr() const { return fromAddr; }
  String getToAddr() const { return toAddr; }
  String getType() const { return type; }
  
  template<typename T>
  T getData(const char* key, T defaultValue) {
    if (!isValid || !jsonData[key]) {
      return defaultValue;
    }
    return jsonData[key].as<T>();
  }

  bool hasDataKey(const char* key) {
    return isValid && jsonData[key];
  }

  const JsonDocument& getJsonData() const { return jsonData; }
};

class PlugSync {
  const String address = generateAddress();
  String deviceType;
  StaticJsonDocument<512> registers;
  
  private:
    typedef std::function<void(Message*)> ActionCallback;
    ActionCallback actionCallback = nullptr;

    const int RS485_DE_RE = 2;  // Data Enable and Receive Enable pin
    const unsigned long RESPONSE_TIMEOUT = 1000;  // 1 second timeout
    const unsigned long MIN_BACKOFF = 100;   // 100ms minimum backoff
    const unsigned long MAX_BACKOFF = 1000;  // 1s maximum backoff
    unsigned long lastTransmitTime = 0;
    bool transmitting = false;
    const unsigned long HEARTBEAT_INTERVAL = 30000;  // 30 seconds between heartbeats
    const unsigned long HEARTBEAT_MAX_BACKOFF = 5000;  // Up to 5 second random delay

    String buffer = "";
    unsigned long lastToggle = 0;
    const unsigned long TOGGLE_INTERVAL = 5000;  // 5 seconds between toggles

    bool initialHeartbeatSent = false;

    void enableTransmit(bool enable) {
      digitalWrite(RS485_DE_RE, enable);
      transmitting = enable;
      if (enable) {
        delayMicroseconds(50);  // Give RS485 chip time to switch modes
      } else {
        delayMicroseconds(50);
      }
    }

    void sendResponse(Message* responseMsg) {
      // Wait for a random backoff period before transmitting
      unsigned long backoff = random(MIN_BACKOFF, MAX_BACKOFF);
      delay(backoff);
      
      // Enable transmitter
      enableTransmit(true);
      
      // Send the message
      Serial.println(responseMsg->toString());
      Serial.flush();  // Wait for transmission to complete
      
      // Disable transmitter and enable receiver
      enableTransmit(false);
      
      lastTransmitTime = millis();
      delete responseMsg;
    }

    bool canTransmit() {
      // Prioritize regular messages over heartbeats
      if (Serial.available()) {
        return false;
      }
      
      // Add extra delay for heartbeats
      if (millis() - lastTransmitTime < MIN_BACKOFF * 2) {
        return false;
      }
      
      return true;
    }

    void handlePeriodicTasks() {
      unsigned long now = millis();

      // Send initial heartbeat after 2 seconds
      if (!initialHeartbeatSent && now > 2000) {
        StaticJsonDocument<200> heartbeatDoc;
        heartbeatDoc["uptime"] = millis();
        
        Message* msg = createMessage("BROADCAST", "HEARTBEAT", heartbeatDoc);
        if (canTransmit()) {
          sendResponse(msg);
          initialHeartbeatSent = true;
        } else {
          delete msg;
        }
      }

      // Regular heartbeat logic
      if (now - lastTransmitTime >= HEARTBEAT_INTERVAL) {
        StaticJsonDocument<200> heartbeatDoc;
        heartbeatDoc["uptime"] = millis();
        
        Message* msg = createMessage("BROADCAST", "HEARTBEAT", heartbeatDoc);
        
        // Use a longer random backoff for heartbeats
        unsigned long backoff = random(1000, HEARTBEAT_MAX_BACKOFF);
        delay(backoff);
        
        if (canTransmit()) {
          sendResponse(msg);
        } else {
          delete msg;
        }
      }
    }

    void processSerialData() {
      while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
          if (buffer.length() > 0) {
            receiveMessage(buffer);
            buffer = "";
          }
        } else {
          buffer += c;
        }
      }
    }

  private:
    static String generateAddress() {
      const char validChars[] = "ABCDEFGHJKMNPQRSTUVWXYZ23456789";
      String result;
      for(int i = 0; i < 6; i++) {
        result += validChars[esp_random() % (sizeof(validChars) - 1)];
      }
      return result;
    }

    void loadRegisters() {
      EEPROM.begin(512);
      String saved;
      
      for (int i = 0; i < 512; i++) {
        char c = EEPROM.read(i);
        if (c == 0) break;
        saved += c;
      }

      DeserializationError error = deserializeJson(registers, saved);
      if (saved.length() == 0 || error != DeserializationError::Ok) {
        registers.clear();
        registers["name"] = "unnamed";
        registers["description"] = "";
        registers["lastUpdate"] = "2017-01-01T18:42:00Z";
        registers["type"] = deviceType;
        registers["address"] = address;
        saveRegisters();
      }

      if (!registers.containsKey("address")) {
        registers["address"] = address;
        saveRegisters();
      }
      
      EEPROM.end();
    }

    void saveRegisters() {
      EEPROM.begin(512);
      
      String jsonString;
      serializeJson(registers, jsonString);
      
      for (int i = 0; i < 512; i++) {
        EEPROM.write(i, 0);
      }
      
      for (unsigned int i = 0; i < jsonString.length(); i++) {
        EEPROM.write(i, jsonString[i]);
      }
      EEPROM.write(jsonString.length(), 0);
      
      EEPROM.commit();
      EEPROM.end();
    }

  public:
    PlugSync(String deviceType) {
      this->deviceType = deviceType;
      
      // Configure RS485 control pin
      pinMode(RS485_DE_RE, OUTPUT);
      enableTransmit(false);  // Start in receive mode
      
      loadRegisters();
    }

    bool setRegister(const char* key, const char* value) {
      if (registers.size() >= 16 && !registers.containsKey(key)) {
        return false;
      }
      registers[key] = value;
      saveRegisters();
      return true;
    }

    String getRegister(const char* key) {
      if (!registers.containsKey(key)) {
        return "";
      }
      return registers[key].as<String>();
    }

    Message* createMessage(String toAddress, String type, JsonDocument& data) {
      String jsonString;
      serializeJson(data, jsonString);
      String messageData = address + "|" + toAddress + "|" + type + "|" + jsonString;
      String messageCRC = generateCRC(messageData);
      return new Message(messageData + "|" + messageCRC);
    }

    void receiveMessage(String message) {
      // Don't process messages while transmitting
      if (transmitting) {
        return;
      }

      Message* msg = new Message(message);
      
      if (!msg->valid()) {
        Serial.println("Error processing message: " + msg->getError());
        delete msg;
        return;
      }

      // Only process messages addressed to us or broadcasts
      if (msg->getToAddr() != address && msg->getToAddr() != "BROADCAST") {
        delete msg;
        return;
      }

      String type = msg->getType();
      if (type == "HEARTBEAT") {
        // Optionally store or process heartbeat data
        const JsonDocument& data = msg->getJsonData();
        String fromAddr = msg->getFromAddr();
        
        // You could store this information in a map to track active devices
        updateDeviceStatus(fromAddr, data);
      } else if (type == "ACTION") {
        handleAction(msg);
      } 
      else if (type == "GET_REGISTER") {
        handleGetRegister(msg);
      } 
      else if (type == "GET_REGISTERS") {
        handleGetRegisters(msg);
      }
      else if (type == "SET_REGISTER") {
        handleSetRegister(msg);
      }
      else if (type == "PING") {
        handlePing(msg);
      }
      else {
        Serial.println("Unknown message type: " + type);
      }

      delete msg;
    }

    void onAction(ActionCallback callback) {
      actionCallback = callback;
    }

    // Add new helper method for creating action messages
    Message* createActionMessage(String toAddress, const char* action, JsonDocument& params) {
      StaticJsonDocument<512> actionDoc;
      actionDoc["action"] = action;
      
      // Copy all parameters from the input document
      for (JsonPair kvp : params.as<JsonObject>()) {
        actionDoc[kvp.key()] = kvp.value();
      }
      
      return createMessage(toAddress, "ACTION", actionDoc);
    }

    // Even simpler helper for basic actions
    template<typename T>
    Message* createSimpleAction(String toAddress, const char* action, const char* paramName, T paramValue) {
      StaticJsonDocument<200> actionDoc;
      actionDoc["action"] = action;
      actionDoc[paramName] = paramValue;
      return createMessage(toAddress, "ACTION", actionDoc);
    }

    // Modified to handle RS485 transmission
    void sendMessage(Message* msg) {
      unsigned long startAttempt = millis();
      
      while (millis() - startAttempt < RESPONSE_TIMEOUT) {
        if (canTransmit()) {
          sendResponse(msg);
          return;
        }
        delay(random(10, 50));  // Random delay before retry
      }
      
      // If we get here, we couldn't send the message
      Serial.println("Failed to send message - timeout");
      delete msg;
    }

  private:
    void handleGetRegisters(Message* msg) {
      StaticJsonDocument<512> response;
      response["status"] = "ok";
      
      JsonObject registersObj = response.createNestedObject("registers");
      for (JsonPair kvp : registers.as<JsonObject>()) {
        registersObj[kvp.key()] = kvp.value();
      }
      
      Message* responseMsg = msg->respond(response);
      sendMessage(responseMsg);
    }

    void handleGetRegister(Message* msg) {
      String key = msg->getData("key", "");
      if (key.isEmpty()) {
        sendError(msg, "Missing key parameter");
        return;
      }

      String value = getRegister(key.c_str());
      
      StaticJsonDocument<512> response;
      response["status"] = "ok";
      response["key"] = key;
      response["value"] = value;
      
      Message* responseMsg = msg->respond(response);
      sendMessage(responseMsg);
    }

    void handleSetRegister(Message* msg) {
      String key = msg->getData("key", "");
      String value = msg->getData("value", "");
      
      if (key.isEmpty()) {
        sendError(msg, "Missing key parameter");
        return;
      }

      if (value.isEmpty()) {
        sendError(msg, "Missing value parameter");
        return;
      }

      bool success = setRegister(key.c_str(), value.c_str());
      
      StaticJsonDocument<512> response;
      response["status"] = success ? "ok" : "error";
      response["message"] = success ? "Register updated" : "Failed to update register";
      
      Message* responseMsg = msg->respond(response);
      sendMessage(responseMsg);
    }

    void handlePing(Message* msg) {
      StaticJsonDocument<512> response;
      response["status"] = "ok";
      response["message"] = "pong";
      response["type"] = deviceType;
      response["address"] = address;
      
      Message* responseMsg = msg->respond(response);
      sendMessage(responseMsg);
    }

    void handleAction(Message* msg) {
      if (!actionCallback) {
        sendError(msg, "No action handler registered");
        return;
      }

      actionCallback(msg);

      StaticJsonDocument<512> response;
      response["status"] = "ok";
      response["message"] = "Action processed";
      
      Message* responseMsg = msg->respond(response);
      sendMessage(responseMsg);
    }

    void sendError(Message* msg, String errorMessage) {
      StaticJsonDocument<512> response;
      response["status"] = "error";
      response["message"] = errorMessage;
      
      Message* responseMsg = msg->respond(response);
      sendMessage(responseMsg);
    }

  public:
    // Helper method to create a get registers request
    Message* createGetRegistersRequest(String toAddress) {
      StaticJsonDocument<200> emptyDoc;
      return createMessage(toAddress, "GET_REGISTERS", emptyDoc);
    }

    // Add getter for transmitting state
    bool isTransmitting() const {
      return transmitting;
    }

    // Add device tracking
    struct DeviceStatus {
      unsigned long lastSeen;
      unsigned long uptime;
      int rssi;
    };
    
    std::map<String, DeviceStatus> knownDevices;

    void updateDeviceStatus(const String& address, const JsonDocument& data) {
      DeviceStatus& status = knownDevices[address];
      status.lastSeen = millis();
      status.uptime = data["uptime"] | 0;
      status.rssi = data["rssi"] | 0;
    }

    // Helper method to check if a device is still active
    bool isDeviceActive(const String& address, unsigned long timeout = 120000) {  // 2 minute timeout
      if (!knownDevices.count(address)) {
        return false;
      }
      return (millis() - knownDevices[address].lastSeen) < timeout;
    }

    // Get list of active devices
    std::vector<String> getActiveDevices(unsigned long timeout = 120000) {
      std::vector<String> active;
      unsigned long now = millis();
      for (const auto& pair : knownDevices) {
        if (now - pair.second.lastSeen < timeout) {
          active.push_back(pair.first);
        }
      }
      return active;
    }

    void handle() {
      processSerialData();
      handlePeriodicTasks();
      delay(1);  // Small delay to prevent tight loops
    }
};

PlugSync* plugSync;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing PlugSync...");
  
  plugSync = new PlugSync("Plug 1");

  plugSync->onAction([](Message* msg) {
    const JsonDocument& data = msg->getJsonData();
    String action = data["action"].as<String>();
    
    if (action == "switch") {
      bool setValue = data["setValue"].as<bool>();
      digitalWrite(LED_BUILTIN, setValue ? HIGH : LOW);
    }
  });
}

void loop() {
  plugSync->handle();
}