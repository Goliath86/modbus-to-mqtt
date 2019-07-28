/**
 * ModbusTCP sunnyboy(3, {192, 168, 123, 123}, 502);
 * where:
 * - 3 = slave's ID
 * - {192, 168, 123, 123} = IP address of the slave
 * - 502 = port number
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp32ModbusTCP.h>
#include <PubSubClient.h>
#include "FS.h"
#include "SD_MMC.h"
#include "ArduinoJson.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <EEPROM.h>

// Constant with the max number of MODBUS registers that can be readed (depend on microcontroller's memory)
#define MAX_REGISTERS 300

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONFIG_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Max EEPROM size on flash RAM
#define EEPROM_SIZE 1024

// Max number of elements in queue
#define MAX_QUEUE 10

// EEPROM functions declarations
void write_string(char add,String data);
String read_string(char add);

// ****** WiFi configs
WiFiClient wifiClient;
WiFiClient httpClient;
const char* ssid = "yout wifi ssid;
const char* pass = "your wifi password";

bool WiFiConnected = false;
bool MQTTConnected = false;
bool MQTTConnect = false;

// ****** MQTT Broker
PubSubClient *client = NULL;

// ****** Modbus
esp32ModbusTCP *sunnyboy = NULL;

// Data structure with all the informations regarding a Modbus register
struct smaData {
  uint16_t address;       // Modbus address
  uint16_t length;        // (Bytes to read) / 2
  uint16_t type;          // Type:      0 - unsigned short int 16bit (2 bytes)
                          //            1 - unsigned integer 32bit (4 bytes)
                          //            2 - signed integer 32bit (4 bytes)
                          //            3 - float 32bit (4bytes)
  uint16_t modbusType;    // Modbus function type: 0 - Read Coils
                          //                       1 - Read Discrete Input Registers
                          //                       3 - Read Input Registers
                          //                       4 - Read Holding Registers
  uint16_t packetId;      // Packet ID assigned to the register by the AsyncTCP module
  const char* dbAddress;  // Database unique ID of the register
};

// Dummy array of registers
smaData smaRegisters[MAX_REGISTERS];

uint16_t counter = 0;         // Actual queued packets counter
uint16_t packetCounter = 0;   // Total packets counter
uint16_t maxRegisters = 0;    // Max number of registers that will be read - Numero di registri che effettivamente si devono leggere
uint8_t readMaxRegister = 0;  // Max number of packets to be queued to be readed by the AsyncTCP module - Numero massimo di pacchetti da mettere in coda per la lettura con il modulo AsyncTCP

// ***********************************
// ********** EEPROM Config **********
// ***********************************

// Slave ID
byte slaveId = 0;

// Slave IP
String slaveIp = "";

// Modbus port
String modbusPort = "";

// WiFi account
String wifiSsid = "";
String wifiPwd = "";

// MQTT topic where to send the data readed from the modbus registers
String mqttTopicSend = "";

// HTTP address where to find the Modbus registers config file - Indirizzo HTTP Host dove trovare il file di configurazione
String httpHostAddress = "";

// Specific HTTP address where to find the Modbus registers config file - Indirizzo specifico dove trovare il file di configurazione
String httpFileAddress = "";

// Broker MQTT IP
String mqtt_server = "";

// Broker MQTT username and password
String mqtt_user = "";
String mqtt_pwd = "";

// Broker MQTT port
String brokerPort = "";

// ***********************************
// ***********************************
// ***********************************

bool tryHttp = false; // If true, the Modbus registers config file must be found on the online repository

/**
 * Function to connect to the WiFi
 */
void wifiConnect() {
  WiFi.disconnect(true);
  WiFi.begin(ssid, pass);

  Serial.print("Conneting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
}

void setup() {
    Serial.begin(115200);

    wifiClient.setTimeout(5);

    // Dummy array initialization
    uint8_t result = inizializzaArray();

    // In case of error on dummy array initialization - In caso di errore durante l'inizializzazione dell'array
    if (result != 0) tryHttp = true;

    // BLE initialization - Inizializzazione BLE
    initBLE();

    // Load the config parameters from the EEPROM - Recupero parametri di configurazione da EEPROM
    loadEEPROMConfig();

    // Wait the BLE config if not already present - Attendi la configurazione tramite BLE se non già presente
    //while(!checkConfig()) {delay(300);};

    Serial.println("Config OK");

    // MQTT initialization - Inizializzazione MQTT
    mqtt_server = "your mqtt broker ip";
    client = new PubSubClient(wifiClient);
    client->setServer(mqtt_server.c_str(), 1883);
    client->setCallback(callback);

    // Modbus master instantiation - Istanziazione Modbus master
    sunnyboy = new esp32ModbusTCP(1, {192, 168, 1, 13}, 502);

    // Modbus callback initialization - Inizializzazione callbacks Modbus
    sunnyboy->onData([](uint16_t packet, uint8_t slave, esp32Modbus::FunctionCode fc , uint8_t* data , uint16_t len) {
      for (uint16_t i = 0; i < maxRegisters; ++i) {
        if (smaRegisters[i].packetId == packet) {

          // Decrement the queued packet counter - Decrementa contatore pacchetti in coda
          counter--;
          
          smaRegisters[i].packetId = 0;
          switch (smaRegisters[i].type) {
            case 0:
              {
                // Unsigned integer 16bit
                uint16_t value = 0;
                value = (data[0] << 8) | (data[1]);

                // Public the readed value to the MQTT topic - Pubblica il valore nel topic
                char str[8];
                sprintf(str, "%u", value);
                mqttTopicSend = "esp32/test/"; // Only for debug - on production will be readed from the EEPROM
                String topic = mqttTopicSend + String(smaRegisters[i].address);
                //client.publish(topic.c_str(), str);
                
                Serial.printf("%u: %u\n", smaRegisters[i].address, value);
                break;
              }
            case 1:
              {
                // Unsigned integer 32bit
                uint32_t value = 0;
                value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
                Serial.printf("%u\n", value);
                break;
              }
            case 2:
              {
                // Signed integer 32bit
                int32_t value = 0;
                value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
                Serial.printf("%i\n", value);
                break;
              }
            case 3:
              {
                // Float 32bit

                union TEMP
                {
                  float f;
                  byte b[4];
                } t;
                
                t.b[0] = data[1];
                t.b[1] = data[0];
                t.b[2] = data[3];
                t.b[3] = data[2];

                // Public the readed value to the MQTT topic - Pubblica il valore nel topic
                char str[20];
                sprintf(str, "%f", t.f);
                //client.publish("esp32/test", str);
                
                Serial.printf("%f\n", t.f);
                break;
              }
            }
          return;
        }
      }
    });

    // ****** Error callback - Callback di errore
    sunnyboy->onError([](uint16_t packet, esp32Modbus::Error e) {
      Serial.printf("Error packet %u: %02x\n", packet, e);
      counter--;
    });

    delay(1000);

    // WiFi event-handlers
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
      Serial.print("WiFi connected. IP: ");
      Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
      WiFiConnected = true;
      MQTTConnect = true;
      //reconnect();
    }, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
    
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
        Serial.print("WiFi lost connection. Reason: ");
        Serial.println(info.disconnected.reason);
        wifiConnect();
        WiFiConnected = false;
    }, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

    // Wifi connection initialization - Inizializza connessione WiFi
    wifiConnect();

    Serial.println();
    Serial.println("Connecting to WiFi...");
}

/**
 * Load the system config from EEPROM - Funzione per il recupero dei parametri di configurazione da EEPROM
 */
void loadEEPROMConfig() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM initialization failed");
    delay(5000);
    ESP.restart(); // Reboot the board after 5 secs - Riavvia la scheda dopo 5 secondi
  }

  //Serial.println("Lettura da EEPROM");

  // Slave ID
  slaveId = EEPROM.read(1000);

  // WiFi account
  wifiSsid = read_string(0);
  wifiPwd = read_string(100);

  // MQTT topic
  mqttTopicSend = read_string(200);

  // HTTP address where to find the config file - Indirizzo HTTP Host dove trovare il file di configurazione
  httpHostAddress = read_string(300);

  // Specific HTTP address where to find the config file - Indirizzo specifico dove trovare il file di configurazione
  httpFileAddress = read_string(400);

  slaveIp = read_string(500);

  mqtt_server = read_string(600);

  // Modbus port - Porta modbus
  modbusPort = read_string(700);

  // MQTT broker port - Porta Broker MQTT
  brokerPort = read_string(800);

  //Serial.printf("%u %u %s %s\n", slaveId, slaveIp[0], wifiSsid.c_str(), httpFileAddress.c_str());
}

/**
 * Function to check if the config parameters are correctly loaded
 */
bool checkConfig() {
  if (slaveIp == "È" || wifiSsid == "È" || wifiPwd == "È" || codiceCliente == "È" || httpHostAddress == "È" || httpFileAddress == "È" || mqtt_server == "È") return false;
  return true;
}

/**
 * Function execute on BLE message received - Funzione eseguita alla ricezione di un messaggio dal BLE
 */
class ServerReadCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);          
        }
        Serial.println();

        // Ricevo un oggetto JSON dal servizio BLE
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, rxValue);
        if (error)
          return;

        // Config Slave
        slaveId = doc["sId"];
        slaveIp = doc["sIp"].as<String>();
        modbusPort = doc["mBp"].as<String>();
        /*
        // Config WiFi
        const char* ifiSsid = doc["wfs"];
        wifiPwd = str(doc["wfp"]);

        codiceCliente = str(doc["ccl"]);

        httpHostAddress = str(doc["hth"]);
        httpFileAddress = str(doc["hfa"]);

        // Broker MQTT IP
        brokerIp0 = doc["bIp0"];
        brokerIp1 = doc["bIp1"];
        brokerIp2 = doc["bIp2"];
        brokerIp3 = doc["bIp3"];
        
        // Porta Broker MQTT
        brokerPort = doc["bpp"];*/
    }
};

/**
 * - BLE server initialization - Funzione di inizializzazione server BLE
 */
void initBLE() {
  BLEDevice::init("IoT Hub"); // BLE server's name
  BLEServer *pServer = BLEDevice::createServer(); // Create the server
  BLEService *pService = pServer->createService(SERVICE_UUID); // Create a service and assign an UUID - Crea un servizio ed assegnagli un UUID
  BLECharacteristic *pCharacteristic = pService->createCharacteristic( // Set a characteristi of the service - Setta una caratteristica del servizio
                                         CONFIG_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setValue("No Config");
  pCharacteristic->setCallbacks(new ServerReadCallbacks());
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  //Serial.println("Characteristic defined!");
}

/**
 * Read a file from the SD MMC slot - Funzione per la lettura di un file in SD_MMC
 */
uint8_t readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Opening file filed");
        return 1;
    }

    Serial.println("Reading file");
    String str;

    // Readed lines counter - Contatore righe lette
    int line = 0;

    // Read line by line the file and retrieve the content as a JSON object - Leggi riga per riga il file e decifra il contenuto come oggetto JSON
    while(file.available()){
      DynamicJsonDocument doc(1024);
      str = file.readStringUntil('\n');
      DeserializationError error = deserializeJson(doc, str);
      if (error)
        return 2;

      uint16_t address = doc["a"];
      uint16_t length = doc["l"];
      uint16_t type = doc["t"];
      uint16_t modbusType = doc["m"];
      const char* dbAddress = doc["i"];
      //uint16_t packetId = 0;

      smaRegisters[line] = { address, length, type, modbusType, 0, dbAddress };
      Serial.println(smaRegisters[line].address);
      
      line++;
    }

    // Save the number of elements to be queued - Stabilisci quanti elementi mettere in coda
    readMaxRegister = min(line, MAX_QUEUE);

    // Store the number of registers to be readed - Memorizza il numero di registri da leggere;
    maxRegisters = line;
    return 0;
}

/**
 * Send a GET request to a server to retrieve the registers list - Funzione per l'invio di una richiesta GET ad un server per recuperare la lista dei registri
 */
 uint8_t sendGET() {
  if (httpClient.connect("www.yoursite.it", 80)) {  // Initialize server connection
    Serial.println("HTTP client connected");

    // Invia richiesta GET al sito per la lettura del file
    httpClient.println("GET /path/registers.txt HTTP/1.1"); // Download the file
    httpClient.println("Host: www.yoursite.it");
    httpClient.println("Connection: closed\r\n");  // Close the site connection - Chiudi la connessione al sito 
    httpClient.println();
  }
  else {
    Serial.println("Connection to the server failed");
    Serial.println();
    return 1;
  }

  while(httpClient.connected() && !httpClient.available()) delay(1); // Wait for the data - Attendi i dati

  Serial.println("Reading from the server...");
  String str;

  // Contatore righe lette
  int line = 0;
    
  while (httpClient.connected() || httpClient.available()) { // Connected or data available - Connesso o dati disponibili
      DynamicJsonDocument doc(1024);
      str = httpClient.readStringUntil('\n');
      
      // Check if the string contains useful data - Verifica che la stringa contenga dati utili
      // Ignore the HTML page header - Permette di ignorare l'header della pagina HTML
      if (str.indexOf("{\"a\":") == -1) continue;
      
      DeserializationError error = deserializeJson(doc, str);
      if (error)
        return 2;

      uint16_t address = doc["a"];        // Modbus register address - Indirizzo registro Modbus
      uint16_t length = doc["l"];         // Number of bytes to read - Numero di bytes da leggere
      uint16_t type = doc["t"];           // Type of data - Tipo di dato
      uint16_t modbusType = doc["m"];     // Modbus function - Tipo funzione Modbus
      const char* dbAddress = doc["i"];   // DB address or index where to save the value of the readed register - Indirizzo nel DB del registro

      smaRegisters[line] = { address, length, type, modbusType, 0, dbAddress };
     
      line++;
  }

  // Save the number of elements to be queued - Stabilisci quanti elementi mettere in coda
  readMaxRegister = min(line, MAX_QUEUE);

  // Store the number of registers to be readed - Memorizza il numero di registri da leggere;
  maxRegisters = line;

  httpClient.stop(); //stop client
  return 0;
}

/** 
 * Init the array containing the data struct of the Modbus registers - Funzione per l'inizializzazione dell'array contenente le strutture dati dei registri Modbus
 */
uint8_t inizializzaArray() {
    if(!SD_MMC.begin()){
        Serial.println("SD mount failed");
        return 1;
    }
    uint8_t cardType = SD_MMC.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD_MMC present");
        return 2;
    }

    Serial.print("SD_MMC type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("Unknown");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD_MMC size: %lluMB\n", cardSize);

    // Read registers from SD MMC file - Lettura dei registri da file SD_MMC
    uint8_t result = readFile(SD_MMC, "/registri.txt");
    return result;
}

/**
 * Callback function to be called on any received message from any MQTT topic - Funzione di callback richiamata all'arrivo di un qualsiasi messaggio su di un topic MQTT
 */
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived from topic: ");
  Serial.print(topic);
  Serial.print(". Payload: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Check if the message is from topic esp32/output - Controlla se il messaggio è arrivato sul topic esp32/output
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
}

void reconnect() {
    client = new PubSubClient(wifiClient);
    client->setServer(mqtt_server.c_str(), 1883);
    client->setCallback(callback);

  //while (!client.state() == 0) {
    Serial.print("Connecting to MQTT broker...");
    // Attempt to connect
    char c[20];
    String str;
    str = String(millis());
    str.toCharArray(c, 20);
    Serial.println(c);
    if (client->connect(c, "mqtt_username", "mqtt_pwd")) {
      MQTTConnected = true;

      // Subscribe
      //client.subscribe("esp32/output");
      Serial.println("Connected to MQTT broker");
    } else {
      MQTTConnect = true;
      MQTTConnected = false;

      Serial.print("error, rc=");
      Serial.print(client->state());
      Serial.println(" reconnecting in 5 secs");
      delay(5000);
    }
  //}
}

/**
 * Write a string in EEPROM - Funzione per la scrittura di una stringa da EEPROM
 */
void write_string(char add, String data)
{
  int _size = data.length();
  int i;
  for(i=0;i<_size;i++)
  {
    EEPROM.write(add+i,data[i]);
  }
  EEPROM.write(add+_size,'\0');   //Add termination null character for String Data
  EEPROM.commit();
}
 
/**
 * Read a string from EEPROM - Funzione per la lettura di una stringa da EEPROM
 */
String read_string(int add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len=0;
  unsigned char k;
  
  k=EEPROM.read(add);
  if (k == 255) return "È"; // Non-valid character - Carattere non valido
  
  while(k != '\0' && len<99)   //Read until null character
  {    
    k=EEPROM.read(add+len);
    data[len]=k;
    len++;
  }
  data[len]='\0';
  return String(data);
}

void loop() {
  //while (WiFi.status != ) { delay(1000); Serial.println("R.."); }
  if (WiFi.status() != WL_CONNECTED) { wifiConnect(); }
  
  if (MQTTConnect) { MQTTConnect = false; reconnect(); }
  
  
  if (!client->loop()) {
    MQTTConnect = true;
    //ESP.restart();
    //reconnect();
    //client.disconnect();
    //MQTTConnected = false;
    //wifiConnect();
    //Serial.println("QUA");
  } else {
    //client->loop();
    //Serial.println("eccoci");
    // If no Wi-Fi connection and there is no SD MMC card - Se è presente la connessione WiFi e se non è stata trovata alcuna SD_MMC
    if (WiFiConnected && maxRegisters == 0) {
      uint8_t result = sendGET();
    }

    if (maxRegisters > 0) {
      if (counter < readMaxRegister && WiFiConnected) {
        // Inserire riconoscimento tipo funzione da usare
        uint16_t packetId = -1;
        switch(smaRegisters[packetCounter].modbusType) {
          case 0:
            // To be implmented on the Modbus library - Da implementare in libreria Modbus
            packetId = -1;
            break;
          case 1:
            packetId = sunnyboy->readDiscreteInputs(smaRegisters[packetCounter].address, smaRegisters[packetCounter].length);
            break;
          case 3:
            packetId = sunnyboy->readInputRegisters(smaRegisters[packetCounter].address, smaRegisters[packetCounter].length);
            break;
          case 4:
            packetId = sunnyboy->readHoldingRegisters(smaRegisters[packetCounter].address, smaRegisters[packetCounter].length);
            break;      
        }
        
        // Check if the comm has been sent correctly - Verifica se è stato trasmesso correttamente il comando
        if (packetId > 0) {
          counter++;
          smaRegisters[packetCounter].packetId = packetId;
          packetCounter++;

          if (packetCounter == maxRegisters) packetCounter = 0;
        } else {
          Serial.print("Read error\n");
        }
      }
    }
  }

  delay(5000);
}
