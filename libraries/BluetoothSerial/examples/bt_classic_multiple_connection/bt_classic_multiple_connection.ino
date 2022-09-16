/*
Marco Daldoss Aug.2022
Example Software to test a simultaneous connection with 2 Bluetooth Classic (SPP) server devices
   This is to be used with the following BluetoothSerial library's fork:
   https://github.com/mdaldoss/arduino-esp32/tree/feature/multi_bt_acceptors
*/

#include <BluetoothSerial.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

// Bluetooth definition Undboning
#define REMOVE_BONDED_DEVICES 1 // <- Set to 0 to view all bonded devices addresses, set to 1 to remove
#define PAIR_MAX_DEVICES 20

uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

BluetoothSerial SerialBT;

bool connected = false;
int prev = millis();
int tdelay = millis() - prev;

#define BT_DISCOVER_TIME 10000

/****  User definitions *************/

// Bluetooth PIN
const char * BT_PIN = "7290";
esp_spp_sec_t sec_mask=ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE; // to request pincode confirmation
esp_spp_role_t role=ESP_SPP_ROLE_MASTER;


// **** Put here your mac address ***
uint8_t addr_trg1[6] = {0x80,0x4b,0x50,0xa8,0x1d,0x1d}; 
uint8_t addr_trg2[6] = {0x84,0xb4,0xdb,0xe3,0x96,0x62};


#define DEV1 addr_trg1
#define DEV2 addr_trg2



bool initBluetooth() {
    if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
    }

    if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
    }
    return true;
}

char * bda2str(const uint8_t * bda, char * str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
    return NULL;
    }
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
    bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

void unBoundBTDevices() {
    initBluetooth();
    Serial.print("ESP32 bluetooth address: ");
    Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));

    // Get the numbers of bonded/paired devices in the BT module
    int count = esp_bt_gap_get_bond_device_num();
    if (!count) {
    Serial.println("No bonded device found.");
    } else {
    Serial.print("Bonded device count: ");
    Serial.println(count);
    if (PAIR_MAX_DEVICES < count) {
        count = PAIR_MAX_DEVICES;
        Serial.print("Reset bonded device count: ");
        Serial.println(count);
    }
    esp_err_t tError = esp_bt_gap_get_bond_device_list( & count, pairedDeviceBtAddr);
    if (ESP_OK == tError) {
        for (int i = 0; i < count; i++) {
        Serial.print("Found bonded device # ");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));
        if (REMOVE_BONDED_DEVICES) {
            esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
            if (ESP_OK == tError) {
            Serial.print("Removed bonded device # ");
            } else {
            Serial.print("Failed to remove bonded device # ");
            }
            Serial.println(i);
        }
        }
    }
    }
}


void setup() {
    Serial.begin(115200);
    delay(400);

    SerialBT.begin("Initiatior", true);
    SerialBT.setPin(BT_PIN);

    unBoundBTDevices();

    // Running scan
    SerialBT.discoverAsync([](BTAdvertisedDevice* pDevice) {
        Serial.printf(">>>>>>>>>>>Found a new device asynchronously: %s\n", pDevice->toString().c_str());
    } );

    delay(10000);
    Serial.print("Stopping discoverAsync... ");
    SerialBT.discoverAsyncStop();


    // Discovering channels
    Serial.println("Starting Discovering BT-SPP server channels...");
    std::map<int,std::string> channels_dev1=SerialBT.getChannels(1, DEV1);
    std::map<int,std::string> channels_dev2=SerialBT.getChannels(2, DEV2);

    // Connecting first device
    connected = SerialBT.connect(DEV1, 1, channels_dev1.begin()->first, sec_mask, role);
    if (connected) {
        Serial.println("Connected DEV1!");
    } else {
        Serial.println("DEV1 not connected...");
    }
        delay(500);

    Serial.println("Disconnecting DEV1...");

    // Disconnecting first device
    connected = SerialBT.disconnect(1);
    if (connected) {
        Serial.println("DEV1 disconnected!");
    } else {
        Serial.println("DEV1 still connected...");
    }
    delay(500);

    // Connecting second device
    connected =  SerialBT.connect(DEV2, 2, channels_dev2.begin()->first, sec_mask, role);

    if (connected) {
            Serial.println("Connected DEV2!");
        } else {
            Serial.println("DEV2 not connected...");
        }

    // Reconnecting first device ->> HERE GIVES ERROR <<<
    delay(500);
    connected = SerialBT.connect(DEV1, 1, channels_dev1.begin()->first, sec_mask, role);

    int connection_retry = 10;
    while (!connected && connection_retry--) {
        connected = SerialBT.connect(DEV1, 1, channels_dev1.begin()->first, sec_mask, role);
        if (connected) {
            Serial.println("Reconnected DEV1! - Simultaneus connection DONE!");
        } else {
            Serial.println("DEV1 not reconnected... FAIL");
        }
        delay(500);
    }
    
    
    delay(2000);
}


void loop() {

 
    prev = millis();
    // senting data to first device
    SerialBT.write('1', 1);
    tdelay = millis() - prev;
    Serial.print("SPP Trasmission duration [ms] for 4 Bytes: ");
    Serial.println(tdelay);

    // waiting for rx
    prev = millis();
    while (!SerialBT.available(1)) {
        ;
    }

    tdelay = millis() - prev;
    Serial.print("01 > SPP answer received after [ms]: ");
    Serial.println(tdelay);
    Serial.write(SerialBT.read(1));
    delay(5);
    prev = millis();
    while (SerialBT.available(1)) {
        Serial.write(SerialBT.read(1));
        delay(5);
    }
    tdelay = millis() - prev;
    Serial.println();
    Serial.print("time to receive data [ms]: ");
    Serial.println(tdelay);

    delay(200);

    // Sending data to second device
    Serial.print("Tx to second device");
    prev = millis();
    
    SerialBT.write('2', 2);
    tdelay = millis() - prev;
    Serial.print("02 > SPP Trasmission duration [ms] for 4 Bytes: ");
    Serial.println(tdelay);

    // waiting for rx
    prev = millis();
    while (!SerialBT.available(2)) {
        ;
    }

    tdelay = millis() - prev;
    Serial.print("01 > SPP answer received after [ms]: ");
    Serial.println(tdelay);
    Serial.write(SerialBT.read(2));
    delay(5);

    prev = millis();
    while (SerialBT.available(2)) {
        Serial.write(SerialBT.read(2));
        delay(5);
    }
    tdelay = millis() - prev;
    Serial.println();
    Serial.print("Lapsed time to receive data [ms]: ");
    Serial.println(tdelay);
    delay(2000);
}
