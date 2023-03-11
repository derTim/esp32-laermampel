/*
 * Display A-weighted sound level measured by I2S Microphone
 * 
 * (c)2019 Ivan Kostoski
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *    
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Sketch samples audio data from I2S microphone, processes the data 
 * with digital IIR filters and calculates A or C weighted Equivalent 
 * Continuous Sound Level (Leq)
 * 
 * I2S is setup to sample data at Fs=48000KHz (fixed value due to 
 * design of digital IIR filters). Data is read from I2S queue 
 * in 'sample blocks' (default 125ms block, equal to 6000 samples) 
 * by 'i2s_reader_task', filtered trough two IIR filters (equalizer 
 * and weighting), summed up and pushed into 'samples_queue' as 
 * sum of squares of filtered samples. The main task then pulls data 
 * from the queue and calculates decibel value relative to microphone 
 * reference amplitude, derived from datasheet sensitivity dBFS 
 * value, number of bits in I2S data, and the reference value for 
 * which the sensitivity is specified (typically 94dB, pure sine
 * wave at 1KHz).
 * 
 * Displays line on the small OLED screen with 'short' LAeq(125ms)
 * response and numeric LAeq(1sec) dB value from the signal RMS.
 */

#define PIN GPIO_NUM_16   //5
#define GND_PIN   GPIO_NUM_21
#define N_LEDS 3 //Anzahl LEDs
#define DB_MIN_DEFAULT 58
#define DB_LOW_DEFAULT 63   //Grün
#define DB_MID_DEFAULT 71   //Gelb
#define DB_HIGH_DEFAULT 74  // Rot
#define DB_MAX_DEFAULT 88   // Alle rot
#define EEPROM_MAGIC_NUMBER 42

#define USE_SERIAL 1
#define BIG_DISPLAY 1
#define BIG_DISPLAY_LEDS 20
#define BIG_DISPLAY_MODE_DEFAULT 1 // 1=individual leds, 2=one color

#define BRIGHTNESS_DEFAULT 200

#define USE_BLE 1



#include <driver/i2s.h>
#include "sos-iir-filter.h"
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>


#if (USE_BLE > 0)
  #include <BLEDevice.h>
  #include <BLEUtils.h>
  #include <BLEServer.h>
#endif

// Preferences
Preferences preferences;
uint32_t db_min;
uint32_t db_low;
uint32_t db_mid;
uint32_t db_high;
uint32_t db_max;
uint32_t big_display_mode;
uint32_t brightness;


// BLE UUIDs
#define SERVICE_UUID                  "cb5f903b-6a3d-4836-b6a9-af3e4d5fb4fd"
#define CHARACTERISTIC_UUID_DB_MIN    "87aa542e-c046-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_DB_LOW    "87aa5726-c046-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_DB_MID    "87aa66e4-c046-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_DB_HIGH   "87aa5898-c046-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_DB_MAX    "87aa59d8-c046-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_MODE      "87aa5b0e-c046-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_BRIGHTNESS "87aa6b0e-c046-11ed-afa1-0242ac120002"


//Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800); // LEDs aktivieren
#if (BIG_DISPLAY > 0)
  Adafruit_NeoPixel strip (BIG_DISPLAY_LEDS, PIN, NEO_GRB + NEO_KHZ400);
#else
  Adafruit_NeoPixel strip (N_LEDS, PIN, NEO_GRB + NEO_KHZ800);
#endif

//
// Configuration
//

#define LEQ_PERIOD        1           // second(s)
#define WEIGHTING         C_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS         "LAeq"      // customize based on above weighting used
#define DB_UNITS          "dBA"       // customize based on above weighting used
#define USE_DISPLAY       0

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER     INMP441    // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB     3.0103      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet) -26
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   116.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      29          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

//
// I2S pins - Can be routed to almost any (unused) ESP32 pin.
//            SD can be any pin, inlcuding input only pins (36-39).
//            SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
//
// Below ones are just example for my board layout, put here the pins you will use
//
// Anschluesse fuer Mikrofon
#define I2S_WS            25 
#define I2S_SCK           32 
#define I2S_SD            33 

// I2S peripheral to use (0 or 1)
#define I2S_PORT          I2S_NUM_0

//
// Setup your display library (and geometry) here
// 
#if (USE_DISPLAY > 0)
  // ThingPulse/esp8266-oled-ssd1306, you may need the latest source and PR#198 for 64x48
  #include <SSD1306Wire.h>
  #define OLED_GEOMETRY     GEOMETRY_64_48
  //#define OLED_GEOMETRY GEOMETRY_128_32
  //#define OLED_GEOMETRY GEOMETRY_128_64
  #define OLED_FLIP_V       1
  SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);
#endif

#if(USE_BLE >0)
  #include "esp_system.h"
  std::string getMacAddress() {
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    char baseMacChr[18] = {0};
    sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    return std::string(baseMacChr);
  }

  class BLE_Callbacks: public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        std::string uuid = pCharacteristic->getUUID().toString();

        if (value.length() > 0) {
          Serial.println(uuid.c_str());
          Serial.print("New value: ");
          for (int i = 0; i < value.length(); i++)
            Serial.print(value[i]);

          Serial.println();
          Serial.println("*********");

          uint32_t newValue = std::stoi(value);
          Serial.println(newValue);
          if(uuid == CHARACTERISTIC_UUID_DB_MIN){
            db_min = newValue;
            preferences.putUInt("db_min", db_min);
            Serial.println("db_min saved!");
          }
          else if(uuid == CHARACTERISTIC_UUID_DB_LOW){
            db_low = newValue;
            preferences.putUInt("db_low", db_low);
            Serial.println("db_low saved!");
          }
          else if(uuid == CHARACTERISTIC_UUID_DB_MID){
            db_mid = newValue;
            preferences.putUInt("db_mid", db_mid);
            Serial.println("db_mid saved!");
          }
          else if(uuid == CHARACTERISTIC_UUID_DB_HIGH){
            db_high = newValue;
            preferences.putUInt("db_high", db_high);
            Serial.println("db_high saved!");
          }
          else if(uuid == CHARACTERISTIC_UUID_DB_MAX){
            db_max = newValue;
            preferences.putUInt("db_max", db_max);
            Serial.println("db_max saved!");
          }
          else if(uuid == CHARACTERISTIC_UUID_MODE){
            big_display_mode = newValue;
            preferences.putUInt("display_mode", big_display_mode);
            Serial.println("big_display_mode saved!");
          }
          else if(uuid == CHARACTERISTIC_UUID_BRIGHTNESS){
            brightness = newValue;
            preferences.putUInt("brightness", brightness);
            strip.setBrightness(brightness);
            Serial.println("brightness saved!");
          }
        }
      }
  };

  bool deviceConnected = false;
  class BLE_Server_Callback: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->getAdvertising()->start();
    }

  };

#endif

//
// IIR Filters
//

// DC-Blocker filter - removes DC component from I2S data
// See: https://www.dsprelated.com/freebooks/filters/DC_Blocker.html
// a1 = -0.9992 should heavily attenuate frequencies below 10Hz
SOS_IIR_Filter DC_BLOCKER = { 
  gain: 1.0,
  sos: {{-1.0, 0.0, +0.9992, 0}}
};

// 
// Equalizer IIR filters to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//
// Filters are represented as Second-Order Sections cascade with assumption
// that b0 and a0 are equal to 1.0 and 'gain' is applied at the last step 
// B and A coefficients were transformed with GNU Octave: 
// [sos, gain] = tf2sos(B, A)
// See: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// NOTE: SOS matrix 'a1' and 'a2' coefficients are negatives of tf2sos output
//

// TDK/InvenSense ICS-43434
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
// B = [0.477326418836803, -0.486486982406126, -0.336455844522277, 0.234624646917202, 0.111023257388606];
// A = [1.0, -1.93073383849136326, 0.86519456089576796, 0.06442838283825100, 0.00111249298800616];
SOS_IIR_Filter ICS43434 = { 
  gain: 0.477326418836803,
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
   {+0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
   {-1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113}
  }
};

// TDK/InvenSense ICS-43432
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/ICS-43432-data-sheet-v1.3.pdf
// B = [-0.45733702338341309   1.12228667105574775  -0.77818278904413563, 0.00968926337978037, 0.10345668405223755]
// A = [1.0, -3.3420781082912949, 4.4033694320978771, -3.0167072679918010, 1.2265536567647031, -0.2962229189311990, 0.0251085747458112]
SOS_IIR_Filter ICS43432 = {
  gain: -0.457337023383413,
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    {-0.544047931916859, -0.248361759321800, +0.403298891662298, -0.207346186351843},
    {-1.909911869441421, +0.910830292683527, +1.790285722826743, -0.804085812369134},
    {+0.000000000000000, +0.000000000000000, +1.148493493802252, -0.150599527756651}
  }
};

// TDK/InvenSense INMP441
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
// B ~= [1.00198, -1.99085, 0.98892]
// A ~= [1.0, -1.99518, 0.99518]
SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696, 
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};

// Infineon IM69D130 Shield2Go
// Datasheet: https://www.infineon.com/dgdl/Infineon-IM69D130-DS-v01_00-EN.pdf?fileId=5546d462602a9dc801607a0e46511a2e
// B ~= [1.001240684967527, -1.996936108836337, 0.995703101823006]
// A ~= [1.0, -1.997675693595542, 0.997677044195563]
// With additional DC blocking component
SOS_IIR_Filter IM69D130 = {
  gain: 1.00124068496753,
  sos: {
    {-1.0, 0.0, +0.9992, 0}, // DC blocker, a1 = -0.9992
    {-1.994461610298131, 0.994469278738208, +1.997675693595542, -0.997677044195563}
  }
};

// Knowles SPH0645LM4H-B, rev. B
// https://cdn-shop.adafruit.com/product-files/3421/i2S+Datasheet.PDF
// B ~= [1.001234, -1.991352, 0.990149]
// A ~= [1.0, -1.993853, 0.993863]
// With additional DC blocking component
SOS_IIR_Filter SPH0645LM4H_B_RB = {
  gain: 1.00123377961525, 
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    {-1.0, 0.0, +0.9992, 0}, // DC blocker, a1 = -0.9992
    {-1.988897663539382, +0.988928479008099, +1.993853376183491, -0.993862821429572}
  }
};

//
// Weighting filters
//

//
// A-weighting IIR Filter, Fs = 48KHz 
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
// B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
// A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
SOS_IIR_Filter A_weighting = {
  gain: 0.169994948147430, 
  sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  }
};

//
// C-weighting IIR Filter, Fs = 48KHz 
// Designed by invfreqz curve-fitting, see respective .m file
// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
// A = [1.0, -1.0325358998928318, -0.9524000181023488, 0.8936404694728326   0.2256286147169398  -0.1499917107550188, 0.0156718181681081]
SOS_IIR_Filter C_weighting = {
  gain: -0.491647169337140,
  sos: { 
    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
};


//
// Sampling
//
#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

// Data we push to 'samples_queue'
struct sum_queue_t {
  // Sum of squares of mic samples, after Equalizer filter
  float sum_sqr_SPL;
  // Sum of squares of weighted mic samples
  float sum_sqr_weighted;
  // Debug only, FreeRTOS ticks we spent processing the I2S data
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;

// Static buffer for block of samples
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

//
// I2S Microphone sampling setup 
//
void mic_i2s_init() {
  // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
  // NOTE: Recent update to Arduino_esp32 (1.0.2 -> 1.0.3)
  //       seems to have swapped ONLY_LEFT and ONLY_RIGHT channels
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,  
    ws_io_num:    I2S_WS,    
    data_out_num: -1, // not used
    data_in_num:  I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0) 
    // Undocumented (?!) manipulation of I2S peripheral registers
    // to fix MSB timing issues with some I2S microphones
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
  #endif
  
  i2s_set_pin(I2S_PORT, &pin_config);

  //FIXME: There is a known issue with esp-idf and sampling rates, see:
  //       https://github.com/espressif/esp-idf/issues/2634
  //       In the meantime, the below line seems to set sampling rate at ~47999.992Hz
  //       fifs_req=24576000, sdm0=149, sdm1=212, sdm2=5, odir=2 -> fifs_reached=24575996  
  //NOTE:  This seems to be fixed in ESP32 Arduino 1.0.4, esp-idf 3.2
  //       Should be safe to remove...
  //#include <soc/rtc.h>
  //rtc_clk_apll_enable(1, 149, 212, 5, 2);
}

//
// I2S Reader Task
//
// Rationale for separate task reading I2S is that IIR filter
// processing cam be scheduled to different core on the ESP32
// while main task can do something else, like update the 
// display in the example
//
// As this is intended to run as separate hihg-priority task, 
// we only do the minimum required work with the I2S data
// until it is 'compressed' into sum of squares 
//
// FreeRTOS priority and stack size (in 32-bit words) 
#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048
//
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    // Block and wait for microphone values from I2S
    //
    // Data is moved from DMA buffers to our 'samples' buffer by the driver ISR
    // and when there is requested ammount of data, task is unblocked
    //
    // Note: i2s_read does not care it is writing in float[] buffer, it will write
    //       integer values to the given address, as received from the hardware peripheral. 
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    // Convert (including shifting) integer microphone values to floats, 
    // using the same buffer (assumed sample size is same as size of float), 
    // to save a bit of memory
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for(int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    // Apply equalization and calculate Z-weighted sum of squares, 
    // writes filtered samples back to the same buffer.
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

    // Apply weighting and calucate weigthed sum of squares
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

    // Debug only. Ticks we spent filtering and summing block of I2S data
    q.proc_ticks = xTaskGetTickCount() - start_tick;

    // Send the sums to FreeRTOS queue where main task will pick them up
    // and further calcualte decibel values (division, logarithms, etc...)
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

//
// Setup and main loop 
//
// Note: Use doubles, not floats, here unless you want to pin
//       the task to whichever core it happens to run on at the moment
// 
void setup() {
  //Set GPIO21 to low to simulate ground
  pinMode(GND_PIN, OUTPUT);
  digitalWrite(GND_PIN, LOW);


  //Init preferences
  preferences.begin("mySetting", false);

  //Load preferences
  db_min = preferences.getUInt("db_min", DB_MIN_DEFAULT);
  db_low = preferences.getUInt("db_low", DB_LOW_DEFAULT);
  db_mid = preferences.getUInt("db_mid", DB_MID_DEFAULT);
  db_high = preferences.getUInt("db_high", DB_HIGH_DEFAULT);
  db_max = preferences.getUInt("db_max", DB_MAX_DEFAULT);
  big_display_mode = preferences.getUInt("display_mode", BIG_DISPLAY_MODE_DEFAULT);
  brightness = preferences.getUInt("brightness", BRIGHTNESS_DEFAULT);

  #if(USE_BLE>0)
    //Setup BLE
    std::string espMac = getMacAddress();
    std::string deviceName = "LaermAmpel_";
    std::string bleName = deviceName+espMac;
    BLEDevice::init(bleName);

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BLE_Server_Callback());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic *pCharacteristic_db_min = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_DB_MIN,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
                                        
    BLECharacteristic *pCharacteristic_db_low = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_DB_LOW,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
                                        
    BLECharacteristic *pCharacteristic_db_high = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_DB_HIGH,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );

    BLECharacteristic *pCharacteristic_db_mid = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_DB_MID,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
                                        
    BLECharacteristic *pCharacteristic_db_max = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_DB_MAX,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
                                        
    BLECharacteristic *pCharacteristic_db_mode = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_MODE,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
                                        
    BLECharacteristic *pCharacteristic_brightness = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_BRIGHTNESS,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
                                        

    pCharacteristic_db_min->setCallbacks(new BLE_Callbacks());
    pCharacteristic_db_low->setCallbacks(new BLE_Callbacks());
    pCharacteristic_db_high->setCallbacks(new BLE_Callbacks());
    pCharacteristic_db_mid->setCallbacks(new BLE_Callbacks());
    pCharacteristic_db_max->setCallbacks(new BLE_Callbacks());
    pCharacteristic_db_mode->setCallbacks(new BLE_Callbacks());
    pCharacteristic_brightness->setCallbacks(new BLE_Callbacks());

    pCharacteristic_db_min->setValue(std::to_string(db_min));
    pCharacteristic_db_low->setValue(std::to_string(db_low));
    pCharacteristic_db_high->setValue(std::to_string(db_high));
    pCharacteristic_db_mid->setValue(std::to_string(db_mid));
    pCharacteristic_db_max->setValue(std::to_string(db_max));
    pCharacteristic_db_mode->setValue(std::to_string(big_display_mode));
    pCharacteristic_brightness->setValue(std::to_string(brightness));

    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();



  #endif

  // If needed, now you can actually lower the CPU frquency,
  // i.e. if you want to (slightly) reduce ESP32 power consumption 
  setCpuFrequencyMhz(80); // It should run as low as 80MHz
  
  #if(USE_SERIAL > 0)
    Serial.begin(112500);
  #endif
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
  // END of Trinket-specific code.

  strip.begin(); //LEDs aktivieren
  strip.clear();

  strip.setBrightness(brightness);

  #if (BIG_DISPLAY > 0)
    for(int i=0; i<20;i++){ //20 LEDs
      strip.setPixelColor(i,strip.Color(0,90,120));
      strip.show();
      delay(50);
    }
    for(int i=19; i>=0;i--){ //20 LEDs
      strip.setPixelColor(i,strip.Color(0,0,0));
      strip.show();
      delay(50);
    }
    strip.show();
  #else
    strip.rainbow(2000,1,255,128,true);
    strip.show(); //LED pixel zuruecksetzen
    delay(1000);
  #endif
  
  #if (USE_DISPLAY > 0)
    display.init();
    #if (OLED_FLIP_V > 0)
      display.flipScreenVertically();
    #endif
    display.setFont(ArialMT_Plain_16);
  #endif

  // Create FreeRTOS queue
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  // Create the I2S reader FreeRTOS task
  // NOTE: Current version of ESP-IDF will pin the task 
  //       automatically to the first core it happens to run on
  //       (due to using the hardware FPU instructions).
  //       For manual control see: xTaskCreatePinnedToCore
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;

  // Read sum of samaples, calculated by 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

    // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;

      #if(USE_SERIAL > 0)
        // Serial output, customize (or remove) as needed
        Serial.printf("\nDB: %.1f | ", Leq_dB);
      #endif

    #if(USE_SERIAL > 0)
        Serial.printf("DISPLAY: %d, DISPLAY_MODE: %d, BRIGHTNESS: %d ", BIG_DISPLAY, big_display_mode, brightness);
        Serial.printf("| DB MIN: %d, LOW: %d, HIGH: %d, MAX: %d ", db_min, db_low, db_high, db_max);
    #endif


    #if (BIG_DISPLAY > 0)
      if(big_display_mode == 1) {
        strip.clear();
        int base=50;
        int db=int(Leq_dB);
        for(int i=0; i<20;i++){ //20 LEDs
          if(db>=base+2*i){ //From 50+0 to 50+38
            strip.setPixelColor(i,strip.Color(0,90,120));
          }
        }
      }
      else if(big_display_mode == 2) {
        // LEDs ansteuern
        strip.clear(); // Set all pixel colors to 'off'     
        // Wenn Pegel groesser 85 ist, dann LED anschalten
        if (Leq_dB >= db_min){
            #if(USE_SERIAL > 0)
            Serial.printf("\r\nDB_MIN (%d)reached", db_min);
            #endif
          for(int i=0; i<20;i++){ //20 LEDs
            strip.setPixelColor(i,strip.Color(0,255,0));
          }
        } 
        // Wenn Pegel groesser 100 ist, dann 2 LEDs anschalten
        if (Leq_dB >= db_low) {
            #if(USE_SERIAL > 0)
            Serial.printf("\r\nDB_LOW (%d)reached", db_low);
            #endif
          for(int i=0; i<20;i++){ //20 LEDs
            strip.setPixelColor(i,strip.Color(190,190,0));
          }
        }
        // Wenn Pegel groesser 110 ist, dann 3 LEDs anschalten
        if (Leq_dB > db_high) {
            #if(USE_SERIAL > 0)
            Serial.printf("\r\nDB_HIGH (%d)reached", db_high);
            #endif
          for(int i=0; i<20;i++){ //20 LEDs
            strip.setPixelColor(i,strip.Color(255,160,0));
          }
        }
        // Wenn Pegel groesser 110 ist, dann 3 LEDs anschalten
        if (Leq_dB > db_max) {
            #if(USE_SERIAL > 0)
            Serial.printf("\r\nDB_MAX (%d)reached", db_max);
            #endif
          for(int i=0; i<20;i++){ //20 LEDs
            strip.setPixelColor(i,strip.Color(255,0,0));
          }
        }
      }
      strip.show();
    #else
      // LEDs ansteuern
      strip.clear(); // Set all pixel colors to 'off'     
      // Wenn Pegel groesser 85 ist, dann LED anschalten
      if (Leq_dB >= db_min){
          #if(USE_SERIAL > 0)
          Serial.printf("\r\nDB_MIN (%d)reached", db_min);
          #endif
        strip.setPixelColor(0,strip.Color(0,90,0));
      } 
      // Wenn Pegel groesser 100 ist, dann 2 LEDs anschalten
      if (Leq_dB >= db_low) {
          #if(USE_SERIAL > 0)
          Serial.printf("\r\nDB_LOW (%d)reached", db_low);
          #endif
        strip.setPixelColor(1,strip.Color(90,60,0));
      }
      // Wenn Pegel groesser 110 ist, dann 3 LEDs anschalten
      if (Leq_dB > db_high) {
          #if(USE_SERIAL > 0)
          Serial.printf("\r\nDB_HIGH (%d)reached", db_high);
          #endif
        strip.setPixelColor(2,strip.Color(90,0,0));    
      }
      // Wenn Pegel groesser 110 ist, dann 3 LEDs anschalten
      if (Leq_dB > db_max) {
          #if(USE_SERIAL > 0)
          Serial.printf("\r\nDB_MAX (%d)reached", db_max);
          #endif
        strip.setPixelColor(0,strip.Color(90,0,0));    
        strip.setPixelColor(1,strip.Color(90,0,0));    
        strip.setPixelColor(2,strip.Color(90,0,0));    
      }
      strip.show();
    #endif


      // Debug only
      //Serial.printf("%u processing ticks\n", q.proc_ticks);
    }

    #if (USE_DISPLAY > 0)

      //
      // Example code that displays the measured value.
      // You should customize the below code for your display 
      // and display library used.
      //
      
      display.clear();

      // It is important to somehow notify when the deivce is out of its range
      // as the calculated values are very likely with big error
      if (Leq_dB > MIC_OVERLOAD_DB) {
        // Display 'Overload' if dB value is over the AOP
        display.drawString(0, 24, "Overload");
      } else if (isnan(Leq_dB) || (Leq_dB < MIC_NOISE_DB)) {
        // Display 'Low' if dB value is below noise floor
        display.drawString(0, 24, "Low");
      }
      
      // The 'short' Leq line
      double short_Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(q.sum_sqr_weighted) / SAMPLES_SHORT) / MIC_REF_AMPL);
      uint16_t len = min(max(0, int(((short_Leq_dB - MIC_NOISE_DB) / MIC_OVERLOAD_DB) * (display.getWidth()-1))), display.getWidth()-1);
      display.drawHorizontalLine(0, 0, len);
      display.drawHorizontalLine(0, 1, len);
      display.drawHorizontalLine(0, 2, len);
      
      // The Leq numeric decibels
      display.drawString(0, 4, String(Leq_dB, 1) + " " + DB_UNITS);
      
      display.display();
      
    #endif // USE_DISPLAY
  }
}

void loop() {
  // Nothing here..
}
