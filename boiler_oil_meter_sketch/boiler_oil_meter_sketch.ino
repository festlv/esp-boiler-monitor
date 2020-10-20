// pin & network configuration
#include "config.h"

//esp32 includes
#include <WiFi.h>
#include <SPIFFS.h>
#include <time.h>

//screen includes
#include <TFT_eSPI.h>
#include <SPI.h>

//temp sensor includes
#include <OneWire.h>
#include <DallasTemperature.h>

#include <arduinoFFT.h>

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_SENSOR_PIN);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature tempSensors(&oneWire);


TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
  
arduinoFFT FFT = arduinoFFT();

const uint32_t NUM_SAMPLES = 256;
const uint32_t SAMPLING_FREQUENCY = 256;

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

bool SPIFFS_initialized = false;

typedef enum {
  STATE_WAIT_RISING = 0,
  STATE_WAIT_FALLING,
  STATE_WAIT_SETTLE
} meas_state_t;


struct {
  float raw_amplitude;
  bool  boiler_on;

  uint64_t last_rising_time_ms;
  uint64_t first_falling_time_ms;
  uint32_t falling_edge_count;
  float current_on_period_s;
  
  uint32_t on_counter;
  
  float day_on_total_s;
  
  uint8_t total_day;

  float month_on_total_s;
  uint8_t total_month;
 
  float outside_temperature_C;
  
  meas_state_t state;
  float day_on_total_l;
  float month_on_total_l;
} measurement;

void setup() {
    
  Serial.begin(1000000);
  
  if(!SPIFFS.begin(true)){
     Serial.println("# An Error has occurred while mounting SPIFFS");
     SPIFFS_initialized = false;
  } else {
    SPIFFS_initialized = true;
  }

  memset(&measurement, 0, sizeof(measurement));
  
  tft.init();
  tft.setRotation(0);
  Serial.println("# tft initialized");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 2);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);  
  tft.setTextSize(1);
  tft.println("Boiler monitoring");

  tempSensors.begin();

  String mac = WiFi.macAddress();
  mac.replace(":", "-");
  
  char mac_cstr[20];
  mac.substring(12).toCharArray(mac_cstr, 6);
  mac_cstr[5] = 0;
  
  char hostname[32];
  snprintf(hostname, sizeof(hostname), "boileresp-%s", mac_cstr);
  
  Serial.print("WiFi: MAC: "); Serial.println(mac);
  Serial.print("WiFi: setting hostname "); Serial.println(hostname);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);

  WiFi.begin(SSID, PASSWORD);
  tft.print("Connecting: ");
  tft.println(SSID);
  
  uint64_t wifi_start_ms = millis64();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if ((millis64() - wifi_start_ms) > 30 * 1000) {
      break;
    }
  }

  if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
  } else {
    Serial.println("mounted SPIFFS");
    if (read_measurement()) {
      Serial.println("measurements read from flash");
    } else {
      Serial.println("failed to read measurements from flash");
    }
  }
  measurement.state = STATE_WAIT_RISING;
  sync_time();
  tempSensors.requestTemperatures();
  measurement.outside_temperature_C = tempSensors.getTempCByIndex(0);
  Serial.println("setup finished"); 
}

bool write_measurement() {
  File file = SPIFFS.open("/measurement.bin", FILE_WRITE);
  uint32_t b_written = file.write((uint8_t*)&measurement, sizeof(measurement));
  file.close();
  return b_written == sizeof(measurement);
}

bool read_measurement() {
  File file = SPIFFS.open("/measurement.bin", FILE_READ);
  uint32_t b_read = file.read((uint8_t*)&measurement, sizeof(measurement));
  file.close();
  return b_read == sizeof(measurement);
}

void print_status() {
  Serial.print("temperature_C=");
  Serial.println(measurement.outside_temperature_C, 2);
  Serial.print("boiler_state=");
  Serial.println(measurement.boiler_on);
  
  Serial.print("day_on_total_s=");
  Serial.println(measurement.day_on_total_s);

  Serial.print("month_on_total_s=");
  Serial.println(measurement.month_on_total_s);
  
  Serial.print("day_on_total_l=");
  Serial.println(measurement.day_on_total_l);

  Serial.print("month_on_total_l=");
  Serial.println(measurement.month_on_total_l);
  
  Serial.print("on_counter=");
  Serial.println(measurement.on_counter);

}
void print_fuel_measurement(uint32_t time_ms, float fuel_l) {
  Serial.print("boiler_fuel_consumed_l=");
  Serial.println(fuel_l, 5);
  
  Serial.print("boiler_on_time_s=");
  Serial.println(time_ms/1000.0f, 2);
}

void display_measurement() {
  
  tft.setCursor(0, 16, 2);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("IP:");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  if (WiFi.status() == WL_CONNECTED) {
    tft.println(WiFi.localIP());
  } else {
    tft.println("N/A");
  }
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Raw ampl: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(measurement.raw_amplitude, 4);


  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("State:     ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  if (measurement.boiler_on)
    tft.println("ON ");
  else
    tft.println("OFF");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("On period: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print(measurement.current_on_period_s, 0);
  tft.println(" s");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("On counter: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(measurement.on_counter);

  char tmp[32];
  snprintf(tmp, sizeof(tmp), "%5.2f l", measurement.day_on_total_l);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Daily: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(tmp);
  
  snprintf(tmp, sizeof(tmp), "%5.2f l", measurement.month_on_total_l);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Monthly: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(tmp);
  
  snprintf(tmp, sizeof(tmp), "%5.1f C", measurement.outside_temperature_C);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Out temp: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println(tmp);

  tft.setCursor(20, 220, 2);
  display_time();
  
}

void sync_time() {
  if (WiFi.status() == WL_CONNECTED) {
    configTime(NTP_GMT_OFFSET, NTP_DST_OFFSET, NTP_SERVER);
    printLocalTime();
  } else {
    Serial.println("wifi not connected, can't sync time");
  }

  struct tm ti;
  if(!getLocalTime(&ti)){
    Serial.println("Failed to obtain time");
    return;
  }
  measurement.total_day = ti.tm_mday;
  measurement.total_month = ti.tm_mon;
}

void display_time() {
  struct tm ti;
  if(!getLocalTime(&ti)){
    Serial.println("Failed to obtain time");
    return;
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.println(&ti, "%d.%m %H:%M:%S");

}

void check_reset_counters() {
  struct tm ti;
  if(!getLocalTime(&ti)){
    Serial.println("Failed to obtain time");
    return;
  }

  if (measurement.total_day != ti.tm_mday) {
    measurement.day_on_total_s = 0.0f;
    measurement.total_day = ti.tm_mday;
  }

  if (measurement.total_month != ti.tm_mon) {
    measurement.month_on_total_s = 0.0f;
    measurement.total_month = ti.tm_mon;
  }
}


void measurement_loop() {
  static uint32_t current_sample_idx = 0;
  const double alpha = 0.1;
  static double previousValue = 0.0;
  static double currentValue = 0.0;
  
  currentValue = alpha * analogRead(CURRENT_TRANSFORMER_PIN) + (1 - alpha) * previousValue;
  previousValue = currentValue;
  //Serial.println(currentValue);
  // Reading potentiometer value
  
  vReal[current_sample_idx] = currentValue;
  vImag[current_sample_idx] = 0.0;

  current_sample_idx++;

  if (current_sample_idx == NUM_SAMPLES) {
    
    //subtract mean
    double sum = 0.0;
    for (int i=0; i < NUM_SAMPLES; i++)
      sum+=vReal[i];

    double mean = sum / NUM_SAMPLES;

    for (int i=0; i < NUM_SAMPLES; i++)
      vReal[i] = vReal[i] - mean;
      
    //Serial.println("Data:");
    //PrintVector(vReal, NUM_SAMPLES, SCL_TIME);
    FFT.Windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); 
    //Serial.println("Weighed data:");
    //PrintVector(vReal, NUM_SAMPLES, SCL_TIME);
    FFT.Compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
    //Serial.println("Computed Real values:");
    //PrintVector(vReal, NUM_SAMPLES, SCL_INDEX);
    //Serial.println("Computed Imaginary values:");
    //PrintVector(vImag, NUM_SAMPLES, SCL_INDEX);
    FFT.ComplexToMagnitude(vReal, vImag, NUM_SAMPLES); 

    //normalize
    
    sum = 0.0;
    for (int i = 0; i < NUM_SAMPLES; i++)
      sum += vReal[i];
    
    for (int i = 0; i < NUM_SAMPLES; i++)
      vReal[i] = vReal[i] / sum;


    //Main debugging entry point: printing computed magnitudes and 50th FFT bin
    //Serial.println("Computed magnitudes:");
    //PrintVector(vReal, (NUM_SAMPLES >> 1), SCL_FREQUENCY);
    //double x = FFT.MajorPeak(vReal, NUM_SAMPLES, SAMPLING_FREQUENCY);
    //Serial.println(x, 6);

    //double around50 = vReal[6] + vReal[7];
    //Serial.print("50Hz: ");
    //Serial.println(vReal[50] * 100.0, 6);
    //Serial.print("Around 50Hz: ");
    //Serial.println(around50 * 100.0, 6);

    double fft_bin = vReal[50];
    bool boiler_on = fft_bin > 0.01;
    measurement.boiler_on = false;
    measurement.raw_amplitude = fft_bin;

    switch (measurement.state) {
      case STATE_WAIT_RISING:
      {
        if (boiler_on) {
          measurement.last_rising_time_ms = millis64();
          measurement.state = STATE_WAIT_FALLING;
          measurement.falling_edge_count = 0;
        }
      }
      break;
      case STATE_WAIT_FALLING:
      {
        measurement.current_on_period_s = (millis64() - measurement.last_rising_time_ms) / 1000.0f;
        if (measurement.current_on_period_s >= 10.0f)
          measurement.boiler_on = true;
          
        if (!boiler_on) {
          if (measurement.falling_edge_count == 0)
            measurement.first_falling_time_ms = millis64();
          
          measurement.falling_edge_count++;
          if (measurement.falling_edge_count > 3) {
            measurement.boiler_on = false;
            uint64_t delta_ms;
            uint64_t millis_ms = measurement.first_falling_time_ms;
            
            if (millis_ms > measurement.last_rising_time_ms)
              delta_ms = millis_ms - measurement.last_rising_time_ms;
            else
              delta_ms = UINT64_MAX - measurement.last_rising_time_ms + millis_ms;
  
            if (delta_ms < 10000) {
              //probably a glitch, reject it
              measurement.state = STATE_WAIT_RISING;
              break;
            } else {
              measurement.current_on_period_s = delta_ms / 1000.0f;
              //register a measurement for delta_ms duration
              measurement.day_on_total_s += delta_ms / 1000.0f;
              measurement.month_on_total_s += delta_ms / 1000.0f;
              measurement.state = STATE_WAIT_RISING;
              measurement.on_counter++;

              measurement.day_on_total_l = measurement.day_on_total_s / 3600.0f * BOILER_L_PER_HOUR;
              measurement.month_on_total_l = measurement.month_on_total_s / 3600.0f * BOILER_L_PER_HOUR;
              print_fuel_measurement(delta_ms, delta_ms / 1000.0f / 3600.0f * BOILER_L_PER_HOUR);
            }
          }
          
        } else {
          measurement.falling_edge_count = 0;
        }
      }
      break;
      
    }
    current_sample_idx = 0;

    

    static uint32_t reset_check_counter = 0;
    reset_check_counter++;
    if (reset_check_counter % 60 == 0) {
      check_reset_counters();
    }

    if (reset_check_counter % 30 == 0) {
      tempSensors.requestTemperatures();
      measurement.outside_temperature_C = tempSensors.getTempCByIndex(0);
    }

    if (reset_check_counter % 5 == 0) {
      print_status();
    }

    if (reset_check_counter % 600 == 0) {
      //write measurement to flash every 10 minutes
      if (write_measurement()) {
        Serial.println("wrote measurement to flash");
      } else {
        Serial.println("failed to write measurement from flash");
      }
    }

    display_measurement();
  }
  
  delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
}

void loop() {
  measurement_loop();
}






void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / SAMPLING_FREQUENCY);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * SAMPLING_FREQUENCY) / NUM_SAMPLES);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}



void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
}

uint64_t millis64() {
return (uint64_t)(esp_timer_get_time() / 1000LL);
}
