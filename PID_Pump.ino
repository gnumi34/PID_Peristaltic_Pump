#include <Wire.h> // Arduino library for I2C
#include <math.h>

#define PUMP_STEP PB8

float get_flow_raw();
void pumpStepSignal();
unsigned long flowToDelay(int spd);

int ret;
bool stop = false;
bool no_flow = false;

uint16_t sensor_flow_value;
int16_t signed_flow_value;
float scaled_flow_value;
int sensor_flow_crc;

unsigned long last_read = 0;

const int ADDRESS = 0x08; // Sensor I2C Address
const float SCALE_FACTOR_FLOW = 500.0; // Scale Factor for flow rate measurement
const float SCALE_FACTOR_TEMP = 200.0; // Scale Factor for temperature measurement
const char *UNIT_FLOW = " ul/min"; //physical unit of the flow rate measurement
const char *UNIT_TEMP = " deg C"; //physical unit of the temperature measurement

// Variabel yang digunakan untuk sistem kendali PID
float u_now;
float last_u = 0;
float error_now = 0.0;
float error_past_1 = 0.0;
float error_past_2 = 0.0;
float sample_time;
float last_time = 0.0;
float delta_time;
int iteration = 0;

// Masukkan nilai parameter yang diinginkan di sini
float kp = 1.0;
float kd = 0.0;
float ki = 0.0;
float set_flow = 500.0;
// float frekuensi = 1.90262 * pow(set_flow, 0.98124);
float frekuensi = 5.0;
float sensed_flow;
unsigned long delay_time;

TwoWire Wire2 (2,0,100000);

void setup() {
    Serial.begin(115200); // initialize serial communication
    Wire2.begin();       // join i2c bus (address optional for master)
    pinMode(PA12, INPUT_PULLUP);
    pinMode(PUMP_STEP, OUTPUT); // STEP input

    //inisialisasi timer untuk sinyal step pompa
    Timer1.pause();
    Timer1.attachInterrupt(TIMER_CH1, pumpStepSignal);
    Timer1.setPeriod(1000000 / (2 * (unsigned long)frekuensi));
    Timer1.refresh();
    Timer1.resume();

    do {
        // Soft reset the sensor
        Wire2.beginTransmission(0x00);
        Wire2.write(0x06);
        ret = Wire2.endTransmission();
        if (ret != 0) {
            Serial.println("Error while sending soft reset command, retrying...");
            delay(500); // wait long enough for chip reset to complete
        }
    } while (ret != 0);

    Serial.println("Time (s),Flow Rate (uL/min),Frequency,Error,U_Now,No Flow Flag");
    delay(30); // wait long enough for chip reset to complete
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  // To perform a measurement, first send 0x3608 to switch to continuous
  // measurement mode (H20 calibration), then read 3x (2 bytes + 1 CRC byte) from the sensor.
  // To perform a IPA based measurement, send 0x3615 instead.
  // Check datasheet for available measurement commands.
  Wire2.beginTransmission(ADDRESS);
  Wire2.write(0x36);
  Wire2.write(0x08);
  ret = Wire2.endTransmission();
  if (ret != 0) {
    Serial.println("Error during write measurement mode command");
  } else {
    delay(1000);
    while (!stop) {
        sample_time = float(millis());  // Mendapatkan waktu pada saat tegangan berhasil dicuplik
        delta_time = sample_time - last_time;   // Menghitung beda waktu antara waktu cuplik sekarang dengan waktu cuplik sebelumnya
        last_time = sample_time; // Menyimpan waktu cuplik sekarang agar dapat digunakan di pencuplikan berikutnya
        sample_time = delta_time / 1000.0;
        sensed_flow = get_flow_raw();
        
        if ((sensed_flow >= (-6.0)) && (sensed_flow <= 6.0)) {
          no_flow = true;
        } else no_flow = false;

        error_now = set_flow - sensed_flow;  // Menghitung nilai error saat ini
        if ((iteration > 2) && (!no_flow)) {    // Menjaga-jaga agar nilai u_now tidak "kacau" pada saat mikrokontroler menyala
          // Perhitungan nilai sinyal PWM dengan PID
          u_now = (error_now * (kp + (ki*sample_time/2.0) + (kd/sample_time))) + (error_past_1 * ((-kp) + (ki*sample_time/2.0) - (kd/sample_time))) + (error_past_2 * (kd/sample_time)) + last_u;
        } else if ((iteration > 2) && (no_flow)) {
          u_now = 5;
        } else {
          u_now = 0;
        }

        if (frekuensi+u_now > 2000.0) {
            frekuensi = 2000.0;
        } else if (frekuensi+u_now <= 5.0) {
            frekuensi = 5.0;
        } else frekuensi += u_now;

        Timer1.setPeriod(1000000 / (2 * (unsigned long)frekuensi));

        // Menyimpan nilai-nilai saat ini agar dapat digunakan di perhitungan selanjutnya
        last_u = u_now;
        error_past_1 = error_now;
        error_past_2 = error_past_1;

        Serial.print(millis()/1000.0);
        Serial.print(",");
        Serial.print(get_flow_raw());
        Serial.print(",");
        Serial.print(frekuensi);
        Serial.print(",");
        Serial.print(error_now);
        Serial.print(",");
        Serial.print(u_now);
        Serial.print(",");
        Serial.print(no_flow);
        Serial.println("");

        iteration++;

        if (digitalRead(PA12) == LOW) {
            stop = true;
        }
    }
    // To stop the continuous measurement, first send 0x3FF9.
    if (stop) {
        Wire2.beginTransmission(ADDRESS);
        Wire2.write(0x3F);
        Wire2.write(0xF9);
        ret = Wire2.endTransmission();
        if (ret != 0) {
            Serial.println("Error during write measurement mode command");
        }

        Timer1.pause(); //menghentikan timer, sehingga pompa berhenti

        delay(999999);
    }
  }
}

float get_flow_raw() {  
  Wire2.requestFrom(ADDRESS, 3);
  if (Wire2.available() < 3) {
    return 0.0;
  }
  sensor_flow_value  = Wire2.read() << 8; // read the MSB from the sensor
  sensor_flow_value |= Wire2.read();      // read the LSB from the sensor
  sensor_flow_crc    = Wire2.read();

  signed_flow_value = (int16_t) sensor_flow_value;
  scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW * 1000.0;

  return scaled_flow_value;
}

// Fungsi yang mengatur sinyal square untuk menjalankan pompa
void pumpStepSignal() {
  static int signal_state = 0;
  if (signal_state == 0) {
    digitalWrite(PUMP_STEP, HIGH);
    signal_state = 1;
  }
  else {
    digitalWrite(PUMP_STEP, LOW);
    signal_state = 0;
  }
}

unsigned long flowToDelay(int spd) {
  double freq = 1.90262 * pow(spd, 0.98124);
  int delay_time = 1000000 / (2 * freq);
  return delay_time;
}