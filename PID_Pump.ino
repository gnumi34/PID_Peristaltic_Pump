#include <Wire.h> // Arduino library for I2C
#include <math.h>

#define PUMP_STEP PB8

float get_flow_raw();
void pumpStepSignal();
unsigned long flowToDelay(int spd);

int ret;
bool stop = false;
bool no_flow = false;

const int ADDRESS = 0x08; // Sensor I2C Address
const float SCALE_FACTOR_FLOW = 500.0; // Scale Factor for flow rate measurement
const char *UNIT_FLOW = " ul/min"; //physical unit of the flow rate measurement
const char *UNIT_TEMP = " deg C"; //physical unit of the temperature measurement

// Variabel yang digunakan untuk sistem kendali PID
float sensed_flow;
float u_now;
float last_u = 0;
float error_now = 0.0;
float error_past_1 = 0.0;
float error_past_2 = 0.0;
float sample_time;
float last_time = 0.0;
float delta_time;
int iteration = 0;
float f_max = 400.0;
float frekuensi = 5.0;

// Masukkan nilai parameter yang diinginkan di sini
float kp = 0.6;
float kd = 0.0;
float ki = 0.05;
float set_flow = 150.0;

// Variabel perhitungan flow rate rata-rata
float ema_a = 0.1;
float ema = 0.0;
float count = 0.0;
float mean = 0.0;

// Variabel untuk Totalizer
float last_flow = 0.0;
float totalizer = 0.0; // in uL

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

  soft_reset();
  start_measure();

  Serial.println("Time (s),Flow Rate (uL/min),Frequency,Error,U_Now,Moving Average Flow Rate (uL/min),Average Flow Rate (uL/min), Volume Dispensed (uL)");
}

void loop() {
  while (!stop) {
    // Hitung waktu sampling
    sample_time = float(millis());  // Mendapatkan waktu pada saat tegangan berhasil dicuplik
    delta_time = sample_time - last_time;   // Menghitung beda waktu antara waktu cuplik sekarang dengan waktu cuplik sebelumnya
    last_time = sample_time; // Menyimpan waktu cuplik sekarang agar dapat digunakan di pencuplikan berikutnya
    sample_time = delta_time / 1000.0;

    // Dapatkan nilai laju aliran saat ini
    sensed_flow = get_flow();

    // Hitung jumlah volume air
    totalizer += ((sensed_flow + last_flow) / 2.0 * (sample_time) / 60.0);
    last_flow = sensed_flow;

    // Hitung laju aliran rata-rata berjalan
    ema = EMA_function(ema_a, sensed_flow, ema);

    // Hitung laju aliran rata-rata setiap saat
    count++;
    mean = mean + (sensed_flow - mean)/count;

    // Hitung PID
    error_now = set_flow - sensed_flow;  // Menghitung nilai error saat ini
    if (iteration > 2) {    // Menjaga-jaga agar nilai u_now tidak "kacau" pada saat mikrokontroler menyala
      // Perhitungan nilai sinyal PWM dengan PID
      u_now = (error_now * (kp + (ki*sample_time/2.0) + (kd/sample_time))) + (error_past_1 * ((-kp) + (ki*sample_time/2.0) - (kd/sample_time))) + (error_past_2 * (kd/sample_time)) + last_u;
    } else {
      u_now = 0;
    }

    // Ubah nilai frekuensi
    if (frekuensi+u_now > f_max) {
        frekuensi = f_max;
    } else if (frekuensi+u_now <= 5.0) {
        frekuensi = 5.0;
    } else frekuensi += u_now;
    Timer1.setPeriod(1000000 / (2 * (unsigned long)frekuensi));

    // Menyimpan nilai-nilai saat ini agar dapat digunakan di perhitungan selanjutnya
    last_u = u_now;
    error_past_1 = error_now;
    error_past_2 = error_past_1;

    // Catat nilai-nilai pengukuran
    Serial.print(millis()/1000.0);
    Serial.print(",");
    Serial.print(sensed_flow);
    Serial.print(",");
    Serial.print(frekuensi);
    Serial.print(",");
    Serial.print(error_now);
    Serial.print(",");
    Serial.print(u_now);
    Serial.print(",");
    Serial.print(ema);
    Serial.print(",");
    Serial.print(mean);
    Serial.print(",");
    Serial.print(totalizer);
    Serial.println("");

    iteration++;

    if (digitalRead(PA12) == LOW) {
        stop = true;
    }

    delay(5); // Beri waktu untuk sistem agar hasil pengendalian stabil stabil
  }

  if (stop) {
    // To stop the continuous measurement, first send 0x3FF9.
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

float get_flow() {  
  uint16_t sensor_flow_value;
  int16_t signed_flow_value;
  float scaled_flow_value;
  int sensor_flow_crc;
  
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

void soft_reset() {
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
    delay(30); // wait long enough for chip reset to complete
    return;
}

void start_measure() {
  do {
      Wire2.beginTransmission(ADDRESS);
      Wire2.write(0x36);
      Wire2.write(0x08);
      ret = Wire2.endTransmission();
      if (ret != 0) {
        Serial.println("Error during write measurement mode command");
      }
    } while (ret != 0);
    delay(12); // wait until measurement is available
    return;
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

float EMA_function(float alpha, float latest, float stored){
  return (alpha*latest) + ((1-alpha)*stored);
}