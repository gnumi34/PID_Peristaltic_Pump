#include <Wire.h> // Arduino library for I2C
#include <math.h>

#define PUMP_STEP PB8

int ret;
bool stop = false;
bool no_flow = false;

const int ADDRESS = 0x08; // Sensor I2C Address
const float SCALE_FACTOR_FLOW = 500.0; // Scale Factor for flow rate measurement
const char *UNIT_FLOW = " ul/min"; //physical unit of the flow rate measurement
const char *UNIT_TEMP = " deg C"; //physical unit of the temperature measurement

// Variabel global sensor
uint16_t aux_value = 0;
int aux_crc;

// Variabel yang digunakan untuk sistem kendali PID
float scaled_flow_value;
float u_now;
float last_u = 0;
float error_now = 0.0;
float error_past_1 = 0.0;
float error_past_2 = 0.0;
float sample_time;
float last_time = 0.0;
float delta_time;
int iteration = 0;
float f_max;
float f_min = 25.0;
float frekuensi = f_min;

// Masukkan nilai parameter yang diinginkan di sini
float kp = 0.5;
float kd = 0.0;
float ki = 0.09;
float set_flow = 100.0;
float realtime_flow = set_flow;
float last_check = 0.0;

// Variabel perhitungan flow rate rata-rata
float ema_a = 0.1;
float ema = 0.0;
float count = 0.0;
float mean = 0.0;

// Variabel untuk Totalizer
float last_flow = 0.0;
float totalizer = 0.0; // in uL

float time;

TwoWire Wire2 (2,0,100000);

// Function Declaration
void init_timer();
void get_flow();
void soft_reset();
void start_measure();
void pumpStepSignal();
float check_max_freq();
float EMA_function(float alpha, float latest, float stored);

void setup() {
  Serial.begin(115200); // initialize serial communication
  Wire2.begin();       // join i2c bus (address optional for master)
  pinMode(PA12, INPUT_PULLUP);
  pinMode(PUMP_STEP, OUTPUT); // STEP input

  init_timer();
  soft_reset();
  start_measure();

  f_max = check_max_freq();  

  Serial.println("Time (s),Flow Rate (uL/min),Frequency,Error,U_Now,Moving Average Flow Rate (uL/min),Average Flow Rate (uL/min),Volume Dispensed (uL),No Flow Flag,Set Point");
}

void loop() {
  while (!stop) {
    // Hitung waktu sampling
    sample_time = float(millis());  // Mendapatkan waktu pada saat tegangan berhasil dicuplik
    delta_time = sample_time - last_time;   // Menghitung beda waktu antara waktu cuplik sekarang dengan waktu cuplik sebelumnya
    last_time = sample_time; // Menyimpan waktu cuplik sekarang agar dapat digunakan di pencuplikan berikutnya
    sample_time = delta_time / 1000.0;

    // Dapatkan nilai laju aliran saat ini
    get_flow();

    if ((aux_value & 1) == 1) {
      scaled_flow_value = 0.0;
    }

    // Hitung jumlah volume fluida
    if (set_flow > 0) {
      totalizer += ((scaled_flow_value + last_flow) / 2.0 * (sample_time) / 60.0);
    }
    else if (set_flow < 0) {
      totalizer -= ((scaled_flow_value + last_flow) / 2.0 * (sample_time) / 60.0);
    }
    last_flow = scaled_flow_value;

    // Hitung laju aliran rata-rata berjalan
    ema = EMA_function(ema_a, scaled_flow_value, ema);

    // Hitung laju aliran rata-rata setiap saat
    count++;
    mean = mean + (scaled_flow_value - mean) / count;
    
    // Hitung PID
    error_now = realtime_flow - scaled_flow_value;  // Menghitung nilai error saat ini
    if ((iteration > 2) && ((aux_value & 1) != 1)) {    // Menjaga-jaga agar nilai u_now tidak "kacau" pada saat mikrokontroler menyala
      // Perhitungan nilai sinyal PWM dengan PID
      u_now = (error_now * (kp + (ki*sample_time/2.0) + (kd/sample_time))) + (error_past_1 * ((-kp) + (ki*sample_time/2.0) - (kd/sample_time))) + (error_past_2 * (kd/sample_time)) + last_u;
    } else {
      u_now = u_now;
    }

    // Ubah nilai frekuensi
    if (set_flow > 0) {
      if (frekuensi + u_now > f_max) {
          frekuensi = f_max;
      } else if (frekuensi + u_now <= f_min) {
          frekuensi = f_min;
      } else frekuensi += u_now;
      Timer1.setPeriod(1000000 / (2 * (unsigned long)frekuensi));
    } else if (set_flow < 0) {
      if (frekuensi - u_now > f_max) {
          frekuensi = f_max;
      } else if (frekuensi - u_now <= f_min) {
          frekuensi = f_min;
      } else frekuensi -= u_now;
      Timer1.setPeriod(1000000 / (2 * (unsigned long)frekuensi));
    }

    // Menyimpan nilai-nilai saat ini agar dapat digunakan di perhitungan selanjutnya
    last_u = u_now;
    error_past_1 = error_now;
    error_past_2 = error_past_1;

    time = millis()/1000.0;
    // Catat nilai-nilai pengukuran
    Serial.print(time, 3);
    Serial.print(",");
    Serial.print(scaled_flow_value);
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
    Serial.print(",");
    Serial.print(aux_value);
    Serial.print(",");
    Serial.print(realtime_flow);
    Serial.println("");

    iteration++;

    if (((time - last_check) > 1.0) && ((aux_value & 1) != 1)) {
      if (mean < set_flow) {
        realtime_flow += 1.0;
      }
      last_check = time;
    }
    if (mean > set_flow) {
      realtime_flow = set_flow;
    }

    if (totalizer > (set_flow * 2)) {
       stop = true;
    }
    
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

void init_timer() {
    //inisialisasi timer untuk sinyal step pompa
  Timer1.pause();
  Timer1.attachInterrupt(TIMER_CH1, pumpStepSignal);
  Timer1.setPeriod(1000000 / (2 * (unsigned long)frekuensi));
  Timer1.refresh();
  Timer1.resume();

  return;
}

void get_flow() {  
  uint16_t sensor_flow_value, sensor_temp_value;
  int16_t signed_flow_value;
  int sensor_flow_crc, sensor_temp_crc;
  
  Wire2.requestFrom(ADDRESS, 9);
  if (Wire2.available() < 9) {
    scaled_flow_value = 0.0;
  }

  sensor_flow_value  = Wire2.read() << 8; // read the MSB from the sensor
  sensor_flow_value |= Wire2.read();      // read the LSB from the sensor
  sensor_flow_crc    = Wire2.read();
  sensor_temp_value  = Wire2.read() << 8; // read the MSB from the sensor
  sensor_temp_value |= Wire2.read();      // read the LSB from the sensor
  sensor_temp_crc    = Wire2.read();
  aux_value          = Wire2.read() << 8; // read the MSB from the sensor
  aux_value         |= Wire2.read();      // read the LSB from the sensor
  aux_crc            = Wire2.read();

  signed_flow_value = (int16_t) sensor_flow_value;
  scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW * 1000.0;
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

float check_max_freq() {
  // Set maksimum frekuensi berdasarkan set point
  if ((set_flow > 180.0) || (set_flow < -180.0)) {
    return 800.0;
  } else if ((set_flow > 140.0) || (set_flow < -140.0)) {
    return 600.0;
  }
  else {
    return 400.0;
  }
}

float EMA_function(float alpha, float latest, float stored) {
  return (alpha*latest) + ((1-alpha)*stored);
}