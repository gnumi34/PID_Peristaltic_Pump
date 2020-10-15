/** Program pengendali pompa peristaltik dengan PID **/

/** Keterangan variabel yang dapat diubah:
 * kp : konstanta proporsional
 * kd : konstanda derivatif
 * ki : konstanta integral
 * f_min : frekuensi minimum pompa
 * set_flow : set point laju aliran
**/

/** Keterangan fungsi:
 * void init_timer() : inisialisasi timer
 * void get_flow() : mendapatkan data laju aliran dan data air-in-line flag dari sensor
 * void sensor_soft_reset() : melakukan soft reset pada sensor
 * void sensor_start_measure() : memberikan perintah memulai pengukuran
 * void pumpStepSignal() : memberikan sinyal untuk pompa peristaltik jenis stepper
 * void pid_control() : algoritma pengendali PID utama (beserta perhitungan volume dan rata-rata laju aliran)
 * float check_max_freq() : melakukan pengecekan frekuensi maksimum pompa yang diperbolehkan berdasarkan nilai set point
 * float EMA_function(float alpha, float latest, float stored) : fungsi untuk mendapatkan data Exponential Moving Average
 * void check_set_point() : melakukan pengecekan dan pertambahan nilai set point untuk mempercepat sistem menuju set point
 * void write_data() : melakukan penulisan data-data yang ingin diukur pada saluran serial
 * void stop_sensor_and_pump() : melakukan penghentian pengukuran pada sensor dan penghentian pompa peristaltik
**/

/** Variabel yang dapat diambil datanya:
 * time : waktu terkini pengukuran
 * scaled_flow_value : laju aliran terukur setiap saat
 * frekuensi : frekuensi setiap saat
 * error_now : nilai error kontrol (set_point - laju aliran terukur)
 * u_now : nilai kontrol yang akan diberikan untuk frekuensi pompa
 * ema : data Exponential Moving Average dari laju aliran
 * mean : nilai rata-rata laju aliran
 * totalizer : nilai volume yang telah dialirkan oleh sistem
 * (aux_value & 1) : Tanda adanya bubble pada saluran aliran fluida
**/

/** belum ada pin disable driver motor stepper **/

#include <Wire.h>
#include <math.h>

#define PUMP_STEP PB3
#define PUMP_DIR PB4

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
float frekuensi = 100.0;

// Masukkan nilai parameter yang diinginkan di sini
float kp = 0.5;
float ki = 0.25;
float kd = 0.00008;
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
void sensor_soft_reset();
void sensor_start_measure();
void pumpStepSignal();
void pid_control();
float check_max_freq();
float EMA_function(float alpha, float latest, float stored);
void check_set_point();
void write_data();
void stop_sensor_and_pump();

void setup() {
  Serial.begin(115200); // initialize serial communication
  Wire2.begin();       // join i2c bus (address optional for master)
  pinMode(PA12, INPUT_PULLUP);
  pinMode(PUMP_STEP, OUTPUT); // STEP input
  pinMode(PUMP_DIR, OUTPUT); // DIR input

  // Atur arah putaran pompa berdasarkan nilai set point laju aliran
  if (set_flow < 0) {
    digitalWrite(PUMP_DIR, HIGH);
  }
  else if (set_flow > 0) {
    digitalWrite(PUMP_DIR, LOW);
  }

  init_timer();
  sensor_soft_reset();
  sensor_start_measure();

  f_max = check_max_freq();  

  Serial.println("Time (s),Flow Rate (uL/min),Frequency,Error,U_Now,Moving Average Flow Rate (uL/min),Average Flow Rate (uL/min),Volume Dispensed (uL),Bubble Flag,Sample Time");
}

void loop() {
  while (!stop) {
    pid_control();
    write_data();
 
    // Function for volume check
    if (totalizer > fabs(set_flow)) {
      stop = true;
    }
    
    if (digitalRead(PA12) == LOW) {
        stop = true;
    }
  }

  if (stop) {
    stop_sensor_and_pump();
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
    aux_value = 3;
    return;
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
  return;
}

void sensor_soft_reset() {
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

void sensor_start_measure() {
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

void pid_control() {
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

  // Hitung laju aliran rata-rata setiap saat (hanya bila ada fluida)
  if ((aux_value & 1) != 1) {
    count++;
    mean = mean + (scaled_flow_value - mean) / count;
  }
  
  // Hitung PID
  error_now = realtime_flow - scaled_flow_value;  // Menghitung nilai error saat ini
  if ((iteration > 2) && ((aux_value & 1) != 1)) {    // Menjaga-jaga agar nilai u_now tidak "kacau" pada saat mikrokontroler menyala
    // Perhitungan nilai sinyal PWM dengan PID
    u_now = (error_now * (kp + (ki*sample_time/2.0) + (kd/sample_time))) + (error_past_1 * ((-kp) + (ki*sample_time/2.0) - (kd/sample_time))) + (error_past_2 * (kd/sample_time)) + last_u;
  } else {
    u_now = 0.0;
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
  if ((aux_value & 1) == 0) {
    last_u = u_now;
    error_past_1 = error_now;
    error_past_2 = error_past_1;
  }
  

  // Check set point
  if (((mean < set_flow) && (set_flow > 0)) || ((mean > set_flow) && (set_flow < 0))) {
    check_set_point();
  } else {
    realtime_flow = set_flow;
  }

  iteration++;
  delay(5);
}

float check_max_freq() {
  if ((set_flow >= 1000.0) || (set_flow <= -1000.0)) {
    return set_flow + 2400.0;
  } else if ((set_flow >= 900.0) || (set_flow <= -900.0)) {
    return set_flow + 2100.0;
  } else if ((set_flow >= 800.0) || (set_flow <= -800.0)) {
    return set_flow + 1750.0;
  } else if ((set_flow >= 700.0) || (set_flow <= -700.0)) {
    return set_flow + 1500.0;
  } else if ((set_flow >= 600.0) || (set_flow <= -600.0)) {
    return set_flow + 1300.0;
  } else if ((set_flow >= 500.0) || (set_flow <= -500.0)) {
    return set_flow + 1100.0;
  } else if ((set_flow >= 400.0) || (set_flow <= -400.0)) {
    return set_flow + 900.0;
  } else if ((set_flow >= 300.0) || (set_flow <= -300.0)) {
    return set_flow + 800.0;
  } else if ((set_flow >= 200.0) || (set_flow <= -200.0)) {
    return set_flow + 650.0;
  } else if ((set_flow >= 175.0) || (set_flow <= -175.0)) {
    return 650.0;
  } else if ((set_flow >= 150.0) || (set_flow <= -150.0)) {
    return 550.0;
  } else if ((set_flow >= 125.0) || (set_flow <= -125.0)) {
    return 450.0;
  } else if ((set_flow >= 100.0) || (set_flow <= -100.0)) {
    return 375.0;
  } else if ((set_flow >= 70.0) || ((set_flow) <= -70.0)) {
    return 300.0;
  } else if ((set_flow >= 50.0) || ((set_flow) <= -50.0)) {
    return 225.0;
  } else if ((set_flow >= 25.0) || ((set_flow) <= -25.0)) {
    return 175.0;
  } else {
    return 100.0;
  }
}

float EMA_function(float alpha, float latest, float stored) {
  return (alpha*latest) + ((1-alpha)*stored);
}

void check_set_point() {
  time = millis()/1000.0;
  if (((time - last_check) > 2.0) && ((aux_value & 1) != 1)) {
    if (set_flow > 0) {
      realtime_flow += 1.0;
    }
    else if (set_flow < 0) {
      realtime_flow -= 1.0;
    }
    last_check = time;
  }
  return;
}

void write_data() {
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
    Serial.print(aux_value & 1);
    Serial.print(",");
    Serial.print(sample_time, 3);
    Serial.println("");
    return;
}

void stop_sensor_and_pump() {
  // Kirim data 0x3FF9 untuk menghentikan pengukuran pada sensor
    Wire2.beginTransmission(ADDRESS);
    Wire2.write(0x3F);
    Wire2.write(0xF9);
    ret = Wire2.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    }

    Timer1.pause(); //menghentikan timer, sehingga pompa berhenti
}