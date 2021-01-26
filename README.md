# PID Controller for Peristaltic Pump with Stepper Motor

Program untuk mengendalikan pompa peristaltik dengan motor stepper dengan menggunakan algoritma PID. Anda dapat mengubah parameter-parameter yang dapat diubah seperti nilai *set point* laju aliran, konstanta Kp, Ki, dan Kd. Pompa tersebut digunakan untuk pengaliran fluida dalam aplikasi *microfluidics* (cairan dalam skala mikroliter).

Dirancang untuk sistem dengan mikrokontroler STM32F103C6 (Blue Pill), FT232RL USB-to-TTL (atau sejenisnya) dan sensor Sensirion SLF3S-1300F.

Data dapat direkam dengan menggunakan PuTTY atau aplikasi perekam data dari Serial Monitor yang lain dan disimpan dalam bentuk CSV.

Untuk membuat plot dari data yang telah diukur, jalankan file Plot.py dengan Python dengan perintah `plot.py file_csv mode_number`
(pastikan telah meng-install library yang diperlukan terlebih dahulu pada requirements.txt dengan 'pip3 install -r requirements.txt')

Program digunakan untuk Tugas Akhir Teknik Elektro ITB "*Sistem Pompa untuk Pemrosesan Sampel pada Platform Mikrofluida*".
