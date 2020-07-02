# PID Controller for Peristaltic Pump with Stepper Motor

Program untuk mengendalikan pompa peristaltik dengan motor stepper dengan menggunakan algoritma PID. Anda dapat mengubah parameter-parameter yang dapat diubah seperti konstanta Kp, Ki, Kd, atau sample time.

Dirancang untuk sistem dengan mikrokontroler STM32F103C6 (Blue Pill) dan sensor Sensirion SLF3S-1300F

Data dapat direkam dengan menggunakan PuTTY atau aplikasi perekam data dari Serial Monitor yang lain dan disimpan dalam bentuk CSV.

Untuk membuat plot dari data yang telah diukur, jalankan file Plot.py dengan Python, lalu masukkan nama file hasil perekaman data yang telah dibuat.
(pastikan telah meng-install library yang diperlukan terlebih dahulu pada requirements.txt)