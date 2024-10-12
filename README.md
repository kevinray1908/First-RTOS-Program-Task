# Dokumentasi Program STM32 - ADC, Button, dan OLED

Deskripsi Proyek
Proyek ini adalah implementasi dari sistem berbasis STM32 yang mengukur tegangan menggunakan ADC dan menampilkannya pada layar OLED. Sistem ini juga memungkinkan interaksi melalui tombol-tombol fisik dan antarmuka UART untuk memantau serta mengendalikan fungsi tertentu.

Diagram Node/Task
Berikut adalah diagram alur kerja dari task-task utama dalam sistem ini:



Inisialisasi: Mengatur GPIO, ADC, I2C, dan UART.
Task ADC: Membaca nilai tegangan dari input ADC.
Task OLED: Menampilkan hasil pengukuran pada layar OLED.
Task UART: Mengelola komunikasi dan input dari pengguna melalui antarmuka UART.
Task Tombol: Memantau status tombol fisik untuk fungsi tertentu.
Foto Hardware
Berikut adalah foto perangkat keras yang digunakan dalam proyek ini, yang mencakup STM32, OLED, dan dua tombol fisik:



Demo Video/GIF
Berikut adalah demo video proyek ini yang menunjukkan cara kerja sistem dalam membaca nilai tegangan dan menampilkannya di OLED, serta bagaimana pengguna dapat berinteraksi melalui tombol dan UART.

<div align="center" style="padding: 0; margin: 0;"> <video src="https://github.com/user-attachments/assets/f496946b-6c1a-47e8-8f1b-1d19548f5a75" controls style="border: none; outline: none; max-width: 100%; height: auto;"></video> </div>
