# ESP32 + FPGA Secure RFID/NFC Access Controller  
**Hardware Anti-Replay Protection using Intel MAX 10 (DE10-Lite)**  
*December 2025 – Personal Project*  
Logan Browning – Mississippi State University ECE '27

### Demo Video (30 sec)
coming

### Features
- PN532 NFC module (I²C) reading MIFARE cards
- 64-bit token generation (UID + monotonic counter)
- **Hardware replay-attack detection** in Verilog on DE10-Lite FPGA
- Physical servo lock actuation
- Real-time feedback on 16×2 LCD (LCD needs i2c addapter to free up pins) + status LEDs
- Proven 100 % replay prevention in hardware

### Serial Log Proof
Card detected! UID = 9C27F905
FPGA_VALID=1 FPGA_REPLAY=0    ← ACCESS GRANTED
Card detected! UID = 9C27F905
FPGA_VALID=0 FPGA_REPLAY=1    ← REPLAY ATTACK DETECTED

### Hardware
- Hosyond ESP32-WROOM-32 DevKit
- PN532 NFC module
- Intel DE10-Lite (MAX 10 FPGA)
- 16×2 LCD (parallel mode, upgrading to I²C)
- SG90 servo

### Repo Structure
- `/esp32` → Final Arduino sketch
- `/fpga` → Verilog + Quartus .qsf pin assignments
- `/media` → Video, photos, serial logs

### Coming in v2 (Winter 2025–26)
- Custom 2-layer PCB (KiCad + JLCPCB)
- I²C LCD adapter
- 3D-printed enclosure + real electric strike

Open source — MIT License  
Feel free to fork & improve!

linkedin.com/in/logan-browning
