
* Python acts as an external controller
* YRC1000 executes motion commands or JOB programs
* Communication is done via High-Speed Ethernet (UDP)

---

## 🖥️ Features

### 🎮 High-level control (GUI)
* ▶️ Run predefined robot JOBs
* ⚡ Adjustable motion speeds
* 🔄 Real-time status monitoring
* 🏠 Automatic homing
* 🛑 Servo ON/OFF control

### ⚙️ Low-level control (Python API)
* 📍 Read robot Cartesian position
* ➡️ Send linear motion commands (`move_straight`)
* 🔢 Write robot variables (INT / REAL)
* 🧠 Parameter-based motion control

---

## 🧠 Control Modes

This project demonstrates two complementary control approaches:

### 1. JOB-based control (recommended for production)

Python:
- sets variables
- triggers JOB execution

Robot:
- executes motion logic internally

✔ safe  
✔ deterministic  
✔ industry standard  

---

### 2. Direct motion control (experimental)

Python sends commands like:

```python
yrc.move_straight(x, y, z, rx, ry, rz, speed)
