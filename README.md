# 🦾 HC10 Robot Ethernet Control Demo

## 📌 Overview

This project demonstrates how to control a **Yaskawa Motoman HC10 robot** from a Python application using **High-Speed Ethernet (UDP communication)**.

It is designed as a practical example of integrating industrial robotics with external software systems — without requiring a PLC.

The project supports two main approaches:
- **JOB-based control (recommended)**
- **Direct motion control via Python (advanced use)**

---

## ⚙️ System Architecture

[ PC (Python Application) ] <--UDP--> [ YRC1000 Controller ] --> [ HC10 Robot ]

- Python application sends commands via Ethernet  
- YRC1000 controller executes motion  
- Robot performs physical movement  

---

## 🖥️ Features

### GUI Control
- Run predefined robot JOBs  
- Adjust speed parameters dynamically  
- Monitor robot state (Servo, Mode, Running)  
- Send robot to HOME position  
- Force servo OFF  

### Python API (Low-level)
- Read robot position (Cartesian)  
- Execute linear motion (`move_straight`)  
- Write robot variables (INT / REAL)  
- Start JOB programs remotely  

---

## 🧠 How Control Works

### 1. JOB-Based Control (recommended)

Python:
- sets variables (e.g. speed)  
- selects and starts JOB  

Robot:
- executes motion logic internally  

Example:

yrc.int_variable_write(100, 50)  
yrc.job_select("ETHERNET-EXAMPLE-JOB", 0)  
yrc.job_start()  

---

### 2. Direct Motion Control (advanced)

Python sends motion commands directly:

yrc.move_straight(x, y, z, rx, ry, rz, speed)

Notes:

- Movement is absolute (BASE coordinate system)  
- Units:  
  - Position: micrometers (1000 = 1 mm)  
  - Orientation: 0.0001 degrees  
  - Speed: 0.1 mm/s  
- Commands are non-blocking → user must handle timing  

---

## 🧩 Project Structure

project/
│
├── main.py                      # GUI + application logic  
├── window.py                    # UI layout (Qt Designer)  
├── yrc_high_speed_ethernet.py  # Robot communication layer  

---

## 🚀 Quick Start

### 1. Install dependencies

pip install PyQt5 numpy  

---

### 2. Set robot IP

Edit in main.py:

self.yrc = ClientOfYRC('192.168.xxx.xxx')  

---

### 3. Run application

python main.py  

---

## 🎮 GUI Controls

| Button          | Function                     |
|----------------|-----------------------------|
| Run speed 1     | Slow movement               |
| Run speed 2     | Medium speed                |
| Run speed 3     | Fast movement               |
| Go home         | Move robot to home position |
| Force servo off | Disable servo               |

---

## 🔧 Robot Setup

### Required JOBs

You must create the following JOBs on the robot:

- GO-HOME-ETHERNET-EXAMPLE  
- ETHERNET-EXAMPLE-JOB  

---

### Variables

Python writes to robot variables:

| Variable | Purpose                |
|----------|----------------------|
| I50      | TCP speed (mm/s × 10) |
| I51      | Joint speed (% × 10)  |

Example in JOB:

MOVL V=I50  
MOVJ VJ=I51  

---

### Robot Mode

- Set controller to REMOTE or PLAY  
- Ensure no alarms  
- Enable servo  

---

### Network

PC and robot must be in same subnet:

PC:     192.168.65.100  
Robot:  192.168.65.249  

Test:

ping 192.168.65.249  

---

## 🧪 Useful Examples

### Read robot position

data = yrc.robot_position_data_read()  

---

### Write variable

yrc.int_variable_write(100, 50)  
yrc.real_type_variable_write(3.14, 1)  

---

### Simple motion

yrc.move_straight(500000, 160000, 530000, 1800000, 0, 0, 200)  

---

### Start JOB

yrc.job_select("ETHERNET-EXAMPLE-JOB", 0)  
yrc.job_start()  

---

## ⚠️ Safety

This system controls a real industrial robot.

Always:
- keep workspace clear  
- use low speeds for testing  
- verify motion before execution  
- ensure emergency stop is available  

---

## 🧪 Troubleshooting

| Issue                        | Cause                          |
|-----------------------------|--------------------------------|
| Robot does not move         | Wrong mode                     |
| JOB does not start          | Wrong JOB name                 |
| No connection               | Wrong IP / network             |
| Servo turns OFF             | Safety or job finished         |
| Motion incomplete / drifting| Commands sent too quickly      |

---

## 🏗️ Purpose

Developed as a hackathon demonstrator to showcase:

- industrial robot integration  
- real-time control  
- HMI (GUI + control)  
- Ethernet-based communication  
