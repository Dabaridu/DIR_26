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

```python
yrc.int_variable_write(100, 50)
yrc.job_select("ETHERNET-EXAMPLE-JOB", 0)
yrc.job_start()
