# 🦾 HC10 Robot Ethernet Control Demo

## 📌 Overview

This project demonstrates remote control of a **Yaskawa Motoman HC10 robot** using a Python-based application over **High-Speed Ethernet (UDP communication)**.

The system provides a simple GUI interface for:

* running predefined robot jobs
* adjusting motion speed parameters
* monitoring robot status in real time

---

## ⚙️ System Architecture

```
[ PC (Python App) ]  <--UDP-->  [ YRC1000 Controller ]  -->  [ HC10 Robot ]
```

* Python application acts as a high-level controller
* YRC1000 executes robot jobs
* Communication is done via Ethernet (no PLC required)

---

## 🖥️ Features

* ▶️ Run robot jobs from GUI
* ⚡ Adjustable motion speeds (TCP / rotational / joint)
* 🔄 Real-time status monitoring
* 🏠 Automatic homing on startup
* 🛑 Safe servo control (ON/OFF)

---

## 🧠 How it works

1. Application connects to robot controller via IP
2. Servo power is enabled
3. Robot is sent to HOME position
4. User triggers predefined jobs
5. Speed parameters are sent as variables
6. Robot executes JOB program
7. Status is continuously monitored

---

## 🧩 Key Components

### GUI

* Built with PyQt5
* Buttons for job execution and control
* Status display (Servo / Job)

### Robot Communication

* UDP-based protocol
* Implemented in:

```
yrc_high_speed_ethernet.py
```

### Main Application

* Handles logic and UI interaction
* File:

```
main.py
```

### UI Layout

* Generated from Qt Designer
* File:

```
window.py
```

---

## 🚀 Usage

### 1. Requirements

```bash
pip install PyQt5 numpy
```

---

### 2. Configure robot IP

In `main.py`:

```python
self.yrc = ClientOfYRC('192.168.xxx.xxx')
```

---

### 3. Run application

```bash
python main.py
```

---

## 🎮 Controls

| Button          | Function                    |
| --------------- | --------------------------- |
| Run speed 1     | Slow movement               |
| Run speed 2     | Medium speed                |
| Run speed 3     | Fast movement               |
| Go home         | Move robot to home position |
| Force servo off | Disable servo               |

---

## 🔧 Robot Setup (IMPORTANT)

Before running the application, make sure the robot is properly configured:

### Required JOBs

The following JOBs must exist on the controller:

* `GO-HOME-ETHERNET-EXAMPLE`
* `ETHERNET-EXAMPLE-JOB`

👉 These JOBs are executed from the Python application.

---

### Variables used

The application writes to robot variables:

| Variable | Purpose                |
| -------- | ---------------------- |
| 50       | TCP speed (mm/s * 100) |
| 51       | Joint speed (% * 100)  |

👉 These variables must be used inside the JOB program.

---

### Robot Mode

* Set controller to **REMOTE or PLAY mode**
* Ensure **no active alarms**
* Ensure **servo can be enabled**

---

### Network Configuration

* PC and robot must be in the same subnet

Example:

```
PC:     192.168.65.100
Robot:  192.168.65.249
```

Test connection:

```bash
ping 192.168.65.249
```

---

## ⚠️ Safety Notice

This project controls an industrial robot.

* Ensure no person is inside robot workspace
* Always verify emergency stop functionality
* Use low speeds during testing
* Follow Yaskawa safety guidelines

---

## 🧪 Troubleshooting

| Problem                     | Possible cause                   |
| --------------------------- | -------------------------------- |
| Robot does not move         | Wrong mode (not REMOTE/PLAY)     |
| Job does not start          | JOB name incorrect               |
| No connection               | Wrong IP or network              |
| Servo turns OFF immediately | Job finished or safety condition |

---

## 🏗️ Demo Purpose

This project was developed as a **hackathon demonstrator** to showcase:

* industrial robot integration
* real-time control systems
* human-machine interfaces (HMI)
* Ethernet-based robot communication

---

## 📈 Future Improvements

* ROS integration
* Vision system (camera-based control)
* Web interface instead of desktop GUI
* AI-based motion optimization
