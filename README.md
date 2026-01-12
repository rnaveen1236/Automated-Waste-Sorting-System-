# 🗑️ Automated Waste Sorting System (MLP / SLP / SUP)

## 📌 Project Overview
The Automated Waste Sorting System is an AI-powered robotics project designed to automatically detect and segregate plastic waste into **MLP (Multi-Layered Plastic)**, **SLP (Single-Layered Plastic)**, and **SUP (Single-Use Plastic)** categories.  
The system uses a deep learning object detection model along with a robotic arm controlled by a microcontroller to perform real-time pick-and-place operations.

---

## 🎯 Objective
To reduce manual waste segregation by using **computer vision, machine learning, and robotics** for accurate and efficient plastic waste sorting.

---

## 🧠 System Architecture
- Camera captures live video of waste objects  
- AI model detects waste type and location  
- Object coordinates are processed in Python  
- Control commands are sent to Arduino  
- Robotic arm picks and places waste into the correct bin  

---

## 🛠️ Technologies Used

### 💻 Software
- Python  
- OpenCV  
- YOLO (Object Detection)  
- PyTorch  
- Roboflow (Dataset & Annotation)  

### 🤖 Hardware
- USB / Webcam  
- Arduino  
- Servo-based Robotic Arm  
- Power Supply  

## 📂 Project File Structure
```text
project-folder/
│
├── detection.py            # YOLO-based object detection script
├── best.pt                 # Trained YOLO model (MLP / SLP / SUP)
├── data.yaml               # Dataset configuration file
├── arduino_control.ino     # Arduino code for robotic arm control
├── README.dataset.txt      # Dataset information
├── README.roboflow.txt     # Roboflow project details
├── venv/                   # Python virtual environment
└── .vscode/                # VS Code configuration
``` 
## ▶️ How to Run the Project
### 1️⃣ Setup Python Environment
```bash
python -m venv venv
``` 
### Activate Virtual Environment
#### Windows
```bash
venv\Scripts\activate
``` 
###Linux / macOS
```bash
source venv/bin/activate
``` 
##Install Required Libraries

##Install all the necessary dependencies using pip:
```bash
pip install ultralytics opencv-python torch
``` 
2️⃣ Run Object Detection
```bash
python detection.py
```
This will:

Open the camera

Detect MLP, SLP, and SUP plastics

Display bounding boxes and labels

3️⃣ Upload Arduino Code

Open arduino_control.ino in Arduino IDE

Select the correct board and COM port

Upload the code

Connect servos to the robotic arm

🔁 Working Flow

Camera captures waste image

YOLO model identifies waste category

Object center is calculated

Coordinates are sent to Arduino

Robotic arm sorts waste into bins

📊 Output

Real-time waste detection on camera feed

Automatic robotic arm movement

Correct segregation of plastic types

🌍 Applications

Smart waste management systems

Plastic recycling automation

Smart city projects

Academic and industrial robotics

✅ Advantages

Reduces human effort

Improves segregation accuracy

Scalable for more waste categories

Real-time AI and robotics integration

🚀 Conclusion

This project demonstrates the effective integration of AI-based vision systems and robotic automation to solve real-world environmental problems. It serves as a strong prototype for smart plastic waste sorting and industrial automation solutions.
