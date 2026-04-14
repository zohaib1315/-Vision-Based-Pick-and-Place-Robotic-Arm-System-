# -Vision-Based-Pick-and-Place-Robotic-Arm-System-
This project presents a real-time vision-guided robotic manipulation system built using MATLAB and Simulink. The system detects objects using a camera, computes their real-world position, and autonomously performs pick-and-place operations.
---

## 🎯 Objectives
- Detect objects using computer vision techniques
- Convert image coordinates to real-world positions
- Control a robotic arm using inverse kinematics
- Execute autonomous pick-and-place operations

---

## 🧠 System Architecture

Camera → Image Processing → Coordinate Transformation → Inverse Kinematics → Simulink Control → QArm

---

## 🔍 Key Features

### 📷 Vision System
- HSV color-based object detection
- Noise removal using morphological operations
- Centroid extraction for object localization

### 🌍 Coordinate Mapping
- Camera calibration for accurate pixel-to-world conversion
- Real-time position estimation

### 🤖 Robotics Control
- Inverse kinematics for QArm positioning
- State machine for pick-and-place sequencing:
  - Move above object
  - Descend
  - Grasp
  - Lift
  - Place

### ⚙️ Simulink Integration
- MATLAB Function blocks for logic implementation
- Real-time execution using QUARC
- Smooth trajectory control

---

## 🛠️ Technologies Used

- MATLAB
- Simulink
- Computer Vision Toolbox
- Robotics System Toolbox
- Quanser QUARC
- Quanser QArm Hardware

---

## ▶️ How to Run the Project

The project is divided into three sequential MATLAB stages. Each stage must be executed in order for the system to function correctly.

### 📂 Files
- `stage1.m` — Vision Processing (Object Detection)
- `stage2.m` — Coordinate Transformation (Pixel → World)
- `stage3.m` — Control & Pick-and-Place Execution (Simulink + QArm)

---

### 🔄 Execution Order

1. **Run Stage 1**
   - Captures camera input
   - Detects object using image processing
   - Outputs object pixel coordinates

2. **Run Stage 2**
   - Converts pixel coordinates into real-world coordinates
   - Uses camera calibration parameters

3. **Run Stage 3**
   - Performs inverse kinematics
   - Executes pick-and-place using Quanser QArm
   - Runs Simulink control model

---

### ⚠️ Important Notes
- All stages must be run **in sequence (Stage 1 → Stage 2 → Stage 3)**
- Do not skip any stage
- Ensure camera is connected before running Stage 1
- Ensure Quanser QArm hardware and QUARC are properly configured before Stage 3

---

### 💡 Tip
For best results:
- Keep all files in the same working directory
- Run MATLAB as administrator (if using QUARC)
- Use a fixed-step solver in Simulink for stable real-time execution

## 📈 Results
- Successfully achieved real-time object detection and tracking
- Accurate object localization using calibrated camera
- Smooth and stable robotic arm motion
- Fully autonomous pick-and-place task execution

---

## ⚠️ Challenges & Solutions

| Challenge | Solution |
|----------|---------|
| Noisy image detection | Applied morphological filtering |
| Coordinate mismatch | Performed camera calibration |
| Jerky arm motion | Implemented trajectory smoothing |
| Hardware timing issues | Used fixed-step solver in Simulink |

---

## 🚀 Future Improvements
- Multi-object detection and sorting
- Deep learning-based object recognition
- Path optimization algorithms
- ROS integration

---

## 👨‍💻 Author
**Zohaib Ahmed Mughal**  
Electrical Engineering Student  
Machine Learning • Signal Processing • AI Systems  

GitHub: https://github.com/zohaib1315
