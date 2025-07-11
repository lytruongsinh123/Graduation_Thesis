# 🎓 Graduation Thesis: Audio Classification on Edge Devices

Welcome to the project for deploying an **Audio Classification Model** on STM32 edge devices! This repository contains all the code, models, and documentation for building, training, and deploying a deep learning model for audio event recognition using Mel-spectrogram features.

---

## 📁 Project Structure

```
CodeAI/
  Audio_Classification_Melspectrogram/   # Python code, data, and models
  Implement-audio-model-on-edge-device-STM32-master/   # STM32 firmware and deployment
```

### Main Folders

-   **Audio_Classification_Melspectrogram/**: Data preprocessing, feature extraction, model training, and evaluation in Python.
-   **Implement-audio-model-on-edge-device-STM32-master/**: STM32CubeIDE project for deploying the trained model on STM32F7 microcontroller.

---

## 🚀 Quick Start

### 1. Data Preparation & Model Training (Python)

1. Place your audio files in the `data/` directory.
2. Run the Jupyter notebooks and scripts in `Audio_Classification_Melspectrogram/` to preprocess data and train the model.
3. The trained model will be saved as `.h5` in the `models/` folder.

### 2. Model Deployment (STM32)

1. Convert the trained model to a C array using STM32Cube.AI or X-CUBE-AI tools.
2. Import the model into the STM32 project under `Implement-audio-model-on-edge-device-STM32-master/X-CUBE-AI/App/`.
3. Build and flash the firmware to your STM32F7 board.

---

## 🛠️ Key Technologies

-   Python, Keras/TensorFlow, NumPy, Librosa
-   STM32CubeIDE, X-CUBE-AI
-   STM32F7 Series Microcontroller

---

## 📊 Results

Include your model accuracy, confusion matrix, and sample predictions here.

---

## 🤝 Contributors

-   [Your Name]
-   [Other Contributors]

---

## 📄 License

This project is licensed under the MIT License.
