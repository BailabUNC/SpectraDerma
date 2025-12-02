# SpectraDerma

A comprehensive photoplethysmography (PPG) analysis system for spectral dermatological applications, featuring machine learning models for physiological signal processing and analysis.

## 🔬 Overview

SpectraDerma is a research platform designed for analyzing PPG signals and spectral data in dermatological applications. The system includes data processing pipelines, machine learning models, and visualization tools for comprehensive physiological signal analysis.

## 🚀 Key Features

- **PPG Signal Processing**: Advanced preprocessing and filtering of photoplethysmography signals
- **Machine Learning Models**: PyTorch-based neural networks for signal classification and analysis
- **Spectral Analysis**: Comprehensive spectral data processing and visualization
- **Data Pipeline**: Automated data processing workflows for finger and wrist measurements
- **Visualization Tools**: Interactive notebooks for data exploration and analysis
- **Power Analysis**: Power consumption monitoring and optimization tools

## 📁 Project Structure

```
SpectraDerma/
├── notebooks/                    # Jupyter notebooks for analysis
│   ├── All_Data_Analysis.ipynb          # Comprehensive data analysis
│   ├── Blood_Spectra_Visualization.ipynb # Blood spectra visualization
│   ├── Data_Pipeline_Finger.ipynb       # Finger measurement pipeline
│   ├── Data_Pipeline_Wrist.ipynb        # Wrist measurement pipeline
│   ├── Training.ipynb                    # Model training notebook
│   ├── Power_Consumption.ipynb           # Power analysis
│   └── ...                              # Additional analysis notebooks
├── data/                         # Data storage and processing
│   ├── as7341/                          # AS7341 sensor data
│   ├── blood_alcohol_spectra/           # Blood alcohol spectral data
│   ├── blood_acetaldehyde_spectra/      # Acetaldehyde spectral data
│   ├── ecoflex/                         # Ecoflex material data
│   ├── ppg_segments/                    # PPG signal segments
│   └── power/                           # Power consumption data
├── models/                       # Trained models and configurations
│   ├── model.pth                        # Main trained model
│   └── model_info.pt                    # Model metadata
├── firmware/                     # Embedded system firmware
│   └── spectraderma/                    # SpectraDerma device firmware
├── figures/                      # Generated plots and visualizations
├── model.py                      # Neural network model definitions
├── spectraderma_processor.py     # Signal processing utilities
├── spectraderma_ppg_analyzer.py  # PPG analysis tools
└── pt_probe.py                   # PyTorch model probing utilities
```

## 🛠️ Installation

### Prerequisites

- Python 3.8+
- PyTorch
- NumPy
- Pandas
- Scikit-learn
- Matplotlib
- Jupyter Notebook

### Setup

1. Clone the repository:
```bash
git clone https://github.com/BailabUNC/SpectraDerma.git
cd SpectraDerma
```

2. Install required packages:
```bash
pip install torch torchvision numpy pandas scikit-learn matplotlib jupyter tqdm pywavelets scipy
```

3. Launch Jupyter notebooks:
```bash
jupyter notebook notebooks/
```

## 💻 Usage

### Data Processing

```python
from spectraderma_processor import SpectraDermaProcessor

# Initialize processor
processor = SpectraDermaProcessor(sample_rate=30)

# Load and process data
processor.load_data('data/your_data.csv')
processed_data = processor.process_signals()
```

### Model Training

```python
from model import SpectraDermaPPGDataset, SpectraDermaCNN

# Load dataset
dataset = SpectraDermaPPGDataset(segments_path, labels)

# Initialize and train model
model = SpectraDermaCNN()
# Training code here...
```

### PPG Analysis

```python
from spectraderma_ppg_analyzer import SpectraDermaPPGAnalyzer

# Initialize analyzer
analyzer = SpectraDermaPPGAnalyzer()

# Perform analysis
results = analyzer.analyze_ppg_data(ppg_data)
```

## 📊 Data Format

The system processes various data types:

- **PPG Signals**: Time-series photoplethysmography data
- **Spectral Data**: Multi-channel spectral measurements
- **Metadata**: Subject demographics and experimental conditions
- **Power Data**: Device power consumption measurements

## 🔍 Analysis Capabilities

### Signal Processing
- Bandpass filtering
- Wavelet denoising
- Peak detection
- Heart rate variability analysis

### Machine Learning
- CNN-based signal classification
- Feature extraction and selection
- Model evaluation and validation
- Cross-validation frameworks

### Visualization
- Interactive spectral plots
- PPG signal visualization
- Statistical analysis plots
- Power consumption monitoring

## 📈 Key Notebooks

1. **All_Data_Analysis.ipynb**: Comprehensive analysis of all collected data
2. **Training.ipynb**: Model training and evaluation pipeline
3. **Data_Pipeline_Finger.ipynb**: Finger measurement processing
4. **Data_Pipeline_Wrist.ipynb**: Wrist measurement processing
5. **Blood_Spectra_Visualization.ipynb**: Blood spectral data analysis
6. **Power_Consumption.ipynb**: Device power analysis

## 🔧 Model Architecture

The system uses convolutional neural networks optimized for PPG signal processing:

- **Input**: Multi-channel PPG segments
- **Architecture**: 1D CNN with attention mechanisms
- **Output**: Binary classification or regression targets
- **Optimization**: Adam optimizer with adaptive learning rates

## 📄 Data Sources

- AS7341 spectral sensor data
- Blood alcohol and acetaldehyde spectra
- Ecoflex material characterization
- PPG signals from finger and wrist measurements
- Power consumption monitoring data

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is part of research conducted at the University of North Carolina at Chapel Hill. Please contact the authors for licensing information.

## 👥 Authors

- Research Team at Baila Lab, UNC Chapel Hill

## 📞 Contact

For questions or collaboration opportunities, please contact the research team.

## 🔬 Research Applications

This system has been used for:
- Photoplethysmography signal analysis
- Blood alcohol detection research
- Spectral dermatological measurements
- Wearable device optimization
- Physiological signal processing research

---

*This README provides an overview of the SpectraDerma system. For detailed implementation information, please refer to the individual notebook files and code documentation.*