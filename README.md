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
│   ├── All_Data_Analysis.ipynb          # Basic data initialization and setup
│   ├── Blood_Spectra_Visualization.ipynb # Blood alcohol spectra analysis and plotting
│   ├── Data_Pipeline_Finger.ipynb       # Comprehensive finger PPG data processing
│   ├── Data_Pipeline_Wrist.ipynb        # Wrist PPG measurement analysis pipeline
│   ├── Training.ipynb                    # GRU-based PPG classifier training (520S2 dataset)
│   ├── Power_Consumption.ipynb           # Device power consumption analysis
│   ├── Metadata_Stat.ipynb              # Study design and participant statistics
│   ├── Ecoflex_Comprehensive.ipynb      # Ecoflex material color normalization
│   ├── Ecoflex.ipynb                    # Basic Ecoflex analysis
│   ├── Sensor_Spectra_VS_Chromophore.ipynb # Sensor response vs chromophore comparison
│   └── Short_Alcohol_Data.ipynb         # Abbreviated alcohol data visualization
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
from model import SpectraDermaPPGDataset, SpectraDermaGRU

# Load PPG dataset for binary classification
dataset = SpectraDermaPPGDataset(segments_path, labels)

# Initialize GRU-based model for alcohol detection
model = SpectraDermaGRU()
# Training pipeline in Training.ipynb
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

## 📈 Key Notebooks

1. **Training.ipynb**: GRU-based binary classifier for alcohol detection using 520S2 PPG segments
2. **Data_Pipeline_Finger.ipynb**: Complete finger PPG data processing and analysis workflow
3. **Data_Pipeline_Wrist.ipynb**: Wrist measurement data processing and visualization pipeline  
4. **Blood_Spectra_Visualization.ipynb**: Blood alcohol concentration spectral analysis and plotting
5. **Metadata_Stat.ipynb**: Study participant demographics and experimental design statistics
6. **Ecoflex_Comprehensive.ipynb**: Ecoflex material optical property normalization from photos
7. **Power_Consumption.ipynb**: Device power usage monitoring and analysis
8. **Sensor_Spectra_VS_Chromophore.ipynb**: Comparison of sensor spectral response with chromophore absorption

## 🔧 Model Architecture

The system uses GRU-based neural networks optimized for PPG signal classification:

- **Input**: PPG segment sequences from 520S2 dataset
- **Architecture**: GRU (Gated Recurrent Unit) with fully connected layers
- **Output**: Binary classification (normal vs. alcohol conditions)
- **Training**: Binary cross-entropy loss with Adam optimizer
- **Application**: Alcohol detection from physiological signals

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

- Yihan Liu et al

## 📞 Contact

For questions or collaboration opportunities, please contact the research team.

## 🔬 Research Applications

This system has been used for:
- **Blood Alcohol Detection**: PPG-based classification of alcohol consumption states
- **Spectral Analysis**: Blood alcohol and acetaldehyde concentration measurements
- **Wearable PPG Monitoring**: Finger and wrist-based physiological signal acquisition
- **Material Characterization**: Ecoflex optical property analysis for device design
- **Power Optimization**: Device energy consumption monitoring and analysis
- **Sensor Validation**: AS7341 spectral sensor response characterization

---

*This README provides an overview of the SpectraDerma system. For detailed implementation information, please refer to the individual notebook files and code documentation.*