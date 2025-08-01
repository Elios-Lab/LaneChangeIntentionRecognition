# Lane Change Intention Recognition Framework
<div align="center">
    <a href="https://elios-lab.github.io/LC-Intention-Framework"><img src="https://img.shields.io/badge/Project%20Page-0E472C?logo=GitHub" alt="Project Page" style="width: 120px;"></a>
    <a href="https://doi.org/10.5281/zenodo.16686054"><img src="https://zenodo.org/badge/DOI/10.5281/zenodo.16686054.svg" alt="DOI"></a>
</div>
## Overview

This framework introduces: (i) a novel public dataset of human-performed simulated lane change maneuvers; (ii) a dedicated CARLA highway map designed for extended driving sessions; and (iii) tools to facilitate data collection and model evaluation. Below is a high-level overview of the workflow:

![Workflow](pics/workflow.svg)

The map can be downloaded from [this link](http://bit.ly/4l6Rb3Q). Unzip it into the `maps` folder of the repository.

---

## Data Collection

The `main.py` script facilitates data collection for training and evaluating the lane change intention recognition model.

### Process

1. **Sensor Integration**: Interfaces with vehicle sensors (e.g., steering angle, speed, turn signals) to gather real-time driving data.
2. **Environment Data Collection**: Collects data from the CARLA simulator, including vehicle dynamics and environmental conditions.
3. **Data Logging**: Saves the collected data into structured log files for later analysis.
4. **Configuration**: Customize parameters like sampling rate and storage paths in the script.

Refer to `main.py` for implementation details.

### Setup Instructions

1. **Map Configuration**: Set the map path in `config.json`.
2. **Create a Virtual Environment**:
    - Using `venv`:
      ```bash
      python -m venv venv
      source venv/bin/activate  # On Windows: venv\Scripts\activate
      ```
    - Using `conda`:
      ```bash
      conda create -n LC-data-collection python=3.8
      conda activate LC-data-collection
      ```
3. **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

### Running the CARLA Manual Control Client

The `main.py` script serves as the entry point for interacting with the CARLA simulator.

#### Features

- **Configuration Loading**: The script loads a `config.json` file to customize gameplay settings, such as baseline or treatment modes.
- **Command-Line Arguments**:
    - `--host`, `--port`: Specify CARLA server IP and port.
    - `--autopilot`: Enable autopilot mode.
    - `--record`, `--savevideo`: Record frames and save simulation videos.
    - `--baseline`: Use baseline mode.
    - `--treatment`: Use treatment mode.
- **Synchronous Mode**: Ensures precise simulation timing.
- **Steering Wheel Simulation**: Optionally activate a steering wheel controller.

#### Usage

Run the script with desired options:

```bash
python main.py [options]
```

Examples:

```bash
python main.py --port 2000 --wheel --baseline
python main.py --port 2000 --wheel --treatment
```

#### Output

Collected data is saved to a log file for replication or conversion into training and evaluation datasets.

---

## Data Preparation and Machine Learning

This repository includes the pipeline for preparing data and training models to predict lane changes.

### Workflow Overview

1. **Log Conversion**: Convert `.txt` logs to `.h5` (CDF format) using `generate_CDF_Carla_log_to_H5.py`.
2. **Feature Extraction**: Extract 30 features and label timestamps using `h5_to_CSV_and_labeling.py`:
    - **Labels**:
        - `0–4`: Lane change direction.
        - `4.1`: Free ride (no lane change).
        - `"-"`: Ignored timestamps (up to 1.5 seconds after a lane change).
3. **Windowing**: Segment data into overlapping 5-second windows at 10 Hz.
4. **Normalization**: Scale data for consistency.
5. **Model Training**: Train models using Bayesian optimization.

### Folder Structure

- **`../maps`**: Place the `LC_Simulator` folder here. ⚠️ Ensure it matches the map used during data collection.
- **`h5/`**: Stores generated `.h5` files.
- **`dataset/`**: Contains processed `.csv` files with features and labels.
- **`machine_learning/`**: Scripts and notebooks for model training and evaluation.

---

### Setup

#### Installation

1. Create and activate a conda environment:
    ```bash
    conda create -n lc_data_preparation python=3.10.14
    conda activate lc_data_preparation
    ```
2. Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```

---

### Detailed Workflow

#### Step 0: Insert Map

Place the `LC_Simulator` folder into the `../maps/` directory. ⚠️ Ensure it matches the map used to generate `.txt` logs.

#### Step 1: Convert Logs to `.h5`

- Open `generate_CDF_Carla_log_to_H5.py` and specify the log files.
- Run the script. If a `RuntimeError: time-out...` occurs, re-run it.
- Output `.h5` files are saved in the `h5/` folder.

#### Step 2: Convert `.h5` to `.csv`

- Open `h5_to_CSV_and_labeling.py` and specify the `.h5` files.
- Extract features and label timestamps:
    - `0–4`: Lane change direction.
    - `4.1`: Free ride.
    - `"-"`: Ignored timestamps.
- Output `.csv` files are saved in the `dataset/` folder.

#### Step 3: Prepare Training, Validation, and Test Sets

- Split users into folders for user-independent training:
    - **Validation**: Users 5, 8, 10, 12, 16, 19, 27.
    - **Testing**: Users 2, 7, 13, 18, 25, 31, 36.
    - **Training**: Remaining users.

#### Step 4: Generate 5-Second Windows

Run `CSV_to_windows_by_users_5s.ipynb` in the `machine_learning/` folder. Output is saved in the `data/` directory.

#### Step 5: Normalize Data & Train Models

- Navigate to `machine_learning/regression_lc_fr_5s/`.
- Run `regression_lc_fr.ipynb`:
    - Normalize the dataset.
    - Train models using Bayesian optimization.
    - Output is saved in the `data_prepared/` folder.

--- 

Enjoy using the Lane Change Intention Recognition Framework!
