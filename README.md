# Lane Change Intention Recognition Framework

Below is a high-level overview of the workflow for the Lane Change Intention Recognition Framework:

![Workflow](pics/workflow.svg)

The map can be downloaded from [this link](http://bit.ly/4l6Rb3Q). We suggest unzipping it in the `maps` folder of the repository.

## Data Collection

The `main.py` script includes functionality for collecting data to train and evaluate the lane change intention recognition model. Below is an overview of the data collection process:

1. **Sensor Integration**: The script interfaces with vehicle sensors (e.g., steering angle, speed, turn signals) to gather real-time driving data.
2. **Data Logging**: Collected data is logged and stored in a CARLA log file for further processing.
3. **Configuration**: Parameters such as sampling rate and data storage paths can be configured in the script for flexibility.

Refer to the `main.py` file for implementation details and customization options.

### Setup Instructions

1. **Map**: Set the map path in the `config.json` file.
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
3. **Install Dependencies**: Install the required packages from `requirements.txt`:
    ```bash
    pip install -r requirements.txt
    ```

#### Running the CARLA Manual Control Client

The `main.py` script serves as the entry point for running the CARLA Manual Control Client. This script provides various functionalities for interacting with the CARLA simulator, including manual control, autopilot, and data recording. Below is an overview of its features and usage:

#### Features

1. **Configuration Loading**: The script loads a `config.json` file to customize gameplay settings, such as baseline or treatment modes.
2. **Command-Line Arguments**: A wide range of arguments can be passed to the script to control its behavior, including:
    - `--host` and `--port` for specifying the CARLA server's IP and port.
    - `--autopilot` to enable autopilot mode.
    - `--record` and `--savevideo` for recording frames and saving videos of the simulation.
    - `--baseline` to toggle between baseline and treatment modes.
3. **Synchronous Mode**: The script supports synchronous mode execution for precise control over simulation timing.
4. **Steering Wheel Simulation**: Optionally, a steering wheel controller can be activated for enhanced realism.

#### Usage

To run the script, use the following command:

```bash
python main.py [options]
```

Replace `[options]` with the desired command-line arguments. For example:

```bash
python main.py --port 2000 --wheel --baseline
```
or

```bash
python main.py --port 2000 --wheel --treatment
```

#### Output
After a successful run, the script will output the collected data to a specified log file, which can be later used for replicating the run or can be converted for training and evaluating the lane change intention recognition model.


## Data Preparation and Machine Learning

This repository contains the data preparation and machine learning pipeline for predicting lane changes in CARLA-based driving scenarios.

### Overview

Data collected from a CARLA session is first saved as a .txt log file (e.g., log_001.txt). These log files should be placed inside the logs/ folder and converted into .h5 format (e.g., log_001.h5) using the script generate_CDF_Carla_log_to_H5.py, which follows the Hi-Drive Common Data Format (CDF) specification.

Each .h5 file stores a wide range of signals from the driving session. To simulate real-world sensor availability, 30 selected features are extracted from the .h5 file. This selection and labeling is performed by the script h5_to_CSV_and_labeling.py, which also assigns labels to each timestamp:

0–4: Lane change direction

4.1: Free ride (no lane change)

"-": Ignored timestamps (up to 1.5 seconds after a lane change)

The result is a .csv file saved in the dataset/ folder, containing the selected features and labels at 10 Hz.

For machine learning, the .csv data is segmented into overlapping 5-second windows, capturing the temporal context of the driving behavior. These windows are then normalized (scaled) to ensure consistency across users. Finally, machine learning models are trained using this windowed and scaled dataset, with Bayesian optimization used to find the best hyperparameters for lane change intention prediction.

### Folders:
#### ../maps: Put the LC_Simulator folder for CARLA here. ⚠️ Important: This must match the map used during data collection.
#### h5: Generated .h5 CDF files will be stored here.
#### dataset: Processed .csv files with selected features and labels.
#### machine_learning: Contains scripts and notebooks for model training and evaluation.

### Setup

#### Installation
1. Create and activate conda environment:

        conda create -n lc_data_preparation python=3.10.14
        conda activate lc_data_preparation
2. Install required packages:
        
        pip install -r requirements.txt

#### Workflow
⚠️If you have already downloaded the dataset, you can skip directly to step 3.

0. Insert Map

    Put the LC_Simulator folder (containing the CARLA map used during data collection) into the ../maps/ directory. 
    ⚠️ Make sure this is the same map used to generate the .txt log files.

  
1. Convert .txt Logs to .h5 (CDF Format)
    
    Open generate_CDF_Carla_log_to_H5.py and specify in the first lines the log file(s) you want to convert. (the logs file are created during the data collection phase inside the data_collection/logs folder)

    Run the script. If you encounter a RuntimeError: time-out..., simply re-run it.

    The output .h5 file(s) will be saved in the h5/ folder, retaining the original filename.

2. Convert .h5 to .csv with Labels

    Open h5_to_CSV_and_labeling.py and specify the .h5 file(s) to convert.

    The script extracts 30 selected features and adds labels:

    0–4: Lane change direction

    4.1: Free ride

    "-": Ignored timestamps (up to 1.5s after a lane change)

    The resulting .csv file will be saved in the dataset/ folder.

3. Prepare Training, Validation, and Test Sets

    Inside the machine_learning/ folder, split users by folder for a user-independent training setup:

    validation/: Users 5, 8, 10, 12, 16, 19, 27

    testing/: Users 2, 7, 13, 18, 25, 31, 36

    training/: All remaining users

    This split follows the experimental setup used in the reference publication.

4. Generate 5s Windows for ML
    
    Run the CSV_to_windows_by_users_5s.ipynb notebook in the machine_learning/ folder to generate windowed datasets. The output will be saved in the data/ directory.

5. Normalize Data & Train Models
    
    Navigate to machine_learning/regression_lc_fr_5s/.

    Run the regression_lc_fr.ipynb notebook:

    This script will normalize the windowed dataset.

    It will also train machine learning models using Bayesian optimization.

    Output is saved in the data_prepared/ folder.