## Data Collection

The `main.py` script includes functionality for collecting data to train and evaluate the lane change intention recognition model. Below is an overview of the data collection process:

1. **Sensor Integration**: The script interfaces with vehicle sensors (e.g., steering angle, speed, turn signals) to gather real-time driving data.
2. **Data Logging**: Collected data is logged and stored in a CARLA log file for further processing.
3. **Configuration**: Parameters such as sampling rate and data storage paths can be configured in the script for flexibility.

Refer to the `main.py` file for implementation details and customization options.

### Setup Instructions
### Setup Instructions

1. **Download the Map**: Download the required map from [this link](http://bit.ly/4l6Rb3Q), unzip it, and set the path to the map in the `config.json` file.
2. **Create a Virtual Environment**:
    - Using `venv`:
      ```bash
      python -m venv venv
      source venv/bin/activate  # On Windows: venv\Scripts\activate
      ```
    - Using `conda`:
      ```bash
      conda create -n lanechange python=3.8
      conda activate lanechange
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

#### Output
After a successful run, the script will output the collected data to a specified log file, which can be later used for replicating the run or can be converted for training and evaluating the lane change intention recognition model.

