# Self-Driving Car Decision Planning: Windows Setup Instructions

This document provides step-by-step instructions for setting up the project environment to run the examples. For convinience in windows we are providing the instructions only for **Anaconda**. 

## Anaconda setup instructions

1. Install Visual Studio Code.
2. Install [Anaconda](https://docs.anaconda.com/free/anaconda/install/windows/).
3. Open the **Anaconda Prompt** and run the following command to initialize the command prompt:

    ```shell
    conda init cmd.exe
    ```

4. Open Anaconda and create a new environment called **decision** with Python **3.7** (IMPORTANT). For this you can use either: 
    - Anaconda Navigator
    - Anaconda Prompt and type:

    ```shell
    conda create -n decision python=3.7
    ```

5. Open Visual Studio Code in the root of this repository.
6. Open a new terminal in Visual Studio Code and activate the **decision** environment by running the following command:

    ```shell
    conda activate decision
    ```

    Note: Make sure that the terminal has successfully activated the environment. You should see the name of the environment at the beginning of the command prompt, as shown in the following picture:

    ![command_prompt](https://user-images.githubusercontent.com/27258035/234564451-b9e5dad9-926d-421c-87f7-59756e685081.png)

7. Install the required dependencies by running the following command:

    ```shell
    cd project_folder_path\
    python -m pip install -r requirements.txt
    python -m pip install PythonAPI\carla\dist\carla-0.9.10-cp37-cp37m-win_amd64.whl
    ```

## Carla Simulator Setup

1. Download the CARLA simulator [version 0.9.10 for Windows](https://github.com/carla-simulator/carla/releases/tag/0.9.10). You don't need to download the additional maps file.
2. Decompress the downloaded file into a directory of your preference.
3. Navigate to the decompressed folder, open a new terminal, and run the following command:

    ```shell
    .\CarlaUE4.exe -quality-level=Low
    ```

4. The simulator should open a window similar to the one shown in the following image:

    ![carla](https://user-images.githubusercontent.com/27258035/234566270-f40a6ee6-aff7-473c-b1bc-2cfe2d983eee.png)

## Test that everything works

1. Run the CARLA simulator with the **Low Level quality** setting (follow the instructions in the [CARLA Setup](#carla-setup) section).
2. In Visual Studio Code, open a new terminal and activate the **decision** environment by running the following command:

    ```shell
    conda activate decision
    ```

3. Navigate to the `PythonAPI/examples` folder by running the following command:

    ```shell
    cd {path_to_project}/Project/PythonAPI/examples
    ```

4. Run the `manual_control.py` script by running the following command:

    ```shell
    python manual_control.py
    ```

5. A new window with the CARLA simulator should open. In this window, you should see a vehicle that you can control using your keyboard. Press **h** for more information on how to control the simulation.

## Troubleshooting

### ModuleNotFoundError: No module named 'carla'

Ensure that you are installing Carla 0.9.10 Python API to your setup. Remember that it is important to use an environment with **Python 3.7**.

```
conda activate decision
python -m pip install {path_to_project}/Project/PythonAPI/carla/dist/carla-0.9.10-cp37-cp37m-win_amd64.whl
```

Do not install Carla with <code>python -m pip install carla </code> ass this will install the API for Carla 0.9.15, which has not been tested with this project.