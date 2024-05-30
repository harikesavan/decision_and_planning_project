# Linux Build Instructions

This document outlines the installation instructions necessary to set up the project environment to **run the examples**. We offer guidelines for using Anaconda or Docker as preferred environment management tools. Note: Carla on Linux works only with a GPU. If you want to use only a CPU, we strongly advise using the Windows instructions and running on Windows.

## Carla Simulator Setup

For this setup, we will rely on Carla's Docker images to run the simulation. This is a versatile way of running the simulator without having to compile and install the whole Carla project.

1. **Install Docker Engine:**
   
   Install Docker Engine for Linux from the [Docker website](https://docs.docker.com/engine/install/ubuntu/).

2. **Install NVIDIA Driver:**
   
   Install the latest [NVIDIA drivers](https://www.nvidia.com/download/index.aspx) for your graphics card.

3. **Install NVIDIA GPU Docker:**
   
   Install [NVIDIA Container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

4. **Run Carla:**
   
   Run the `run_carla.sh` script:

    ```
    ./run_carla.sh
    ```

   Note: The simulator will not open a new window. You will only notice some warnings in the terminal.

## Development Setup

Choose one of the following setup methods based on your preference:

- [Anaconda Setup Instructions](#anaconda-setup-instructions)
- [Devcontainer Setup Instructions](#devcontainer-setup-instructions)

### Anaconda setup instructions

For this setup, we will rely on Carla's Docker images to run the simulation. This is a versatile way of running the simulator without having to compile and install the whole Carla project.

1. **Install Anaconda:**
   
   Follow the instructions to install Anaconda from the [official Anaconda documentation](https://docs.anaconda.com/free/anaconda/install/linux/).

   ```
   conda init
   ```

2. **Create a New Environment:**

    Create a new environment named decision with Python **3.7** **(VERY IMPORTANT)**:

    ```
    conda create -n decision python=3.7
    ```

3. **Activate the New Environment**
   
    ```
    conda activate decision
    ```

4. **Navigate to the Project Root and install requiremements**
   

    ```
    cd path_to_project/
    python -m pip install ./Project/PythonAPI/carla/dist/carla-0.9.10-cp37-cp37m-manylinux_2_27_x86_64.whl # Installs Carla Python API
    python -m pip install -r requirements.txt # Installs python dependencies
    ```

### Devcontainer setup instructions

For this setup, we will rely on Carla's Docker images to run the simulation. This is a versatile way of running the simulator without having to compile and install the whole Carla project.

1. **Install Visual Studio Code:**

    Download and install Visual Studio Code from the [official site](https://code.visualstudio.com/).

2. **Install VSCode Extensions**

    Install the following extensions in Visual Studio Code:

    - Remote SSH
    - Dev Container

3. **Prepare Visual Studio Code:**

    - Open Visual Studio Code.
    - Open your project folder using the shortcut `Ctrl+K Ctrl+O` (ensure - you select the root directory of the project).
    
4. **Reopen in Container:**

    - Open the Command Palette in VSCode (lower left blue button).
    - Choose **Reopen in Container** to restart VSCode in the container environment.

5. **Verify X Display Forwarding:**

    - Open a new terminal in VSCode and type `xclock` to ensure the display is being forwarded correctly.
    - If the terminal `shows bash: xclock: command not found`, install xclock:

        ```
        sudo apt install x11-apps
        ```

## Testing the Setup

We have provided several examples to verify if the simulator and the setup are working correctly.

1. **Run the CARLA Simulator:**
    
    Open a new terminal and run:

    ```
    cd path_to_project\
    ./run_carla.sh
    ```

2. **Run the Environment:**

    - If using the Devcontainer setup, run the project devcontainer in Visual Studio Code.
    - If using Anaconda, open a new terminal and activate the decision environment:
    
        ```
        conda activate decision
        ```

3. **Navigate to the Carla Examples Folder:**

    Navigate to the directory filled with [Carla examples scripts](../Project/PythonAPI/examples/)
    
    ```
    cd project_folder/Project/PythonAPI/examples
    ```
4. **Run the manual control example**

    Carla provides a [manual control script](../Project/PythonAPI/examples/manual_control.py) that allows users to run a car in a 3D simulated environment, run:

    ```
    python manual_control.py
    ```

## Troubleshooting

### ModuleNotFoundError: No module named 'carla'

Ensure that you are installing Carla 0.9.10 Python API to your setup. Remember that it is important to use an environment with **Python 3.7**.

```
conda activate decision
python -m pip install {path_to_project}/Project/PythonAPI/carla/dist/carla-0.9.10-cp37-cp37m-manylinux_2_27_x86_64.whl 
```

Do not install Carla with <code>python -m pip install carla </code> ass this will install the API for Carla 0.9.15, which has not been tested with this project.

### ERROR: carla-0.9.10-cp37-cp37m-manylinux_2_27_x86_64.whl is not a supported wheel on this platform.

This error occurs when trying to install the API with a different Python version from 3.7. Ensure you are running Python 3.7 and that the pip version is linked to your environment.

### ImportError: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0

This error likely occurs under Ubuntu 20.04. Solve this problem by deleting libffi-7 from your environment:

```
conda activate decision
rm $CONDA_PREFIX/lib/libffi.7.so $CONDA_PREFIX/lib/libffi.so.7 
```

### ImportError: <PATH_TO_ENVIRONMENT>/bin/../lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found

This error indicates that you need the latest GCC in Anaconda. Run:

```
conda activate decision
conda install -c conda-forge gcc=12.1.0
```





