# OptiSort: Optical Sorting Cell

Welcome to the **OptiSort** GitHub repository! This project focuses on developing an automatic optical sorting station designed for sorting of tiny objects.
It's a project developed at the Smart Mini Factory lab of the Free University of Bozen/Bolzano (Italy).

## Project Overview

**OptiSort** aims to revolutionize the sorting process for monosilicon by leveraging advanced optical technologies. This system is designed to automatically identify and items based on specific criteria such as size, shape and morhpology. The goal is to achieve seamless automation.

## Features

- **Automatic Sorting**: Fully automated system that requires minimal human intervention.
- **Optical Detection**: Utilizes high-resolution cameras and sensors to accurately identify monosilicon characteristics.
- **Real-time Processing**: Capable of processing and sorting materials in real-time, ensuring high throughput.
- **Customizable Criteria**: Allows for the customization of sorting parameters to meet specific requirements.
- **Data Logging**: Records sorting data for analysis and optimization.


# Installation

To install the OptiSort System correctly, you have two possibilities:

A. Install a prepared VM (Win10 Educational Licensed by Unibz)

B. Follow the Installation guide

## A. Install OptiSort Virtual Machine

1. Install VirtualBox: https://www.virtualbox.org/
2. Download OptiSort VM (.vdi file) from the SMF's onedrive (you will need to request access): ...
3. You will find login credentials inside the SMF's onedrive folder

### Description of the VM environment
All the software required for the OptiSort system is already installed and prepared for you to use. Software types:

- Development (Visual Studio Community, Pycharm): used to write code and contribute to project's repository (you might need to login to github)
- Robotics (ACE 3.6, Flexibowl parameters): used to control Scara and Flexibowls robot
- Cameras (IDS cockpits, Basler cockpits, Luxonis packages, SDKs, ...): used to connect or parameterize cameras
- MQTT (background broker, MQTTX): needed for internal or external message exchange or development supervision

The document folder contains all the installers used for the virtual machine in case you might need to replicate installations somewhere else. On the desktop, you will find a folder called "OptiSort_project", which contains the source code (this repository cloned). Remember to pull the last version of the right branch before modifying something.

## B. Install Optisort System Manually

### Download required software

1. [Download](https://visualstudio.microsoft.com/vs/community) and install Visual Studio Community: select **.NET development** when asked by the updater service 
2. [Download](https://www.python.org/) and install the latest stable Python Release: Mark both checkboxes "**Admin privileges**" and "**Add to PATH**" when asked by the installer; then "**Disable PATH length limit**")
3. [Download](https://www.jetbrains.com/pycharm/download/?section=window) Pycharm Community: Add it to PATH when asked by the installer
4. [Download](https://mosquitto.org/download/) and install Eclipse Mosquitto (MQTT broker)
5. [Download](https://mqttx.app/downloads) and install MQTTX (for MQTT supervision during development)
6. [Download](https://www.baslerweb.com/en/downloads/software/2012599532/?srsltid=AfmBOorplO9MAmNuyzK0a-u_12KqZsKkbDvuvwWtmepAgUatpbWpUdGQ) and install Pylon 8.1.0 (basler camera cockpit and parametrization)
7. [Download]() the ACE 3.6.zip folder, extract, and install it: **installation of dependancies will probably fail!!** If that's your case, enter the "dependencies" folder and manually launch the installers inside the folders "OPC" and "Sentinel" or whatever installation failed in your case
8. [Download]() ids-software-suite-win-full-4.94.2-64.zip file, extract, and install it: select **USB and GigE** options
3. [Download]() the ids-peak-win-standard-setup-64-2.14.0.0.zip folder, extract, and install it: install **transport layer** (required to access UI camera models)
9. Restart your PC

### Download source code

Open Visual Studio and clone (```https://github.com/SmartMiniFactory/opti-sort.git```) the OptiSort repository in a local folder of your preference: 


### Install Python Dependancies
1. Open Pycharm
3. Select "Open Project"
4. Search for the local folder where you cloned this directory, and find src>python
5. Select the python folder as project folder
6. When PyCharm asks to install git, accept the package installation
7. PyCharm will detect a requirements.txt file indicating which dependancies and versions need to be installed and will ask to create a virtual environment (venv); accept it and ensure the interpreter, requirement, and venv are correctly detected/set:

<img width="455" alt="image" src="https://github.com/user-attachments/assets/d3464641-75ca-4748-91b4-16ce54cd6d8d" />

This will automatically install all the needed dependencies in a dedicated environment to avoid conflicts with eventual global installations. Some final notes:
- take note of the installation output: if something fails, you will need to proceed with manual installation; search for the package online and ensure you are installing the correct version
- remember to install dependancies within the venv
- remember to setup a PyCharm interpreter or the project will not run



# Development

## Maintenance

### Python requirements
If you modify dependencies, please make sure to update the requirements.txt file accordingly. You can automate this process by following the next steps. If you are using a VENV, you can copy the following command to the VENV's bash:

```
pip freeze > requirements.txt
```

Otherwise, if you are not using a VENV, the command above would track all the libraries installed in your system rather than used in the project. Thus, you need to install the [pipreqs](https://pypi.org/project/pipreqs/0.1.4/) library and run the following command:

```
pipreqs /path/to/project
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contact

For any questions or inquiries, please contact us at [dgalli@unibz.it](mailto:dgalli@unibz.it) and [dmorelato@unibz.it](mailto:dmorelato@unibz.it) 

---

Thank you for your interest in **OptiSort**! We look forward to your contributions and feedback.
