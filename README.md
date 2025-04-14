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

1. Install VirtualBox on your host PC: https://www.virtualbox.org/
2. Download OptiSort VM (.vbox & -vdi file) from the SMF's OneDrive (you must request access); you will find login credentials inside the SMF's OneDrive folder
3. Move the files to a folder where they will remain permanently
4. Open Virtual Box > Add > Select .vbox file
5. Ensure your host PC has a network card/adapter set in the workstation's local network (you should probably set a static IP address)
6. In the VM settings > network, make sure "bridged adapter" is selected and set to the correct network card (host PC)
7. If you need internet access in the VM, you need to set up a NAT network (the VM can access the internet via the host PC, but the internet cannot reach the VM). Steps 8-9:
8. In VirtualBox > file > tools > Network Manager > NAT networks > add > leave defautls and take note of the NAT name
9. Select the optisort VM > settings > network > adapter 2 > NAT network > select the NAT network just created

If you encounter any troubles using the VM, try following the manual OptiSort installation (B) to try to fix eventual issues.

### Description of the VM environment
All the software required for the OptiSort system is already installed and prepared for you to use. Software types:

- Development (Visual Studio Community, Pycharm): used to write code and contribute to project's repository (you might need to login to github)
- Robotics (ACE 3.6, Flexibowl parameters): used to control Scara and Flexibowls robot
- Cameras (IDS cockpits, Basler cockpits, Luxonis packages, SDKs, ...): used to connect or parameterize cameras
- MQTT (background broker, MQTTX): needed for internal or external message exchange or development supervision

The document folder contains all the installers used for the virtual machine in case you might need to replicate installations somewhere else. On the desktop, you will find a folder called "OptiSort_project", which contains the source code (this repository cloned). Please remember to pull the last version of the right branch before you modify something.

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
7. PyCharm will detect a requirements.txt file indicating which dependancies and versions need to be installed and will ask to create a virtual environment (venv); choose this carefully considering [issue #69](https://github.com/SmartMiniFactory/opti-sort/issues/69)

<img width="455" alt="image" src="https://github.com/user-attachments/assets/d3464641-75ca-4748-91b4-16ce54cd6d8d" />

This will automatically install all the needed dependencies in a dedicated environment to avoid conflicts with eventual global installations. Some final notes:
- note the installation output: if something fails, you will need to proceed with manual installation; search for the package online and ensure you are installing the correct version
- remember to install dependancies within the venv
- remember to setup a PyCharm interpreter (venv's interpreter) or the project will not run

**Important**: most of the dependancies can be installed at higher versions, except for paho-mqtt. Higher versions of this dependency include deprecated functionalities that continuously generate errors. Make sure to install exactly version 1.6.1 or find a way to fix such issues.

**Tip**: in case the automated installation process failed installation of some dependancies, or installed at higher versions than required, use the PyCharm integrated bash to be directly located in your VENV, so reinstallation commands are already executed at the right spot:


# Development

## Maintenance

### Python requirements
If you modify dependencies, please make sure to update the requirements.txt file accordingly. You can automate this process by following the next steps. If you are using a VENV, you can copy the following command to the VENV's bash:

```
pip freeze > requirements.txt
```

Otherwise, if you are not using a VENV, the command above would track all the libraries installed in your system rather than those used in the project. Thus, you need to install the [pipreqs](https://pypi.org/project/pipreqs/0.1.4/) package and run the following command:

```
pipreqs /path/to/project
```

**IMPORTANT**: _pipreqs_ tries to infer the package names, but it might confuse hyphens (`-`) for underscores (`_`). This is an unwanted behavior when exporting the requirements.txt file because subsequent dependency installations will fail. Make sure to check every suspicious underscore and substitute it with the correct package name.

**Tip**: _pipreqs_ generates a strict dependency relationship requiring specific versions to be installed (e.g. numpy==1.0.0); you can manually modify this syntax to allow installation of higher versions if you are sure this won't generate errors (e.g. numpy>=1.0.0)


Example of wrong requirements.txt:

<img width="150" alt="image" src="https://github.com/user-attachments/assets/3a2869bd-255d-4d24-ac83-be5f987eee32" />

Example of correct requirements.txt:

<img width="150" alt="image" src="https://github.com/user-attachments/assets/ebbded26-296d-46e2-8410-b8e8c5d8f8b8" />


## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contact

For any questions or inquiries, please contact us at [dgalli@unibz.it](mailto:dgalli@unibz.it) and [dmorelato@unibz.it](mailto:dmorelato@unibz.it) 

---

Thank you for your interest in **OptiSort**! We look forward to your contributions and feedback.
