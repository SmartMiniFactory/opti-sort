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
2. Download OptiSort VM: ...

## B. Install Optisort System Manually

### Download required software

1. [Download](https://visualstudio.microsoft.com/vs/community) and install Visual Studio Community: select **.NET development** when asked by the updater service 
2. [Download](https://www.python.org/) and install the latest stable Python Release: Mark both checkboxes "**Admin privileges**" and "**Add to PATH**" when asked by the installer; then "**Disable PATH length limit**")
3. [Download](https://www.jetbrains.com/pycharm/download/?section=window) Pycharm Community: Add it to PATH when asked by the installer
4. [Download](https://mosquitto.org/download/) and install Eclipse Mosquitto (MQTT broker)
5. [Download](https://mqttx.app/downloads) and install MQTTX (for MQTT supervision during development)
6. [Download](https://www.baslerweb.com/en/downloads/software/2012599532/?srsltid=AfmBOorplO9MAmNuyzK0a-u_12KqZsKkbDvuvwWtmepAgUatpbWpUdGQ) and install Pylon 8.1.0 (basler camera cockpit and parametrization)
7. Restart your PC

### Download source code

Open Visual Studio and clone the OptiSort repository in a local folder of your preference: 
```https://github.com/SmartMiniFactory/opti-sort.git```

### Install Vendor-specific software

Inside the project's folder, you will find a directory called "installers", from here:
1. Copy elsewhere the ACE 3.6.zip folder, extract, and install it: **installation of dependancies will probably fail!!** To fix this, enter the "dependencies" folder and manually launch the installer inside the folders "OPC" and "Sentinel"
2. Copy elsewhere ids-software-suite-win-full-4.94.2-64.zip folder, extract, and install it: select **USB and GigE** options
3. Copy elsewhere the ids-peak-win-standard-setup-64-2.14.0.0.zip folder, extract, and install it: install **transport layer** (required to access UI camera models)



## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contact

For any questions or inquiries, please contact us at [dgalli@unibz.it](mailto:dgalli@unibz.it) and [dmorelato@unibz.it](mailto:dmorelato@unibz.it) 

---

Thank you for your interest in **OptiSort**! We look forward to your contributions and feedback.
