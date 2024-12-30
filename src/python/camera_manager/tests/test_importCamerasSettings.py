import configparser
import pathlib
from ids_peak import ids_peak  # Assuming you're using IDS Peak SDK

# Load the .ini file
script_dir = pathlib.Path(__file__).parent.resolve()
calib_file = (script_dir / "../../OptiSort/HMI/config/camera_parameters.ini").resolve()

config = configparser.ConfigParser()
config.read(calib_file, encoding='cp1250')

sensor_name = config.get('Sensor', 'Sensor')
image_width = config.get('Image size', 'Width')
image_height = config.get('Image size', 'Height')
exposure = config.get('Timing', 'Exposure')

print(f"Sensor: {sensor_name}, Width: {image_width}, Height: {image_height}, Exposure: {exposure}")


# Initialize the camera
device_manager = ids_peak.DeviceManager()
camera = device_manager.get_device(0)  # Assuming single camera setup

# Apply configuration
def apply_ini_config(camera, config):
    # Set image size
    camera.Width.set_value(int(config['Image size']['Width']))
    camera.Height.set_value(int(config['Image size']['Height']))

    # Set exposure
    camera.ExposureTime.set_value(float(config['Timing']['Exposure']))

    # Set pixel clock
    camera.PixelClock.set_value(int(config['Timing']['Pixelclock']))

    # Additional settings
    # Repeat similar mappings for other parameters

apply_ini_config(camera, config)

