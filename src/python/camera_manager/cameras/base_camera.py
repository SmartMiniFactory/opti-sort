class BaseCamera:
    def __init__(self, camera_id, config_path):
        self.camera_id = camera_id
        self.config_path = config_path

    def initialize(self):
        pass

    def configure(self):
        pass

    def acquisition_start(self):
        pass

    def acquisition_stop(self):
        pass

    def capture_frame(self):
        pass
