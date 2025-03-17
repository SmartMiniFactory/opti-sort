class BaseCamera:
    def __init__(self, camera_id):
        self.camera_id = camera_id

    def initialize(self):
        pass

    def configure(self, config_path):
        pass

    def acquisition_start(self):
        pass

    def acquisition_stop(self):
        pass

    def capture_frame(self):
        pass
