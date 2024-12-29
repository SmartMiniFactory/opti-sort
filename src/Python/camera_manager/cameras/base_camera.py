class BaseCamera:
    def __init__(self, camera_id, config):
        self.camera_id = camera_id
        self.config = config

    def initialize(self):
        pass

    def configure(self):
        pass

    def start_streaming(self):
        pass

    def stop_streaming(self):
        pass

    def capture_frame(self):
        pass
