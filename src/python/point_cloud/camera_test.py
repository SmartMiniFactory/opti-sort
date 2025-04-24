from python.camera_manager.cameras.ids import Ids
from machine_vision import OptiSortVision
import cv2

camera = Ids(camera_id="ids")
camera.initialize()
camera.acquisition_start()

mv = OptiSortVision()

image = camera.capture_frame()


mg_array = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
mv.show_frame(mg_array)

camera.acquisition_stop()