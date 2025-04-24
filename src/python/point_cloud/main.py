from python.camera_manager.cameras.basler import Basler
from machine_vision import OptiSortVision
import cv2


mv = OptiSortVision()
camera = Basler(camera_id="basler")
camera.initialize()
camera.acquisition_start()

while True:
    image = camera.capture_frame()
    mv.get_flexibowl_centre(image)

    if cv2.waitKey(1) == ord('q'):
        break

camera.acquisition_stop()