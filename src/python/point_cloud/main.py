from python.camera_manager.cameras.basler import Basler
from python.camera_manager.cameras.ids import Ids
from python.camera_manager.cameras.luxonis import Luxonis
from machine_vision import OptiSortVision
import cv2


mv = OptiSortVision()
#basler = Basler(camera_id="basler")
#basler.initialize()
#basler.acquisition_start()

ids = Ids(camera_id="ids")
ids.initialize()
#ids.acquisition_start()

#luxonis = Luxonis(camera_id="luxonis")
#luxonis.initialize()

try:
    while True:
        image = ids.capture_frame()
        mg_array = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        mv.get_centre(mg_array)

        if cv2.waitKey(1) == ord('q'):
            break

    ids.acquisition_stop()
except:
    ids.acquisition_stop()