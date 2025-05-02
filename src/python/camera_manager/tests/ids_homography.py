import cv2
import numpy as np
from ids_peak import ids_peak
from ids_peak_ipl import ids_peak_ipl
from numpy.matlib import empty


# ----------- IDS Camera Wrapper (light version for simplicity) ------------

class IdsCamera:
    def __init__(self):
        ids_peak.Library.Initialize()
        self.device_manager = ids_peak.DeviceManager.Instance()
        self.device = None
        self.node_map = None
        self.data_stream = None

    def initialize(self):
        self.device_manager.Update()
        if self.device_manager.Devices().empty():
            raise RuntimeError("No IDS camera found")

        for dev in self.device_manager.Devices():
            if dev.IsOpenable():
                self.device = dev.OpenDevice(ids_peak.DeviceAccessType_Control)
                self.node_map = self.device.RemoteDevice().NodeMaps()[0]
                break
        if self.device is None:
            raise RuntimeError("No openable IDS camera found")

        self.data_stream = self.device.DataStreams()[0].OpenDataStream()

    def start_acquisition(self):
        self.data_stream.Flush(ids_peak.DataStreamFlushMode_DiscardAll)

        for buffer in self.data_stream.AnnouncedBuffers():
            self.data_stream.RevokeBuffer(buffer)

        payload_size = self.node_map.FindNode("PayloadSize").Value()
        num_buffers = self.data_stream.NumBuffersAnnouncedMinRequired()

        for _ in range(num_buffers):
            buffer = self.data_stream.AllocAndAnnounceBuffer(payload_size)
            self.data_stream.QueueBuffer(buffer)

        self.data_stream.StartAcquisition(ids_peak.AcquisitionStartMode_Default,
                                          ids_peak.DataStream.INFINITE_NUMBER)
        self.node_map.FindNode("TLParamsLocked").SetValue(1)
        self.node_map.FindNode("AcquisitionStart").Execute()

    def capture_frame(self):
        buffer = self.data_stream.WaitForFinishedBuffer(5000)
        if buffer is None:
            return None

        image = ids_peak_ipl.Image.CreateFromSizeAndBuffer(
            buffer.PixelFormat(),
            buffer.BasePtr(),
            buffer.Size(),
            buffer.Width(),
            buffer.Height()
        )

        img_array = image.get_numpy().astype(np.uint8)
        self.data_stream.QueueBuffer(buffer)

        return img_array

    def stop_acquisition(self):
        self.node_map.FindNode("AcquisitionStop").Execute()
        self.node_map.FindNode("TLParamsLocked").SetValue(0)
        self.data_stream.StopAcquisition()
        self.device.Close()

# ----------- Calibration -----------

import cv2
import numpy as np

def calibrate_camera(img, grid_size, square_size_mm):
    """ Calibrate camera with a checkerboard grid """
    pattern_size = grid_size  # (cols, rows)
    objp = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)
    objp *= square_size_mm

    # Find corners
    ret, corners = cv2.findChessboardCorners(img, pattern_size)
    if not ret:
        raise ValueError("Chessboard corners not found")

    # Refine corners
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

    # img_pts = detected corners (pixels)
    # obj_pts = real world coords (mm)
    img_pts = corners2.reshape(-1,2)
    obj_pts = objp[:,:2]

    # Compute homography
    H_cam_to_grid, status = cv2.findHomography(img_pts, obj_pts)
    return H_cam_to_grid

def compute_grid_to_scara_transform(grid_pts, scara_pts):
    """ Compute affine transform from grid coords to scara_plane coords """
    if len(grid_pts) != 3 or len(scara_pts) != 3:
        raise ValueError("Need exactly 3 point correspondences")

    grid_pts_np = np.array(grid_pts, np.float32)
    scara_pts_np = np.array(scara_pts, np.float32)

    T_grid_to_scara = cv2.getAffineTransform(grid_pts_np, scara_pts_np)
    return T_grid_to_scara

def combine_transforms(H_cam_to_grid, T_grid_to_scara):
    """ Combine homography and affine transform into one 3x3 matrix """
    T_affine_3x3 = np.vstack([T_grid_to_scara, [0,0,1]])
    H_cam_to_scara = T_affine_3x3 @ H_cam_to_grid
    return H_cam_to_scara

def pixel_to_scara(H_cam_to_scara, pixel_point):
    """ Transform pixel coordinate to scara_plane coordinate """
    pt_pixel = np.array([[pixel_point]], dtype=np.float32)
    pt_scara = cv2.perspectiveTransform(pt_pixel, H_cam_to_scara)
    return tuple(pt_scara[0][0])

# Example usage
if __name__ == "__main__":
    # Parameters (example)
    grid_size = (7, 5)           # 7 cols, 5 rows inner corners
    square_size_mm = 10.0        # each square = 10mm

    # Load calibration image
    img = cv2.imread("calibration_image.jpg")

    # Step 1 - Calibrate camera
    H_cam_to_grid = calibrate_camera(img, grid_size, square_size_mm)

    # Step 2 - Define 3 corresponding points grid <-> scara
    # These values are just examples. You measure them!
    grid_pts = [
        (0, 0),        # grid origin (mm)
        (60, 0),       # 6 squares right
        (0, 40)        # 4 squares down
    ]

    scara_pts = [
        (100.0, 200.0),
        (160.0, 200.0),
        (100.0, 240.0)
    ]

    # Step 3 - Compute grid to scara transform
    T_grid_to_scara = compute_grid_to_scara_transform(grid_pts, scara_pts)

    # Step 4 - Combine transforms
    H_cam_to_scara = combine_transforms(H_cam_to_grid, T_grid_to_scara)

    # Step 5 - Detect an object (example pixel coordinate)
    detected_pixel = (520, 380)

    # Transform pixel → scara_plane
    X_scara, Y_scara = pixel_to_scara(H_cam_to_scara, detected_pixel)

    print(f"Object at pixel {detected_pixel} → Scara coords: ({X_scara:.2f}, {Y_scara:.2f}) mm")
