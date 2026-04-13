import cv2
import numpy as np
from typing import Optional

class CameraInterface:
    def __init__(self, use_zed=False, camera_index=0, width=1280, height=720, fps=60):
        self.use_zed, self.camera_index, self.width, self.height, self.fps = use_zed, camera_index, width, height, fps
        self.cap, self.zed, self.sl = None, None, None
        self.camera_matrix, self.dist_coeffs = None, None

        if self.use_zed: self._open_zed()
        else: self._open_standard()

    def _open_zed(self):
        import pyzed.sl as sl
        self.sl, self.zed = sl, sl.Camera()
        params = sl.InitParameters()
        params.camera_resolution, params.camera_fps, params.depth_mode = sl.RESOLUTION.HD1080, self.fps, sl.DEPTH_MODE.NONE
        if self.zed.open(params) != sl.ERROR_CODE.SUCCESS: raise RuntimeError("ZED Open Failed")
        calib = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.camera_matrix = np.array([[calib.fx, 0, calib.cx], [0, calib.fy, calib.cy], [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.array(calib.disto, dtype=np.float64).flatten()[:5].reshape(-1, 1)

    def _open_standard(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def load_standard_calibration(self, yaml_path):
        if yaml_path:
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            self.camera_matrix, self.dist_coeffs = fs.getNode("K").mat(), fs.getNode("D").mat()
            fs.release()

    def get_frame(self):
        if self.use_zed:
            image = self.sl.Mat()
            if self.zed.grab() == self.sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(image, self.sl.VIEW.LEFT)
                frame = image.get_data()
                return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) if frame.shape[-1] == 4 else frame
            return None
        ret, frame = self.cap.read()
        return frame if ret else None

    def close(self):
        if self.zed: self.zed.close()
        if self.cap: self.cap.release()
        cv2.destroyAllWindows()