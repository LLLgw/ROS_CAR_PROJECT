import cv2
import numpy as np
from rknnlite.api import RKNNLite

from .postprocess import draw_detections, letterbox, post_process


# Map string parameters from ROS 2 into RKNN runtime core selections.
CORE_MASK_MAP = {
    "0": RKNNLite.NPU_CORE_0,
    "1": RKNNLite.NPU_CORE_1,
    "2": RKNNLite.NPU_CORE_2,
    "0_1_2": RKNNLite.NPU_CORE_0_1_2,
    "auto": None,
}


class RKNNDetector:
    def __init__(self, model_path, input_size, obj_threshold, nms_threshold, class_names, core_mask):
        self.model_path = model_path
        self.input_size = input_size
        self.obj_threshold = obj_threshold
        self.nms_threshold = nms_threshold
        self.class_names = class_names
        self.rknn = RKNNLite()

        # Load the converted YOLOv11n RKNN model from disk.
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"load_rknn failed: {model_path}")

        # Initialize RKNN runtime. "auto" falls back to default scheduling.
        mask = CORE_MASK_MAP.get(core_mask)
        if mask is None:
            ret = self.rknn.init_runtime()
        else:
            ret = self.rknn.init_runtime(core_mask=mask)
        if ret != 0:
            raise RuntimeError(f"init_runtime failed for core_mask={core_mask}")

    def infer(self, frame):
        # RKNN model expects RGB NHWC input after letterbox resize.
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized, ratio, padding = letterbox(rgb, (self.input_size, self.input_size))
        resized = np.expand_dims(resized, 0)

        outputs = self.rknn.inference(inputs=[resized], data_format=["nhwc"])
        # Decode raw RKNN outputs into boxes/classes/scores.
        boxes, classes, scores = post_process(
            outputs,
            input_size=self.input_size,
            obj_threshold=self.obj_threshold,
            nms_threshold=self.nms_threshold,
        )

        annotated = frame.copy()
        if boxes is not None:
            # Draw the detections back onto the original image coordinate space.
            annotated = draw_detections(
                annotated,
                boxes,
                scores,
                classes,
                ratio,
                padding,
                self.class_names,
            )

        return annotated, boxes, classes, scores

    def release(self):
        self.rknn.release()
