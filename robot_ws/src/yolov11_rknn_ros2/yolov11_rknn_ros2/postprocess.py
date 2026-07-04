import cv2
import numpy as np


def filter_boxes(boxes, box_confidences, box_class_probs, obj_threshold):
    # Keep only predictions whose objectness * class confidence passes threshold.
    box_confidences = box_confidences.reshape(-1)
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    class_pos = np.where(class_max_score * box_confidences >= obj_threshold)
    scores = (class_max_score * box_confidences)[class_pos]
    boxes = boxes[class_pos]
    classes = classes[class_pos]
    return boxes, classes, scores


def nms_boxes(boxes, scores, nms_threshold):
    # Standard NMS to remove overlapping boxes for the same class.
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        index = order[0]
        keep.append(index)

        xx1 = np.maximum(x[index], x[order[1:]])
        yy1 = np.maximum(y[index], y[order[1:]])
        xx2 = np.minimum(x[index] + w[index], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[index] + h[index], y[order[1:]] + h[order[1:]])

        overlap_w = np.maximum(0.0, xx2 - xx1 + 1e-5)
        overlap_h = np.maximum(0.0, yy2 - yy1 + 1e-5)
        inter = overlap_w * overlap_h

        iou = inter / (areas[index] + areas[order[1:]] - inter)
        inds = np.where(iou <= nms_threshold)[0]
        order = order[inds + 1]

    return np.array(keep)


def dfl(position):
    # Decode Distribution Focal Loss outputs into distances for box regression.
    n, c, h, w = position.shape
    parts = 4
    channels = c // parts

    position = position.reshape(n, parts, channels, h, w)
    exp_pos = np.exp(position - np.max(position, axis=2, keepdims=True))
    position = exp_pos / np.sum(exp_pos, axis=2, keepdims=True)

    acc = np.arange(channels).reshape(1, 1, channels, 1, 1)
    return (position * acc).sum(2)


def box_process(position, input_size):
    # Convert feature-map outputs into xyxy boxes in resized input coordinates.
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([input_size // grid_h, input_size // grid_w]).reshape(1, 2, 1, 1)

    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    return np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)


def post_process(outputs, input_size, obj_threshold, nms_threshold):
    # This model uses three detection branches; each branch contributes box and class tensors.
    boxes, scores, classes_conf = [], [], []
    branch_count = 3
    pair_per_branch = len(outputs) // branch_count

    for branch_index in range(branch_count):
        boxes.append(box_process(outputs[pair_per_branch * branch_index], input_size))
        classes_conf.append(outputs[pair_per_branch * branch_index + 1])
        scores.append(
            np.ones_like(outputs[pair_per_branch * branch_index + 1][:, :1, :, :], dtype=np.float32)
        )

    def flatten_branch(branch_output):
        channels = branch_output.shape[1]
        branch_output = branch_output.transpose(0, 2, 3, 1)
        return branch_output.reshape(-1, channels)

    boxes = np.concatenate([flatten_branch(item) for item in boxes])
    classes_conf = np.concatenate([flatten_branch(item) for item in classes_conf])
    scores = np.concatenate([flatten_branch(item) for item in scores])

    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf, obj_threshold)
    if len(classes) == 0:
        return None, None, None

    # Run NMS independently per class.
    nboxes, nclasses, nscores = [], [], []
    for class_id in set(classes):
        inds = np.where(classes == class_id)
        class_boxes = boxes[inds]
        class_ids = classes[inds]
        class_scores = scores[inds]
        keep = nms_boxes(class_boxes, class_scores, nms_threshold)

        if len(keep) > 0:
            nboxes.append(class_boxes[keep])
            nclasses.append(class_ids[keep])
            nscores.append(class_scores[keep])

    if not nboxes:
        return None, None, None

    return np.concatenate(nboxes), np.concatenate(nclasses), np.concatenate(nscores)


def letterbox(image, new_shape=(640, 640), color=(0, 0, 0)):
    # Resize with aspect ratio preserved and pad to model input size.
    shape = image.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    scale = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    ratio = (scale, scale)
    new_unpad = (int(round(shape[1] * scale)), int(round(shape[0] * scale)))
    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))
    image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return image, ratio, (left, top)


def draw_detections(image, boxes, scores, classes, ratio, padding, class_names):
    # Map boxes from resized image space back to the original frame and draw them.
    for box, score, class_id in zip(boxes, scores, classes):
        top, left, right, bottom = box

        top = int((top - padding[0]) / ratio[0])
        left = int((left - padding[1]) / ratio[1])
        right = int((right - padding[0]) / ratio[0])
        bottom = int((bottom - padding[1]) / ratio[1])

        class_name = class_names[class_id] if class_id < len(class_names) else str(class_id)
        label = f"{class_name} {score:.2f}"

        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 255), 2)
        cv2.putText(
            image,
            label,
            (top, max(left - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return image
