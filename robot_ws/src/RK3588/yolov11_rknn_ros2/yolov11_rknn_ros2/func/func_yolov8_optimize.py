import cv2
import numpy as np

OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = 640
OUT_WIN = "output_style_full_screen"

CLASSES = ("pothole",)


def filter_boxes(boxes, box_confidences, box_class_probs):
    box_confidences = box_confidences.reshape(-1)

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[class_pos]

    boxes = boxes[class_pos]
    classes = classes[class_pos]
    return boxes, classes, scores


def nms_boxes(boxes, scores):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]

    return np.array(keep)


def dfl(position):
    n, c, h, w = position.shape
    p_num = 4
    mc = c // p_num
    y = position.reshape(n, p_num, mc, h, w)

    e_y = np.exp(y - np.max(y, axis=2, keepdims=True))
    y = e_y / np.sum(e_y, axis=2, keepdims=True)

    acc_metrix = np.arange(mc).reshape(1, 1, mc, 1, 1)
    y = (y * acc_metrix).sum(2)
    return y


def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE // grid_h, IMG_SIZE // grid_w]).reshape(1, 2, 1, 1)

    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)
    return xyxy


def yolov8_post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    default_branch = 3
    pair_per_branch = len(input_data) // default_branch

    for i in range(default_branch):
        boxes.append(box_process(input_data[pair_per_branch * i]))
        classes_conf.append(input_data[pair_per_branch * i + 1])
        scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    nboxes, nclasses, nscores = [], [], []
    for class_id in set(classes):
        inds = np.where(classes == class_id)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)
    return boxes, classes, scores


def draw_box_corner(draw_img, top, left, right, bottom, length, corner_color):
    cv2.line(draw_img, (top, left), (top + length, left), corner_color, thickness=3)
    cv2.line(draw_img, (top, left), (top, left + length), corner_color, thickness=3)
    cv2.line(draw_img, (right, left), (right - length, left), corner_color, thickness=3)
    cv2.line(draw_img, (right, left), (right, left + length), corner_color, thickness=3)
    cv2.line(draw_img, (top, bottom), (top + length, bottom), corner_color, thickness=3)
    cv2.line(draw_img, (top, bottom), (top, bottom - length), corner_color, thickness=3)
    cv2.line(draw_img, (right, bottom), (right - length, bottom), corner_color, thickness=3)
    cv2.line(draw_img, (right, bottom), (right, bottom - length), corner_color, thickness=3)


def draw_label_type(draw_img, top, left, label_text, label_color):
    label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 6)[0]

    if left - label_size[1] - 3 < 0:
        box_coords = (top, left + 5, top + label_size[0], left + label_size[1] + 3)
        text_pos = (top, left + label_size[0] + 3)
    else:
        box_coords = (top, left - label_size[1] - 3, top + label_size[0], left - 3)
        text_pos = (top, left - 3)

    cv2.rectangle(draw_img, box_coords[0:2], box_coords[2:4], color=label_color, thickness=-1)
    cv2.putText(draw_img, label_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), thickness=2)


def draw(image, boxes, scores, classes, ratio, padding):
    for box, score, class_id in zip(boxes, scores, classes):
        top, left, right, bottom = box

        top = int((top - padding[0]) / ratio[0])
        left = int((left - padding[1]) / ratio[1])
        right = int((right - padding[0]) / ratio[0])
        bottom = int((bottom - padding[1]) / ratio[1])

        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 255), 2)
        draw_box_corner(image, top, left, right, bottom, 15, (0, 255, 0))
        draw_label_type(image, top, left, f"{CLASSES[class_id]}", (255, 0, 255))


def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    shape = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, ratio, (left, top)


def myFunc(rknn_lite, img):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_rgb, ratio, padding = letterbox(img_rgb)
    img_rgb = np.expand_dims(img_rgb, 0)

    outputs = rknn_lite.inference(inputs=[img_rgb], data_format=["nhwc"])
    boxes, classes, scores = yolov8_post_process(outputs)

    if boxes is not None:
        draw(img, boxes, scores, classes, ratio, padding)

    return img
