import ultralytics.engine
import ultralytics.engine.results
from easyGL.transform import *
from easyGL.utils import *
import ultralytics
from easyGL import get_logger
import numpy as np
import cv2
from typing import List
from collections import defaultdict

logger = get_logger(__name__)

def reproduce_3D():
    u, v = get_uv()
    rotation_mat = get_rotation()
    translation_mat = get_translation()
    inverse_E = construct_inverse_extrinsic(rotation_mat, translation_mat)
    inverse_K = construct_inverse_intrinsic()
    unproject(u, v, inverse_K, inverse_E)


        

def track(model:ultralytics.YOLO,image:np.ndarray)->ultralytics.engine.results:
    result = model.track(image)[0]
    return result


def get_target_box(result:ultralytics.engine.results,target_category:List[str],target_track_id:int): 
    boxes:ultralytics.engine.results.Boxes = result[0].boxes
    res_target_boxes = []
    res_target_id = []
    res_target_category = []

    for box in boxes:
        category = box.cls  # 对象类别
        track_id = box.id # 跟踪 ID
        bbox = box.xywh  # 边界框信息
        # 检查是否为 ID 为 2 的车辆
        if category in target_category and track_id in target_track_id:
           res_target_boxes.append(bbox)
           res_target_id.append(track_id)
           res_target_category.append(target_track_id)
    return res_target_boxes,res_target_category,res_target_id


def get_uv(boxes:List[ultralytics.engine.results.Boxes]):
    uv_list = []
    for box in boxes:
        fisrt_box = box[0]
        x = fisrt_box[0]
        y = fisrt_box[1]
        uv_list.append((int(x),int(y)))
        return  uv_list


def get_annotated_frame(result:ultralytics.engine.results.Results,u,v):
    result_temp = copy.deepcopy(result)
    label_key = str(result.boxes.cls)
    xyxy = result_temp.boxes.xyxy
    first_box = xyxy[0]
    x1, y1, x2, y2 = first_box[0], first_box[1], first_box[2], first_box[3]
    x1 = int(round(x1.item()))
    y1 = int(round(y1.item()))
    x2 = int(round(x2.item()))
    y2 = int(round(y2.item()))
    temp = cv2.rectangle(result_temp.orig_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    temp = cv2.putText(temp, label_key, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
    temp = cv2.circle(temp, (u, v), radius=20, color=(255, 0, 0), thickness=-1)  # 使用蓝色圆点标记
    return temp


    
