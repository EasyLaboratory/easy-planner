from easyGL.airsim_gl import *
from ultralytics import YOLO
from pathlib import Path
import cv2

current_dir = Path(__file__).resolve().parent
project_root = current_dir.parent
model_base_path = project_root/"model"
model_path = model_base_path/"yolov8n_v1.pt"
image_path = current_dir/"car.webp"


model = YOLO(model_path)
results = track(model,image_path)
boxes,category,id = get_target_box(results,[7],[1])
u,v = get_uv(boxes)
annotated_frame =  get_annotated_frame(results,u,v)
cv2.imshow("test",annotated_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()





