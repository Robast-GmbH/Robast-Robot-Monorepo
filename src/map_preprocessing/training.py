from ultralytics import YOLO

model = YOLO('yolov8n.pt') 

results = model.train(data='coco128.yaml', epochs=100, imgsz=640)