import cv2
from ultralytics import YOLO

# 加载YOLO模型
model = YOLO('yolov8n.pt')  # 默认COCO模型

# 打开视频文件或摄像头，改成0表示摄像头
cap = cv2.VideoCapture('test_video.mp4')

# 获取视频参数
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps    = cap.get(cv2.CAP_PROP_FPS)

# 设置保存输出视频
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('yolo.mp4', fourcc, fps, (width, height))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 直接检测，不限制类别
    results = model(frame, device='cuda')

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())

            # 获取类别名称
            cls_name = model.names[cls]

            # 画框和类别
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{cls_name} {conf:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow('YOLO Detection', frame)
        # 保存当前帧
    out.write(frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC退出
        break

cap.release()
out.release()
cv2.destroyAllWindows()
