import cv2
import numpy as np
from ultralytics import YOLO

# 加载模型（你可以改为自己训练的纸箱模型）
model = YOLO('yolov8n.pt')

# 视频输入（也可以用摄像头：cv2.VideoCapture(0)）
cap = cv2.VideoCapture('test_video.mp4')

# 获取视频参数
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

# 设置保存输出视频
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output_video.mp4', fourcc, fps, (width, height))

# 设定颜色范围（根据纸箱颜色调整）
lower_hsv = np.array([174, 0, 153])  # 示例：棕黄色
upper_hsv = np.array([179, 97, 201])

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO 检测
    results = model(frame, classes=[i for i in range(1, model.model.model[-1].nc)], device='cuda')


    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            # 获取识别框坐标
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            roi = frame[y1:y2, x1:x2]

            # 转 HSV 并掩码
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, lower_hsv, upper_hsv)

            # 查找颜色区域轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            tracked = False
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if ((area > 2000) and (area < 4000)):  # 最小面积过滤
                    tracked = True
                    x, y, w, h = cv2.boundingRect(cnt)

                    # 框出颜色区域
                    cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 0, 255), 2)

                    # 计算颜色区域中心
                    cx = x1 + x + w // 2
                    cy = y1 + y + h // 2
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(frame, f'Box Pos: ({cx},{cy})', (cx, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # 框出YOLO检测区域
            if tracked:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(frame, 'Box Tracked', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # 显示当前帧
    cv2.imshow('Tracking', frame)

    # 保存当前帧
    out.write(frame)

    # 按 ESC 退出
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 释放资源
cap.release()
out.release()
cv2.destroyAllWindows()
