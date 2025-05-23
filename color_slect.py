import cv2
import numpy as np

# 加载图像（你可以替换成自己的图像路径）
image = cv2.imread("test_pic.png")  # 替换为你的图像路径
image = cv2.resize(image, (600, 400))  # 调整大小方便显示

# 创建滑块窗口
cv2.namedWindow("Trackbars")

# 初始化滑块
def nothing(x):
    pass

# 创建滑块：Hue 取值范围为 0-179，Saturation 和 Value 为 0-255
cv2.createTrackbar("H Min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("H Max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

print("按 's' 可打印当前HSV范围，按 'ESC' 退出")

while True:
    # 获取滑块值
    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")

    # 转换为HSV图像
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 创建掩码
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)

    # 应用掩码
    result = cv2.bitwise_and(image, image, mask=mask)

    # 显示图像
    cv2.imshow("Original", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Filtered", result)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC键退出
        break
    elif key == ord('s'):  # 按下s键打印当前HSV范围
        print(f"Lower HSV: [{h_min}, {s_min}, {v_min}]")
        print(f"Upper HSV: [{h_max}, {s_max}, {v_max}]")

cv2.destroyAllWindows()
