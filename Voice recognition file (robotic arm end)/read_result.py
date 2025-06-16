import requests
import urllib3
import time

# 忽略 HTTPS 证书警告
urllib3.disable_warnings()

url = "https://www.trajectoires.cn:20394/api/get_result"

last_result = None  # 存储上一次的识别结果

while True:
    try:
        response = requests.get(url, verify=False)
        response.raise_for_status()
        result = response.json().get("result", "")

        if result != last_result and result.strip() != "":
            print("新识别结果为：", result)
            last_result = result

    except requests.exceptions.RequestException as e:
        print("请求失败:", e)

    time.sleep(2)  # 每2秒轮询一次

