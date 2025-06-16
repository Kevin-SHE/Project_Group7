from flask import Flask, jsonify, render_template, request
import requests
import urllib3
import subprocess
import os
import json

app = Flask(__name__, static_folder='static', static_url_path='')

urllib3.disable_warnings()

API_URL = "https://www.trajectoires.cn:20394/api/get_result"


@app.route('/get_result_once', methods=['GET'])
def get_result_once():
    try:
        response = requests.get(API_URL, verify=False, timeout=5)
        response.raise_for_status()
        result = response.json().get("result", "").strip()

        matched_herb = ""
        if result:
            # 加载 herbs.json 文件
            with open("herbs.json", "r", encoding="utf-8") as f:
                herb_list = json.load(f)

            # 遍历匹配
            for herb in herb_list:
                if herb in result:
                    matched_herb = herb
                    break

        return jsonify({
            "success": True,
            "result": result,
            "matched_herb": matched_herb
        })
    except requests.exceptions.RequestException as e:
        return jsonify({
            "success": False,
            "message": str(e),
            "result": "",
            "matched_herb": ""
        })

@app.route('/run_script1', methods=['GET'])
def run_script1():
    try:
        # 假设 script1.py 在 script 文件夹下（相对于当前工作目录）
        script_path = os.path.join(os.getcwd(), 'script', 'script1.py')
        completed_process = subprocess.run(
            ['python', script_path],   #linux改为python3
            capture_output=True,
            text=True,
            timeout=30
        )
        if completed_process.returncode == 0:
            return jsonify({"success": True, "output": completed_process.stdout})
        else:
            return jsonify({"success": False, "error": completed_process.stderr})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})
    
@app.route('/run_script_by_herb', methods=['POST'])
def run_script_by_herb():
    data = request.json
    herb = data.get("herb", "")

    try:
        # 加载药材列表
        with open("herbs.json", "r", encoding="utf-8") as f:
            herb_list = json.load(f)

        if herb not in herb_list:
            return jsonify({"success": False, "message": f"药材 '{herb}' 不在字典中"})

        index = herb_list.index(herb)
        script_path = f"./script/script{index + 1}.py"

        # 执行对应脚本
        result = subprocess.run(['python', script_path], capture_output=True, text=True)
        output = result.stdout + result.stderr

        return jsonify({"success": True, "output": output})

    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
