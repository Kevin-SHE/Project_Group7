# stop_sorting.py
import subprocess
import os
import requests

def run_command(cmd):
    print(f"\n运行命令: {cmd}")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    print("STDOUT:\n", result.stdout)
    print("STDERR:\n", result.stderr)

def main():
    run_command('rosservice call /object_sorting/set_running "data: false"')
    run_command('rosservice call /object_sorting/exit "{}"')

    pid_file = "/tmp/ros_sorting_pids.txt"
    if os.path.exists(pid_file):
        with open(pid_file, "r") as f:
            pids = f.read().splitlines()
        for pid in pids:
            subprocess.run(f"kill {pid}", shell=True)
        os.remove(pid_file)
        print("后台进程已终止，服务关闭完成。")
    else:
        print("未找到进程 PID 文件，可能服务已被提前终止。")

if __name__ == '__main__':
    main()
