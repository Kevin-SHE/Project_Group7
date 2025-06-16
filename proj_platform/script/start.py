# start.py
import subprocess
import time
import sys

def run_command(cmd, wait=True):
    print(f"\n运行命令: {cmd}")
    if wait:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        print("STDOUT:\n", result.stdout)
        print("STDERR:\n", result.stderr)
        return result.returncode
    else:
        return subprocess.Popen(cmd, shell=True)

def wait_for_node(node_name, timeout=30):
    print(f"等待 ROS 节点 {node_name} 启动...")
    for _ in range(timeout):
        result = subprocess.run("rosnode list", shell=True, capture_output=True, text=True)
        if node_name in result.stdout:
            print(f"节点 {node_name} 已就绪。")
            return True
        time.sleep(1)
    raise TimeoutError(f"等待节点 {node_name} 超时")

def wait_for_service(service_name, timeout=30):
    print(f"等待服务 {service_name} 可用...")
    for _ in range(timeout):
        result = subprocess.run("rosservice list", shell=True, capture_output=True, text=True)
        if service_name in result.stdout:
            print(f"服务 {service_name} 已就绪。")
            return True
        time.sleep(1)
    raise TimeoutError(f"等待服务 {service_name} 超时")

def main():
    if len(sys.argv) != 3:
        print("用法: python3 start_sorting.py <color> <tag>")
        sys.exit(1)

    color = sys.argv[1]
    tag = sys.argv[2]

    run_command("sudo ./stop_ros.sh")
    time.sleep(2)
    run_command("rosnode kill /usb_cam")

    bringup_process = run_command("roslaunch armpi_fpv_bringup bringup.launch", wait=False)
    print("正在启动 bringup.launch ...")
    time.sleep(5)

    wait_for_node("/usb_cam")
    wait_for_service("/object_sorting/enter")
    wait_for_service("/object_sorting/set_running")
    wait_for_service("/object_sorting/set_target")

    run_command('rosservice call /object_sorting/enter "{}"')
    time.sleep(2)

    viewer_process = run_command("python3 /home/ubuntu/armpi_fpv/src/image_viewer.py", wait=False)
    print("已启动图像显示程序")
    time.sleep(2)

    run_command('rosservice call /object_sorting/set_running "data: true"')
    run_command(f"rosservice call /object_sorting/set_target \"{{color: ['{color}'], tag: ['{tag}']}}\"")
    print("程序已启动，系统正在运行中。")

    # 保存进程 PID 以便停止脚本使用
    with open("/tmp/ros_sorting_pids.txt", "w") as f:
        f.write(f"{bringup_process.pid}\n")
        f.write(f"{viewer_process.pid}\n")

if __name__ == '__main__':
    main()
