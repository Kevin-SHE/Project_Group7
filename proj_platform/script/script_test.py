import subprocess
import time

def run_command(cmd, wait=True):
    print(f"\n运行命令: {cmd}")
    if wait:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        print("STDOUT:\n", result.stdout)
        print("STDERR:\n", result.stderr)
        return result.returncode
    else:
        process = subprocess.Popen(cmd, shell=True)
        return process

def wait_for_node(node_name, timeout=30):
    print(f"等待 ROS 节点 {node_name} 启动...")
    for _ in range(timeout):
        result = subprocess.run(
            "rosnode list",
            shell=True, capture_output=True, text=True
        )
        if node_name in result.stdout:
            print(f"节点 {node_name} 已就绪。")
            return True
        time.sleep(1)
    raise TimeoutError(f"等待节点 {node_name} 超时")

def wait_for_service(service_name, timeout=30):
    print(f"等待服务 {service_name} 可用...")
    for _ in range(timeout):
        result = subprocess.run(
            "rosservice list",
            shell=True, capture_output=True, text=True
        )
        if service_name in result.stdout:
            print(f"服务 {service_name} 已就绪。")
            return True
        time.sleep(1)
    raise TimeoutError(f"等待服务 {service_name} 超时")

def main():
    # 停止手机APP自启服务
    run_command("sudo ./stop_ros.sh")
    time.sleep(2)

    # 杀掉可能残留节点
    run_command("rosnode kill /usb_cam", wait=True)

    # 启动 bringup.launch（后台运行）
    bringup_process = run_command("roslaunch armpi_fpv_bringup bringup.launch", wait=False)
    print("正在启动 bringup.launch ...")
    time.sleep(5)

    # 等待节点和服务启动
    wait_for_node("/usb_cam")
    wait_for_service("/object_sorting/enter")
    wait_for_service("/object_sorting/set_running")
    wait_for_service("/object_sorting/set_target")

    # 进入
    run_command('rosservice call /object_sorting/enter "{}"')
    time.sleep(2)

    # 启动图像显示程序
    viewer_process = run_command("python3 /home/ubuntu/armpi_fpv/src/image_viewer.py", wait=False)
    print("已启动图像显示程序")
    time.sleep(2)

    # 启动
    run_command('rosservice call /object_sorting/set_running "data: true"')
    time.sleep(1)

    # 设置颜色和标签
    run_command('rosservice call /object_sorting/set_target "{color: [\'red\'], tag: [\'tag1\']}"')
    print("程序已启动，系统正在运行中。")

    input("按下回车键停止程序...")

    # 停止
    run_command('rosservice call /object_sorting/set_running "data: false"')

    # 退出
    run_command('rosservice call /object_sorting/exit "{}"')

    # 关闭后台服务
    bringup_process.terminate()
    viewer_process.terminate()
    print("程序已停止，服务已关闭。")

if __name__ == '__main__':
    main()