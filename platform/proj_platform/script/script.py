# script.py
import sys

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法: python3 script.py <color> <tag>")
    else:
        color = sys.argv[1]
        tag = sys.argv[2]
        print(f"接收到的参数: color = {color}, tag = {tag}")
