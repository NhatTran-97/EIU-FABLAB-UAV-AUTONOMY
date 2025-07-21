import subprocess

# Chạy main.py và tắt stderr
subprocess.Popen(
    ["python3", "main.py"],
    stderr=subprocess.DEVNULL,
    stdout=subprocess.DEVNULL
)
