import subprocess

def launch_aruco_script():
    cmd = [
        "python3", 
        "ArUcoMovement.py", 
        "--use-zed", 
        "--ugv-marker-id", "5", 
        "--dest-marker-id", "0"
    ]
    
    print("Launching ArUco Navigation...")
    subprocess.run(cmd)

if __name__ == "__main__":
    launch_aruco_script()