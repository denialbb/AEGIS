import os
import shutil
from typing import List, Optional

def get_temp_directories() -> List[str]:
    """Get list of potential temporary directories to search."""
    dirs = ["/tmp", "/var/tmp"]
    # Check Windows temp directories mounted in WSL
    win_users_path = "/mnt/c/Users"
    if os.path.exists(win_users_path):
        try:
            for user in os.listdir(win_users_path):
                user_temp = os.path.join(win_users_path, user, "AppData", "Local", "Temp")
                if os.path.exists(user_temp):
                    dirs.append(user_temp)
        except Exception:
            pass
    
    # Also add standard env vars if defined
    for env_var in ["TEMP", "TMP", "TMPDIR"]:
        val = os.environ.get(env_var)
        if val and os.path.exists(val):
            dirs.append(val)
            
    # Clean duplicates and normalize paths
    unique_dirs = list(set(os.path.normpath(d) for d in dirs))
    return [d for d in unique_dirs if os.path.isdir(d)]

def find_latest_handoff() -> Optional[str]:
    """Find the path to the most recent handoff file."""
    search_dirs = get_temp_directories()
    handoff_files: List[str] = []
    
    for directory in search_dirs:
        try:
            for file_name in os.listdir(directory):
                # Search case-insensitively for "handoff" in the filename with .md or .txt extension
                if "handoff" in file_name.lower() and (file_name.endswith(".md") or file_name.endswith(".txt")):
                    full_path = os.path.join(directory, file_name)
                    if os.path.isfile(full_path):
                        handoff_files.append(full_path)
        except Exception:
            pass
            
    if not handoff_files:
        return None
        
    # Sort files by modification time, latest first
    handoff_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
    return handoff_files[0]

def load_handoff() -> None:
    """Finds, prints, and copies the latest handoff file."""
    latest = find_latest_handoff()
    if not latest:
        print("No handoff files found in temporary directories.")
        return
        
    print(f"Found latest handoff file: {latest}")
    print("=" * 40)
    
    try:
        with open(latest, 'r', encoding='utf-8') as f:
            content = f.read()
            print(content)
    except Exception as e:
        print(f"Error reading handoff file: {e}")
        return
        
    print("=" * 40)
    
    # Copy to workspace shared temp location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dest_dir = os.path.abspath(os.path.join(script_dir, "..", "shared", "temp"))
    os.makedirs(dest_dir, exist_ok=True)
    
    # Determine target filename extension based on source
    ext = os.path.splitext(latest)[1]
    dest_path = os.path.join(dest_dir, f"latest_handoff{ext}")
    
    try:
        shutil.copy2(latest, dest_path)
        print(f"Successfully copied handoff to {dest_path}")
    except Exception as e:
        print(f"Error copying handoff file: {e}")

if __name__ == "__main__":
    load_handoff()
