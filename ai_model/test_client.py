import os
import requests

# Path to your testing images folder
folder = "testing images"

# Server endpoint
url = "http://192.168.68.107:5000/upload"

# Loop through all files in the folder
for filename in os.listdir(folder):
    if filename.lower().endswith((".jpg", ".jpeg", ".png")):
        filepath = os.path.join(folder, filename)
        print(f"Sending {filepath}...")
        with open(filepath, "rb") as f:
            r = requests.post(url, files={"file": f})
            print("Response:", r.text)
