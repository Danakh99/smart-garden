from flask import Flask, request, jsonify
from ultralytics import YOLO
import cv2
import numpy as np

app = Flask(__name__)

# Load YOLOv8 model
model = YOLO("best.pt")

@app.route("/upload", methods=["POST"])
def upload():
    img_bytes = None

    # Case 1: multipart form (Python client)
    if "file" in request.files:
        img_bytes = request.files["file"].read()

    # Case 2: raw bytes (ESP32-CAM)
    elif request.data:
        img_bytes = request.data

    if not img_bytes:
        return jsonify({"error": "No image data received"}), 400

    # Decode image
    nparr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        return jsonify({"error": "Failed to decode image"}), 400

    # Run YOLO inference
    results = model(img)

    detections = []
    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        xyxy = box.xyxy[0].tolist()
        detections.append({
            "class": model.names[cls_id],
            "confidence": round(conf, 4),
            "bbox": [round(x, 2) for x in xyxy]
        })

    return jsonify({"detections": detections})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
