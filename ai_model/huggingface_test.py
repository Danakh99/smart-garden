from ultralytics import YOLO

model = YOLO("plant_leaf.pt")   # your downloaded Hugging Face weights
results = model.predict(source="test_leaf.jpeg", save=True)
print(results)
