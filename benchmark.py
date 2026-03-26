import time
import timeit
import random

class DummyModel:
    def __init__(self):
        # 80 coco classes
        self.names = {i: f"class_{i}" for i in range(80)}
        self.names[0] = "cup"
        self.names[1] = "motorcycle"
        self.names[2] = "clock"
        self.names[3] = "potted plant"
        self.names[4] = "bottle"
        self.names[5] = "person"
        self.names[6] = "car"

class DummyBoxes:
    def __init__(self, num_boxes):
        self.cls = [random.randint(0, 79) for _ in range(num_boxes)]

model = DummyModel()
allowed_classes = {
    'cup': 'cup',
    'motorcycle': 'motorcycle',
    'clock': 'clock',
    'potted plant': 'plant',
    'plant': 'plant',
    'bottle': 'water bottle',
    'water bottle': 'water bottle',
}

# Pre-compute cache
cached_classes = {}
for class_id, class_name in model.names.items():
    norm = allowed_classes.get(str(class_name).lower())
    if norm is not None:
        cached_classes[class_id] = norm

boxes = DummyBoxes(100) # 100 detections

def original_loop():
    count = 0
    for idx in range(len(boxes.cls)):
        detected_class_id = int(boxes.cls[idx])
        detected_class_name = str(model.names[detected_class_id]).lower()
        normalized_name = allowed_classes.get(detected_class_name)
        if normalized_name is not None:
            count += 1
    return count

def optimized_loop():
    count = 0
    for idx in range(len(boxes.cls)):
        detected_class_id = int(boxes.cls[idx])
        normalized_name = cached_classes.get(detected_class_id)
        if normalized_name is not None:
            count += 1
    return count

print("Original:")
orig_time = timeit.timeit(original_loop, number=10000)
print(orig_time)

print("Optimized:")
opt_time = timeit.timeit(optimized_loop, number=10000)
print(opt_time)

print(f"Speedup: {orig_time / opt_time:.2f}x")
