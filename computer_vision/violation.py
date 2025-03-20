import cv2
import numpy as np
import supervision as sv
from ultralytics import YOLO

model = YOLO("yolo11n.pt") 

# Define traffic violation detection area (Region of Interest, ROI) 
ROI_POINTS = np.array([[200, 500], [600, 500], [600, 550], [200, 550]])  #to be modified

# Define object tracker
tracker = sv.ByteTrack()

# Video source
cap = cv2.VideoCapture("traffic.mp4")

# violation detection function
def is_violation(box, roi):
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  #Center point of the bounding box
    return cv2.pointPolygonTest(roi, (cx, cy), False) >= 0  #Check if inside ROI

frame_count = 0  #Frame counter
violation_count = 0  #Number of violations

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    detections = model(frame)[0]  # Perform object detection

    boxes = detections.boxes.xyxy.cpu().numpy()  # Bounding boxes
    confs = detections.boxes.conf.cpu().numpy()  # Confidence scores
    classes = detections.boxes.cls.cpu().numpy()  # Class IDs

    tracked_objects = tracker.update(boxes, confs, classes)

    for obj in tracked_objects:
        x1, y1, x2, y2, track_id, class_id, conf = obj

        #If detected object is a vehicle (adjust class_id based on model)
        if int(class_id) in [2, 3, 5, 7]:  # Cars, trucks, buses, bikes
            if is_violation((x1, y1, x2, y2), ROI_POINTS):
                violation_count += 1
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2) #BGR: 255 defines the color
                cv2.putText(frame, f"Violation #{violation_count}", (int(x1), int(y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Save violation frame
                cv2.imwrite(f"violation_{violation_count}.jpg", frame)

    # Draw ROI on frame
    cv2.polylines(frame, [ROI_POINTS], isClosed=True, color=(255, 0, 0), thickness=2)

    # Show frame
    cv2.imshow("Traffic Violation Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()