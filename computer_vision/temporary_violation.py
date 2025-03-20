import argparse
import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv
import time
import os

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Traffic Violation Detection using YOLO")
    
    parser.add_argument("--source_video_path", required=True, help="Path to source video file", type=str)
    parser.add_argument("--output_frame_size", required=False, help="Desired output frame size as width,height", type=str, default="640,480")

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    
    # Load YOLO model
    model = YOLO("yolo11n.pt")

    # Get video information
    video_info = sv.VideoInfo.from_video_path(args.source_video_path)
    byte_track = sv.ByteTrack(frame_rate=video_info.fps)

    # Annotation settings
    thickness = sv.calculate_optimal_line_thickness(resolution_wh=video_info.resolution_wh)
    text_scale = sv.calculate_optimal_text_scale(resolution_wh=video_info.resolution_wh)
    box_annotator = sv.BoxAnnotator(thickness=1)
    label_annotator = sv.LabelAnnotator(text_scale=text_scale, text_thickness=thickness)

    # Create folder for violation images
    if not os.path.exists("violations"):
        os.makedirs("violations")

    # Define stop line for red-light detection
    STOP_LINE_Y = 250
    frame_generator = sv.get_video_frames_generator(args.source_video_path)
    output_width, output_height = map(int, args.output_frame_size.split(','))

    frame_count = 0  # Initialize frame count

    for frame in frame_generator:
        frame_count += 1  # Track frames
        
        # Run YOLO detection
        results = model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)
        detections = byte_track.update_with_detections(detections=detections)

        # Annotate frame
        annotated_frame = frame.copy()
        annotated_frame = box_annotator.annotate(scene=annotated_frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)
        resized_frame = cv2.resize(annotated_frame, (output_width, output_height))

        # Process detected objects
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_id = int(box.cls[0])

            # Check if detected object is a vehicle (COCO: Car, Motorcycle, Bus, Truck)
            if class_id in [2, 3, 5, 7]:
                vehicle_center_y = (y1 + y2) // 2

                # Simulated traffic light status (Replace with actual detection)
                traffic_light = "red" if frame_count % 300 < 150 else "green"

                # Check for red-light violation
                if traffic_light == "red" and vehicle_center_y > STOP_LINE_Y:
                    cv2.rectangle(resized_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    violation_image_path = f"violations/violation_{time.time()}.jpg"
                    cv2.imwrite(violation_image_path, resized_frame)
                    print(f"Violation detected! Image saved: {violation_image_path}")
                else:
                    cv2.rectangle(resized_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # **Show the frame inside the loop**
        frame = cv2.resize(frame, (1280, 720))
        cv2.imshow("Annotated Frame", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
