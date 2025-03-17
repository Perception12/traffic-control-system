import argparse
from ultralytics import YOLO
import supervision as sv
import cv2


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Vehicle speed estimation using Inference and Supervision"
    )
    
    parser.add_argument(
        "--source_video_path",
        required=True,
        help="Path to source video file",
        type=str,
        )
    parser.add_argument(
        "--output_frame_size",
        required=False,
        help="Desired output frame size as width,height",
        type=str,
        default="640,480"
        )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    
    video_info = sv.VideoInfo.from_video_path(args.source_video_path)
    model = YOLO("yolov8x.pt")
    
    byte_track = sv.ByteTrack(frame_rate=video_info.fps)
    
    thickness = sv.calculate_optimal_line_thickness(resolution_wh=video_info.resolution_wh)
    text_scale = sv.calculate_optimal_text_scale(resolution_wh=video_info.resolution_wh)
    box_annotator = sv.BoxAnnotator(thickness=4)
    label_annotator = sv.LabelAnnotator(text_scale=text_scale, text_thickness=thickness)
    
    frame_generator = sv.get_video_frames_generator(args.source_video_path)
    
    output_width, output_height = map(int, args.output_frame_size.split(','))

    for frame in frame_generator:
        result = model(frame)[0]
        detections = sv.Detections.from_ultralytics(result)
        detections = byte_track.update_with_detections(detections=detections)
        
        annotated_frame = frame.copy()
        annotated_frame = box_annotator.annotate(scene=annotated_frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)
        
        # Resize the annotated frame
        resized_frame = cv2.resize(annotated_frame, (output_width, output_height))
        
        cv2.imshow("Annotated Frame", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()



