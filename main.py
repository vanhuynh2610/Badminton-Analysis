from utils import (read_video , save_video, measure_distance,convert_pixel_distance_to_meter,draw_player_stats
)
import constants
from trackers import PlayerTracker , BallTracker
import cv2
from court_line_detector import CourtLineDetector
from mini_court import MiniCourt
from copy import deepcopy
import pandas as pd


def main():
    # Read Video
    input_video_path = "input_video/Video5.mp4"
    video_frames = read_video(input_video_path)

    # Detect player , ball
    player_tracker = PlayerTracker("yolov8x.pt")
    ball_tracker = BallTracker("models/last.pt")
   
    player_detections = player_tracker.detect_frames(video_frames,
                                                     read_from_stub =True ,
                                                     stub_path="tracker_stubs/player_detections.pkl"
                                                    )
    
    ball_detections = ball_tracker.detect_frames(video_frames,
                                                    read_from_stub =True ,
                                                    stub_path="tracker_stubs/ball_detections.pkl"
                                                    )


    ball_detections = ball_tracker.interpolate_ball_position(ball_detections)    
    court_model_path = "models/best_keypoint_v3.pt"

    court_line_detector = CourtLineDetector(court_model_path)

    mini_court = MiniCourt(video_frames[0])
    court_keypoints = court_line_detector.predict(video_frames[0])
    player_detections = player_tracker.choose_and_filter_player(court_keypoints, player_detections)
    court = court_keypoints.flatten().tolist()
    player_mini_court_detections = mini_court.convert_bounding_boxes_to_mini_court_coordinates(player_detections, 
                                                                                                          ball_detections,
                                                                                                          court)

    def compute_player_speeds(player_detections, reference_height_in_meters=1.80, fps=30):
        from collections import defaultdict
        player_tracks = defaultdict(list)

        for frame_idx, detections in enumerate(player_detections):
            for player_id, bbox in detections.items():
                x_center = (bbox[0] + bbox[2]) / 2
                y_center = (bbox[1] + bbox[3]) / 2
                player_tracks[player_id].append((frame_idx, (x_center, y_center), bbox))

        player_speeds = {}

        for player_id, track in player_tracks.items():
            speeds = []
            for i in range(0,len(track),30):
                if i == 0:
                    speeds.append((track[i][0], 0.0))
                    continue

                prev_frame, prev_center, prev_bbox = track[i - 30]
                curr_frame, curr_center, curr_bbox = track[i]

                pixel_dist = measure_distance(prev_center, curr_center)

                ref_height_px = ((prev_bbox[3] - prev_bbox[1]) + (curr_bbox[3] - curr_bbox[1])) / 2

                if ref_height_px == 0:
                    speed = 0.0
                else:
                    meter_dist = convert_pixel_distance_to_meter(pixel_dist, reference_height_in_meters, ref_height_px)
                    delta_time = (curr_frame - prev_frame) / fps
                    speed = meter_dist / delta_time if delta_time > 0 else 0.0
                for j in range(i+1,i + 30):
                    speeds.append((j, speed))
                speeds.append((curr_frame, speed))
             
            player_speeds[player_id] = speeds

        return player_speeds


    player_speeds = compute_player_speeds(player_detections)


    # Draw bounding boxes
    output_video_frames = player_tracker.draw_bboxes(video_frames, player_detections)
    output_video_frames = ball_tracker.draw_bboxes(video_frames, ball_detections)

    ## Draw court lines
    output_video_frames = court_line_detector.draw_keypoint_on_video(output_video_frames, court_keypoints)

    # Draw mini court
    output_video_frames = mini_court.draw_mini_court(output_video_frames)
    output_video_frames = mini_court.draw_points_on_mini_court(output_video_frames,player_mini_court_detections)


    # Draw frame number on top left
    for i, frame in enumerate(output_video_frames):
        cv2.putText(frame, f"Frame: {i}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 225, 0), 2)
        for player_id, speeds in player_speeds.items():
            for frame_idx, speed in speeds:
                if frame_idx == i:
                    if player_id in player_detections[i]:
                        bbox = player_detections[i][player_id]
                        x = int((bbox[0] + bbox[2]) / 2)
                        y = int(bbox[3]) + 20
                        cv2.putText(frame, f"{speed*3.6:.2f} km/h", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
   
    save_video(output_video_frames, "output_videos/output_video.avi")

if __name__ == "__main__":
    main()