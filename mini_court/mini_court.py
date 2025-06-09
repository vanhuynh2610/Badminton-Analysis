import cv2
import sys
sys.path.append("../")
import constants
from utils import (
    convert_meter_distance_to_pixel,
    convert_pixel_distance_to_meter,
    get_foot_position,
    get_closest_keypoint_index,
    get_height_of_bbox,
    measure_xy_distance,
    get_center_of_bbox,
    measure_distance
    )
import numpy as np

class MiniCourt():
    def __init__(self, frame):
        self.drawing_rectangle_width = 150  # Reduced from 250
        self.drawing_rectangle_height = 300  # Reduced from 450
        self.buffer = 30     # Reduced from 50
        self.padding_court = 10
        self.set_canvas_background_box_position(frame)
        self.set_mini_court_position()
        self.set_court_drawing_key_points()
        self.set_court_lines()

    def convert_meters_to_pixel(self, meters):
        return convert_meter_distance_to_pixel(
            meters,
            constants.DOUBLE_LINE_WIDTH,
            self.court_drawing_width
        )
    def convert_meters_to_pixel_y(self, meters):
        return convert_meter_distance_to_pixel(
            meters,
            constants.HALF_COURT_LINE_HEIGHT * 2,
            self.court_drawing_height
        )

    def set_court_drawing_key_points(self):
        drawing_key_points = [0]*24
        #point 0
        drawing_key_points[0]  = int(self.court_start_x)
        drawing_key_points[1] = int(self.court_start_y) + self.convert_meters_to_pixel(constants.HALF_COURT_LINE_HEIGHT*2)

        #point 1
        drawing_key_points[2] = 1240 #int(self.court_start_x) + self.convert_meter_to_pixel(constants.HALF_COURT_LINE_HEIGHT*2)
        drawing_key_points[3] = drawing_key_points[1]

        #point 2 
        drawing_key_points[4] = drawing_key_points[0] + self.convert_meters_to_pixel(constants.NO_MANS_LAND_HEIGHT)
        drawing_key_points[5] = drawing_key_points[1] - self.convert_meters_to_pixel(constants.NO_MANS_LAND_HEIGHT)

        #point 3 
        drawing_key_points[6] = drawing_key_points[2] - self.convert_meters_to_pixel(constants.NO_MANS_LAND_HEIGHT)
        drawing_key_points[7] = drawing_key_points[5]

        #point 4
        drawing_key_points[8] = drawing_key_points[4] 
        drawing_key_points[9] = drawing_key_points[5] - self.convert_meters_to_pixel(constants.SERVICE_LINE_WIDTH)

        #point 5
        drawing_key_points[10] = drawing_key_points[6]
        drawing_key_points[11] = drawing_key_points[9]
        

        #point 10
        drawing_key_points[20] , drawing_key_points[21] = int(self.court_start_x), int(self.court_start_y)

        #point 8
        drawing_key_points[16] = drawing_key_points[8]
        drawing_key_points[17] = drawing_key_points[21] + self.convert_meters_to_pixel(constants.NO_MANS_LAND_HEIGHT)

        #point 9 
        drawing_key_points[18] = drawing_key_points[6]
        drawing_key_points[19] = drawing_key_points[21] + self.convert_meters_to_pixel(constants.NO_MANS_LAND_HEIGHT)

        #point 6
        drawing_key_points[12] = drawing_key_points[16] 
        drawing_key_points[13] = drawing_key_points[17] + self.convert_meters_to_pixel(constants.SERVICE_LINE_WIDTH)

        #point 7
        drawing_key_points[14] = drawing_key_points[18]
        drawing_key_points[15] = drawing_key_points[13]

        #point 11
        drawing_key_points[22] , drawing_key_points[23] = int(self.court_end_x), int(self.court_start_y)
        self.drawing_key_points=drawing_key_points

    

    def set_mini_court_position(self):
        self.court_start_x = self.start_x + self.padding_court
        self.court_start_y = self.start_y + self.padding_court
        self.court_end_x = self.end_x - self.padding_court
        self.court_end_y = self.end_y - self.padding_court
        self.court_drawing_width = self.court_end_x - self.court_start_x
        self.court_drawing_height = self.court_end_y - self.court_end_x

    
    def set_court_lines(self):
        self.lines = [
            (10, 0),
            (8, 2),
            (9,3),
            (11,1),
            
            (10,11),
            (8,9),
            (6,7),
            (4,5),
            (2,3),
            (0,1)
        ]

    def set_canvas_background_box_position(self, frame):
        frame = frame.copy()

        self.end_x = frame.shape[1] - self.buffer
        self.end_y = self.buffer + self.drawing_rectangle_height
        self.start_x = self.end_x - self.drawing_rectangle_width
        self.start_y = self.end_y - self.drawing_rectangle_height

    def draw_court(self, frame):
        for i in range(0 , len(self.drawing_key_points),2):
            x = int(self.drawing_key_points[i])
            y = int(self.drawing_key_points[i+1])
            cv2.circle(frame, (x, y), 5, (0, 0, 225), -1)

        for line in self.lines:
            if line == (8,2) or line == (9,3):
                start_point = (int(self.drawing_key_points[line[0]*2]), int(self.drawing_key_points[10*2+1]))
                end_point = (int(self.drawing_key_points[line[1]*2]), int(self.drawing_key_points[0*2+1]))
            elif line == (2,3) or line == (4,5) or line == (6,7) or line == (8,9):
                start_point = (int(self.drawing_key_points[0*2]), int(self.drawing_key_points[line[0]*2+1]))
                end_point = (int(self.drawing_key_points[1*2]), int(self.drawing_key_points[line[1]*2+1]))
            else:
                start_point = (int(self.drawing_key_points[line[0]*2]), int(self.drawing_key_points[line[0]*2+1]))
                end_point = (int(self.drawing_key_points[line[1]*2]), int(self.drawing_key_points[line[1]*2+1]))
            cv2.line(frame, start_point, end_point, (0, 0, 0), 2)
        
        net_start_point = (self.drawing_key_points[0], int((self.drawing_key_points[1] + self.drawing_key_points[21])/2))
        net_end_point = (self.drawing_key_points[2], int((self.drawing_key_points[1] + self.drawing_key_points[21])/2))
        
        cv2.line(frame, net_start_point, net_end_point, (255, 0, 0), 2)

        width_half_start_point = (int((self.drawing_key_points[0] + self.drawing_key_points[2])/2), int(self.drawing_key_points[1]))
        width_half_end_point = (int((self.drawing_key_points[0] + self.drawing_key_points[2])/2), int(self.drawing_key_points[9]))
        cv2.line(frame, width_half_start_point, width_half_end_point, (0, 0, 0), 2)

        width_half_start_point2 = (int((self.drawing_key_points[0] + self.drawing_key_points[2])/2), int(self.drawing_key_points[21]))
        width_half_end_point2 = (int((self.drawing_key_points[0] + self.drawing_key_points[2])/2), int(self.drawing_key_points[13]))
        cv2.line(frame, width_half_start_point2, width_half_end_point2, (0, 0, 0), 2)


        return frame

    def draw_background_rectangle(self, frame):
        shapes = np.zeros(frame.shape, dtype=np.uint8)
        cv2.rectangle(
            shapes,
            (self.start_x, self.start_y),
            (self.end_x, self.end_y),
            (225, 225, 225),
            cv2.FILLED
        )
        out = frame.copy()
        alpha = 0.5
        mask = shapes.astype(bool)
        out[mask] = cv2.addWeighted(frame, alpha, shapes, 1 - alpha, 0)[mask]
        return out
    
    def draw_mini_court(self, frames):
        output_frames = []
        for frame in frames:
            frame = self.draw_background_rectangle(frame)
            frame = self.draw_court(frame)
            output_frames.append(frame)
        return output_frames
    
    def get_start_point_of_mini_court(self):
        return (self.court_start_x,self.court_start_y)
    
    def get_width_of_mini_court(self):
        return self.court_drawing_width

    def get_court_drawing_keypoints(self):
        return self.drawing_key_points
    
    def get_mini_court_coordinates(self,
                                   object_position,
                                   closest_key_point, 
                                   closest_key_point_index, 
                                   player_height_in_pixels,
                                   player_height_in_meters, player_id
                                   ):
        distance_from_keypoint_x_pixels, distance_from_keypoint_y_pixels =  measure_xy_distance(object_position, closest_key_point)
        # Conver pixel distance to meters
        distance_from_keypoint_x_meters = convert_pixel_distance_to_meter(distance_from_keypoint_x_pixels,
                                                                           player_height_in_meters,
                                                                           player_height_in_pixels
                                                                           )
        distance_from_keypoint_y_meters = convert_pixel_distance_to_meter(distance_from_keypoint_y_pixels,
                                                                                player_height_in_meters,
                                                                                player_height_in_pixels
                                                                          )

        # Convert to mini court coordinates

        if player_id == 1 and closest_key_point_index == 1:
            mini_court_y_distance_pixels = self.convert_meters_to_pixel_y(distance_from_keypoint_y_meters)
            mini_court_x_distance_pixels = self.convert_meters_to_pixel(-distance_from_keypoint_x_meters)
        elif player_id == 1 and closest_key_point_index == 4:
            mini_court_y_distance_pixels = self.convert_meters_to_pixel_y(-distance_from_keypoint_y_meters)
            mini_court_x_distance_pixels = self.convert_meters_to_pixel(distance_from_keypoint_x_meters)
        
        elif player_id == 2 and closest_key_point_index == 6 :
            mini_court_y_distance_pixels = self.convert_meters_to_pixel_y(distance_from_keypoint_y_meters)
            mini_court_x_distance_pixels = self.convert_meters_to_pixel(distance_from_keypoint_x_meters)

        elif player_id == 2 and closest_key_point_index == 9:
            mini_court_y_distance_pixels = self.convert_meters_to_pixel_y(-distance_from_keypoint_y_meters)
            mini_court_x_distance_pixels = self.convert_meters_to_pixel(-distance_from_keypoint_x_meters)

        else:
            mini_court_y_distance_pixels = self.convert_meters_to_pixel_y(-distance_from_keypoint_y_meters)
            mini_court_x_distance_pixels = self.convert_meters_to_pixel(-distance_from_keypoint_x_meters)


        closest_mini_coourt_keypoint = ( self.drawing_key_points[closest_key_point_index*2],
                                        self.drawing_key_points[closest_key_point_index*2+1]
                                        )
        
        mini_court_player_position = (closest_mini_coourt_keypoint[0]+mini_court_x_distance_pixels,
                                      closest_mini_coourt_keypoint[1]+ mini_court_y_distance_pixels
                                        )

        return  mini_court_player_position

    def convert_bounding_boxes_to_mini_court_coordinates(self,player_boxes, ball_boxes, original_court_key_points ):
        player_heights = {
            1: constants.PLAYER_1_HEIGHT_METERS,
            2: constants.PLAYER_2_HEIGHT_METERS
        }
        output_player_boxes= []

        for frame_num, player_bbox in enumerate(player_boxes):
            ball_box = ball_boxes[frame_num][1]
            ball_position = get_center_of_bbox(ball_box)
            output_player_bboxes_dict = {}
            for player_id, bbox in player_bbox.items():

                foot_position = get_foot_position(bbox)
                
                # Get The closest keypoint in pixels
                closest_key_point_index = get_closest_keypoint_index(foot_position,original_court_key_points, [1,4,6,9])
                closest_key_point = (original_court_key_points[closest_key_point_index*2], 
                                     original_court_key_points[closest_key_point_index*2+1])
                # Get Player height in pixels
                frame_index_min = max(0, frame_num-20)
                frame_index_max = min(len(player_boxes), frame_num+10)
                bboxes_heights_in_pixels = [get_height_of_bbox(player_boxes[i][player_id]) for i in range (frame_index_min,frame_index_max)]
                max_player_height_in_pixels = max(bboxes_heights_in_pixels)

                mini_court_player_position = self.get_mini_court_coordinates(foot_position,
                                                                            closest_key_point, 
                                                                            closest_key_point_index, 
                                                                            max_player_height_in_pixels,
                                                                            player_heights[player_id], 
                                                                            player_id
                                                                            )
                
                output_player_bboxes_dict[player_id] = mini_court_player_position
            output_player_boxes.append(output_player_bboxes_dict)
        return output_player_boxes #, output_ball_boxes
    
    def draw_points_on_mini_court(self,frames,postions, color=(0,255,0)):
        for frame_num, frame in enumerate(frames):
            for _, position in postions[frame_num].items():
                x,y = position
                x= int(x)
                y= int(y)
                cv2.circle(frame, (x,y), 5, color, -1)
        return frames