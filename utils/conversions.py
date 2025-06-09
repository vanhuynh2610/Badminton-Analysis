def convert_pixel_distance_to_meter(pixel_distance, reference_height_in_meters, reference_height_in_pixels):
    return (pixel_distance * reference_height_in_meters) / reference_height_in_pixels

def convert_meter_distance_to_pixel(meter, reference_height_in_meters, reference_height_in_pixels):
    return (meter * reference_height_in_pixels) / reference_height_in_meters  