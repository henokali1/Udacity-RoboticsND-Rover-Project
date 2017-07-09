import numpy as np
import cv2

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# Filter out only the pixles between the hsv threshold
def target_filter(img, hsv_thresh_min, hsv_thresh_max):    
    # Blur image to remove noise
    img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

    # Convert image from BGR to HSV
    img_filter = cv2.cvtColor(img_filter, cv2.COLOR_RGB2HSV)

    # Set pixels to white if in color range, others to black (binary bitmap)
    img_binary = cv2.inRange(img_filter.copy(), hsv_thresh_min, hsv_thresh_max)
    
    
    # Dilate image to make white blobs larger
    img_binary = cv2.dilate(img_binary, None, iterations = 1)
    
    # Return the binary image
    return(img_binary)


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    img = Rover.img
    # Define source and destination points for perspective transform
    img_size = (img.shape[1], img.shape[0])
    dst_size = 5
    bottom_offset = 6
    src = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    dst = np.float32([[img_size[0] / 2 - dst_size, img_size[1] - bottom_offset],
                      [img_size[0] / 2 + dst_size, img_size[1] - bottom_offset],
                      [img_size[0] / 2 + dst_size, img_size[1] - 2 * dst_size - bottom_offset],
                      [img_size[0] / 2 - dst_size, img_size[1] - 2 * dst_size - bottom_offset],
                      ])


    # Apply perspective transform
    warped = perspect_transform(img, src, dst)


    # Apply color threshold to identify navigable terrain/obstacles/rock samples
    # HSV color thresholds for navigable area
    NAVIGABLE_LOW = (0, 0, 190)
    NAVIGABLE_HIGH = (150, 45, 255)
    # HSV color thresholds for obstacles
    OBSTACLE_LOW = (0, 0, 0)
    OBSTACLE_HIGH = (165, 255, 62)
    # HSV color thresholds for Rock samples
    ROCK_LOW = (20, 155, 106)
    ROCK_HIGH = (26, 255, 255)

    navigable = target_filter(warped, NAVIGABLE_LOW, NAVIGABLE_HIGH)
    obstacle = target_filter(warped, OBSTACLE_LOW, OBSTACLE_HIGH)
    rock = target_filter(warped, ROCK_LOW, ROCK_HIGH)


    # Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle
    Rover.vision_image[:,:,1] = rock
    Rover.vision_image[:,:,2] = navigable


    # Convert map image pixel values to rover-centric coords
    rover_xpos, rover_ypos = Rover.pos
    rover_yaw = Rover.yaw
    worldmap = Rover.worldmap
    scale = 10
    
    navigable_xpix, navigable_ypix = rover_coords(navigable)
    rock_sample_xpix, rock_sample_ypix = rover_coords(rock)
    obstacle_xpix, obstacle_ypix = rover_coords(obstacle)


    # Convert rover-centric pixel values to world coordinates
    rover_xpos, rover_ypos = Rover.pos
    rover_yaw = Rover.yaw
    worldmap = Rover.worldmap
    scale = 10

    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix, rover_xpos,
                                    rover_ypos, rover_yaw,
                                    worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(rock_sample_xpix, rock_sample_ypix, rover_xpos,
                                    rover_ypos, rover_yaw,
                                    worldmap.shape[0], scale)

    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, obstacle_ypix, rover_xpos,
                                    rover_ypos, rover_yaw,
                                    worldmap.shape[0], scale)

    

    # Convert rover-centric pixel positions to polar coordinates
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(navigable_xpix, navigable_ypix)
    rock_dist, rock_angle = to_polar_coords(rock_sample_xpix, rock_sample_ypix)

    

    # Update Rover pixel distances and angles
    # If there is a rock sample in vision, change 'Rover.mode' to 'rock_nav' to navigate the rover into the location of the rover
    if((len(rock_dist) > 0) or (len(rock_angle) > 0)):
        Rover.mode = 'rock_nav'
        Rover.nav_dists = rock_dist
        Rover.nav_angles = rock_angle
    elif((len(rover_centric_pixel_distances) > 0) or (len(rover_centric_angles) > 0)):
        Rover.nav_dists = rover_centric_pixel_distances
        Rover.nav_angles = rover_centric_angles

        # Update Rover worldmap (to be displayed on right side of screen) if and only if the pitch and roll angles are less than 0.5
        if((Rover.roll >=0 and Rover.roll <= 0.5) and (Rover.pitch >=0 and Rover.pitch <= 0.5)):
            Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] +=1
            Rover.worldmap[rock_y_world, rock_x_world, 1] +=1
            Rover.worldmap[navigable_y_world, navigable_x_world, 2] +=1

    # Otherwise navigate the rover along the navigable terrain
    else:
        Rover.mode = 'stop'
        Rover.nav_dists = rover_centric_pixel_distances
        Rover.nav_angles = rover_centric_angles
    return Rover   
    