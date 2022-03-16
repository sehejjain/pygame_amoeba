import math

power_exponent = 1.3  # between 1.0 and 2.0
    # the larger the parameter, the slower the windows grows with swarm size; vice versa
# for converting from physical world to display world
pixels_per_length = 50  # this is to be fixed



# formation configuration
comm_range = 0.65  # communication range in the world
desired_space_ratio = 0.8  # ratio of the desired space to the communication range
    # should be larger than 1/1.414=0.71, to avoid connections crossing each other
desired_space = comm_range * desired_space_ratio
# deviate robot heading, so as to avoid robot travlling perpendicular to the walls
perp_thres = math.pi/18  # threshold, range from the perpendicular line
devia_angle = math.pi/9  # deviate these much angle from perpendicualr line






# visualization configuration
color_white = (255,255,255)
color_black = (0,0,0)
color_grey = (128,128,128)
color_red = (255,0,0)
# distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
#     (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
#     (0,128,128), (230,190,255), (170,110,40), (255,250,200), (128,0,0),
#     (170,255,195), (128,128,0), (255,215,180), (0,0,128), (128,128,128))
distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
    (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
    (0,128,128), (230,190,255), (170,110,40), (128,0,0),
    (170,255,195), (128,128,0), (0,0,128))
color_quantity = 17
# sizes for formation simulations
robot_size_formation = 5  # robot size in formation simulations
robot_width_empty = 2
conn_width_formation = 2  # connection line width in formation simulations
# sizes for consensus simulations
robot_size_consensus = 7  # robot size in consensus simulatiosn
conn_width_thin_consensus = 2  # thin connection line in consensus simulations
conn_width_thick_consensus = 4  # thick connection line in consensus simulations
robot_ring_size = 9  # extra ring on robot in consensus simulations
# the sizes for formation and consensus simulations are set to same for visual consistency
