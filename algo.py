import pygame
import sys, os, getopt, math, random
import numpy as np

swarm_size = 7  # default size of the swarm
manual_mode = False  # manually press enter key to proceed between simulations

# Function to create a unit vector from a given vector
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

power_exponent = 1.3  # between 1.0 and 2.0
    # the larger the parameter, the slower the windows grows with swarm size; vice versa
# for converting from physical world to display world
pixels_per_length = 50  # this is to be fixed
# calculate world_side_coef from a desired screen size for 30 robots
def cal_world_side_coef():
    desired_screen_size = 400  # desired screen size for 30 robots
    desired_world_size = float(desired_screen_size) / pixels_per_length
    return desired_world_size / pow(7, 1/power_exponent)
world_side_coef = cal_world_side_coef()
world_side_length = world_side_coef * pow(swarm_size, 1/power_exponent)
world_size = (world_side_length, world_side_length)  # square physical world
# screen size calculated from world size
screen_side_length = int(pixels_per_length * world_side_length)
screen_size = (screen_side_length, screen_side_length)  # square display world

# formation configuration
comm_range = 0.65  # communication range in the world
desired_space_ratio = 0.8  # ratio of the desired space to the communication range
    # should be larger than 1/1.414=0.71, to avoid connections crossing each other
desired_space = comm_range * desired_space_ratio
# deviate robot heading, so as to avoid robot travlling perpendicular to the walls
perp_thres = math.pi/18  # threshold, range from the perpendicular line
devia_angle = math.pi/9  # deviate these much angle from perpendicualr line
# # consensus configuration
# loop_folder = "loop-data2"  # folder to store the loop shapes
# shape_catalog = ["airplane", "circle", "cross", "goblet", "hand", "K", "lamp", "square",
#     "star", "triangle", "wrench"]
# shape_quantity = len(shape_catalog)  # the number of decisions
# shape_decision = -1  # the index of chosen decision, in range(shape_quantity)
#     # also the index in shape_catalog
# assignment_scheme = np.zeros(swarm_size)
# # variable to force shape to different choices, for video recording
# force_shape_set = range(shape_quantity)

# robot properties
robot_poses = np.random.rand(swarm_size, 2) * world_side_length  # initialize the robot poses
dist_table = np.zeros((swarm_size, swarm_size))  # distances between robots
conn_table = np.zeros((swarm_size, swarm_size))  # connections between robots
    # 0 for disconnected, 1 for connected
conn_lists = [[] for i in range(swarm_size)]  # lists of robots connected

# function for all simulations, update the distances and connections between the robots
def dist_conn_update():
    global dist_table
    global conn_table
    global conn_lists
    conn_lists = [[] for i in range(swarm_size)]  # empty the lists
    for i in range(swarm_size):
        for j in range(i+1, swarm_size):
            dist_temp = np.linalg.norm(robot_poses[i] - robot_poses[j])
            dist_table[i,j] = dist_temp
            dist_table[j,i] = dist_temp
            if dist_temp > comm_range:
                conn_table[i,j] = 0
                conn_table[j,i] = 0
            else:
                conn_table[i,j] = 1
                conn_table[j,i] = 1
                conn_lists[i].append(j)
                conn_lists[j].append(i)
dist_conn_update()  # update the distances and connections
disp_poses = []  # display positions
# function for all simulations, update the display positions

def disp_poses_update():
    global disp_poses
    poses_temp = robot_poses / world_side_length
    poses_temp[:,1] = 1.0 - poses_temp[:,1]
    poses_temp = poses_temp * screen_side_length
    # disp_poses = poses_temp.astype(int)  # convert to int and assign to disp_poses
    disp_poses = np.array([[0,0] for i in range(swarm_size)])  # 
disp_poses_update()
# deciding the seed robots, used in simulations with moving robots
seed_percentage = 0.1  # the percentage of seed robots in the swarm
seed_quantity = min(max(int(swarm_size*seed_percentage), 1), swarm_size)
    # no smaller than 1, and no larger than swarm_size
robot_seeds = [False for i in range(swarm_size)]  # whether a robot is a seed robot
    # only seed robot can initialize the forming a new group
seed_list_temp = np.arange(swarm_size)
np.random.shuffle(seed_list_temp)
for i in seed_list_temp[:seed_quantity]:
    robot_seeds[i] = True
    
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

current_swarm_size = 0  # the current swarm size
sim_haulted = False
# set up the simulation window
try:
    pygame.display.quit()
    pygame.quit()
except:
    pass
pygame.init()
disp_poses_update()
font = pygame.font.SysFont("Cabin", 12)
# icon = pygame.image.load("icon_geometry_art.jpg")
# pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Demo 1")
# draw the network
screen.fill(color_white)
for i in range(swarm_size):
    pygame.draw.circle(screen, color_black, disp_poses[i], robot_size_formation,
        robot_width_empty)
    # pygame.draw.circle(screen, color_black, disp_poses[i],
    #     int(comm_range*pixels_per_length), 1)
pygame.display.update()


while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # close window button is clicked
            print("program exit in simulation 1")
            sys.exit()  # exit the entire program
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_SPACE:
                sim_haulted = not sim_haulted  # reverse the pause flag
    if sim_haulted: continue
    
    current_pos = disp_poses[0]
    if current_swarm_size == 1:
        random_vector = unit_vector((np.random.rand(2)-0.5)*2)
        disp_poses[0] = current_pos + np.array((1,2))*10
    else:
        for i in range(current_swarm_size):
            current_pos = disp_poses[i]
            # disp_poses[i] = disp_poses[i] + np.array([1,1])
            random_vector = unit_vector(np.random.uniform(low = 0, high =disp_poses[i-1]))                
            disp_poses[i] = np.array(disp_poses[i])+ random_vector
            
        
        
    if current_swarm_size < swarm_size:
        print(True)
        current_swarm_size+=1 
    # disp_poses_update()
    screen.fill(color_white)
    for i in range(swarm_size):
        pygame.draw.circle(screen, color_black, disp_poses[i], robot_size_formation,
            robot_width_empty)
        # pygame.draw.circle(screen, color_black, disp_poses[i],
        #     int(comm_range*pixels_per_length), 1)
    pygame.display.update()
