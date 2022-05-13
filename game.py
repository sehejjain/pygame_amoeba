#!/usr/bin/env python
# coding: utf-8

# In[1]:


import pygame, time
import sys, os, getopt, math, random
import numpy as np
from constants import *


# In[108]:


swarm_size = 3  # default size of the swarm
manual_mode = False  # manually press enter key to proceed between simulations


# In[109]:


def S14_closest_robot(robot_host, robot_neighbors):
    # "robot_host": the robot to measure distance from
    # "robot_neighbors": a list of robots to be compared with
    robot_closest = robot_neighbors[0]
    dist_closest = dist_table[robot_host,robot_closest]
    for i in robot_neighbors[1:]:
        dist_temp = dist_table[robot_host,i]
        if dist_temp < dist_closest:
            robot_closest = i
            dist_closest = dist_temp
    return robot_closest


# In[110]:

# Function to get centroid
def getcenteroid(arr):
    length, dim = arr.shape
    return np.array([np.sum(arr[:, i])/length for i in range(dim)])

def get_dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def euclidean_distance(a, b):
    """Euclidean Distance

    Args:
        a (np.array): a
        b (np.array): b

    Returns:
        float: dist
    """
    return np.linalg.norm(a-b)


# In[111]:


# general function to steer robot away from wall if out of boundary (following physics)
# use global variable "world_side_length"
def robot_boundary_check(robot_pos, robot_ori):
    new_ori = robot_ori
    if robot_pos[0] >= screen.get_width():  # outside of right boundary
        if math.cos(new_ori) > 0:
            new_ori = reset_radian(2*(math.pi/2) - new_ori)
            # further check if new angle is too much perpendicular
            if new_ori > 0:
                if (math.pi - new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
            else:
                if (new_ori + math.pi) < perp_thres:
                    new_ori = new_ori + devia_angle
    elif robot_pos[0] <= 0:  # outside of left boundary
        if math.cos(new_ori) < 0:
            new_ori = reset_radian(2*(math.pi/2) - new_ori)
            if new_ori > 0:
                if new_ori < perp_thres:
                    new_ori = new_ori + devia_angle
            else:
                if (-new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
    if robot_pos[1] >= screen.get_height():  # outside of top boundary
        if math.sin(new_ori) > 0:
            new_ori = reset_radian(2*(0) - new_ori)
            if new_ori > -math.pi/2:
                if (new_ori + math.pi/2) < perp_thres:
                    new_ori = new_ori + devia_angle
            else:
                if (-math.pi/2 - new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
    elif robot_pos[1] <= 0:  # outside of bottom boundary
        if math.sin(new_ori) < 0:
            new_ori = reset_radian(2*(0) - new_ori)
            if new_ori > math.pi/2:
                if (new_ori - math.pi/2) < perp_thres:
                    new_ori = new_ori + devia_angle
            else:
                if (math.pi/2 - new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
    return new_ori


# general function to reset radian angle to [-pi, pi)
def reset_radian(radian):
    while radian >= math.pi:
        radian = radian - 2*math.pi
    while radian < -math.pi:
        radian = radian + 2*math.pi
    return radian


# In[112]:


# calculate world_side_coef from a desired screen size for 30 robots
def cal_world_side_coef():
    desired_screen_size = 500  # desired screen size for 30 robots
    desired_world_size = float(desired_screen_size) / pixels_per_length
    return desired_world_size / pow(30, 1/power_exponent)
world_side_coef = cal_world_side_coef()
# world_side_length = world_side_coef * pow(swarm_size, 1/power_exponent)
world_side_length = world_side_coef * pow(30, 1/power_exponent)
world_size = (world_side_length, world_side_length)  # square physical world
# screen size calculated from world size
screen_side_length = int(pixels_per_length * world_side_length)
screen_size = (screen_side_length, screen_side_length)  # square display world


# In[113]:


# Function to create a unit vector from a given vector
def unit_vector(vector):
    return np.abs(vector / np.linalg.norm(vector))
def get_swarm_vector(robot_oris):
    vector = np.array([0.0,0.0])
    for ori in robot_oris:
        vec = np.array([math.cos(ori), math.sin(ori)])
        vector += vec
    return vector

def get_radial_distance(robot_poses, current_swarm_size):
    centroid = np.array([np.sum(robot_poses[:, 0]), np.sum(robot_poses[:, 1])])/current_swarm_size
    radial_dist = 0
    for robot in range(current_swarm_size):
        radial_dist+= euclidean_distance(centroid, robot_poses[robot])
        
    return radial_dist


# In[114]:


try:
    pygame.display.quit()
    pygame.quit()
except:
    pass
pygame.init()

font = pygame.font.SysFont("Cabin", 12)

screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Demo 1")
# draw the network
screen.fill(color_white)

pygame.display.update()


# In[115]:


robot_poses = np.array([[10.0,10.0] for i in range(swarm_size)])  # initialize the robot poses
dist_table = np.zeros((swarm_size, swarm_size))  # distances between robots
conn_table = np.zeros((swarm_size, swarm_size))  # connections between robots
current_swarm_size = 1  # the current swarm size
robot_oris = np.random.rand(swarm_size) * math.pi   # in range of [0, pi/2)
past = []


# In[97]:


def plot_swarm(robot_poses, robot_oris):
    # draw the network
    screen.fill(color_white)
    for i in range(swarm_size):
        pygame.draw.circle(screen, color_black, robot_poses[i], robot_size_formation, robot_width_empty)
        pygame.draw.line(screen, color_grey, robot_poses[i], robot_poses[i]+np.array([math.cos(robot_oris[i]), math.sin(robot_oris[i])])*50)
    pygame.display.update()
plot_swarm(robot_poses, robot_oris)


# In[106]:


step = 20
# init = 100
delta_c = 1.5
delta_r = 1.5
step_normal = 3.0
swarm_ori = np.random.random()*math.pi
sim_haulted = False


# In[107]:


while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # close window button is clicked
            print("program exit in simulation 1")
            sys.exit()  # exit the entire program
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_SPACE:
                sim_haulted = not sim_haulted  # reverse the pause flag
    if sim_haulted: continue
    print(f"Halted = {sim_haulted}, swarm size = {current_swarm_size}")
    print(f"current swarm size is: {current_swarm_size} out of {swarm_size}")
    current_pos = robot_poses[0]
    centroid = np.array([np.sum(robot_poses[:, 0]), np.sum(robot_poses[:, 1])])/current_swarm_size
    # print(centroid)
    robot_poses_t = robot_poses.copy()  # stores the old copy of robot positions
    if current_swarm_size == 0: pass #continue
    elif current_swarm_size < 3:
        old_centroid = np.array([1,1])
        # random_vector = unit_vector((np.random.rand(2)-0.5)*2)
        for j in range(current_swarm_size):
        
            robot_oris[j] = robot_boundary_check(robot_poses_t[j], robot_oris[j])
            robot_poses[j] = robot_poses_t[j] + (step/5 * np.array([math.cos(robot_oris[j]), math.sin(robot_oris[j])]))
            past.append((robot_poses[j], robot_poses_t[j]))
        old_centroid = centroid
        changed_distance = euclidean_distance(old_centroid, centroid)
        old_radial = get_radial_distance(robot_poses, current_swarm_size)
        
    else:
        # Add robot to swarm
        if current_swarm_size < swarm_size:

            j = current_swarm_size
            robot_oris[j] = robot_boundary_check(robot_poses_t[j], robot_oris[j])
            robot_poses[j] = robot_poses_t[j] + (step * np.array([math.cos(robot_oris[j]), math.sin(robot_oris[j])]))
        
        # Pseudopod Grouping
        robot_ids = list(range(current_swarm_size))
        for div in range(current_swarm_size//3):
            init = np.random.choice(robot_ids)
            # print("Robot chosen is: ", init)
            robot_ids.remove(init)
            closest = S14_closest_robot(init, robot_ids)
            robot_ids.remove(closest)
            # print("closest robot id is: ", closest)
            
            # Choose another robot that is closest to the two
            closest1 = S14_closest_robot(init, robot_ids)
            closest2 = S14_closest_robot(closest, robot_ids)
            if closest1 == closest2:
                triad = [init, closest, closest1]
            else:
                if dist_table[init, closest1] < dist_table[closest, closest2]:
                    triad = [init, closest, closest1]
                    robot_ids.remove(closest1)
                else:
                    triad = [init, closest, closest2]
                    robot_ids.remove(closest2)
            # print("triad is: ", triad) # Direction of Last robot in this list is updated, other two are pseudopods
            dist = [euclidean_distance(robot_poses[triad[i]], centroid) for i in range(len(triad))]
            
            # dist.sort(reverse=True)
            vector1 = np.array([math.cos(robot_oris[triad[0]]), math.sin(robot_oris[triad[0]])])
            vector2 = np.array([math.cos(robot_oris[triad[1]]), math.sin(robot_oris[triad[1]])])
            
            current_vector = np.array([math.cos(robot_oris[triad[2]]), math.sin(robot_oris[triad[2]])])
            random_ori = np.random.random()* math.pi
            random_vector = np.array([math.cos(random_ori), math.sin(random_ori)])
            centroid_vector = np.array(centroid) - np.array(robot_poses[triad[2]])
            avg_vector = get_swarm_vector(robot_oris)
            
            vector = 4*np.random.random()*(vector1 + vector2) + 2 * np.random.random()* current_vector + np.random.random() * centroid_vector + np.random.random() * avg_vector + np.random.random()*random_vector
            
            
            robot_oris[triad[2]] = math.atan2(vector[1], vector[0])
            # print(vector)
            # robot_oris[triad[2]] = math.atan2(new[1], new[0])

            step_dist = 0
            coord_old = robot_poses[triad[2]]
            
            sum_coord = sum(robot_poses[:current_swarm_size])
            
            robot_oris[triad[2]] = robot_boundary_check(robot_poses_t[triad[2]], robot_oris[triad[2]])
            
            for step_candidate in np.linspace(step, step/2, 10):
                new_pos = robot_poses_t[triad[2]] + (step_candidate * np.array([math.cos(robot_oris[triad[2]]), math.sin(robot_oris[triad[2]])]))
                new_centroid = np.array(np.sum(new_pos), np.sum(new_pos))/current_swarm_size
                # print(euclidean_distance(old_centroid, centroid))
                if euclidean_distance(new_centroid, centroid)/changed_distance < delta_c:
                    break
                
            changed_distance = euclidean_distance(new_centroid, centroid) - euclidean_distance(old_centroid, centroid)
            robot_poses[triad[2]] = new_pos
            centroid = np.array([np.sum(robot_poses[:, 0]), np.sum(robot_poses[:, 1])])/current_swarm_size
            past.append((robot_poses[triad[2]], robot_poses_t[triad[2]]))
            
            # robot_oris[triad[2]] = robot_boundary_check(robot_poses_t[triad[2]], robot_oris[triad[2]])
            # robot_poses[triad[2]] = robot_poses_t[triad[2]] + (step * np.array([math.cos(robot_oris[triad[2]]), math.sin(robot_oris[triad[2]])]))
            
        curr_radial_dist = get_radial_distance(robot_poses, current_swarm_size)
        print(robot_ids)
        robot_ids.sort(key=lambda x: get_dist(centroid, robot_poses[x]), reverse=True)
        for robot in robot_ids:
            current_vector = np.array([math.cos(robot_oris[robot]), math.sin(robot_oris[robot])])
            centroid_vector = np.array(centroid) - np.array(robot_poses[robot])
            final_vector = np.random.random()* current_vector+np.random.random()*centroid_vector
            
            ori = math.atan2(final_vector[1], final_vector[0])
            ori = robot_boundary_check(robot_poses_t[robot], ori)
            
            for step_candidate in np.linspace(step, step/2, 10):
                new_pos = robot_poses_t[robot] + (step_candidate * np.array([math.cos(ori), math.sin(ori)]))
                new_radial = curr_radial_dist - euclidean_distance(centroid, robot_poses_t[robot])+ euclidean_distance(centroid, new_pos)
                
                if (new_radial)/old_radial < delta_r:
                    robot_poses[robot] = new_pos
                    past.append((robot_poses[robot], robot_poses_t[robot]))

            centroid = np.array([np.sum(robot_poses[:, 0]), np.sum(robot_poses[:, 1])])/current_swarm_size
            
        
        old_radial = new_radial
    old_centroid = centroid
    prev = robot_poses_t[:(current_swarm_size-1)] if current_swarm_size < swarm_size else robot_poses_t
    # print("previous robot positions: ", prev)         
    # print("robot positions: ", robot_poses[:current_swarm_size]) 
    # time.sleep(1)
    if current_swarm_size < swarm_size:
        current_swarm_size+=1 
        
    plot_swarm(robot_poses, robot_oris)
    


# In[86]:


screen.fill(color_white)
pygame.draw.circle(screen, color_black, (500, 100), 5, 0)
pygame.display.update()


# In[87]:


world_side_length


# In[ ]:




