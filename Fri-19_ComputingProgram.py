import time
import sys
sys.path.append('../')

from Common_Libraries.p2_lib import *

import os
from Common_Libraries.repeating_timer_lib import repeating_timer

def update_sim ():
    try:
        arm.ping()
    except Exception as error_update_sim:
        print (error_update_sim)

arm = qarm()

update_thread = repeating_timer(2, update_sim)

import random
from random import shuffle               #imported random module and shuffle for the randomized loop
## *******************************************

#Made by Calvyn Siong (siongc1) and Renee Twyford (twyfordr)


is_gripper_open = True
draweropen = False              #global booleon variables initialized for opening/closing gripper and each autoclave drawers


threshold = 0.3 ## Defining the value of threshold

##Calvyn found all of the coordinates
small_red_drop_off = [-0.6121,0.2411,0.3704]
small_green_drop_off = [0,-0.6578,0.3784]
small_blue_drop_off = [0,0.6578,0.3784]
large_red_drop_off = [-0.4058,0.1639,0.1600]
large_green_drop_off = [0,-0.4229,0.2080]
large_blue_drop_off = [0,0.4229,0.2080]

drop_off_list = [small_red_drop_off, small_green_drop_off, small_blue_drop_off,
                large_red_drop_off, large_green_drop_off, large_blue_drop_off]
    
## "Top" Locations are used to avoid hitting other autoclaves or drawers
red_top = [-0.3768, 0.1522, 0.4826]
green_top = [0, -0.4064, 0.4826]
blue_top = [0, 0.4064, 0.4826]
    
drop_off_top_list = [red_top, green_top, blue_top]




##Calvyn wrote this function
## "Top" locations are used to avoid hitting the drawers, since the arm moves in a 2 step motion (moves to top, then to final coordinates to prevent collisions)
##For large containers, this also serves the purpose of preventing the drawer from hitting the arm in the final position, since the arm dips into the drawer to minimize impact from dropping a large container
def move_end_effector_to_top(drop_off_location):
    print("Moving to top position, flex only the left arm.")
    
    while True:           #while statement allows for main function to keep checking if the condition is met, before proceeding to the next subfunction (used in subsequent subfunctions)
        if(arm.emg_left() > threshold) and (arm.emg_right() == 0):
            if (is_gripper_open == False):
                arm.move_arm(0.4064, 0, 0.4826)
            time.sleep(.25)
            if (drop_off_location == 1) or (drop_off_location == 4): ## if it is a red container
                arm.move_arm(drop_off_top_list[0][0], drop_off_top_list[0][1], drop_off_top_list[0][2]) ## move it to the "top" red location
            elif (drop_off_location == 2) or (drop_off_location == 5): ## if it is a green container
                arm.move_arm(drop_off_top_list[1][0], drop_off_top_list[1][1], drop_off_top_list[1][2]) ## move it to the "top" green location
            elif (drop_off_location == 3) or (drop_off_location == 6): ## if it is a blue container
                arm.move_arm(drop_off_top_list[2][0], drop_off_top_list[2][1], drop_off_top_list[2][2]) ## move it to the "top" blue location
            break                   #break is used so that the statement executes once when the condition is met, instead of being stuck in an infinite loop (used in subsequent sub functions)



##Renee wrote this function
## accessing the list of drop off locations to know where to place the drawer
## move end effector to the final position IF gripper is closed AND left arm is above threshold AND right arm is fully extended
def move_end_effector_to_drop_off(drop_off_location):
    print("To move the container to the appropriate end position, flex only the left arm.")
    while True:
        if(arm.emg_left() > threshold) and (arm.emg_right() == 0) and (is_gripper_open == False):
            arm.move_arm(drop_off_location[0], drop_off_location[1], drop_off_location[2])
            break



##Calvyn wrote this function
## Moves end effector to pick up coordinates of every container IF left arm is above threshold,
## right arm is fully extended, and gripper is open
def move_end_effector_to_pickup():
    print("Move the arm to the pick up container location by flexing only the left arm.")
    while True:
        if(arm.emg_left() > threshold) and (arm.emg_right() == 0) and (is_gripper_open == True):
            arm.move_arm(0.5321,0,0.035)
            break

        

## Renee wrote this function
## Opens or closes the gripper based on whether it is already open or closed and IF left AND right arm are above threshold
def open_close_gripper():
    global is_gripper_open
    print("To open or close the gripper, flex both arms.")
    while True:
        if (arm.emg_left() > threshold) and (arm.emg_right() > threshold):
            if (is_gripper_open == True): ## if the gripper is open, then...
                arm.control_gripper(45) ## close the gripper
                is_gripper_open = False ## mark the gripper as closed
                break
        
            elif (is_gripper_open == False): ## if the gripper is closed, then...
                arm.control_gripper(-45) ## open the gripper
                is_gripper_open = True ## mark the gripper as open
                break

## Calvyn wrote this function

def toggle_drawer(container):                          #open/closes autoclave drawer
    global draweropen                             #brings global booleon value into function to be edited
    print ('Flex right arm ONLY to open and close drawer') #prompt

    

    while True:
        if (container > 3) and (arm.emg_left() == 0) and (arm.emg_right() > threshold):
            if draweropen == False:      #Similar booleon logic from gripper() function is used 
                if container == 4:                              #reads argument from function parameter to determine color of large container
                    arm.open_red_autoclave(True)
                    draweropen = True                       #booleon is initially false, and set to True so it can be closed later
                    break
                elif container == 5:
                    arm.open_green_autoclave(True)
                    draweropen = True
                    break
                elif container == 6:
                    arm.open_blue_autoclave(True)
                    draweropen = True
                    break
            elif draweropen == True:   #elif segments from first if statement,activates when drawer is open/booleon is True
                if container == 4:
                    arm.open_red_autoclave(False)               #drawer is closed
                    draweropen = False                      #booleon is set to False (like in the beginning) so that it can be opened again
                    break
                elif container  == 5:
                    arm.open_green_autoclave(False)
                    draweropen = False
                    break
                elif container == 6:
                    arm.open_blue_autoclave(False)
                    draweropen = False
                    break

        elif (container < 4):
            print("It is not a large container, and you don't need to close a drawer. Moving on.")
            break

def home():                             #arm moves home
    print("Move the arm home by flexing only the left arm.")
    while True:
        if(arm.emg_left() > threshold) and (arm.emg_right() == 0):
            arm.move_arm(0.4064, 0, 0.4826)
            break

'''
Pseudocode:
    The function needs to perform one of three actions based on the muscle-sensor
    emulator.
    Logically, because the only functions that would "work" together would be the
    control gripper function and the move end-effector function, we should use
    the following scenarios for each function:
    1) When the muscle sensor surpasses a threshold of (we used 0.3) on L side,
            Move the end-effector (BUT R HAS TO BE = 0)
    2) When the muscle sensor surpasses a threshold of (we used 0.3) on R side,
            Open/Close the autoclave drawer (BUT L HAS TO BE = 0)
    3) When the muscle sensor surpasses a threshold of (we used 0.3) on L AND R side,
            Open/Close gripper

'''

## The function below is used for each iteration of a container
def one_cycle(drop_off_location):
    
    arm.spawn_cage(drop_off_location) ## creates a container based on the input
    time.sleep(.25)
    move_end_effector_to_pickup() ## moves arm to pick up a container
    time.sleep(.25)
    open_close_gripper() ## closes the gripper
    time.sleep(.25)
    move_end_effector_to_top(drop_off_location) ## moves the arm to the "top" locations, to avoid hitting other autoclaves
    time.sleep(.25)
    toggle_drawer(drop_off_location) ## opens the drawer if there is a large container
    time.sleep(.25)
    move_end_effector_to_drop_off(drop_off_list[drop_off_location-1]) ## lowers the container to the "final drop position"
    time.sleep(.25)
    open_close_gripper() ## drops the container
    time.sleep(.25)
    if (drop_off_location > 3):
        move_end_effector_to_top(drop_off_location) ## if it is a large container, the arm needs to be moved out of the way of the drawer
    time.sleep(.25)
    toggle_drawer(drop_off_location)         ## closes the drawer if there is a large container
    ## return arm to the home position
    time.sleep(.25)
    home()
    time.sleep(.25)
    


def six_iterations():
    randomized = [i for i in range(1,7)]          #creates a list with 6 elements (1-6)
    shuffle(randomized)                         #with the shuffle function imported from the random module, it shuffles the list in a randomized order
    print(randomized)                        

    for i in randomized:                       #iterates through all 6 elements, using the randomized container ids as input
        time.sleep(1)
        one_cycle(int(i))
        print("")
        print("")
        print("New container being generated...")
        print("")
        print("")

    print("All containers are properly placed.")

six_iterations()

