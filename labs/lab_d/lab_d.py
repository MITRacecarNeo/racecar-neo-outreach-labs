"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: lab_d.py

Title: Lab D - Driving in Shapes

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Create a script to enable semi-autonomous driving for the RACECAR. Button presses
enable a series of instructions sent to the RACECAR, which enable it to drive in various shapes.
Complete the lines of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the "A" button is pressed, drive in a circle
- When the "B" button is pressed, drive in a square
- When the "X" button is pressed, drive in a figure eight
- When the "Y" button is pressed, drive in any shape of your choice
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# A queue of driving steps to execute
# Each entry is a list containing (time remaining, speed, angle)
queue = []

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # Begin at a full stop
    rc.drive.stop()

    # Begin with an empty queue
    queue.clear()

    # Print start message
    # TODO Part 1: Add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = drive in a circle\n"
        "   B button = drive in a square\n"
        "   X button = drive in a figure eight\n"
        "   Y button = drive in a <shape of your choice>\n"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global queue

    # When the A button is pressed, add instructions to drive in a circle
    if rc.controller.was_pressed(rc.controller.Button.A):
        drive_circle()

    # When the B button is pressed, add instructions to drive in a square
    if rc.controller.was_pressed(rc.controller.Button.B):
        drive_square()

    # When the X button is pressed, add instructions to drive in a figure eight
    if rc.controller.was_pressed(rc.controller.Button.X):
        drive_figure_eight()

    # When the Y button is pressed, add instructions to drive in our chosen shape
    if rc.controller.was_pressed(rc.controller.Button.Y):
        drive_chosen_shape()

    # TODO Analyze the following code segment that executes instructions from the queue.
    # Determine how the script processes the instructions and then sends the correct speed
    # and angle commands to the RACECAR.

    # If the queue is not empty, follow the current drive instruction
    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        if queue[0][0] <= 0:
            queue.pop(0)

    # Send speed and angle commands to the RACECAR
    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the RACECAR to drive in a circle
def drive_circle():
    global queue

    # Tune these constants until the car completes a full circle
    CIRCLE_TIME = ___
    BRAKE_TIME = ___

    queue.clear()

    # TODO Part 2: Append the correct variables in the correct order in order
    # for the RACECAR to drive in a perfect circle
    queue.append([_____, 1, 1])
    queue.append([_____, -1, 1])

# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the RACECAR to drive in a square
def drive_square():
    global queue

    # TODO Part 3: Create constants that represent the RACECAR driving through
    # different parts of the square (ex: first slower edge, corners, fast edges)
    
    queue.clear()

    # TODO Part 4: Append the instructions into the queue that represent the RACECAR
    # driving in a perfect square.
    

# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the RACECAR to drive in a figure 8
def drive_figure_eight():
    global queue

    # TODO Part 5: Create constants that represent the RACECAR driving through
    # different parts of the figure 8, and then append the instructions in the
    # correct order into the queue for execution

    queue.clear()


# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the RACECAR to drive in the shape of your choice
def drive_chosen_shape():
    global queue

    # TODO Part 6: Create constants that represent the RACECAR driving through
    # different parts of the shape, and then append the instructions in the
    # correct order into the queue for execution

    queue.clear()


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
