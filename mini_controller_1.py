#!/usr/bin/env python3

"""mini_controller_1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

TIME_STEP = 64
MAX_SPEED = 6.28

def run_robot(robot):

    timestep = int(robot.getBasicTimeStep())

    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')

    leftMotor.setPotisition(float('inf'))
    leftMotor.setVelocity(0.1 * MAX_SPEED)
    
    rightMotor.setPotisition(float('inf'))
    rightMotor.setVelocity(0.1 * MAX_SPEED)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
        pass
    
        
# Enter here exit cleanup code.

if __name__ == "__main__":
    robot = Robot(robot)
    run_robot()
