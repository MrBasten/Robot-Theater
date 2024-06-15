import os
import time

import numpy as np
import pybullet as p
from qibullet import SimulationManager
from qibullet import PepperVirtual


def calculate_robot_moves(robot, robot_2) -> None:
    distance = 0.5
    current_position_robot1 = robot.getPosition()
    current_position_robot2 = robot_2.getPosition()

    # Преобразование глобальных координат робота 2 в локальные координаты робота 1
    local_position_robot2 = [current_position_robot2[0] - current_position_robot1[0], current_position_robot2[1] - current_position_robot1[1]]

    # Вычисление нового положения робота
    new_robot1_position = [local_position_robot2[0] - distance if local_position_robot2[0] >= 0 else local_position_robot2[0] + distance, 
                           local_position_robot2[1] - distance if local_position_robot2[1] >= 0 else local_position_robot2[1] + distance]

    # Установка нового положения робота
    robot.moveTo(new_robot1_position[0], new_robot1_position[1], 0, speed=0.1, _async=True)


def calculate_distance(position1, position2):
    return np.linalg.norm(np.array(position1) - np.array(position2))


def arms(pepper) -> None:
    # Угол, на который нужно опустить руки
    angle = 1.5

    # Опускаем правую руку
    pepper.setAngles('RShoulderPitch', angle, 0.2)

    # Опускаем левую руку
    pepper.setAngles('LShoulderPitch', angle, 0.2)


def check_who_can_talk(pepper1, pepper2, pepper3, threshold, user_input) -> bool:
    positions = [pepper1.getPosition(), pepper2.getPosition(), pepper3.getPosition()]
    talking_pairs = []
    for i in range(3):
        for j in range(i+1, 3):
            distance = calculate_distance(positions[i], positions[j])
            if distance < threshold:
                # Add pair of robots that can talk to the list
                talking_pairs.append((i+1, j+1))

    if len(talking_pairs) == 3 and user_input == 'All':
        return True

    else:
        try:
            robot1, robot2 = map(int, user_input.split(' and '))
        except ValueError:
            print("Invalid input. Enter the numbers of two robots, separated by ' and '.")
            return False
        
        if robot1 in [1, 2, 3] and robot2 in [1, 2, 3]:
            robots = [pepper1, pepper2, pepper3]
            if (robot1, robot2) in talking_pairs or (robot2, robot1) in talking_pairs:
                return True
            else:
                print(f"Robots {robot1} and {robot2} cannot talk.")
                return False
        else:
            print("Invalid input. Enter the numbers of two robots from 1 to 3, separated by ' and '.")
            return False


# def check_who_can_talk(pepper1, pepper2, pepper3, threshold, user_input) -> list:
#     positions = [pepper1.getPosition(), pepper2.getPosition(), pepper3.getPosition()]
#     talking_pairs = []
#     for i in range(3):
#         for j in range(i+1, 3):
#             distance = calculate_distance(positions[i], positions[j])
#             if distance < threshold:
#                 # Add pair of robots that can talk to the list
#                 talking_pairs.append((i+1, j+1))

#     if len(talking_pairs) > 3:
#         print("All three are talking")

#     else:
#         try:
#             robot1, robot2 = map(int, user_input.split(' and '))
#             if robot1 in [1, 2, 3] and robot2 in [1, 2, 3]:
#                 robots = [pepper1, pepper2, pepper3]
#                 if (robot1, robot2) in talking_pairs or (robot2, robot1) in talking_pairs:
#                     for _ in [pepper1, pepper2, pepper3]:
#                         # Perform an action if the distance is less than the threshold
#                         print(f"Robots {robot1} and {robot2} are can talking.")
#                     return True
#                 else:
#                     print(f"Robots {robot1} and {robot2} cannot talk.")
#                     return False
#             else:
#                 print("Invalid input. Enter the numbers of two robots from 1 to 3, separated by ' and '.")
#                 return False
#         except ValueError:
#             print("Invalid input. Enter the numbers of two robots, separated by ' and '.")
#             return False



def talk(pepper1, pepper2, pepper3, talking_pairs, user_input) -> None:
    if user_input == "Все":
        print("All Robots are talking")

    elif user_input != "Все":
        robot1, robot2 = map(int, user_input.split(' and '))
        print(f"Robots {robot1} and {robot2} talk.")

    else:
        print("All cannot talk")
        return False


def commands(pepper1, pepper2, pepper3) -> None:
    # Dictionary to store robots
    robots = {
        '1': pepper1,    
        '2': pepper2,
        '3': pepper3}

    while True:
        user_massage = input("\nEnter command: ")
        # Splitting input into two separate robots

        if user_massage == 'Move':
            user_input = input("Which robot will go to which one? (format input 'robot1 -> robot2'): ")
            robot1, robot2 = user_input.split(' -> ')
            # Checking for robots in the dictionary
            if robot1 in robots and robot2 in robots:    # Calling function to move robots
                calculate_robot_moves(robots[robot1], robots[robot2])    
                time.sleep(15)

            else:    
                print("One or both robots not found.")

        if user_massage == 'Talk':
            user_input = input("Who will talk to whom? (format input 'robot1 and robot2' or 'All'): ")
            threshold_value = 2 # TODO: move value to user-assigned
            if check_who_can_talk(pepper1, pepper2, pepper3, threshold_value, user_input):
                talk(pepper1, pepper2, pepper3, threshold_value, user_input)

        if user_massage == 'Help':
                print("You can enter the following commands: Move, Talk, Stop")

        if user_massage == 'Stop':
            p.disconnect()
            break  # Add break to exit the loop


def main() -> None:
        cwd = os.getcwd()
        os.chdir(cwd)

        simulation_manager = SimulationManager()
        client_id = simulation_manager.launchSimulation(gui=True)

        pepper1 = PepperVirtual()
        pepper2 = PepperVirtual()
        pepper3 = PepperVirtual()

        pepper1.loadRobot(translation=[-1, 1, 0], quaternion=[0, 0, 0, 1], physicsClientId=client_id)
        pepper2.loadRobot(translation=[0, 0, 0], quaternion=[0, 0, 0, 1], physicsClientId=client_id)
        pepper3.loadRobot(translation=[2, 1, 0], quaternion=[0, 0, 0, 1], physicsClientId=client_id)

        peper_list = [pepper1, pepper2, pepper3]

        for k in peper_list:
            arms(k)

        commands(pepper1, pepper2, pepper3)

main()