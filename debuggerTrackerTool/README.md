# IEEE Robot Autonomous Debugger/Tracking Tool
By Skylar Araujo and Danny Johnson

# NOTE!
**The Raspberry Pi 4b is connected to Skylar Araujo's github account 'hurpo', hence why there are far more commits from that account compared to Danny's. However, we both pushed commits through the Raspberry Pi 4b, so be aware that the commit history is not an accurate idicator of how we split the work.**

## Introduction to our Project
This is our midterm project repo for ECEN-4293 Python with Numerical Methods. We tied our midterm into our senior design capstone project. Our capstone project objective is to build a fully autonomous robot to score the most amount of points in the IEEE R5 Conference Robotics Competition. While programming the autonomous state machine for our robot, we saw the need to track certain variable values, sensor data, and current state in the autonomous process. We saw our python midterm as an opportunity to create a much needed debugging tool for our capstone project. We had the following objective for our program:

- Wirelessly connect client (our software) to the server (the robot) over a network (eduroam).
- Connection error handling
- Data transfer accross the connection to display the following robot information on the GUI:
    - Camera Feed
    - Robot position and orientation on the game field estimate
    - Robot's current state 
    - Other information from the robot (Sensor data, saved data from auto route)
- Graceful disconnection
- State machine controls:
    - Start
    - Stop
    - Pause
    - Resume

Since this project ties into our capstone project, a lot of the server side code relies on files on our robot. For example, in server.py a new thread is created for the robots state machine, which it imports from game.py located in the robots directory. Robot files can be found on our capstone repo linked below:

https://github.com/hurpo/capstone_project_S26

## How to Use Our Software
1. Run server.py on the robot
    This can be done by directly interfacing with the Raspberry Pi 4b on the robot, or by SSHing into it to start it remotely. The venv located in capstone_project_S26 will need to be activated first. Use python interpretor under the capstone_project_S26 to run /PythonProject1/Server/server.py.
2. Launch the client.
3. Enter the IP address.
    The robots ip address can be found by running ifconfig in the terminal on the Raspberry Pi 4b.
4. Press 'Connect'.
If the connection was successful you will be transferred to the interface.

## Data Displayed
- Robot marker plotted on map*
- Position and Orientation values for robot position on map*
- Encoder values
- Current State in state machine
- Remaining data is displayed in a list
- Camera feed

## Controls
- Start
    Starts the state machine in the INIT state.
- Stop
    Stops the state machine and disconnects from the server.
- Pause
    Pauses between states.
- Resume
    Resumes to the next state.
- Command Line Interface
    Commands can typed and sent to the server. This gives the flexibility to add new commands in the future, without the need to create new GUI for every command added.
