# Self Driving Visualization
A localization and A* visualization based on the first 4 lessons of
[Sebastian Thrun's Artificial Inteligence for Robotics](https://classroom.udacity.com/courses/cs373)
course on [Udacity](https://udacity.com). For a full overview of the material, I recommend checking
out this course. However, I will try to describe the techniques and algorithms used in this demo
at a high level.

We simulate an environment with obstacles that the robot must navigate to get to the destination. Our
focus is to use particle filters to predict where exactly the robot is and to use A* to efficiently find
a path to the destination without colliding into obstacles. One simplifying assumption we make is that
unlike a car which must turn, the robot can instantly rotate to any direction.

Note that the code is generally written assuming the user will not try to break the simulation. Thus it
does not gracefully handle most failures and makes assumptions that if broken will cause erroneous
behavior.

## Requirements

1. [Python 2.7.x](https://www.python.org/downloads/)

## Running the Simulation

1. Clone the repository: `git clone https://github.com/akshaynanavati/localization`
2. Navigate to the src directory: `cd src`
3. Run the simulation: `python main.py`
4. Navigate to the tkinter window
5. Click on a white square (note that clicking on an obstacle will likely cause the code to crash). If
   done correctly, this should draw a path to the square you clicked on.
6. Hit space to begin the simulation (note there must be a path to start the simulation). Hit space
   at any time to pause it.

If at any time anything goes wrong, it can usually be fixed by killing and restarting the program (either by closing the window or hitting ctrl-c from the terminal). If the issue is not resolved this way feel free to open an issue or submit
a pull request.

## Demo

Below is an example run of the demo. Note that the orange represents obstacles, white are cells the robot can go
over, the gold circle is the robot, the green dots are the particles, and the red spokes coming out of the robot are the
simulated "lidar" measurements from the robot determining the distance to the closest obstacle.

As you can see, our particles begin far apart but as the robot moves they converge nicely to the robot's precise location.

![alt text](https://media.githubusercontent.com/media/akshaynanavati/localization/master/demo.gif)

## Further Reading

For more information on the code itself and the underlying algorithms, check out the project wiki
[here](https://github.com/akshaynanavati/localization/wiki). If you are still interested in learning more,
head over to the Udacity link above and check out the course.
