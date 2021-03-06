Solve a Maze
============

[![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/from-referrer)

# Assignment
Design and implement an autonomous driving system that will move the craft toward the target while avoiding the obstacles.

![maze](/assets/maze.png)

To solve the quest you need to know the following things.

### The maze
The maze is a **2D square area** containing the target, the craft and a number of
obstacles. The **origin `(0,0)`** is located at the bottom-left corner of the picture
above, whereas the x-axis is oriented left->right the y-axis is oriented down->up.

### The craft
The craft is equipped with **2 motors** that enable clock/counter-clock wise rotation
so as forward/backward linear motion. You can control the **speed** of the motors
independently as a percentage of the full-scale, i.e. **`[-100%, 100%]`**.
The motors exhibit their own dynamics, therefore e.g. the craft cannot come to a
complete stop at once.

In addition, a **radar** is available that allows acquiring a complete **scan of the maze** plus the **odometry of the craft**.

### The obstacles
The obstacles are always in a **fixed number**, **circular**, of **different size** and **located randomly**
within the maze.

### Driving the craft
It is required to send a `yarp::os::Bottle` to the port `/assignment_solve-maze-handler/motor:i` containing two double representing the angular and the linear speed:
```c++
yarp::os::Bottle &speed=portMotor.prepare();
speed.clear();
speed.addDouble(-60.0); // angular speed in percentage [-100%, 100%]
speed.addDouble(100.0); // linear speed in percentage [-100%, 100%]
port.write();
```

### Scanning the maze
It is required to receive a `yarp::os::Property` from the port `/assignment_solve-maze-handler/radar:o` containing the up-to-date results of the scan and odometry in the following format:
```
(length <l>) (time <t>) (craft (<x> <y> <d>)) (target (<x> <y> <r>)) (obstacles ((<x> <y> <r>) (<x> <y> <r>) ...)) (state <s>)
```
Where:
- `<l>` represents the length of the maze wall (integer).
- `<t>` is the current time in seconds (double).
- `<x>`, `<y>` are the coordinates of the object within the maze (double).
- `<d>` accounts for the current craft direction in `[0, 360]` degrees (double).
- `<r>` represents the radius of the object (double).
- `<s>` is the current state of the maze: one of {`"running"`, `"crashed"`, `"expired"`, `"solved"`} (string).

### Crashing & accomplishing the task
The craft is considered to be a **point** and can crash into obstacles as well as
into maze walls. By contrast, the maze is solved whenever the craft is driven
into the target area (still circular).

## How to proceed
We provide in [**`src/craft.cpp`**](./src/craft.cpp) a code controlling the craft that performs a very basic random
search. Will you be able to modify the code to implement a better strategy? :wink:

Once done, you can test your code in two ways:

1. **Manually**: running the _yarpmanager scripts_ provided from within [**app/scripts**](./app/scripts). This will help you refine your solution.
1. **Automatically**: [running the script **test.sh**](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-run-smoke-tests.md) in the **smoke-test** directory. In this case, time limitation
to solve the maze will be activated.

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
