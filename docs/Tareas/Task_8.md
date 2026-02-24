[Home](../index.md)

# Ros Basics Capstone

## 1.- Instructions

### Turtlesim “Catch Them All” Project

You’ll build a small multi-node ROS 2 application using turtlesim to visualize behavior.

### How to work

1. Design first: draw the nodes, topics, services, and what each one does.

2. Implement step-by-step.

### Nodes

1. turtlesim_node (from turtlesim package)

2. turtle_controller (custom) — controls turtle1

3. turtle_spawner (custom) — spawns and manages “alive” turtles

Create a new package, e.g. turtlesim_catch_them_all, to store your nodes.

### turtle_spawner requirements

* Uses turtlesim services:

    - Calls /spawn to create turtles at random (x, y, theta)

    - Calls /kill to remove turtles

* Maintains an array/list of alive turtles (name + pose used at spawn time)

* Publishes alive turtles on /alive_turtles (TurtleArray)

* Provides service /catch_turtle (CatchTurtle) which:

    - Receives turtle name to catch

    - Calls /kill

    - Removes that turtle from the alive list

    - Republishes updated /alive_turtles

* Spawn coordinates: choose random x, y in [0.5, 10.5] (recommended) and theta in [0, 2π].
turtle_controller requirements:

    - Subscribes to /turtle1/pose

    - Publishes velocity commands to /turtle1/cmd_vel

    - Runs a control loop (timer, high rate) implementing a simplified P controller to reach a target turtle.

    - Subscribes to /alive_turtles and selects a target turtle to catch.

    - When turtle1 reaches the target (e.g. distance < 0.3), calls /catch_turtle with the target name.

### Custom interfaces (suggested)

Create these in my_robot_interfaces:

**Turtle.msg**:

* string name

* float32 x

* float32 y

* float32 theta

**TurtleArray.msg**

* Turtle[] turtles
CatchTurtle.srv

* Request: string name

* Response: bool success

**Parameters**

* /turtle_spawner

    - spawn_frequency (float)

    - turtle_name_prefix (string)

* /turtle_controller

### Suggested implementation steps

1. Controller basics: subscribe to /turtle1/pose, implement control loop to reach an
arbitrary fixed goal, publish /turtle1/cmd_vel.

2. Spawner basics: timer spawns turtles at a rate by calling /spawn.

3. Alive list + topic: spawner maintains list and publishes /alive_turtles; controller
subscribes and targets the first turtle.

4. Catch service: controller calls /catch_turtle when close; spawner kills turtle,
updates list, republishes.

5. Launch + params: create YAML + launch file to start all three nodes.

## 2.- turtle sim node

We don't have to create this node, this ons already exist and we will only use it to extract infromation like **pose**, **turtle names**, **kill** and **spawn**.

For calling this node we will put on the ubuntu terminal the next:

```bash
ros2 run turtlesim turtlesim_node
```

![WIP](../recursos/imgs/Task_8/turtlesim.jpeg)

## 3.- msg and srv files (interfaces)

Before creating the nodes we will make the 3 files that act like our interfaces for saving information.

**TurtleArray.msg**

Save the name of the turtles spawned so that later the function **kill** can use them to make the kill action. **Turtle** is theh type of data and **turtles** the name of the sapawned turtles.

```msg
Turtle[] turtles
```

**Turtle.msg**

Save the position of the turtle1 to later use it to move the turtle1 next to the other turtles.

```msg
string name
float32 x
float32 y
```

**CatchTurtle.srv**

Compares and verify that the turtle that we wanted was the one that we caught.

```srv
string turtle_name
---
string caught_turtle
```

## 4.- turtle_controller

### Libraries imported

Importing Modules and Packages. This were 
extracted from turtlesim, python and a package that you must have to create to make the previos interfaces.

```python
#!/usr/bin/env python3

from turtle import distance

import rclpy
from rclpy.node import Node
import math
import numpy as np

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from joseph_interfaces.msg import TurtleArray 
from joseph_interfaces.srv import CatchTurtle
```

### Define the name of the node and the functions

Node name: **TurtleController**

```python
class TurtleController(Node):
```

functions:

**def __init__(self):**

Initialize the node and establish how it will communicate with others.

- State Variables: Define x_target, y_target, and pose as None to ensure the robot doesn't move until it receives real data.

- Subscribers: Listen to turtle1/pose to find out the robot's location.

- Listen to alive_turtles to receive the list of available targets.

- Publisher: Create the turtle1/cmd_vel channel to send move commands.

- Client: Create a client for the catch_turtle service, which is the action of "killing" or eliminating the target turtle.

```python
    def __init__(self):
        super().__init__('turtle_controller')
        self.x_target = None
        self.y_target = None
        self.target_name = None # Added initialization
        self.pose = None
        
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.alive_subscriber = self.create_subscription(TurtleArray, 'alive_turtles', self.alive_callback, 10)
        self.client_catch = self.create_client(CatchTurtle, 'catch_turtle')
        
        self.is_catching = False
        self.accumulated_error = 0
        self.get_logger().info('Turtle Controller Node has been started.')
```

**def pose_callback(self, msg):**

This logic runs repeatedly each time the main turtle reports its position. It contains the following control logic:

- Error calculation: Calculates the distance and angle needed to reach the target using arctan² and hypot.

- Angle normalization: The line (angle_error + np.pi) % (2 * np.pi) - np.pi is a funtion to turn degrees to radians (avoiding unnecessary turns greater than 180°).

Here is how the logic works:

- If the distance is less than 0.3, the turtle stops and calls for capture.

- If the angle error is large, the turtle stops and only rotates on its own axis.

- If the angle is correct, the turtle moves forward using PI (Proportional-Integral) control to adjust its linear speed.

```python
def pose_callback(self, msg):
        self.pose = msg
        
        # Safety check: Wait for a target to be assigned
        if self.x_target is None:
            return

        twist = Twist()
        dy = self.y_target - msg.y
        dx = self.x_target - msg.x
        target_angle = np.arctan2(dy, dx)
        
        angle_error = target_angle - msg.theta
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
        distance = math.hypot(dx, dy)
        
        if distance <= 0.3:
            # Arrived! Stop and reset
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.accumulated_error = 0
            self.get_logger().info(f"Target {self.target_name} reached!")
            # TODO: Add catch service call here
            if not self.is_catching:
                self.is_catching = True
                self.call_catch_turtle_service(self.target_name)
    
                
            
        elif abs(angle_error) <= 0.3:
            # Facing target: Move forward
            self.accumulated_error += distance
            ki_linear = 0.01
            kp_linear = 2.0
            twist.linear.x = (kp_linear * distance) + (ki_linear * self.accumulated_error)
            twist.angular.z = 0.0
        else:
            # Wrong way: Rotate in place
            twist.linear.x = 0.0
            twist.angular.z = 2.0 * angle_error
            
        self.cmd_vel_publisher.publish(twist)
```

**def call_catch_turtle_service(self, turtle_name):**

It is activated each time the Spawner node publishes the list of live turtles.

- Nearest Neighbor: It traverses the entire list of turtles (msg.turtles) and calculates which one is closest to the turtle1 current position.

- Assignment: If it finds a turtle# and the turtle1 is not busy capturing another (if not self.is_catching), it updates the target coordinates (x_target, y_target).

```python
def call_catch_turtle_service(self, turtle_name):
        request = CatchTurtle.Request()
        request.turtle_name = turtle_name
        future = self.client_catch.call_async(request)
        future.add_done_callback(self.callback_catch_turtle)
```

**def callback_catch_turtle(self, future):**

It's a helper function for making the service call asynchronously, preparing the request with the name of the turtle we want to delete.

- Uses **call_async** so the code doesn't freeze while waiting for the other node to respond.

- Adds a callback that will only be triggered when the service confirms the turtle has died.

```python
def callback_catch_turtle(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle {response.caught_turtle} has been caught!")
            
            # Reset our state
            self.is_catching = False
            self.x_target = None
            self.y_target = None
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
```

**def alive_callback(self, msg):**

It only runs when **CathTurtle** service responds.

Prints to the terminal that the turtle was successfully captured and reset to set **is_catching** to **False** and clears the targets. This tells the node that it is now free to search for the next turtle in the list.

```python
def alive_callback(self, msg):
        # Safety check: Wait until we know our own position
        if self.pose is None or len(msg.turtles) == 0:
            return

        closest_turtle = None
        min_dist = float('inf')

        for turtle in msg.turtles:
            dist = math.hypot(turtle.x - self.pose.x, turtle.y - self.pose.y)
            if dist < min_dist:
                min_dist = dist
                closest_turtle = turtle

        if closest_turtle is not None:
            if not self.is_catching:  # Only update target if we're not currently catching
                self.x_target = closest_turtle.x
                self.y_target = closest_turtle.y
                self.target_name = closest_turtle.name
```

### Keep the node on loop and activate it

```python
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run it we will have to put on the ubuntu terminal this:

```bash
ros2 run myrobot_pkg turtle_controller
```

![WIP](../recursos/imgs/Task_8/controller.jpeg)

Complete code

```python
#!/usr/bin/env python3

from turtle import distance

import rclpy
from rclpy.node import Node
import math
import numpy as np

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from joseph_interfaces.msg import TurtleArray
from joseph_interfaces.srv import CatchTurtle

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.x_target = None
        self.y_target = None
        self.target_name = None # Added initialization
        self.pose = None
        
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.alive_subscriber = self.create_subscription(TurtleArray, 'alive_turtles', self.alive_callback, 10)
        self.client_catch = self.create_client(CatchTurtle, 'catch_turtle')
        
        self.is_catching = False
        self.accumulated_error = 0
        self.get_logger().info('Turtle Controller Node has been started.')

    def pose_callback(self, msg):
        self.pose = msg
        
        # Safety check: Wait for a target to be assigned
        if self.x_target is None:
            return

        twist = Twist()
        dy = self.y_target - msg.y
        dx = self.x_target - msg.x
        target_angle = np.arctan2(dy, dx)
        
        angle_error = target_angle - msg.theta
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
        distance = math.hypot(dx, dy)
        
        if distance <= 0.3:
            # Arrived! Stop and reset
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.accumulated_error = 0
            self.get_logger().info(f"Target {self.target_name} reached!")
            # TODO: Add catch service call here
            if not self.is_catching:
                self.is_catching = True
                self.call_catch_turtle_service(self.target_name)
    
                
            
        elif abs(angle_error) <= 0.3:
            # Facing target: Move forward
            self.accumulated_error += distance
            ki_linear = 0.01
            kp_linear = 2.0
            twist.linear.x = (kp_linear * distance) + (ki_linear * self.accumulated_error)
            twist.angular.z = 0.0
        else:
            # Wrong way: Rotate in place
            twist.linear.x = 0.0
            twist.angular.z = 2.0 * angle_error
            
        self.cmd_vel_publisher.publish(twist)

    def call_catch_turtle_service(self, turtle_name):
        request = CatchTurtle.Request()
        request.turtle_name = turtle_name
        future = self.client_catch.call_async(request)
        future.add_done_callback(self.callback_catch_turtle)

    def callback_catch_turtle(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle {response.caught_turtle} has been caught!")
            
            # Reset our state
            self.is_catching = False
            self.x_target = None
            self.y_target = None
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def alive_callback(self, msg):
        # Safety check: Wait until we know our own position
        if self.pose is None or len(msg.turtles) == 0:
            return

        closest_turtle = None
        min_dist = float('inf')

        for turtle in msg.turtles:
            dist = math.hypot(turtle.x - self.pose.x, turtle.y - self.pose.y)
            if dist < min_dist:
                min_dist = dist
                closest_turtle = turtle

        if closest_turtle is not None:
            if not self.is_catching:  # Only update target if we're not currently catching
                self.x_target = closest_turtle.x
                self.y_target = closest_turtle.y
                self.target_name = closest_turtle.name

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.- turtle_spawner

### Libraries imported

Importing Modules and Packages. This were 
extracted from turtlesim, python and a package that you must have to create to make the previos interfaces.

```python
#mp!/usr/bin/env python3

from random import uniform



import rclpy

from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from joseph_interfaces.msg import Turtle
from joseph_interfaces.msg import TurtleArray
from joseph_interfaces.srv import CatchTurtle
```

### Define the name of the node and the functions

Node name: **TurtleSpawner**

```python
class TurtleSpawner(Node):
```

functions:

**def __init__(self):**

Configure the necessary tools to manage the turtle ecosystem:

- Service Clients: Create two clients (spawn and kill) to communicate directly with the original turtlesim node.

- Timer: Set a timer that automatically calls the spawn_turtle function every 2.0 seconds.

- Publisher: Create the alive_turtles topic. This is vital because this is where the Controller learns which turtles are being targeted.

- Service Server: Create the catch_turtle service, this node provides the service that the controller uses when it encounters a turtle.

```python
def __init__(self):
        super().__init__('turtle_spawner')
        self.spawn_client= self.create_client(Spawn, 'spawn')
        self.kill_client= self.create_client(Kill, 'kill')
        self.get_logger().info('Turtle Spawner Node has been started.')
        self.create_timer(2.0, self.spawn_turtle) #para bajar tiempo de aparicion
        self.alive_turtles = []
        self.alive_turtles_publisher= self.create_publisher (TurtleArray, 'alive_turtles', 10)
        self.server_catch = self.create_service(CatchTurtle, 'catch_turtle', self.catch_turtle_callback)
```

**def spawn_turtle(self):**

Generate the data for a new turtle:

- Use uniform(0.5, 10.5) to choose a random position within the visible limits of the turtlesim window.

- Send an asynchronous request to the turtlesim service to physically display a turtle on the screen.

- Temporarily store the x and y coordinates so they can be recalled when the turtle officially appears.

```python
def spawn_turtle(self):
        request = Spawn.Request()
        request.x = uniform(0.5, 10.5)
        request.y = uniform(0.5, 10.5)
        request.theta = uniform(0, 3.14159)
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        self.last_spawn_x = request.x
        self.last_spawn_y = request.y
```

**def spawn_callback(self, future):**

Runs when Turtlesim confirms the turtle has been created:

- Registration: Creates a Turtle-type message (from your custom interfaces) with the name Turtlesim assigned and the saved coordinates.

- Living List: Adds this new turtle to the **self.alive_turtles list**.

- Notification: Publishes the updated list to the alive_turtles topic so the Controller knows there's a new target on the map.

```python
def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Turtle spawned with name: {response.name}')
            Turtle_msg = Turtle()
            Turtle_msg.name = response.name
            Turtle_msg.x = self.last_spawn_x
            Turtle_msg.y = self.last_spawn_y
            self.alive_turtles.append(Turtle_msg)

            # 1. Create the 'envelope' (The Array Message)
            msg = TurtleArray()
            # 2. Put your 'storage box' (the list) into the envelope's 'turtles' field
            msg.turtles = self.alive_turtles
            # 3. Mail it!
            self.alive_turtles_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {e}')
```

**def catch_turtle_callback (self, request, response):**

Coordinates the removal:

- List Cleanup: Uses a Python list comprehension ([t for t in .. if t.name != ..]) to remove the captured turtle from the program's memory.

-Update: Immediately publishes the updated turtle list (now without the dead turtle) so that no one else tries to capture it.

- Kill Call: Sends the command to the Turtlesim /kill service to visually remove the turtle from the screen.

- Response: Confirms to the Controller that the turtle was successfully processed.

```python
def catch_turtle_callback (self, request, response): 
            response.caught_turtle = request.turtle_name
            self.alive_turtles = [t for t in self.alive_turtles if t.name != request.turtle_name]
            TurtleArray_msg = TurtleArray()
            TurtleArray_msg.turtles = self.alive_turtles
            self.alive_turtles_publisher.publish(TurtleArray_msg)
            Kill_request = Kill.Request()
            Kill_request.name = request.turtle_name
            future = self.kill_client.call_async(Kill_request)
            future.add_done_callback(self.kill_callback)
            return response
```

**def kill_callback(self, future):**

Checks if the call to the Turtlesim "kill" service completed successfully. If there was an error, it reports it in the log so that it appears in the terminal.

```python
def kill_callback(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Kill service call failed: {e}")
```

### Keep the node on loop and activate it

```python
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner() # Initialize your class
    rclpy.spin(node)       # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
```bash
ros2 run myrobot_pkg turtle_spawner
```

![WIP](../recursos/imgs/Task_8/spawn.jpeg)

Here is the complete code:

```python
#mp!/usr/bin/env python3

from random import uniform



import rclpy

from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from joseph_interfaces.msg import Turtle
from joseph_interfaces.msg import TurtleArray
from joseph_interfaces.srv import CatchTurtle

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.spawn_client= self.create_client(Spawn, 'spawn')
        self.kill_client= self.create_client(Kill, 'kill')
        self.get_logger().info('Turtle Spawner Node has been started.')
        self.create_timer(2.0, self.spawn_turtle) #para bajar tiempo de aparicion
        self.alive_turtles = []
        self.alive_turtles_publisher= self.create_publisher (TurtleArray, 'alive_turtles', 10)
        self.server_catch = self.create_service(CatchTurtle, 'catch_turtle', self.catch_turtle_callback)

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = uniform(0.5, 10.5)
        request.y = uniform(0.5, 10.5)
        request.theta = uniform(0, 3.14159)
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        self.last_spawn_x = request.x
        self.last_spawn_y = request.y
        
    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Turtle spawned with name: {response.name}')
            Turtle_msg = Turtle()
            Turtle_msg.name = response.name
            Turtle_msg.x = self.last_spawn_x
            Turtle_msg.y = self.last_spawn_y
            self.alive_turtles.append(Turtle_msg)

            # 1. Create the 'envelope' (The Array Message)
            msg = TurtleArray()
            # 2. Put your 'storage box' (the list) into the envelope's 'turtles' field
            msg.turtles = self.alive_turtles
            # 3. Mail it!
            self.alive_turtles_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {e}')

    def catch_turtle_callback (self, request, response): 
            response.caught_turtle = request.turtle_name
            self.alive_turtles = [t for t in self.alive_turtles if t.name != request.turtle_name]
            TurtleArray_msg = TurtleArray()
            TurtleArray_msg.turtles = self.alive_turtles
            self.alive_turtles_publisher.publish(TurtleArray_msg)
            Kill_request = Kill.Request()
            Kill_request.name = request.turtle_name
            future = self.kill_client.call_async(Kill_request)
            future.add_done_callback(self.kill_callback)
            return response

    def kill_callback(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Kill service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner() # Initialize your class
    rclpy.spin(node)       # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.- CMakeLists.txt

Add the 3 files of the interfaces to the **rosidl_generate_interfaces(${PROJECT_NAME}**

```txt
"msg/Turtle.msg"
"msg/TurtleArray.msg"
"srv/CatchTurtle.srv"
```

## 7.- package.xml

Add the **depend** and **test_depend** to it's own necesary group, like the turtle sim or our interfaces.

```xml
  <depend>rclpy</depend>
  <depend>example_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>functools</depend>
  <depend>threading</depend>
  <depend>joseph_interfaces</depend>
  <depend>turtlesim</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
```

## 8.- setup.py

Add the 2 names of the nodes that had been created on the **entry_point**

```python
'turtle_controller= myrobot_pkg.turtle_controller:main',
            'turtle_spawner= myrobot_pkg.turtle_spawner:main',
```
## 9.- Launch the 3 nodes

For this part we will have to create another package, then in the folder **launch**  you will find create a file **xml** ours will be called **app_launch.xml** and then add on which node is the code and the name of your node.

```xml
<launch>
    <node pkg="turtlesim" exec="turtlesim_node"/>
    <node pkg="myrobot_pkg" exec="turtle_controller"/>
    <node pkg="myrobot_pkg" exec="turtle_spawner"/>
</launch>
```
Then you will have to save and colcon build, we will launch our 3 nodes with this line

```bash
ros2 launch myrobot_bringup app_launch.xml
```

Here is how it looks

![WIP](../recursos/imgs/Task_8/launch.jpeg)


## 10.- Results (Terminal and rqt_graph)

Once the nodes are finished we can open with the launch line or we will open 4 ubuntu terminals in this order

1. Run turtle sim.

```bash
ros2 run turtlesim turtlesim_node
```

2. Run the spawner.

```bash
ros2 run myrobot_pkg turtle_spawner
```

3. Run the controller.

```bash
ros2 run myrobot_pkg turtle_controller
```

4. Graph

```bash
rqt_graph
```

Now when we run first 2 nodes, the turtle sim terminal appears with new information about the turtles spawned.

![WIP](../recursos/imgs/Task_8/turtles_spawned.jpeg)

This is the result while running the 4 terminals.

![WIP](../recursos/imgs/Task_8/turtles_result.jpeg)
