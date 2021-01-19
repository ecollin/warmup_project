# warmup_project
## Driving in a Square
### High-level approach
*For each robot behavior, describe the problem and your approach at a high-level. Include any relevant diagrams or pictures that help to explain your approach.*

The high level approach for driving of a square was fairly simple. A square is just two sets of parallel lines, joined with right angles. Our robot just needs to move straight for an equal period of time 4 separate times, and turn 90° between each of these periods of straight movement, and the resulting shape will be a square. (You could also turn -90° as long as you are consistent.) 
### Structure of the code
*Describe the structure of your code. For the functions you wrote, describe what each of them does.*

My code is structured around a class called DriveSquare; the only functions written are the __init__ and run methods. In light of the high-level picture described above, the only ROS topic needed for this movement is /cmd_vel. As such, the __init__ method just registers the node and sets up a publisher to /cmd_vel. The run method then just defines a few parameters that affect the time it takes for the robot to traverse the square (such as the time linear speed used and the time spent drawing the lines at that speed), and then uses a never-ending while loop to actually traverse the square. Inside the while loop are the two steps to continuously draw the lines of a square: first, the linear.x speed is set to the parameter defined earlier, that is published, and the program sleeps for some time, letting the robot move along the line. Then, the linear.x speed is set to 0 and the angular.z is set such that 90 degrees will be rotated in a certain parameterized period of time. The first step also sets angular.z speed to be 0 so that this does not remain when the lines of the square are drawn.

### Relevant GIFs
*While recording your robot's behavior in a rosbag conducting each type of behavior, also record a gif of the robot visually. Include this gif in your writeup and use it for analysis if needed. For instructions on how to make a gif recording, look at Gazebo simulator.*

The first gif below is a gif of the robot moving in a square using my final code. The second gif is a gif of the robot moving in a straight line with no change in velocity over a long distance--it demonstrates the effect of drift on robot trajectory even without changing velocity. The third gif shows the accuracy with which the robot turns 90° at a time in place when it is not moving linearly--for some reason, the angle seems to be ruined when combined with the linear moving, as you can see in the first gif. Note that all three gifs were created using only slight variations of my final code where different lines were commented out, so the differences are not due to different ways of implementing (although the linear speed is higher in the linear_drift gif for demonstrative purposes). 
![gif1](gifs/square_drive.gif)
![gif2](gifs/linear_drift.gif)
![gif3](gifs/turns_in_place.gif)
### Challenges
*Describe the challenges you faced and how you overcame them.*

The first challenge I faced was that, despite going through exercises in class, I still had a very poor understanding of the general ROS framework/how to work with topics and publish messages. I combated this by going over the work we'd done so far thoroughly, and attending Pouya's office hours on Sunday. The office hours were immensely helpful, as exercise 2 in class uses /cmd_vel as well, and after attending I understood the basic ROS framework well.

Before coming to the high-level idea listed above, I conceived of an equivalent idea: for some specified time move in the +x direction, then for the same specified time in the +y direction, then for the same time in the -x direction, and finally for the same period of time in the -y direction. Doing this will create a square if you draw it out. Of course, I ran into lower-level implementation problems here--when I coded it up, the bot would only move in the +x direction and -x direction and not the +y or -y. I asked on slack about this and Jason Lin explained: "The bot can only move forward/backward and spin around because its wheels are fixed to the chassis." I then went back to the drawing board, conceived the high-level idea above, and implemented that.

The final challenge I faced was dealing with the fact that the turtlebot emulates the real world and is not perfect--there is drift, for instance. Despite, I believe, implementing the correct logic behind the high-level idea described above, my turtlebot did not move in a perfect square, as you can see in the gif above. I spent a long time trying to figure out the cause of this. One thing I did, for instance, is import pi from python rather than just using 3.14, to improve the accuracy when turning. In the end, I settled for the resulting gif above. In other experiments, I found that my code with just a few lines changed could cause the turtlebot to rotate seemingly perfectly by 90° at a time IN PLACE, and that it could move approximately linearly, but that drift occured over larger distances/speeds. But, putting together I supposed compounded errors and resulting in the imperfect gif above. I did not quite overcome this challenge, but I did improve upon it and come to understand it better. 

### If I Had More Time, How Would I Improve the Behavior?
*If you had more time, how would you improve your robot behaviors?*

As evidenced by the gif above, my robot doesn't really move in in a square that well. For some reason, combining turning and moving forward leads to the angle turned being wrong. If I had more time, this is what I would spend most of my time on. After thinking about my code for some time, and messaging Pouya on slack, I think that the error with the way I've done it might just be due to the fact that you can only get so accurate moving in a square by relying on rospy.sleep to turn/move for a certain angle or distance. So, I would probably try an entirely different approach using odometry. 

### Key Takeaways 
*What are your key takeaways from this project that would help you/others in future robot programming assignments? For each takeaway, provide a few sentences of elaboration.*

Working with ROS topics: as mentioned above, starting this project I had a poor grasp of how to handle ROS topics--how to gather information on them, publish to them, and subscribe to them. In having to work on this basic example of robot behavior, I had to learn all about these. While I'm sure I still have a lot to learn, perhaps my biggest takeaway from this will be the understanding of these topics I gained in working through the project

My other big takeaway from this project is that when working with robotics--even if you're just simulating them--you have to deal with real world noise and imperfections. It is very easy to draw out a diagram of a square with perfect 90° angles and say you just need to make the robot move for the same amount of time when going on each line and making the angles--as I did in my high-level summary--but in the real world there is drift and other factors such that even if you code things up well you won't get the result you want. 
