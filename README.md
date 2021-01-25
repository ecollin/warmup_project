# warmup_project
## Driving in a Square
### High-level approach
*For each robot behavior, describe the problem and your approach at a high-level. Include any relevant diagrams or pictures that help to explain your approach.*

The high level approach for driving in a square was fairly simple. A square is just two sets of parallel lines, joined with right angles. Our robot just needs to move straight for an equal period of time 4 separate times, and turn 90° between each of these periods of straight movement, and the resulting shape will be a square. (You could also turn -90° as long as you are consistent.) 
### Structure of the code
*Describe the structure of your code. For the functions you wrote, describe what each of them does.*

My code is structured around a class called DriveSquare; the only functions written are the __init__ and run methods. In light of the high-level picture described above, the only ROS topic needed for this movement is /cmd_vel. As such, the __init__ method just registers the node and sets up a publisher to /cmd_vel. The run method then just defines a few parameters that affect the time it takes for the robot to traverse the square (such as the linear speed used and the time spent moving in the lines of the square at that speed), and then uses a never-ending while loop to actually traverse the square. Inside the while loop are two steps to continuously draw the lines of a square: first, the linear.x speed is set to the parameter defined earlier, that is published, and the program sleeps for some time, letting the robot move along the line. Then, the linear.x speed is set to 0 and the angular.z is set such that 90 degrees will be rotated in a certain parameterized period of time, which is then waited for. The first step also sets angular.z speed to be 0 so that this does not remain when the lines of the square are drawn.

### Relevant GIFs
*While recording your robot's behavior in a rosbag conducting each type of behavior, also record a gif of the robot visually. Include this gif in your writeup and use it for analysis if needed. For instructions on how to make a gif recording, look at Gazebo simulator.*

The first gif below is a gif of the robot moving in a square using my final code. The second gif is a gif of the robot moving in a straight line with no change in velocity over a long distance--it demonstrates the effect of drift on robot trajectory even without changing velocity. The third gif shows the accuracy with which the robot turns 90° at a time in place when it is not moving linearly--for some reason, the angle seems to be ruined when combined with the linear moving, as you can see in the first gif. Note that all three gifs were created using only slight variations of my final code where different lines were commented out, so the differences are not due to different ways of implementing (although the linear speed is higher in the linear_drift gif for demonstrative purposes). 
![gif1](gifs/square_drive.gif)
![gif2](gifs/linear_drift.gif)
![gif3](gifs/turns_in_place.gif)
### Challenges
*Describe the challenges you faced and how you overcame them.*

The first challenge I faced was that, despite going through exercises in class, I still had a very poor understanding of the general ROS framework/how to work with topics and publish messages. I combated this by going over the work we'd done so far thoroughly, and attending Pouya's office hours on Sunday. The office hours were immensely helpful, as exercise 2 in class uses /cmd_vel as well, and after attending I understood the basic ROS framework well.

Before coming to the high-level idea listed above, I conceived of another idea: for some specified time move in the +x direction, then for the same specified time in the +y direction, then for the same time in the -x direction, and finally for the same period of time in the -y direction. Doing this will create a square if you draw it out. Of course, I ran into lower-level implementation problems here--when I coded it up, the bot would only move in the +x direction and -x direction and not the +y or -y. I asked on slack about this and Jason Lin explained: "The bot can only move forward/backward and spin around because its wheels are fixed to the chassis." I then went back to the drawing board, conceived the high-level idea above, and implemented that.

The final challenge I faced was dealing with the fact that the turtlebot emulates the real world and is not perfect--there is drift, for instance. Despite, I believe, implementing the correct logic behind the high-level idea described above, my turtlebot did not move in a perfect square, as you can see in the gif above. I spent a long time trying to figure out the cause of this. One thing I did, for instance, is import pi from python rather than just using 3.14, to improve the accuracy when turning. In the end, I settled for the resulting gif above. In other experiments, I found that my code with just a few lines changed could cause the turtlebot to rotate seemingly perfectly by 90° at a time IN PLACE, and that it could move approximately linearly (both of these are displayed in gifs above), although drift occured over larger distances/speeds. But, putting together I supposed compounded errors resulting in the imperfect gif of my final code version above. I did not quite overcome this challenge to perfectly move in a square, but I did improve upon facing it and learn from it.

### If I Had More Time, How Would I Improve the Behavior?
*If you had more time, how would you improve your robot behaviors?*

As evidenced by the gif above, my robot doesn't really move in in a square that well. For some reason, combining turning and moving forward leads to the angle turned being wrong. If I had more time, this is what I would spend most of my time on. After thinking about my code for some time, and messaging Pouya on slack, I think that the error with the way I've done it might just be due to the fact that you can only get so accurate moving in a square by relying on rospy.sleep to turn/move for a certain angle or distance. I think that small differences due to noise will compound into an inaccurate path, hence my problems. So, I would probably try an entirely different approach using odometry to improve further.

### Key Takeaways 
*What are your key takeaways from this project that would help you/others in future robot programming assignments? For each takeaway, provide a few sentences of elaboration.*

Working with ROS topics: as mentioned above, starting this project I had a poor grasp of how to handle ROS topics--how to gather information on them, publish to them, and subscribe to them. In having to work on this basic example of robot behavior, I had to learn all about these. While I'm sure I still have a lot to learn, perhaps my biggest takeaway from this will be the understanding of these topics I gained in working through the project

Imprefect robotics: my other big takeaway from this project is that when working with robotics--even if you're just simulating them--you have to deal with real world noise and imperfections. It is very easy to draw out a diagram of a square with perfect 90° angles and say you just need to make the robot move for the same amount of time when going on each line and making the angles--as I did in my high-level summary--but in the real world there is drift and other factors such that even if you code things up well you won't get the result you want. Similarly, the first high-level idea I conceived above described above (that involves moving in the +y and -y directions) makes perfect conceptual sense, but isn't implementable for turtlebot. Robotics don't work in the perfect world of Platonic ideals but in our world, and you need to program taking this into account, and perhaps be willing to accept imperfections. 


# Wall Follower
### High-level approach
*For each robot behavior, describe the problem and your approach at a high-level. Include any relevant diagrams or pictures that help to explain your approach.*


### Structure of the code
*Describe the structure of your code. For the functions you wrote, describe what each of them does.*


### Relevant GIFs
*While recording your robot's behavior in a rosbag conducting each type of behavior, also record a gif of the robot visually. Include this gif in your writeup and use it for analysis if needed. For instructions on how to make a gif recording, look at Gazebo simulator.*

![gif1](gifs/wall_follower.gif)
### Challenges
*Describe the challenges you faced and how you overcame them.*

### If I Had More Time, How Would I Improve the Behavior?
*If you had more time, how would you improve your robot behaviors?*


### Key Takeaways 
*What are your key takeaways from this project that would help you/others in future robot programming assignments? For each takeaway, provide a few sentences of elaboration.*



# Person Follower
### High-level approach
*For each robot behavior, describe the problem and your approach at a high-level. Include any relevant diagrams or pictures that help to explain your approach.*

The task can be broken down into a few steps: turning towards the nearest object, moving towards the nearest object, and knowing to stop when you get close to the object. To solve the problem, I just had to make the robot do each of these tasks independently: turn towards the object while moving towards it, and if you get close or are facing it, stop turning/stop moving.

### Structure of the code
*Describe the structure of your code. For the functions you wrote, describe what each of them does.*

The code is a very basic ROS program all within a PersonFollower class. The class has an init function which sets up publishing and subscribing to /cmd_vel and /scan respectively. The work of the program is in the function scan_callback, which is the subscription function for scan. Whenever it receives data, it finds the distance to the closest object and its corresponding angle by iterating through data.ranges. It then updates the angular velocity using a basic PID update if the robot is not facing the object or sets it to 0 if it is facing the object, and sets the linear velocity to a constant 0.3 value unless the robot is within 0.5 of the object in which case it updates it to 0. There is also a basic run function that just calls rospy.spin() to keep the code running.

### Relevant GIFs
*While recording your robot's behavior in a rosbag conducting each type of behavior, also record a gif of the robot visually. Include this gif in your writeup and use it for analysis if needed. For instructions on how to make a gif recording, look at Gazebo simulator.*

![gif1](gifs/person_follower.gif)
### Challenges
*Describe the challenges you faced and how you overcame them.*
Conceptually this problem was not very challenging, and any trickiness the implementation's details might have solved were greatly ameliorated by having first spent lots of time on WallFollower. The biggest challenges I faced were deciding how to handle getting the robot to stop turning/moving when it was facing the object/near to it and choosing the k value to be used in PID. The former challnege just required some thinking at which point I realized I could just set the velocities to 0 if we were close enough to the desired value. The latter challenge was solved by seeing what values of k I had used in other programs and experimenting based on that.

### If I Had More Time, How Would I Improve the Behavior?
*If you had more time, how would you improve your robot behaviors?*
While I think my robot achieves all the asked for goals pretty well, it does sometimes, when it gets to an object, spin more than it needs to (for instance, doing a 360 before stopping facing the object; this can be witnessed in my gif briefly). If I had more time, I would try to fix this problem, which I think is caused by variation in the angle of the nearest object that happens when you get close to an object. To expand the robots behavior, I would also explore how it behaves when you add lots of objects around it, as there might be a similar problem in that case where the angles lead the robot to move quite indirectly or not be able to decide which object to go to since more than one may appear to be the closest object at different moments in its movement.


### Key Takeaways 
*What are your key takeaways from this project that would help you/others in future robot programming assignments? For each takeaway, provide a few sentences of elaboration.*

* My first takeaway is that to get PID to work with angles you generally need to have a very small k value. I first started trying to point at the object using k values ~50 times as large as the one I ended up using and the robot continuously spun. More generally, I could say I learned that you should experiment with vastly different values for k, as something 100x larger or smaller may be what you need, which is a wider range of values than I am used to experimenting with.

* Turning is weird and hard to get right with the methods I have been using. All the things I would improve if I had more time involve the robot spinning too much, and I had similar problems with the wall follower. Its easy to overshoot an angle and then if you're only turning one way, have to keep spinning a whole other rotation. This also means that small/natural fluctuations in the nearest angle can cause a whole extra rotation.
