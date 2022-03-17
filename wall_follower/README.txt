HOW TO RUN: please use $roslaunch wall_follower wall_follower.launch

This will launch all three of the nodes, pid, driver, and scan values handler, correctly.

This is my submission for the wall follower PA.
I started With August's skeleton code and then used my own PD algorithm which only uses the P and D component without I component to follow the wall.


My robot does publish the movement comments depending on each unique states. I have three states. State zero means I have to find a wall. I’m looking for a wall and my robot would continue to just go straight ahead until it has any walls nearby. State one is a state where my robot has found a wall then it would turn either right or left which ever is the shortest and turn to align the wall on the left side of the robot. State two is following the robot that I connected with my PD algorithm and let the PD algorithm keep the wall on the left.

I have completed all required features such as following a single wall following the inside of L-shaped wall following the inside of a rectangular wall and I’ve also completed the extra challenge to follow a rectangular wall on the outside. I could not do the corridor challenge because I did not know how to make those corridors in the Gazebo. Also I did I really wanted to try this with a physical robot but because my I shattered my arm and I cannot leave my house. I’m looking forward to try this out later just for my curiosity 


