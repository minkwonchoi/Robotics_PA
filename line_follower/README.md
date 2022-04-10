Readme for the line follower PA.

This PA started with the linefollower_p.py from prrexample, and worked in 3 folds. 

  Part 1, Running linefollower_p.py.
    When I ran the code, the robot had trouble locating the yellow line. I edited the lower and upper bound of my yellow definition each by 20 units. 
    
    lower_yellow = numpy.array([ 20, 0, 0]) # was 40
    upper_yellow = numpy.array([ 140, 255, 255]) # was 120
    
    I tried to change value and saturation, but they did not have noticible impact on line detacting. I could safly enlarge the window of detection 
    since in the virtual enviournemt in Gazebo, there is only two colors, grey and yellow. 
    
  Part 2, Making safer line follower
    I found out that sometimes the err variable can be very large (>30000) so I made a new variable called cap. This sets the limit to err to cap.
    500 was the best number for cap. Limiting the angular velocity helped the robot to stick to the line in case of missing the line. 
    
  Part 3, Extra challenges
    Finished two extra challenges. 
    -1- Robot is able to double back along the line allowing it to follow the line infinitely
        Added an else statement that spinss the robot in place when it looses the yellow line. Wwhen the robot reaches the end of the line, it would
        spin until it sees the line again. 
    -2- Robot starts somewhere where the line is not immediately visible
        Completed this by adding forward movement to the robot. It would move forward until it sees a yellow line. 
        
  Interesting thing happened
    The TA Kelly and I spent 30 minute on a strange phenomenon. The robot would make circles, spirals, or random movements near the line. We 
    could see the robot picking up the line and correctly showing the center of the detected line with a red dot. We checked the angular velocity 
    and linear velocity but we could not find the error. Later we found out that using reset feature on Gazebo using alt+r can reset the yellow lines
    in the given Gazebo world. The strange thing is sometimes all the lines would rotate so they are parellel to each other, but does not display it
    correctly. Thus user would be looking at perfectly normal line following world, when real world has parellel lines. 
    

