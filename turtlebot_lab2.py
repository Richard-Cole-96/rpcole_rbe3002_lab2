#!/usr/bin/env python

import rospy, tf, math, numpy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used

# Publish a Twist message with the linearVelocity and angularVelocity
def publishTwist(linearVelocity, angularVelocity):
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    print "spin!"
    print "move!"
    print "spin!"
    print "done"
    pass # Delete this 'pass' once implemented


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(.25,.6)
    rotate(90)
    driveStraight(.25,.45)
    rotate(-135)


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    #length of turtlebot wheelbase in meters (9 in)
    base_width = 0.2286
    #calculate the needed values for linear and angular velocities from wheel velocities
    lin_vel = (u1 + u2)/2
    ang_vel = (u2 - u1)/base_width
    #While time has not elapsed, send twist message to drive the turtlebot
    start_time = rospy.Time.now().secs

    while(rospy.Time.now().secs - start_time <= time and not rospy.is_shutdown()):
        publishTwist(lin_vel,ang_vel)
    lin_vel = 0.0
    ang_vel = 0.0
    publishTwist(0.0,0.0)
    

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    global xPosition
    global yPosition

    calcOdom()

    iX = xPosition
    iY = yPosition
    atGoal = False

    print "DRIVE!!"

    #loop until distance wanted is same as distanced achieved
    while(not atGoal and not rospy.is_shutdown()):
        calcOdom()
        curX = xPosition
        curY = yPosition
        curDist = math.sqrt(((curX-iX) ** 2)+ ((curY-iY) ** 2))
        if(curDist >= distance):
            atGoal = True
            publishTwist(0,0)
            print "At Point"
        else:
            print "Not There Yet"
            print "Goal=%d, Cur=%d" % (distance,curDist)
            spinWheels(speed,speed,.05)
        publishTwist(0,0)


    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose
    global theta

    print "PIVOT!!"

    if(angle > 180 or angle < -180):
        print "angle is too large or small"
    vel = Twist()
    done = True

    calcOdom()
    dest = angle + theta
    if(dest > 180):
        print "dest is too large"
        excess = dest - 180
        dest = -180 + excess
    if(dest < -180):
        print "dest is too small"
        excess = dest + 180
        dest = -180 + excess
        
    error = dest - theta

    while((abs(error) >= 25) and not rospy.is_shutdown()):
        calcOdom()
        error = dest - theta
        direction = error / abs(error)
        print "dest=%d, angle=%d, error=%d" % (dest,theta,error)
        spinWheels(direction*-.2,direction*.2,.05)

    while((abs(error) >= 10) and not rospy.is_shutdown()):
        calcOdom()
        error = dest - theta
        direction = error / abs(error)
        print "dest=%d, angle=%d, error=%d" % (dest,theta,error)
        spinWheels(direction*-.1,direction*.1,.04)

    while((abs(error) >= 3) and not rospy.is_shutdown()):
        calcOdom()
        error = dest - theta
        direction = error / abs(error)
        print "dest=%d, angle=%d, error=%d" % (dest,theta,error)
        spinWheels(direction*-.05,direction*.05,.04)

    vel.angular.z = 0.0
    publishTwist(0.0,0.0)
    print "DONE!"


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    #length of turtlebot wheelbase in meters (9 in)
    base_width = 0.2286

    Vr = speed(radius+(base_width/2))
    Vl = speed(radius-(base_width/2))


    pass  # Delete this 'pass' once implemented


#Bumper Event Callback function
def readBumper(msg):
    print "Bumper Callback"
    if (msg.state == 1): #change back to 1 when running later
        print "Bumper Pressed!"
        executeTrajectory()
        #driveStraight(.25,.5)
        #rotate(90)

#calculate the odometry parameters
def calcOdom():
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('odom','base_footprint',rospy.Time(0),rospy.Duration(1.0))
    #finds the position and oriention of two objects relative to each other
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) 
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]

    xPosition = position[0]
    yPosition = position[1]

    odomW = orientation
    q = [odomW[0],odomW[1],odomW[2],odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)

    #convert yaw to degrees
    pose.pose.orientation.z = yaw
    theta = math.degrees(yaw)

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta

    #calcOdom() 


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list

    pose = PoseStamped()

    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    # Set the time to run the program for
    run_t = 90

    start_t = rospy.Time.now().secs

    while(rospy.Time.now().secs - start_t <= run_t and not rospy.is_shutdown()):
        #make the robot keep doing something... 
        rospy.Timer(rospy.Duration(.01), timerCallback)

    #rotate(90)

    # Make the robot do stuff...
    print "Lab 2 complete!"

