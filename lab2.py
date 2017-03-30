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
    global pose
    global xPosition
    global yPosition
    global theta

    calcOdom()

    iX = xPosition
    iY = yPosition

    #get all info about the pose
    goalX = goal.pose.position.x
    goalY = goal.pose.position.y

    quat = goal.pose.orientation
    quat = [quat.x, quat.y, quat.z, quat.w]

    roll, pitch, yaw = euler_from_quaternion(quat)

    #convert yaw to degrees
    goalAng = math.degrees(yaw)

    print "spin!"
    firstAng = math.degrees(math.atan2(goalY-yPosition,goalX-xPosition))
    if (firstAng < 0):
        print "TOO SMALL!!!!!"
        firstAng = firstAng + 360
    rotate(firstAng-theta)

    print "move!"
    curDist = math.sqrt(((iX - goalX) ** 2)+ ((iY - goalY) ** 2))
    driveStraight(.25,curDist)

    print "spin!"
    calcOdom()
    rotate(goalAng - theta)

    print "done"


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
            print "Goal=%f, Cur=%f" % (distance,curDist)
            spinWheels(speed*.5,speed*.5,.01)
        publishTwist(0,0)
    print "THERE!"


    
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
        print "ONE, dest=%d, angle=%d, error=%d" % (dest,theta,error)
        spinWheels(direction*-.03,direction*.03,.05)

    while((abs(error) >= 10) and not rospy.is_shutdown()):
        calcOdom()
        error = dest - theta
        direction = error / abs(error)
        print "TWO, dest=%d, angle=%d, error=%d" % (dest,theta,error)
        spinWheels(direction*-.01,direction*.01,.04)

    while((abs(error) >= 4) and not rospy.is_shutdown()):
        calcOdom()
        error = dest - theta
        direction = error / abs(error)
        print "THREE, dest=%d, angle=%d, error=%d" % (dest,theta,error)
        spinWheels(direction*-.005,direction*.005,.02)

    vel.angular.z = 0.0
    publishTwist(0.0,0.0)
    print "DONE!"


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global pose
    global xPosition
    global yPosition
    global theta

    angVel = speed/radius
    calcOdom()

    #convert yaw to degrees
    goalAng = theta + angle

    while (theta <= goalAng):
        
        calcOdom()

        spinWheels(speed * .25,angVel * .25, .01)
        print "dest=%d, angle=%d" % (goalAng,theta)


#Bumper Event Callback function
def readBumper(msg):
    print "Bumper Callback"
    if (msg.state == 0): #change back to 1 when running later
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

    odom_list.waitForTransform('map','base_footprint',rospy.Time(0),rospy.Duration(1.0))
    #finds the position and oriention of two objects relative to each other
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) 
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
    goal_sub = rospy.Subscriber('move_base_simple/goal',PoseStamped,navToPose,queue_size=1)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    # Set the time to run the program for
    run_t = 240

    start_t = rospy.Time.now().secs

    driveArc(.2,.25,90)

    while(rospy.Time.now().secs - start_t <= run_t and not rospy.is_shutdown()):
        #make the robot keep doing something... 
        rospy.Timer(rospy.Duration(.01), timerCallback)

    #rotate(90)

    # Make the robot do stuff...
    print "Lab 2 complete!"

