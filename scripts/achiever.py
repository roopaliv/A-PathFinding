#!/usr/bin/env python
import sys
import rospy
import geometry_msgs.msg
import math
from nav_msgs.msg import Odometry
import tf

xPos = -8.0
yPos = -2.0
orientation = 0.000

def callbackGroundPos(msg):
    global xPos
    global yPos
    global orientation
    xPos = 	msg.pose.pose.position.x;
    yPos = 	msg.pose.pose.position.y;
    euler = tf.transformations.euler_from_quaternion(
    [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    orientation = euler[2]

    #rospy.logerr("inside callback: " + str(math.degrees(orientation)))

def heuristic(x1, x2, y1, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def achiever():
    rospy.init_node('achiever', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    map = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
           [0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
           [0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0],
           [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
           [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
           [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
           [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
           [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0],
           [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1]]
    initialXCoord = -8.0;
    initialYCoord = -2.0;
    goalXCoord = rospy.get_param('goalx');
    goalYCoord = rospy.get_param('goaly');
    initialXBox = int(math.floor(9 + initialXCoord))
    initialYBox = int(math.floor(10 - initialYCoord))
    goalXBox = int(math.floor(9 + goalXCoord))
    goalYBox = int(math.floor(10 - goalYCoord))

    gScore = {}
    fScore = {}
    givenMap = {}
    cameFrom ={}
    openList = []
    closedList =[]
    for y in range(20):
        for x in range(18):
            gScore[str(x)+','+str(y)]= -1
            fScore[str(x)+','+str(y)] = -1
            cameFrom[str(x)+','+str(y)] = [-1,-1]
            givenMap[str(x)+','+str(y)] = map[y][x]
    gScore[str(initialXBox) + ','+ str(initialYBox)] = 0
    fScore[str(initialXBox) + ','+ str(initialYBox)] = heuristic(initialXBox,goalXBox, initialYBox, goalYBox)

    openList.append([initialXBox,initialYBox]);
    pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callbackGroundPos, queue_size=10)
    while(len(openList)>0):

        cmd_vel = geometry_msgs.msg.Twist()
        currentX = openList[0][0]
        currentY = openList[0][1]
        currentFscore = fScore[str(currentX) + ','+ str(currentY)]
        for openItem in openList:
            fScoreTemp = fScore[str(openItem[0]) + ','+ str(openItem[1])]
            if fScoreTemp < currentFscore:
                currentX = openItem[0];
                currentY = openItem[1];
                currentFscore = fScoreTemp
        if currentX == goalXBox and currentY == goalYBox:
            openList.remove([currentX, currentY])
            #rospy.logerr("Done")
            path = [];
            pathCurrent = [currentX, currentY]
            pathCurrentX = currentX
            pathCurrentY = currentY
            for cI in range(len(cameFrom)):
                path.append([pathCurrentX,pathCurrentY])
                pathCurrent = cameFrom[str(pathCurrentX) + ',' + str(pathCurrentY)]
                pathCurrentX = pathCurrent[0]
                pathCurrentY = pathCurrent[1]
                if(pathCurrentY == -1):
                    break;
            '''for pathStep in path:
                map[pathStep[1]][pathStep[0]] = 9
            for testi in range(20):
                rospy.logerr(map[testi])'''
            for pathI in range(len(path)):
                index = len(path) - 1 - pathI
                nextXbox = path[index][0]
                nextYbox = path[index][1]
                nextXcoord = nextXbox - 9
                nextYcoord = 10 - nextYbox
                #rospy.logerr("X: " + str(nextXcoord)+" Y: " + str(nextYcoord))
                angle = math.atan2((nextYcoord - yPos), (nextXcoord - xPos)) - orientation;
                distance = math.hypot(nextXcoord - xPos, nextYcoord - yPos);
                duration = rospy.Rate(5)
                vel = geometry_msgs.msg.Twist()
                for s in range(0,10):
                     vel.angular.z = angle/2
                     vel.linear.x = distance/2
                     pub.publish(vel)
                     duration.sleep()

        else:
            closedList.append([currentX,currentY])
            openList.remove([currentX,currentY])
            neighbours=[]

           
            if currentY + 1 <= 19:
                neighbours.append([(currentX),(currentY + 1),(10)])
            if currentX - 1 >= 0:
                neighbours.append([(currentX - 1),(currentY),(10)])
            if currentY - 1 >= 0:
                neighbours.append([(currentX),(currentY - 1),(10)])
 	    if currentX + 1 <= 17:
                neighbours.append([(currentX + 1),(currentY),(10)])
            #if currentX + 1 <= 17 and currentY - 1 >= 0:
                #neighbours.append([(currentX + 1),(currentY - 1),(14)])
            #if currentX + 1 <= 17 and currentY + 1 <= 19:
                #neighbours.append([(currentX + 1),(currentY + 1),(14)])
            #if currentX - 1 >= 0 and currentY - 1 >= 0:
                #neighbours.append([(currentX - 1),(currentY - 1),(14)])
            #if currentX-1 >=0 and currentY + 1 <=19:
                #neighbours.append([(currentX - 1),(currentY + 1),(14)])
            for neighbourX,neighbourY,cost in neighbours:
                neighbourExistsOpen = [neighbourX, neighbourY] in openList;
                neighbourExistsClosed = [neighbourX, neighbourY] in closedList;
                isObstacle = False if(givenMap[str(neighbourX) + ',' + str(neighbourY)] == 0) else True
                if not neighbourExistsClosed and not isObstacle:
                    gScoreTemp = (gScore[str(currentX) + ','+ str(currentY)] if (gScore[str(currentX) + ','+ str(currentY)] != -1) else 0) + cost
                    if gScore[str(neighbourX) + ',' + str(neighbourY)] == -1 or gScoreTemp < gScore[str(neighbourX) + ',' + str(neighbourY)]:
                        cameFrom[str(neighbourX) + ',' + str(neighbourY)] = [currentX, currentY]
                        gScore[str(neighbourX) + ',' + str(neighbourY)] = gScoreTemp
                        fScore[str(neighbourX) + ',' + str(neighbourY)] = gScoreTemp + heuristic(neighbourX, goalXBox, neighbourY, goalYBox)
                        if not neighbourExistsOpen:
                            openList.append([neighbourX, neighbourY])

    cmd_vel.angular.z = 0
    cmd_vel.linear.x = 0
    pub.publish(cmd_vel)
    rate.sleep()
if __name__ == '__main__':
	    try:
		achiever()
	    except rospy.ROSInterruptException:
		pass
