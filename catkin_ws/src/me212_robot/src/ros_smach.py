#!/usr/bin/env python
import rospy
import smach
import smach_ros
from me212_robot.msg import smachAction, smachResult, smachFeedback, smachGoal
import actionlib
from std_msgs.msg import Float64MultiArray
x = 0
y = 0
yaw = 0

class Keyboard_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Keyboard_move')

        while (x < 0.1):
            # print x,y,yaw
            rospy.sleep(0.5)

 
        return 'outcome1'


# define state Foo
class GO_Straight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

        # Set_up_ros_action
        self.client = actionlib.SimpleActionClient('smach_action', smachAction)
        self.client.wait_for_server()

        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Go_Straight')

        if self.counter < 3:
            self.pub_action_goal(20)
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

    def pub_action_goal(self, distance):
        # Creates a goal to send to the action server.
        goal = smachGoal(action_id=0, number=distance)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()


# define state Bar
class Go_circle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        # Set_up_ros_action
        self.client = actionlib.SimpleActionClient('smach_action', smachAction)

    def execute(self, userdata):
        self.pub_action_goal(0.5)
        rospy.loginfo('Executing state Go_circle')
        return 'outcome1'

    def pub_action_goal(self, distance):

        # Creates a goal to send to the action server.
        goal = smachGoal(action_id=1, number=distance)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Keyboard_move', Keyboard_move(), 
                               transitions={'outcome1':'Go_Straight'})
        smach.StateMachine.add('Go_Straight', GO_Straight(), 
                               transitions={'outcome1':'Go_circle', 'outcome2':'outcome4'})
        smach.StateMachine.add('Go_circle', Go_circle(), 
                               transitions={'outcome1':'Go_Straight'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/LAB10_ROS_SMACH')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

def cbPose(msg):
    global x,y,yaw
    x = msg.data[0]
    y = msg.data[1]
    yaw = msg.data[2]
    # print x,y,yaw

if __name__ == '__main__':
    sub_odom = rospy.Subscriber("/odometry", Float64MultiArray, cbPose)
    main()
