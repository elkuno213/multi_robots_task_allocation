#!/home/hvh/miniconda3/envs/MRTA/bin/python

#%% Import
import sys
import rospy
import smach_ros
import moveit_commander
import actionlib
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from smach import State,StateMachine,Sequence
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from mrta.msg import Mission



#%% Destination
destination = { 'MP1': ((10.6, 10.2), (0.0, 0.0, - pi/2)),
                'MP2': ((10.6, 11.5), (0.0, 0.0, - pi/2)),
                'MP3': ((3.6, 11.8), (0.0, 0.0, pi/2)),
                'MP4': ((3.6, 10.4), (0.0, 0.0, pi/2)),
                'AP': ((7.6, 2.5), (0.0, 0.0, 0.0)),
                'stock': ((5.7, 7.6), (0.0, 0.0, pi)), 
                'depot0': ((1.0, 1.0), (0.0, 0.0, 0.0)), 
                'depot1': ((1.0, 2.0), (0.0, 0.0, 0.0)),
                'depot2': ((1.0, 3.0), (0.0, 0.0, 0.0))}



#%% State Machine
# manipulation of arm and gripper
def manipulation(move_group, target_name):
    move_group.set_named_target(target_name)
    if not rospy.is_shutdown():
        move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


# State of 'pickup' used in State Machine
class pickup(State):
    def __init__(self, arm_group, gripper_group, target_pick):
        State.__init__(self, outcomes=['succeeded'])
        # Get parameters for self
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.target_pick = target_pick

    def execute(self, userdata):
        # Pickup
        rospy.loginfo('Pickup at {} !!!'.format(self.target_pick))
        manipulation(self.arm_group, 'arm_home')
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_pick))
        manipulation(self.gripper_group, 'gripper_open')
        manipulation(self.arm_group, 'arm_{}'.format(self.target_pick))
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_pick))
        manipulation(self.arm_group, 'arm_home')
        return 'succeeded'


# State of 'place' used in State Machine
class place(State):
    def __init__(self, arm_group, gripper_group, target_place):
        State.__init__(self, outcomes=['succeeded'])
        # Get parameters for self
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.target_place = target_place

    def execute(self, userdata):
        # Place
        rospy.loginfo('Place at {} !!!'.format(self.target_place))
        manipulation(self.arm_group, 'arm_home')
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_place))
        manipulation(self.arm_group, 'arm_{}'.format(self.target_place))
        manipulation(self.gripper_group, 'gripper_open')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_place))
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_home')
        return 'succeeded'


# State of 'movement' used in State Machine
class movement(State):
    def __init__(self, robot_name, target_name):
        State.__init__(self, outcomes=['succeeded'])
        # Get an action client
        self.client = actionlib.SimpleActionClient('{}/move_base'.format(robot_name), MoveBaseAction)
        self.client.wait_for_server()
        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = destination[target_name][0][0]
        self.goal.target_pose.pose.position.y = destination[target_name][0][1]
        self.goal.target_pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(destination[target_name][1][0], destination[target_name][1][1], destination[target_name][1][2])
        self.goal.target_pose.pose.orientation.x = quaternion[0]
        self.goal.target_pose.pose.orientation.y = quaternion[1]
        self.goal.target_pose.pose.orientation.z = quaternion[2]
        self.goal.target_pose.pose.orientation.w = quaternion[3]

        # Get parameters for self
        self.target_name = target_name

    def execute(self, userdata):
        # Movement
        rospy.loginfo('Movement to {} !!!'.format(self.target_name))
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        # Check whether the pose is occupied
        while self.client.get_state() != actionlib.simple_action_client.GoalStatus.SUCCEEDED:
            rospy.loginfo('Retry movement')
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
        return 'succeeded'


# State of 'transport' (pickup, movement, place) used in State Machine
class transport(State):
    def __init__(self, robot_name, arm_group, gripper_group, target_pick, target_place):
        State.__init__(self, outcomes=['succeeded'])
        # Get an action client
        self.client = actionlib.SimpleActionClient('{}/move_base'.format(robot_name), MoveBaseAction)
        self.client.wait_for_server()
        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = destination[target_place][0][0]
        self.goal.target_pose.pose.position.y = destination[target_place][0][1]
        self.goal.target_pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(destination[target_place][1][0], destination[target_place][1][1], destination[target_place][1][2])
        self.goal.target_pose.pose.orientation.x = quaternion[0]
        self.goal.target_pose.pose.orientation.y = quaternion[1]
        self.goal.target_pose.pose.orientation.z = quaternion[2]
        self.goal.target_pose.pose.orientation.w = quaternion[3] 

        # Get parameters for self
        self.robot_name = robot_name
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.target_pick = target_pick
        self.target_place = target_place

    def execute(self, userdata):
        # Pickup
        rospy.loginfo('Pickup at {} !!!'.format(self.target_pick))
        manipulation(self.arm_group, 'arm_home')
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_pick))
        manipulation(self.gripper_group, 'gripper_open')
        manipulation(self.arm_group, 'arm_{}'.format(self.target_pick))
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_pick))
        manipulation(self.arm_group, 'arm_home')

        # Movement
        rospy.loginfo('Movement to {} !!!'.format(self.target_place))
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        # Check whether the pose is occupied
        while self.client.get_state() != actionlib.simple_action_client.GoalStatus.SUCCEEDED:
            rospy.loginfo('Retry movement')
            self.client.send_goal(self.goal)
            self.client.wait_for_result()

        # Place
        rospy.loginfo('Place at {} !!!'.format(self.target_place))
        manipulation(self.arm_group, 'arm_home')
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_place))
        manipulation(self.arm_group, 'arm_{}'.format(self.target_place))
        manipulation(self.gripper_group, 'gripper_open')
        manipulation(self.arm_group, 'arm_pre_{}'.format(self.target_place))
        manipulation(self.gripper_group, 'gripper_closed')
        manipulation(self.arm_group, 'arm_home')

        return 'succeeded'


# State of 'waiting' used in State Machine
class waiting(State):
    def __init__(self, waiting_time):
        State.__init__(self, outcomes=['succeeded'])
        self.waiting_time = waiting_time

    def execute(self, userdata):
        # Waiting for operation
        rospy.loginfo('Waiting for operation ...')
        rospy.sleep(self.waiting_time)
        return 'succeeded'


# Transform mission message from 1D to 2D list of string
def mission_generation(encoded_mission):
    i, m = 0, 0
    mission = []
    while i < len(encoded_mission):
        if encoded_mission[i] == 'waiting':
            mission.append(['waiting', float(encoded_mission[i+1])])
            i += 2
        elif encoded_mission[i] == 'movement':
            mission.append(['movement', encoded_mission[i+1]])
            i += 2
        elif encoded_mission[i] == 'transport':
            mission.append(['transport', encoded_mission[i+1], encoded_mission[i+2]])
            i += 3
        if i >= len(encoded_mission):
            break
        m += 1
    return mission



#%% robot_action function
def robot_action(msg):

    # Define the group of arm and gripper for moveit_commander 
    arm_group = moveit_commander.MoveGroupCommander('arm', '{}/robot_description'.format(msg.robot_name), msg.robot_name)
    gripper_group = moveit_commander.MoveGroupCommander('gripper', '{}/robot_description'.format(msg.robot_name), msg.robot_name)

    # Move robot to home position
    manipulation(arm_group, 'arm_home')
    manipulation(gripper_group, 'gripper_closed')

    # Define the 'Sequence' type of State Machine
    action_sequence = Sequence(outcomes = ['succeeded','aborted','preempted'], connector_outcome = 'succeeded')
    with action_sequence:
        for i, mission in enumerate(mission_generation(msg.robot_mission)):
            if mission[0] == 'movement':
                Sequence.add('Movement {}'.format(i) , movement(msg.robot_name, mission[1]))
            if mission[0] == 'transport':
                Sequence.add('Transport {}'.format(i) , transport(msg.robot_name, arm_group, gripper_group, mission[1], mission[2]))
            if mission[0] == 'waiting':
                Sequence.add('Waiting {}'.format(i) , waiting(mission[1]))

    # Start the sequence of mission
    rospy.loginfo('Start the {} mission !!!'.format(msg.robot_name))
    start = rospy.get_time()
    action_sequence.execute()
    end = rospy.get_time()
    rospy.loginfo('Finished the {} mission !!!'.format(msg.robot_name))
    rospy.loginfo('The elapsed time of the {} mission: {}'.format(msg.robot_name, end - start))



#%% Main function
def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.Subscriber('robot_mission', Mission, robot_action)
    rospy.init_node("robot_sequencer")
    rospy.spin()



#%% Run main
if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass


