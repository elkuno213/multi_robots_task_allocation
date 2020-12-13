#!/home/hvh/miniconda3/envs/MRTA/bin/python

#%% Import
import rospy
from mrta.msg import Mission



#%% Inputs
robot0_name = "robot0"
robot0_mission = [  'waiting', '94.5', 'movement', 'MP3', 'transport', 'MP3', 'MP2', 'waiting', '60', 'transport', 'MP2', 'AP', 'movement', 'depot0']
robot1_name = "robot1"
robot1_mission = [  'movement', 'stock', 'transport', 'stock', 'MP3', 'movement', 'stock', 'transport', 'stock', 'MP4', 'waiting', '36.0', 'movement', 'MP2',
                    'transport', 'MP2', 'stock', 'waiting', '47.4', 'movement', 'MP4', 'transport', 'MP4', 'MP2', 'movement', 'AP', 'transport', 'AP', 'stock', 'movement', 'depot1']



#%% Main function
def main():

    mission0 = Mission()
    mission0.robot_name = robot0_name
    mission0.robot_mission = robot0_mission

    mission1 = Mission()
    mission1.robot_name = robot1_name
    mission1.robot_mission = robot1_mission

    pub0 = rospy.Publisher('robot0_mission', Mission, queue_size=1)
    pub1 = rospy.Publisher('robot1_mission', Mission, queue_size=1)
    rospy.init_node('robot_mission_publisher')
    pub0.publish(mission0)
    pub1.publish(mission1)
    rospy.spin()



#%% Run main
if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass