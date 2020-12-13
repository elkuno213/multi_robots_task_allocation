#!/usr/bin/env python2.7

import os, rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node("stock_components")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    s = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    path = os.path.join(os.path.expanduser('~'), 'MRTA_2020', 'Simulation', 'usecase_ws', 'src', 'usecase_gazebo', 'models', 'component', 'model.sdf')
    with open(path, "r") as f:
        component_xml = f.read()

    # Delete previous components
    for component_num in range(24):
        item_name = "component_{}".format(component_num)
        delete_model(item_name)
    
    # Insert new components
    orient1 = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    orient2 = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 3.14159265359/2))
    pose = [    [[9.4, 10.3, 0.8], orient2], [[9.6, 10.3, 0.8], orient2], [[9.4, 10.5, 0.8], orient2], [[9.6, 10.5, 0.8], orient2],
                [[9.4, 11.7, 0.8], orient2], [[9.6, 11.7, 0.8], orient2], [[9.4, 11.5, 0.8], orient2], [[9.6, 11.5, 0.8], orient2],
                [[2.4, 10.3, 0.8], orient2], [[2.6, 10.3, 0.8], orient2], [[2.4, 10.5, 0.8], orient2], [[2.6, 10.5, 0.8], orient2],
                [[2.4, 11.7, 0.8], orient2], [[2.6, 11.7, 0.8], orient2], [[2.4, 11.5, 0.8], orient2], [[2.6, 11.5, 0.8], orient2],
                [[5.8, 6.6, 1], orient1], [[6.0, 6.6, 1], orient1], [[5.8, 6.8, 1], orient1], [[6.0, 6.8, 1], orient1],
                [[7.6, 3.4, 1.1], orient1], [[7.8, 3.4, 1.1], orient1], [[7.6, 3.6, 1.1], orient1], [[7.8, 3.6, 1.1], orient1]]
    for component_num in range(24):
        x = pose[component_num][0][0]
        y = pose[component_num][0][1]
        z = pose[component_num][0][2]
        orient = pose[component_num][1]
        
        item_name = "component_{}".format(component_num)
        item_pose = Pose(Point(x=x, y=y, z=z), orient)
        s(item_name, component_xml, "", item_pose, "world")
