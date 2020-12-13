#!/home/hvh/miniconda3/envs/MRTA/bin/python

#%% Import
import numpy as np
import random
import rospy
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from mrta.msg import Mission
from multiprocessing import Process



#%% Tool functions
def intersection(lst1, lst2):
    """
    This function finds the intersection of 2 lists.
    For example,
    lst1 = [1, 2, 3, 4, 5]
    lst2 = [0, 2, 4, 7]
    --->>>
    return [2, 4]
    """
    return list(set(lst1) & set(lst2))


def find_element_index(lst, x):
    """
    This function finds indices of element 'x' inside 'lst'
    For example,
    lst = [0, 2, 4, 6, 8, 2]
    x = 2
    --->>>
    return [1, 5]
    """
    return [i for i, e in enumerate(lst) if e == x]


def find_sublist_indices(lst, sublst):
    """
    This function finds indices of element 'x' inside 'lst'
    For example,
    lst = [0, 2, 4, 6, 8, 2]
    sublst = [0, 2, 8]
    --->>>
    return [0, 1, 5, 4]
    """
    idx = []
    for e in sublst:
        idx += find_element_index(lst, e)
    return idx



#%% Inital population
def initial_population(data, n_Pop, n_Random=1000, lbd1=0.5, lbd2=0.5):
    """
    This function create a population of 'n_Pop' valid chromosomes and returns also its fitness values and missions
    """
    ch_pop, fitness_ch_pop, misison_ch_pop = [], [], []
    while len(ch_pop) < n_Pop:
        ch = list(np.random.randint(data.n_R, size=data.n_T))
        validation = chromosome_fitness(ch, data, n_Random, lbd1, lbd2)
        if validation:
            ch_pop.append(ch)
            fitness_ch_pop.append(validation[0])
            misison_ch_pop.append(validation[1])
    return ch_pop, fitness_ch_pop, misison_ch_pop



#%% Filtered population by fitness values
def filtered_population(pop, fitness_pop, n_choosen=None, best_ratio=None, worst_ratio=None):
    """
    This function returns new poplupaltion sorted by ascending fitness values and choosen from best and worst ratios, which help to find global optimum
    best_ratio = 0.2 means 20% best chromosomes are choosen
    worst_ratio = 0.1 means 10% worst chromosomes are choosen
    For example,
    pop = [ [1, 1, 1, 0, 0], [0, 0, 1, 1, 0], [0, 1, 1, 0, 0], [0, 1, 0, 1, 0], [1, 1, 1, 1, 1],
            [1, 0, 1, 1, 0], [1, 0, 0, 1, 0], [0, 1, 0, 0, 0], [1, 0, 1, 0, 0], [0, 1, 1, 1, 1]]
    fitness_pop = [51.0, 70.5, 61.5, 88.5, 87.5, 78.0, 76.0, 74.5, 87.5, 89.5]
    best_ratio = 0.2
    worst_ratio = 0.1
    --->>>
    pop_sorted = [  [1, 1, 1, 0, 0], [0, 1, 1, 0, 0], [0, 0, 1, 1, 0], [0, 1, 0, 0, 0], [1, 0, 0, 1, 0],
                    [1, 0, 1, 1, 0], [1, 0, 1, 0, 0], [1, 1, 1, 1, 1], [0, 1, 0, 1, 0], [0, 1, 1, 1, 1] ]
    pop_choosen = [ [1, 1, 1, 0, 0], [0, 1, 1, 0, 0], [0, 1, 1, 1, 1]   ]
    """
    pop_sorted = [i for _, i in sorted(zip(fitness_pop, pop))]        # Sort the population by ascending order of the fitness values
    fitness_pop_sorted = fitness_pop[:]
    fitness_pop_sorted.sort()      # Sort the fitness values by ascending order
    if n_choosen:           # If n_choosen exists --->>> choose 'n_choosen' best chromosomes 
        pop_choosen = pop_sorted[:n_choosen]
        fitness_pop_choosen = fitness_pop_sorted[:n_choosen]
    else:                   # If n_choosen doesn't exist                 
        if best_ratio:      # If best_ratio exists
            best_threshold = int(best_ratio * len(pop))
            if worst_ratio: # If worst_ratio exists --->>> choose best and worst chromosomes
                worst_threshold = int((1 - worst_ratio) * len(pop))
                pop_choosen = pop_sorted[:best_threshold] + pop_sorted[worst_threshold:]
                fitness_pop_choosen = fitness_pop_sorted[:best_threshold] + fitness_pop_sorted[worst_threshold:]
            else:           # If worst_ratio exists --->>> choos best chromosomes
                pop_choosen = pop_sorted[:best_threshold]
                fitness_pop_choosen = fitness_pop_sorted[:best_threshold]
        else:               # If best_ratio doesn't exist 
            if worst_ratio: # If worst_ratio exists --->>> choose worst chromosomes
                worst_threshold = int((1 - worst_ratio) * len(pop))
                pop_choosen = pop_sorted[worst_threshold:]
                fitness_pop_choosen = fitness_pop_sorted[worst_threshold:]
            else:           # If worst_ratio doesn't exist --->>> choose all sorted chromosomes
                pop_choosen = pop_sorted
                fitness_pop_choosen = fitness_pop_sorted
    return pop_choosen, fitness_pop_choosen



#%% Fitness functions
def horizontal_random(lst):
    """
    This function does random of 2D list
    For example,
    lst = [[0, 1, 2, 3, 4], [5, 6, 7]]
    --->>>
    random_lst = [[3, 4, 1, 0, 2], [7, 6, 5]]
    """  
    random_lst = []
    for e in lst:
        row = e
        random.shuffle(row)     # Shuffle this row
        random_lst.append(row)  # Append the shuffled row to new list
    return random_lst


def reparation_transport_order(transformed_ch, priority):
    """
    This function repairs transformed_ch by considering priority on each robot
    For example,
    priority = [[0, 1, 2, 3, 4], [5, 6, 7, 8]]
    transformed_ch = [[2, 7, 0, 3, 5, 8], [4, 1, 6]]
    --->>>
    transformed_ch = [[0, 5, 2, 3, 7, 8], [1, 4, 6]]
    """
    for tr in transformed_ch:
        for tp in priority:           
            sub_transports = intersection(tr, tp)                   # Find transport of product 'tp' inside transport of robot 'tr'
            sub_idx = find_sublist_indices(tr, sub_transports)      # Find all indices of these transports inside transport of robot 'tr'
            if sorted(sub_idx) != sub_idx:                          # Check ascending order and repair
                sub_transports.sort()
                sub_idx.sort()
                i = 0
                for j in sub_idx:
                    tr[j] = sub_transports[i]
                    i += 1 
    return transformed_ch


def chromosome_transformation(ch, n_R):
    """
    This function transforms a chromosome to another form of scheduling 
    For example,
    ch = [0, 1, 0, 1, 1, 1, 0, 1]
    n_R = 2
    --->>>
    transformed_ch = [[0, 2, 6], [1, 3, 4, 5, 7]]
    """  
    transformed_ch = []
    for i in range(n_R):
        idx = find_element_index(ch, i)
        transformed_ch.append([j for j in idx])
    return transformed_ch


def mission_generating(transformed_ch, data):


    def find_todo_robot(transformed_ch, todo):
        """
        This function finds corresponding robots for 'todo' list
        For example,
        priority = [[0, 1, 2, 3, 4], [5, 6, 7], [8, 9, 10]]
        todo = [5, 0, 9]
        --->>>
        todo_robot = [1, 0, 2]
        """
        todo_robot = []
        for td in todo:
            for robot, tp in enumerate(transformed_ch):
                if find_element_index(tp, td):
                    todo_robot.append(robot)
        return todo_robot


    def product_transport_checking(priority, trans):
        """
        This function finds the position of a transport 'trans' inside the 'transport' list
        For example,
        priority = [[0, 1, 2, 3, 4, 5], [6, 7, 8, 9, 10, 11]]
        trans = 5
        --->>>
        return (0, 5)
        """
        for i, x in enumerate(priority):
            if trans in x:      return (i, x.index(trans))


    def todo_generating(counter, transformed_ch):
        """
        This function generates new 'todo' list from 2D chromosome and robot's transport counters
        For example,
        counter = [0, 0]
        transformed_ch = [[0, 1, 3, 4, 6, 7, 8, 11], [2, 5, 9, 10]]
        --->>>
        todo = [0, 2]
        """
        todo = []
        for i, tc in enumerate(counter):
            if tc < len(transformed_ch[i]):     todo.append(transformed_ch[i][tc])
        return todo


    def same_product_filtering(todo, priority):
        """
        This function filters new 'todo' list from the old one by removing same product's transports and keeping only the 'smallest' one
        For example,
        priority = [[0, 1, 2, 3, 4, 5], [6, 7, 8, 9, 10, 11]]
        todo = [0, 4, 6, 10]
        --->>>
        new_todo = [0, 6]
        """
        new_todo = []
        for _, trans in enumerate(priority):                  # For each product
            inter = intersection(trans, todo)                       # Find intersection between todo and transports of each product
            if len(inter) > 0:      new_todo.append(min(inter))     # Filter transport of each product
        return new_todo


    def operation_order_filtering(todo, done, priority):
        """
        This function filters new 'todo' list from the old one by considering the 'done' list
        For example,
        priority = [[0, 1, 2, 3, 4, 5], [6, 7, 8, 9, 10, 11]]
        todo = [0, 8]
        done = [6, 7]
        --->>>
        new_todo = [0, 8]
        """
        new_todo = []
        for _, trans in enumerate(todo):
            prod, trans_idx = product_transport_checking(priority, trans)

            if trans_idx == 0:      new_todo.append(trans)      # If this is the first transport of the product 'prod' -> add to new 'todo' list

            else:       # If this is not the first transport of the product 'prod' -> check the existence of the previous transport
                prev_trans = priority[prod][trans_idx - 1]

                if prev_trans in done:      new_todo.append(trans)      # If the previous transport exists in 'done' list -> add the transport to new 'todo' list
        return new_todo
        

    def mission_updating(todo, todo_robot, mission, actual_position, actual_time, transport_time, data):
        """
        This function updates missions for each robot by considering conditions
        """
        for i, robot in enumerate(todo_robot):
            trans = todo[i]                         # Define transport of this robot
            init_pose = data.pose_list[trans][0]    # Initial pose of the transport
            final_pose = data.pose_list[trans][1]   # Final pose of the transport
            actual_pose = actual_position[robot]    # Actual pose of the robot

            # Consider the actual position of robot, the initial and final positions of transport to generate mission
            if init_pose == actual_pose:        # If robot is right now at the initial pose of transport 'trans'

                # Do waiting
                oper_dt = dt('operation', actual_pose)      # Operation dt   
                mission[robot].append(['waiting', oper_dt, actual_pose])    # Do waiting
                actual_time[robot] += oper_dt               # Actual time of robot

                # Do transport
                transport_time[trans][0] = actual_time[robot]       # Transport initial time
                trans_dt = dt('transport', init_pose, final_pose)   # Transport dt
                mission[robot].append(['transport', trans_dt, init_pose, final_pose, trans])     # Do transport
                actual_time[robot] += trans_dt                      # Actual time of robot
                transport_time[trans][1] = actual_time[robot]       # Transport final time

            else:                               # If robot is NOT at the initial pose of transport 'trans'

                move_dt = dt('movement', actual_pose, init_pose)        # Movement dt

                # Constraint of order of operations
                prod, trans_idx = product_transport_checking(data.priority, trans)      # Product and transport index

                if trans_idx > 0:       # If the transport is not the first one of product 'prod'

                    prev_trans = data.priority[prod][trans_idx - 1]   # Previous transport
                    prev_trans_final_pose = data.pose_list[prev_trans][1]   # Previous transport final pose
                    prev_trans_final_time = transport_time[prev_trans][1]   # Previous transport final time
                    prev_oper_final_time = prev_trans_final_time + dt('operation', prev_trans_final_pose)       # Previous operation final time
    
                    if prev_oper_final_time > actual_time[robot] + move_dt:       # If the previous operation has not been finished yet

                        # Do waiting at the initial pose of transport 'trans'
                        wait_dt = prev_oper_final_time - actual_time[robot] - move_dt   # Waiting dt
                        mission[robot].append(['waiting', wait_dt, init_pose])          # Do waiting
                        actual_time[robot] += wait_dt                                   # Actual time of robot
                
                # Do movement to the initial pose
                mission[robot].append(['movement', move_dt, actual_pose, init_pose])      # Do movement
                actual_time[robot] += move_dt                       # Actual time of robot

                # Do transport to the final pose of transport 'trans'
                transport_time[trans][0] = actual_time[robot]       # Transport initial time
                trans_dt = dt('transport', init_pose, final_pose)   # Transport dt
                mission[robot].append(['transport', trans_dt, init_pose, final_pose, trans])    # Do transport
                actual_time[robot] += trans_dt                      # Actual time of robot
                transport_time[trans][1] = actual_time[robot]       # Transport final time

            # Update robots' position
            actual_position[robot] = data.pose_list[trans][1]            # Robot's actual position



    """
    This function finds best mission of robots from transformed chromosome
    For example,
    transformed_ch = [[0, 3, 4, 6, 7, 10, 11], [1, 2, 5, 8, 9]]
    --->>>
    mission = [['movement', 1, 'depot0', 'AP'], ['waiting', 18, 'AP'], ['transport', 12, 'AP', 'MP1'], ...]
    """
    random_transformed_ch = horizontal_random(transformed_ch)       # Random transformed chromosome
    random_transformed_ch = reparation_transport_order(random_transformed_ch, data.priority)      # Repair random transformed chromosome considering order of transports on each robot

    transport_time = [[0, 0]] * data.n_T    # Define time (inital and final) of each transport
    counter = [0] * data.n_R                # Define counter of transports for each robot
    done = []                               # Define 'done' list of done transports

    # Define actual position and time of each robot
    actual_position = []
    for i in range(data.n_R):       actual_position.append('depot{}'.format(i))
    actual_time = [0] * data.n_R

    # Define list of mission of each robot
    mission = []
    for _ in range(data.n_R):       mission.append([])
    
    # Mission generating
    while len(done) < data.n_T:
        initial_todo = todo_generating(counter, transformed_ch)                 # Generate inital todo list
        todo = same_product_filtering(initial_todo, data.priority)        # Check if transports of same product (for each product)
        todo = operation_order_filtering(todo, done, data.priority)       # Check if previous transport exists in 'done' list

        if len(todo):       # If 'todo' list exists
            todo_robot = find_todo_robot(transformed_ch, todo)                  # This is the list of robots who make 'todo' list
            mission_updating(todo, todo_robot, mission, actual_position, actual_time, transport_time, data)    # Do 'todo'
            for _, idx in enumerate(todo_robot):    counter[idx] += 1           # Change to new round
            done += todo        # Update 'done' list

        else:               # Otherwise
            mission = None
            break

    return mission


def fitness_calculation(mission, lbd1=0.5, lbd2=0.5):
    """
    This function calculates the fitness value from mission
    For example,
    mission = [ [['movement', 8, 'depot0', 'stock'], ['transport', 5, 'stock', 'MP4'], ['waiting', 4, 'MP4']],
                [['movement', 12, 'depot1', 'MP2'], ['waiting', 18, 'MP2'], ['transport', 9, 'MP2', 'MP1']] ]
    --->>>
    fitness = 30.5
    """
    total_waiting_dt = 0    # Total waiting time of multi-robots
    makespan = 0            # Makespan of multi-robots
    for rm in mission:
        rob_ms = 0          # Makespan of each robot
        # Loop in mission of each robot
        for m in rm:
            rob_ms += m[1]
            if m[0] == 'waiting':       total_waiting_dt += m[1]
        # Filter maximum makespan
        if makespan < rob_ms:       makespan = rob_ms
    fitness_value = lbd1 * makespan + lbd2 * total_waiting_dt     # Calculate fitness value
    return fitness_value


def chromosome_fitness(ch, data, n_Random=1000, lbd1=0.5, lbd2=0.5):
    """
    This function returns fitness value, mission of a chromosome if validated
    """
    # Create transformation of chromosome : ch = [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0] --->>> transformed_ch = [[2, 3, 4, 5, 10, 11], [0, 1, 6, 7, 8, 9]]
    transformed_ch = chromosome_transformation(ch, data.n_R)

    # Create all possible missions
    mission_group = []
    for _ in range(n_Random):
        mission = mission_generating(transformed_ch, data)       # Mission of random transformed ch
        if mission:     mission_group.append(mission)       # Add to group of possible missions for each chromosome

    # Find best mission based on minimum fitness value
    fitness_min = 99999
    if mission_group:       # If the mission group exists
        for m in mission_group:
            f = fitness_calculation(m, lbd1, lbd2)
            if f < fitness_min:
                fitness_min = f
                mission_min = m
        return fitness_min, mission_min


def population_fitness(pop, data, n_Random=1000, lbd1=0.5, lbd2=0.5):
    """
    This function returns all fitness values, missions of a population. Pay attention to the population, which can be shortened
    """
    ch_pop, fitness_pop, mission_pop = [], [], []
    for ch in pop:
        validation = chromosome_fitness(ch, data, n_Random, lbd1, lbd2)
        # Add to new population
        if validation:
            ch_pop.append(ch)
            fitness_pop.append(validation[0])
            mission_pop.append(validation[1])
    return ch_pop, fitness_pop, mission_pop



#%% Crossover
def multi_site_crossover(p1, p2, n_sites=1):
    """
    This function returns childrens crossovered by multi-sites from 2 parents. n_sites are random.

    For example,
    p1 = [1, 1, 1, 0, 0]
    p2 = [0, 0, 1, 1, 0]
    --->>>
    n_sites = 2
    sites = [0, 2, 4, 5]
    c1 = [1, 1, 1, 1, 0]
    c2 = [0, 0, 1, 0, 0]
    """
    size = min(len(p1), len(p2))        # Get minimim size of 2 parents in case of different sizes

    # Create list of crossover points
    if n_sites < size:
        sites = random.sample(range(1, size), n_sites)
        sites.sort()
        sites = [0] + sites + [size]
    else:
        sites = list(range(size))

    # Create children
    c1, c2 = p1[:], p2[:]
    switch = 0
    for i in range(len(sites)-1):
        if switch == 1:
            c1[sites[i]:sites[i+1]] = p2[sites[i]:sites[i+1]]
            c2[sites[i]:sites[i+1]] = p1[sites[i]:sites[i+1]]
            switch = 0
        else:
            switch = 1
    return c1, c2


def crossover(parent_pop, n_sites=1):
    """
    This function returns a population of childrens crossovered by n_sites from a population of parents. n_sites are random.
    """
    child_pop = []
    # For each parent, do crossover with all other parents
    for i in range(len(parent_pop) - 1):
        for j in range(i + 1, len(parent_pop)):
            child1, child2 = multi_site_crossover(parent_pop[i], parent_pop[j], n_sites)
            child_pop.append(child1)
            child_pop.append(child2)
    return child_pop
    


#%% Mutation
def shuffle_indices(ch, p_Mut = 0.5):
    """
    This function does mutation of a chromosome by replacing a gen by another gen, whose position is random. The decision of replacement depends on random probability
    For example,
    ch = [0, 1, 2, 3, 4]
    --->>>
    ch = [4, 1, 0, 3, 2]
    """
    for i in range(len(ch)):
        if random.random() < p_Mut:   # Probability to decide whether there is mutation
            idx_swap = np.random.randint(0, len(ch)-2)
            if idx_swap >= i:       idx_swap += 1
            ch[i], ch[idx_swap] = ch[idx_swap], ch[i]
    return ch


def mutation(ch_pop, p_Mut = 0.5):
    """
    This function does mutation of a population of chromosomes
    For example,
    ch_pop = [[0, 1, 2, 3, 4], [5, 6, 7]]
    --->>>
    ch_pop = [[3, 4, 1, 0, 2], [7, 6, 5]]
    """  
    for i, child in enumerate(ch_pop):
        ch_pop[i] = shuffle_indices(child, p_Mut)
    return ch_pop



#%% Time function
def dt(type_of_action, init_pose=None, final_pose=None):
    """
    This function returns the time of different actions of robot
    """
    if type_of_action == 'operation':
        for i, ope in enumerate(poses):
            if init_pose == ope:
                dt = operation_dt[i]
                break

    elif type_of_action == 'manipulation':
        for i, mani in enumerate(poses):
            if init_pose == mani:
                dt = manipulation_dt[i]
                break

    elif type_of_action == 'movement':
        for i, init in enumerate(poses):
            for j, final in enumerate(poses):
                if init == init_pose and final == final_pose:
                    dt = movement_dt[i][j]

    elif type_of_action == 'transport':
        for i, init in enumerate(poses):
            for j, final in enumerate(poses):
                if init == init_pose and final == final_pose:
                    dt = manipulation_dt[i] + movement_dt[i][j] + manipulation_dt[j]
    return dt



#%% Mission simplification
def mission_simplification(mission):
    simplified_mission = []
    for rm in mission:
        robot_mission = []
        for m in rm:
            if m[0] == 'waiting':
                robot_mission.append('waiting')
                robot_mission.append('{}'.format(m[1]))
            if m[0] == 'movement':
                robot_mission.append('movement')
                robot_mission.append(m[3])
            if m[0] == 'transport':
                robot_mission.append('transport')
                robot_mission.append(m[2])
                robot_mission.append(m[3])

        simplified_mission.append(robot_mission)
    return simplified_mission




#%% Mission plotting
def mission_plot(mission):
    """
    This function plots mission of all robots
    """
    # Config of plot
    _, ax = plt.subplots(dpi=244)
    plt.xlim(-10, 800)
    plt.ylim(-1, 4)
    ax.set_title('Scheduling')
    ax.set_xlabel('Time')
    ax.set_ylabel('Robot')
    ax.legend(loc='upper right')

    # Plot mission of each robot
    for r, rm in enumerate(mission):
        type_of_mission = [rm[i][0] for i in range(len(rm))]    # List of types of mission

        dt_lst = [rm[i][1] for i in range(len(rm))]              # List of all dt
        time = [0] + [sum(dt_lst[:(i+1)]) for i in range(len(dt_lst))]      # List of all times

        y = [r for _ in range(len(rm)+1)]       # Robot r

        # Plot states
        for i in range(len(rm)):
            if type_of_mission[i] == 'waiting':
                plt.plot(time[i:i+2], y[i:i+2], ':k', linewidth=1)
            if type_of_mission[i] == 'movement':
                plt.plot(time[i:i+2], y[i:i+2], '-b', linewidth=4, dashes=[30, 5, 10, 5])
            if type_of_mission[i] == 'transport':
                plt.plot(time[i:i+2], y[i:i+2], '-r', linewidth=15, dashes=[30, 5, 10, 5])

    # Legend for states
    waiting = mlines.Line2D([], [], color='k', linestyle=':', label='waiting')
    movement = mlines.Line2D([], [], color='b', linewidth=2, label='movement')
    transport = mlines.Line2D([], [], color='r', linewidth=6, label='transport')
    plt.legend(handles=[waiting, movement, transport])

    plt.show()



#%% Publish messages to sequencer
def message_publishing(published_mission):
    message, pub = [], []
    for i in range(len(published_mission)):
        mission = Mission()
        mission.robot_name = 'robot{}'.format(i)
        mission.robot_mission = published_mission[i]
        message.append(mission)
        
        pub.append(rospy.Publisher('robot{}_mission'.format(i), Mission, queue_size=1))

    rospy.init_node('MRTA_Algorithm')

    for i in range(len(published_mission)):
        pub[i].publish(message[i])

    # rospy.spin()



#%% Input class
def pose_list(pose_chain):
    """
    This function changes the input of pose list from a chain to a list
    For example,
    pose_chain = [['MP1', 'MP2', 'MP3'], ['stock', 'AP', 'MP4']]
    pose_list = [['MP1', 'MP2'], ['MP2', 'MP3'], ['stock', 'AP'], ['AP', 'MP4']]
    """
    pose_list = []
    for pose in pose_chain:
        for j in range(len(pose) - 1):
            pose_list.append([pose[j], pose[j+1]])
    return pose_list


def priority(prod_list):
    """
    This function return the priority and number of transports from product list
    For example,
    products = [['stock', 'AP', 'MP1', 'MP2', 'MP4', 'MP3', 'stock'], ['stock', 'MP4', 'MP2', 'MP1', 'MP3', 'MP1', 'stock']]
    prod_list = [0, 1, 1]          # 2 products of type 0 and 1
    --->>>
    priority = [[0, 1, 2, 3, 4, 5], [6, 7, 8, 9, 10, 11], [12, 13, 14, 15, 16, 17]]         # Transport numberized by product
    """
    priority = []
    prod_size = [len(products[prod]) - 1 for prod in prod_list]     
    prod_threshold = [0] + [sum(prod_size[:(i+1)]) for i in range(len(prod_size))]
    for i in range(len(prod_threshold) - 1):
        priority.append(list(range(prod_threshold[i], prod_threshold[i+1])))
    return priority


class Inputs(object):
    """
    This function defines all data inside classe 'Inputs'
    """
    def __init__(self, products, prod_list, n_R):
        self.n_R = n_R                                              # Number of robots
        self.priority = priority(prod_list)             # priority and number of transports
        self.n_T = sum(len(sublst) for sublst in self.priority)               # Number of transports
        self.robot = [i for i in range(n_R)]                        # List of numberized robots, for example [0, 1, 2]
        self.pose_chain = [products[p] for p in prod_list]          # Chain of poses of all products : pose_chain = [['MP1', 'MP2', 'MP3'], ['stock', 'AP', 'MP4']]
        self.pose_list = pose_list(self.pose_chain)                 # List of pose : pose_list = [['MP1', 'MP2'], ['MP2', 'MP3'], ['stock', 'AP'], ['AP', 'MP4']]



#%% Usecase information

# products = [['stock', 'AP', 'MP1', 'MP2', 'MP4', 'MP3', 'stock'], ['stock', 'MP4', 'MP2', 'MP1', 'MP3', 'MP1', 'stock']]
products = [['stock', 'MP3', 'MP2', 'stock'], ['stock', 'MP4', 'MP2', 'AP', 'stock']]

poses = ['MP1', 'MP2', 'MP3', 'MP4', 'AP', 'stock', 'depot0', 'depot1', 'depot2']

# MP1 MP2 MP3 MP4 AP Stock depot0 depot1 depot2
operation_dt = [60, 60, 60, 60, 60, 0, 0, 0, 0]

# MP1 MP2 MP3 MP4 AP Stock depot0 depot1 depot2
manipulation_dt = [16.1,    16.4,   16.3,   16.2,   15.3,   15.2,   0,  0,  0]

# MP1 MP2 MP3 MP4 AP Stock depot0 depot1 depot2
movement_dt = [ [0,     6.8,    26.6,   27.6,   29.4,   30.2,   40.2,   38.8,   37.6],
                [33.6,  0,      24.4,   26.8,   45.6,   28.6,   38.2,   36.8,   34.2],
                [26.4,  25.4,   0,      9,      35,     12.8,   22.6,   22,     20  ],
                [21.6,  24,     12.8,   0,      36.2,   10.6,   20.6,   19.4,   18.2],
                [41.6,  36.6,   34.6,   32.2,   0,      21.8,   14.2,   15.4,   15  ],
                [19.6,  22,     15.6,   12.6,   31.8,   0,      21.8,   21.2,   19.2],
                [41.4,  36.2,   33.2,   31.8,   21.2,   22.2,   0,      10.4,   11  ],
                [38.4,  35,     31,     28,     21.6,   20.6,   10.2,   0,      9   ],
                [36.6,  34.8,   29.8,   27,     20.6,   18,     10.4,   10.8,   0   ]]



#%% GA parameters

# Population size, number of generations
n_Pop, n_G, n_Random = 20, 30, 100

# Fitness --->>> coefficients in fitness function
lbd1, lbd2 = 0.5, 0.5

# Mating pool --->>> ratios to choose parents
best_ratio, worst_ratio = 0.2, 0.2

# Mutation probability, crossover probability and Crossover direction probability
p_Mut = 0.5
n_sites = 3



#%% Main function
def main(prod_list, n_R):

    # Generate data from inputs
    data = Inputs(products, prod_list, n_R)

    # Initial population
    ch_pop, fitness_ch_pop, mission_ch_pop = initial_population(data, n_Pop, n_Random, lbd1, lbd2)

    # Generate loop
    for _ in range(n_G):
        # Parents to mate
        parent_pop, _ = filtered_population(ch_pop, fitness_ch_pop, best_ratio=best_ratio, worst_ratio=worst_ratio)
        mission_parent_pop, fitness_parent_pop = filtered_population(mission_ch_pop, fitness_ch_pop, best_ratio=best_ratio, worst_ratio=worst_ratio)
        
        # Crossover
        child_pop = crossover(parent_pop, n_sites)

        # Mutation
        child_pop = mutation(child_pop, p_Mut)

        # Filter population of children
        child_pop, fitness_child_pop, mission_child_pop = population_fitness(child_pop, data, n_Random, lbd1, lbd2)     # Filter child_pop 
        if len(child_pop) > n_Pop - len(parent_pop):        # Limit number of children
            child_pop, _ = filtered_population(child_pop, fitness_child_pop, n_choosen=n_Pop - len(parent_pop))
            mission_child_pop, fitness_child_pop = filtered_population(mission_child_pop, fitness_child_pop, n_choosen=n_Pop - len(parent_pop))
        
        # Create new population of chromosomes
        ch_pop = parent_pop + child_pop         # New population to mate
        fitness_ch_pop = fitness_parent_pop + fitness_child_pop
        mission_ch_pop = mission_parent_pop + mission_child_pop
        


    # Best solution
    ch_pop, _ = filtered_population(ch_pop, fitness_ch_pop)
    mission_ch_pop, fitness_ch_pop = filtered_population(mission_ch_pop, fitness_ch_pop)
    best_ch = ch_pop[0]
    best_fitness = fitness_ch_pop[0]
    best_mission = mission_ch_pop[0]
    for i in range(n_R):    best_mission[i].append(['movement', dt('movement', 'stock', 'depot{}'.format(i)), 'stock', 'depot{}'.format(i)])    # Add movement to depots

    print 'The best chromosome', best_ch
    print 'The best fitness', best_fitness
    print 'The best mission', best_mission
    print 'Published mission', mission_simplification(best_mission)

    # Publish message and plot scheduling
    mes_pub = Process(target=message_publishing(mission_simplification(best_mission)))
    mes_pub.start()
    mis_plot = Process(target=mission_plot(best_mission))
    mis_plot.start()
    mes_pub.join()      # Publish mission by message
    mis_plot.join()     # Plot mission



#%% Run main
if __name__ == '__main__':

    # Product 0 : ['stock', 'AP', 'MP1', 'MP2', 'MP4', 'MP3', 'stock']
    # Product 1 : ['stock', 'MP4', 'MP2', 'MP1', 'MP3', 'MP1', 'stock']

    # Product 0 : ['stock', 'MP3', 'MP2', 'stock']
    # Product 1 : ['stock', 'MP4', 'MP2', 'AP', 'stock']

    prod_list = [0, 1]          # List of products
    n_R = 2                     # Number of robots

    try:
        main(prod_list, n_R)
    except rospy.ROSInterruptException:
        pass
