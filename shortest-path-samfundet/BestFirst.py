from Map import Map_Obj

# Variables
task_num = 4
timeout_iterations = 2000

# Returns a dictionary with all the nodes after using the A* algorithm on the map_obj
def best_first(map_obj, timeout_iterations):
    map = map_obj
    timeout = timeout_iterations
    nodes = {}
    open_list = []
    closed_list = []
    start_node = tuple(map.get_start_pos())
    goal_node = tuple(map.get_end_goal_pos())

    # Functions for the A*-algorithm to use

    # Returns the estimated cost to go to the goal for a node
    def estimate_h_cost(pos):
        goal = map.get_goal_pos()
        remaining_y = abs(pos[0]-goal[0])
        remaining_x = abs(pos[1]-goal[1])
        return remaining_x + remaining_y

    # Fills the nodes (dictionary) with information from map
    def init_nodes():
        map_values = map.get_maps()[0]
        nodes_dict = {}
        for i in range (0,len(map_values)):
            for j in range(0,len(map_values[i])):
                nodes_dict[(i,j)] = {"cost":map_values[i][j], "parent":None, "g":None, "h":estimate_h_cost((i,j))}
        start = tuple(map.get_start_pos())
        nodes_dict[start]["g"]=0
        return nodes_dict

    # Returns the sum of the cost of getting to the spot (g-cost) and to the goal (h-cost)
    def f(pos):
        if nodes.__contains__(pos) and nodes[pos]["cost"] != -1 and nodes[pos]["g"]:
            return nodes[pos]["g"] + nodes[pos]["h"]

    # Returns all the neighboring nodes that is not an obstacle
    def find_neighbors(pos):
        list = []
        for node in [(pos[0],pos[1]+1),(pos[0],pos[1]-1),(pos[0]+1,pos[1]),(pos[0]-1,pos[1])]:
            if nodes.__contains__(node) and nodes[node]["cost"] != -1:
                list.append(node)
        return list

    # Returns the g-cost of a node
    def g(pos):
        return nodes[pos]["g"]

    # Returns the cost to go from a parent node to a child node
    def cost(pos):
        return nodes[pos]["cost"]

    # Sets the g-cost of a node
    def set_g(pos,value):
        nodes[pos]["g"] = value

    # Sets the parent of a node
    def set_parent(pos_child, pos_parent):
        nodes[pos_child]["parent"] = pos_parent

    # Returns the node with the lowest f-cost, additionally removes it from open_list
    def remove_least_node():
        if len(open_list)==0:
            return None
        elif len(open_list)==1:
            least = open_list[0]
            open_list.remove(least)
            return least
        least_node= open_list[0]
        least_f = f(least_node)
        for i in range(1,len(open_list)):
            f_new = f(open_list[i])
            if f_new<least_f:
                least_node = open_list[i]
                least_f = f_new
        open_list.remove(least_node)
        return least_node

    # A*-algorithm:
    nodes = init_nodes()
    open_list.append(start_node)
    current_node = ()
    times = 1
    while len(open_list)>0 and times<timeout:
        times += 1
        current_node = remove_least_node()
        if current_node == goal_node:
            break
        closed_list.append(current_node)
        neighbors = find_neighbors(current_node)
        for successor_node in neighbors:
            successor_current_cost = g(current_node) + cost(successor_node)
            if successor_node in closed_list:
                pass
            elif not successor_node in open_list:
                set_g(successor_node,successor_current_cost)
                set_parent(successor_node, current_node)
                open_list.append(successor_node)
            elif successor_current_cost<g(successor_node):
                set_g(successor_node,successor_current_cost)
                set_parent(successor_node, current_node)
    print(f"Iterations: {times}")
    if current_node==goal_node:
        print(f"The goal node is reached")
        return nodes
    else:
        print(f"The algorithm timed out at {timeout} loops")
        return None

# Uses map to draw the path to the ending node
def draw_path(map_obj, node_dict, timeout_iterations, node_end=None):
    map = map_obj
    timeout = timeout_iterations
    nodes = node_dict
    path = []
    if not node_end:
        node_end = tuple(map.get_end_goal_pos())

    # Returns parent of a node
    def parent(pos):
        return nodes[pos]["parent"]

    # Fills the path_list with the nodes from the starting point to the given node, default is to the goal_node
    def init_path(pos=node_end):
        path_list = []
        current_node = pos
        path_list.append(current_node)
        times = 1
        while parent(current_node)!=None and times<timeout_iterations:
            times += 1
            current_node = parent(current_node)
            path_list.insert(0,current_node)
        return path_list
        
    path = init_path(tuple(map.get_end_goal_pos()))
    for node in path:
        map.set_cell_value(node,";")
    map.show_map()

#Run
def run(task_num, timeout):
    if task_num not in [1,2,3,4]:
        return print(f"{task_num} is not one of the tasks I have done. Use 1,2,3 or 4 next time.")
    map = Map_Obj(task=task_num)
    nodes = best_first(map, timeout)
    draw_path(map,nodes,timeout)
  
run(task_num, timeout_iterations)