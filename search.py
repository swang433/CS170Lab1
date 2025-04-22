# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to University of California, Riverside and the authors.
# 
# Authors: Pei Xu (peixu@stanford.edu) and Ioannis Karamouzas (ioannis@cs.ucr.edu)
#


"""
 In this assignment, the task is to implement different search algorithms to find 
 a path from a given start cell to the goal cell in a 2D grid map.

 To complete the assignment, you must finish four functions:
   depth_first_search (line 148), breadth_first_search (line 232), 
   uniform_cost_search (line 280), and astar_search (line 327).

 During the search, a cell is represented using a tuple of its location
 coordinates.
 For example:
   the cell at the left-top corner is (0, 0);
   the cell at the first row and second column is (0, 1);
   the cell at the second row and first column is (1, 0).
 You need put these tuples into the open set or/and closed set properly
 during searching.
"""

# ACTIONS defines how to reach an adjacent cell from a given cell
# Important: please check that a cell within the bounds of the grid
# when try to access it.
ACTIONS = (
    (-1,  0), # go up
    ( 0, -1), # go left
    ( 1,  0), # go down
    ( 0,  1)  # go right
)

from utils.search_app import OrderedSet, Stack, Queue, PriorityQueue
"""
 Four different structures are provided as the containers of the open set
 and closed set.

 OrderedSet is an ordered collections of unique elements.

 Stack is an LIFO container whose `pop()` method always pops out the last
 added element. 

 Queue is an FIFO container whose `pop()` method always pops out the first
 added element.

 PriorityQueue is a key-value container whose `pop()` method always pops out
 the element whose value has the highest priority.

 All of these containers are iterable but not all of them are ordered. Use
 their pop() methods to ensure elements are popped out as expected.


 Common operations of OrderedSet, Stack, Queue, PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s


 Unique operations of OrderedSet:
   s.add(x): add the element x to the set s;
             nothing will be done if x is already in s
   s.pop(): return and remove the LAST added element in s;
            raise IndexError if s is empty 
   s.pop(last=False): return and remove the FIRST added element in s;
            raise IndexError if s is empty 
 Example:
   s = Set()
   s.add((1,2))    # add a tuple element (1,2) to the set
   s.remove((1,2)) # remove the tuple element (1,2) from the set
   s.add((1,1))
   s.add((2,2))
   s.add((3,3))
   x = s.pop()
   assert(x == (3,3))
   assert((1,1) in s and (2,2) in s)
   assert((3,3) not in s)
   x = s.pop(last=False)
   assert(x == (1,1))
   assert((2,2) in s)
   assert((1,1) not in s)
   

 Unique operations of Stack:
   s.add(x): add the element x to the back of the stack s
   s.pop(): return and remove the LAST added element in the stack s;
            raise IndexError if s is empty
 Example:
   s = Stack()
   s.add((1,1))
   s.add((2,2))
   x = s.pop()
   assert(x == (2,2))
   assert((1,1) in s)
   assert((2,2) not in s)


 Unique operations of Queue:
   s.add(x): add the element x to the back of the queue s
   s.pop(): return and remove the FIRST added element in the queue s;
            raise IndexError if s is empty
 Example:
   s = Queue()
   s.add((1,1))
   s.add((2,2))
   x = s.pop()
   assert(x == (1,1))
   assert((2,2) in s)
   assert((1,1) not in s)


 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v to the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="max", f=lambda v: abs(v))
   s.put((1,1), -1)
   s.put((2,2), -20)
   s.put((3,3), 10)
   x, v = s.pop()  # the element with maximum value of abs(v) will be popped
   assert(x == (2,2) and v == -20)
   assert(x not in s)
   assert(x.get((1,1)) == -1)
   assert(x.get((3,3)) == 10)
"""


# use math library if needed
import math

def depth_first_search(grid_size, start, goal, obstacles, costFn, logger):
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    open_set = Stack()
    closed_set = OrderedSet()

    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set

    parent = [ #reverse map
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
    actions = [
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
   
    movement = []
    
    # Add the start node to the open set
    open_set.add(start)
    
    while open_set:
        # Get the next node to expand and pop stack
        current_row, current_col = open_set.pop()
        
        # Check if current node is the goal
        if (current_row, current_col) == goal:
            pathNode = goal
            while pathNode != start: #retrace traversal if goal is reached 
                parentNode = parent[pathNode[0]][pathNode[1]]
                movement.insert(0, actions[pathNode[0]][pathNode[1]]) #insert at the front so theres no need to append then reverse 
                pathNode = parentNode
            return movement, closed_set
        
        if (current_row, current_col) not in closed_set: #mark as explored and put in closed set
            closed_set.add((current_row, current_col)) #add to closed set
            for move in ACTIONS: 
                dr = move[0]
                dc = move[1]
                newRow = current_row + dr
                newCol = current_col + dc
                
                if (0 <= newRow < n_rows and 0 <= newCol < n_cols and #check if in bounds
                    (newRow, newCol) not in closed_set and #check if already explored
                    (newRow, newCol) not in open_set and #check if already in open set
                    (newRow, newCol) not in obstacles):

                    cost = costFn((newRow, newCol))
                    
                    open_set.add((newRow, newCol))
                    parent[newRow][newCol] = (current_row, current_col) #set as child to the previous node
                    actions[newRow][newCol] = move #set the action taken to get to this node
    
    # If we get here, no path was found
    return movement, closed_set

def breadth_first_search(grid_size, start, goal, obstacles, costFn, logger):
    """
    BFS algorithm finds the path from the start cell to the
    goal cell in the 2D grid world.
    
    After expanding a node, to retrieve the cost of a child node at location (x,y), 
    please call costFn((x,y)). 

    See depth_first_search() for details.
    """
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    ##########################################
    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = Queue()
    closed_set = OrderedSet()

    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set

    parent = [ # the immediate predecessor cell from which to reach a cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
    actions = [ # the action that the parent took to reach the cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]

    movement = []

    open_set.add(start)

    while open_set: 

        currRow, currCol = open_set.pop()

        if (currRow, currCol) == goal:
            pathNode = goal
            while pathNode != start: 
                parentNode = parent[pathNode[0]][pathNode[1]] #find previously mapped parent
                movement.insert(0, actions[pathNode[0]][pathNode[1]]) #backtrack movement and insert movement at the front of list of movements
                pathNode = parentNode
            return movement, closed_set
        
        if (currRow, currCol) not in closed_set: 
            closed_set.add((currRow, currCol))
            for move in ACTIONS: 
                newRow = currRow + move[0]
                newCol = currCol + move[1]
                
                #new node boundary check
                if (0 <= newRow < n_rows and 0 <= newCol < n_cols and #check if in bounds
                    (newRow, newCol) not in closed_set and #check if already explored
                    (newRow, newCol) not in open_set and #check if already in open set
                    (newRow, newCol) not in obstacles):
                    
                    #cost Fn
                    cost = costFn((newRow, newCol))
                    # print(f"Cost: {cost}")

                    open_set.add((newRow, newCol))
                    parent[newRow][newCol] = (currRow, currCol) #set as child to the previous node
                    actions[newRow][newCol] = move #set the action taken to get to this node


    return movement, closed_set


def uniform_cost_search(grid_size, start, goal, obstacles, costFn, logger):
    """
    Uniform-cost search algorithm finds the optimal path from 
    the start cell to the goal cell in the 2D grid world. 
    
    After expanding a node, to retrieve the cost of a child node at location (x,y), 
    please call costFn((x,y)). 

    See depth_first_search() for details.
    """
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    ##########################################
    # Choose a proper container yourself from
    # OrderedSet, Stack, Queue, PriorityQueue
    # for the open set and closed set.
    open_set = PriorityQueue(order = "min", f = lambda v: abs(v))
    closed_set = OrderedSet()
    ##########################################

    ##########################################
    # Set up visualization logger hook
    # Please do not modify these four lines
    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set
    ##########################################

    parent = [ # the immediate predecessor cell from which to reach a cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
    actions = [ # the action that the parent took to reach the cell
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]

    # put here?
    lowCost = [
        [math.inf for __ in range(n_cols)] for _ in range(n_rows)
    ]   # this will be to find the neighbors
        # will be used to branch out to the neighbors to find the lowest cumulative cost

    movement = []
    # ----------------------------------------
    # finish the code below
    # ----------------------------------------
#############################################################################
    initCost = 0

    open_set.put(start, 0)
    lowCost[start_row][start_col] = initCost
    
    while open_set:
        (curr_row, curr_col), curr_cost = open_set.pop() # eg (1,1), -1

        #check if goal
        if (curr_row, curr_col) == goal:
            #reconstruct path
            pathNode = (goal_row, goal_col)
            while pathNode != start:
                parentNode = parent[pathNode[0]][pathNode[1]]
                movement.insert(0, actions[pathNode[0]][pathNode[1]])
                pathNode = parentNode
            return movement, closed_set
        
        if ((curr_row, curr_col)) not in closed_set:
            closed_set.add((curr_row, curr_col))

            for move in ACTIONS:
                newRow = curr_row + move[0]
                newCol = curr_col + move[1]

                if (0 <= newRow < n_rows and 0 <= newCol < n_cols and #check if in bounds
                    (newRow, newCol) not in closed_set and #check if already explored
                    (newRow, newCol) not in open_set and #check if already in open set
                    (newRow, newCol) not in obstacles):
                        
                        # moveCost = costFn((newRow, newCol))
                        new = curr_cost + costFn((newRow, newCol))

                        # we want to see if the neighbors are cheaper
                        if new < lowCost[newRow][newCol]:
                            lowCost[newRow][newCol] = new
                            # put on open queue to explore
                            open_set.put((newRow, newCol), new)
                            parent[newRow][newCol] = (curr_row, curr_col) #set as child to the previous node
                            actions[newRow][newCol] = move #set the action taken to get to this node



#############################################################################
    return movement, closed_set

def astar_search(grid_size, start, goal, obstacles, costFn, logger):
    n_rows, n_cols = grid_size
    start_row, start_col = start
    goal_row, goal_col = goal

    def heuristic(row, col):
        return abs(row - goal_row) + abs(col - goal_col)

    ##########################################

    open_set = PriorityQueue(order="min", f=lambda v: abs(v))
    closed_set = OrderedSet()

    closed_set.logger = logger
    logger.closed_set = closed_set
    open_set.logger = logger
    logger.open_set = open_set
    ##########################################
    
    parent = [
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]
    actions = [
        [None for __ in range(n_cols)] for _ in range(n_rows)
    ]

    lowCost = [
        [math.inf for __ in range(n_cols)] for _ in range(n_rows)
    ]
    
    movement = []
    initCost = 0

    f_cost = heuristic(start_row, start_col)
    open_set.put(start, f_cost)
    lowCost[start_row][start_col] = initCost

    while open_set: 
        (currRow, currCol), curr_f_cost = open_set.pop()

        if (currRow, currCol) == goal: #backtrack
            pathNode = goal
            while pathNode != start:
                parentNode = parent[pathNode[0]][pathNode[1]]
                movement.insert(0, actions[pathNode[0]][pathNode[1]])
                pathNode = parentNode
            return movement, closed_set
        
        if (currRow, currCol) not in closed_set: #check if already fully explored
            closed_set.add((currRow, currCol))
            
            for move in ACTIONS:
                newRow = currRow + move[0]
                newCol = currCol + move[1]

                if (0 <= newRow < n_rows and 0 <= newCol < n_cols and
                    (newRow, newCol) not in closed_set and
                    (newRow, newCol) not in obstacles):
                    
                    #calculate node costs individually
                    g_cost = lowCost[currRow][currCol] + costFn((newRow, newCol))
                    
                    #check if needs updating
                    if g_cost < lowCost[newRow][newCol]:
                        lowCost[newRow][newCol] = g_cost
                        
                        #find f(n)
                        f_cost = g_cost + heuristic(newRow, newCol)
                        
                        #update or add
                        open_set.put((newRow, newCol), f_cost)
                            
                        parent[newRow][newCol] = (currRow, currCol)
                        actions[newRow][newCol] = move

    return movement, closed_set

if __name__ == "__main__":
    # make sure actions and cost are defined correctly
    from utils.search_app import App
    assert(ACTIONS == App.ACTIONS)

    import tkinter as tk

    algs = {
        "Breadth-First Search": breadth_first_search,
        "Depth-First Search": depth_first_search,
        "Uniform Cost Search": uniform_cost_search,
        "A* Search": astar_search
    }

    root = tk.Tk()
    App(algs, root)
    root.mainloop()