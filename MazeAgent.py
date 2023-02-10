import heapq
import collections


class MazeAgent(object):
    '''
    Agent that uses path planning algorithm to figure out path to take to reach goal
    Built for Malmo discrete environment and to use Malmo discrete movements
    '''

    def __init__(self, grid=None, path_alg=None):
        '''
        Arguments
            grid -- (optional) a 2D list that represents the map
            path_alg -- (optional) a string that represents path algorithm to be
                used. Should be "bf" or "astar"
        '''
        self.__frontier_set = [] # Depends on the case what DSA it is
        self.__explored_set = {} # Depends on case for DSA
        self.__goal_state = None
        self.__grid = grid # Sent over thru set method from env sim run
        if (path_alg is None): # Alg given on agent init 
            self.__path_alg = "bf" 
        else:
            self.__path_alg = path_alg


    def get_eset(self):
        return self.__explored_set

    def get_fset(self):
        return self.__frontier_set

    def get_goal(self):
        return self.__goal_state

    def set_grid(self, grid):
        self.__grid = grid

    def __plan_path_breadth(self, curr_node, goal_node):
        '''Breadth-First tree search
            Arguments:
                curr_node -- Current node of algorithm
                goal_node -- The goal node to which the agent must travel
            Returns:
                ordered list that gives set of commands to move to from init to
                 goal position
        '''
        
        # Starting Inits #
        directions = ((1,0,0), (-1,0,1), (0,-1,2), (0,1,3)) # Tuple of sets of directions coordinates: South is forward 
        commands = ("movenorth 1", "movesouth 1", "moveeast 1", "movewest 1") # Tuple of strs of commands to return
        rows = len(self.__grid) # get the length of rows to iterate over 
        cols = len(self.__grid[0]) # get the length of columns to iterate over 

        # Configure the BFS with the starting data #
        self.__frontier_set = collections.deque() # FIFO Queue
        self.__frontier_set.append(curr_node) 
        self.__explored_set[curr_node] = [] # HASH MAP -> [ KEY->(i,j), VALUE->[CUR.SOL] ] ; kind of comb. the ex. set with a soln. map

        while self.__frontier_set:

            i,j = self.__frontier_set.popleft()
            
            for rowDir, colDir, index in directions:
                newRow = rowDir + i # calculate the new blocks row we are testing
                newCol = colDir + j # calculate the new blocks col we are testing

                if newRow in range(rows) and newCol in range(cols):
                    
                    if self.__grid[newRow][newCol] == 3: # GOAL BLOCK RETURN
                        self.__explored_set[(newRow, newCol)] = self.__explored_set[(i, j)][:]
                        self.__explored_set[(newRow, newCol)].append(commands[index])
                        soln = self.__explored_set[(newRow, newCol)][::-1] # reverse the list, that is how it is parsed

                        return soln

                    elif self.__grid[newRow][newCol] == 1 and (newRow, newCol) not in self.__explored_set:  # VALID BLOCK
                        self.__frontier_set.append((newRow, newCol))
                        self.__explored_set[(newRow, newCol)] = self.__explored_set[(i, j)][:] # Want a new memory chunk
                        self.__explored_set[(newRow, newCol)].append(commands[index]) # add new command to get to this new path onto curr soln

        print("Nil Path") # didnt find a path, oh well
        return []
        
    def heuristic(self, node):
        # MANHATTAN DISTANCE
        x1, y1 = node
        x2, y2 = self.get_goal()
        return abs(x1 - x2) + abs(y1 - y2)
        

    def __plan_path_astar(self, curr_node, goal_node):
        '''A* tree searchReturns:
            ordered list that gives set of commands to move to from init to
             goal position

            Arguments:
                curr_node -- current node to be processed (may be initial node)
                goal_node -- goal node to be tested
            Returns:
                ordered list that gives set of commands to move to from init to
                 goal position
        '''
        # Starting Inits #
        directions = ((1,0,0), (-1,0,1), (0,-1,2), (0,1,3)) # Tuple of sets of directions coordinates: South is forward 
        commands = ("movenorth 1", "movesouth 1", "moveeast 1", "movewest 1") # Tuple of strs of commands to return
        rows = len(self.__grid) # get the length of rows to iterate over 
        cols = len(self.__grid[0]) # get the length of columns to iterate over

        # Configure the BFS with the starting data #
        self.__frontier_set = [] 
        heapq.heappush(self.__frontier_set, (0,curr_node)) # Priority queue, cough cough, a heap || Formated by (weighted heuristic, (i,j))
        self.__explored_set[curr_node] = [] # HASH MAP -> [ KEY->(i,j), VALUE->[CUR.SOL] ] ; kind of comb. the ex. set with a soln. map
        
        while self.__frontier_set:
            score, point = heapq.heappop(self.__frontier_set)
            i, j = point

            for rowDir, colDir, index in directions:
                newRow = rowDir + i # calculate the new blocks row we are testing
                newCol = colDir + j # calculate the new blocks col we are testing

                if newRow in range(rows) and newCol in range(cols):
                    
                    if self.__grid[newRow][newCol] == 3: # GOAL BLOCK RETURN BOI
                        self.__explored_set[(newRow, newCol)] = self.__explored_set[(i, j)]
                        self.__explored_set[(newRow, newCol)].append(commands[index])
                        soln = self.__explored_set[(newRow, newCol)][::-1] # reverse the list, that is how it is parsed

                        return soln

                    elif self.__grid[newRow][newCol] == 1 and (newRow, newCol) not in self.__explored_set:  # VALID BLOCK
                        self.__explored_set[(newRow, newCol)] = self.__explored_set[(i, j)][:] # Want a new memory chunk
                        self.__explored_set[(newRow, newCol)].append(commands[index]) # add new command to get to this new path onto curr soln
                        hX = self.heuristic((newRow, newCol)) # Get distance from new node to goal node
                        gX = len(self.__explored_set[(newRow, newCol)]) + hX # Get distance of path traveled so far (1 action = 1 length unit)
                        heapq.heappush(self.__frontier_set, (gX, (newRow, newCol))) # add our new weight and cell onto the heap

        print("Nil Path") # didnt find a path, oh well
        return []

    def get_path(self):
        '''should return list of strings where each string gives movement command
            (these should be in order)
            Example:
             ["movenorth 1", "movesouth 1", "moveeast 1", "movewest 1"]
             (these are also the only four commands that can be used, you
             cannot move diagonally)
             On a 2D grid (list), "move north" would move us
             from, say, [0][0] to [1][0]
        '''

        # GET LENGTHS OF MATRIX #
        rows = len(self.__grid) # get the length of rows to iterate over 
        cols = len(self.__grid[0]) # get the length of columns to iterate over 

        # INIT OUR STATES #
        startNode = None # Set pair
        endNode = None # Set pair

        # FIND THE | START[i][j] == 2 | and | GOAL [i][j] == 3 | #
        for i in range(rows):
            for j in range(cols):
                if self.__grid[i][j] == 2:
                    startNode = (i,j) # Found our starting node pos
                elif self.__grid[i][j] == 3:
                    endNode = (i,j) # Found our end node pos
                    self.__goal_state = endNode
        
        return self.__plan_path_breadth(startNode, endNode) if self.__path_alg == "bf" else self.__plan_path_astar(startNode, endNode) # Return the path for desired algorithm specified
