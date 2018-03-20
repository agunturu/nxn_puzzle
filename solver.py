#! /usr/bin/python
import sys
import resource
import collections
import itertools
import time
import heapq
import pdb

method=sys.argv[1]
stateString=sys.argv[2]

def printUsage():
    print("Usage: " + sys.argv[0] + " <method> <board>")
    print("     : where method is bfs or dfs or ast")
    print("     :       board is a comma separated list of integers with no spaces")

    if len(sys.argv) != 3:
        print("Error: Invalid number of arguments")
        printUsage()
        exit(-1)

class StatePQ:
    """
    This class implements a priority queue for state objects. This class
    uses heapq module to implement the priority queue.
    """

    def __init__(self):
        self.pq = []
        self.entryFinder = {}
        self.REMOVED = '<removed-task>'
        self.counter = itertools.count()

    def put(self, s):
        """
        This function inserts a state object into the priority queue. This function
        doesn't check if the an object with same state already exists. The caller
        must ensure that an object with same state doesn't already exist. This
        function uses the getEstimatedCost() function of the state object to
        determine the priority.
  
        Arguments:
        s -- an object of the class State
        """
        count = next(self.counter)
        entry = [s.getEstimatedCost(), count, s]
        self.entryFinder[s.boardString] = entry
        heapq.heappush(self.pq, entry)

    def get(self):
        """
        This function dequeues and returns an object with the lowest priority
        in the queue. If the queue is empty it returns None.
        """
        while self.pq:
            p, count, s = heapq.heappop(self.pq)
            if s is not self.REMOVED:
                del self.entryFinder[s.boardString]
                return s

        return None

    def remove(self, s):
        """
        This function removes a state object from the queue.

        Arguments:
        s -- an object of the class State
        """
        entry = self.entryFinder.pop(s.boardString, None)
        if entry is not None:
            entry[-1] = self.REMOVED

    def updatePriority(self, n):
        """
        This function updates the priority of a state object.

        Arguments:
        n -- an object of the class State
        """
        entry = self.entryFinder[n.boardString]
        if entry[2].getEstimatedCost() > n.getEstimatedCost():
            self.remove(entry[2])
            self.put(n)

    def clear(self):
        while self.pq:
            self.pq.get()
        self.entryFinder.clear()

    def isExists(self, s):
        """
        This function checks if a state object exists in the queue.

        Arguments:
        n -- an object of the class State
        """

        if s.boardString in self.entryFinder:
            return True
        else:
            return False

    def isEmpty(self):
        """
        This function checks if the queue is empty.
        """
        if self.entryFinder:
            return False
        else:
            return True

class StateDeque:
    """
    This class implements a FIFO and LIFO for state objects. In addition to
    providing the functions for queueing and dequeing, this class also
    provides an efficient function to check if a state already exists in
    the queue.
    """

    def __init__(self):
        self.q = collections.deque()
        self.qSet = set()

    def append(self, s):
        self.q.append(s)
        self.qSet.add(s.boardString)

    def appendleft(self, s):
        self.q.appendleft(s)
        self.qSet.add(s.boardString)

    def pop(self):
        s = self.q.pop()
        self.qSet.remove(s.boardString)
        return s
 
    def popleft(self):
        s = self.q.popleft()
        self.qSet.remove(s.boardString)
        return s

    def clear(self):
        self.q.clear()
        self.qSet.clear()

    def isExists(self, s):
        if s.boardString in self.qSet:
            return True
        else:
            return False

    def isEmpty(self):
        if self.q:
            return False
        else:
            return True

class State:
    """
    This class represents a state of the n-puzzle"
    """
    GOAL = [0, 1, 2, 3, 4, 5, 6, 7, 8]

    def __init__(self, board, parent=None):
        self.board = board
        self.boardString = ''.join(map(str, board))
        self.parent = parent
        self.path = ""
        if parent is None:
            self.depth = 0
        else:
            self.depth = parent.depth + 1

    def getPathToGoal(self):
        path = []
        p = self

        while p.parent is not None:
            path.append(p.path)
            p = p.parent

        path.reverse()
        return path

    def isGoalState(self):
        if self.board == State.GOAL:
            return True
        else:
            return False

    def getManhattanDistance(self):
        """
        This function returns the manhatten distance for this node.
        """
        return sum(abs(val%3 - i%3) + abs(val//3 - i//3) for i, val in enumerate(self.board) if val)
  
    def getEstimatedCost(self):
        """
        This function retuns the estimated cost of the cheapest 
        solution through this node. If uses node depth to estimate the cost to reach
        this node and manhanttan distance to estimate the cost to from this node to
        the goal.
        """
        return self.depth + self.getManhattanDistance()

    def getNeighbors(self):
        """
        This function returns the list neighbors at the current state.
        """
        i = self.board.index(0)
  
        if i in [0, 1, 2]:
            """When the blank is in the first row, it can't move up"""
            up = None
        else:
            temp = list(self.board)
            temp[i] = temp[i-3]
            temp[i-3] = 0
            up = State(temp, self)
            up.path = "Up"
   
        if i in [6, 7, 8]:
            """When the blank is in the last row, it can't move down"""
            down = None
        else:
            temp = list(self.board)
            temp[i] = temp[i+3]
            temp[i+3] = 0
            down = State(temp, self)
            down.path = "Down"
  
        if i in [0, 3, 6]:
            """When the blank is in the left most row, it can't move left"""
            left = None
        else:
            temp = list(self.board)
            temp[i] = temp[i-1]
            temp[i-1] = 0
            left = State(temp, self)
            left.path = "Left"
  
        if i in [2, 5, 8]:
            """When the blank is in the right most row, it can't move right"""
            right = None
        else:
            temp = list(self.board)
            temp[i] = temp[i+1]
            temp[i+1] = 0
            right = State(temp, self)
            right.path = "Right"
  
        neighbors = []
        if up is not None:
            neighbors.append(up)
        if down is not None:
            neighbors.append(down)
        if left is not None:
            neighbors.append(left)
        if right is not None:
            neighbors.append(right)
   
        return neighbors
 
class Solver:
    """
    This is a solver class for the n-puzzle
    """

    def __init__(self, state):
        self.startState=state
        self.frontier = None
        self.explored = None
        self.nodesExpanded = 0
        self.maxDepth = 0
        self.startTime = time.time()

    def outputResults(self, s):
        path = s.getPathToGoal()
        elapsedTime = time.time() - self.startTime
        maxrss = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        maxrss /= 1024
        with open("output.txt", "w") as outFp:
            outFp.write("path_to_goal: {0}\n".format(path))
            outFp.write("cost_of_path: {0}\n".format(len(path)))
            outFp.write("nodes_expanded: {0}\n".format(self.nodesExpanded))
            outFp.write("search_depth: {0}\n".format(len(path)))
            outFp.write("max_search_depth: {0}\n".format(self.maxDepth))
            outFp.write("running_time: %.8f\n" % elapsedTime)
            outFp.write("max_ram_usage: %.8f\n" % maxrss)

    def bfs(self):
        """
        This function solves the n-puzzle using the breadth-first-search algorithm"
        """
        self.startTime = time.time()
        self.frontier = StateDeque()
        self.explored = StateDeque()
        self.frontier.append(self.startState)
        self.nodesExpanded = 0
  
        while not self.frontier.isEmpty():
            s = self.frontier.popleft()
            self.explored.append(s)
   
            if s.isGoalState():
                self.outputResults(s)
                return s
   
            neighbors = s.getNeighbors()
            self.nodesExpanded += 1
            #if self.nodesExpanded % 1000 == 0:
            #  print("nodes expanded: " + str(self.nodesExpanded))

            for neighbor in neighbors:
                if not self.explored.isExists(neighbor) and not self.frontier.isExists(neighbor):
                    self.frontier.append(neighbor)
                    if neighbor.depth > self.maxDepth:
                        self.maxDepth = neighbor.depth
    
        return None
  
    def dfs(self):
        """
        This function solves the n-puzzle using the depth-first-search algorithm
        """
        self.startTime = time.time()
        self.frontier = StateDeque()
        self.explored = StateDeque()
        self.frontier.append(self.startState)
        self.nodesExpanded = 0
  
        while not self.frontier.isEmpty():
            s = self.frontier.pop()
            self.explored.append(s)
   
            if s.isGoalState():
                self.outputResults(s)
                return s
   
            neighbors = s.getNeighbors()
            self.nodesExpanded += 1
            #if self.nodesExpanded % 1000 == 0:
            #  print("nodes expanded: " + str(self.nodesExpanded))

            neighbors.reverse()
            for neighbor in neighbors:
                if not self.explored.isExists(neighbor) and not self.frontier.isExists(neighbor):
                    self.frontier.append(neighbor)
                    if neighbor.depth > self.maxDepth:
                        self.maxDepth = neighbor.depth
    
        return None
   
    def ast(self):
        """
        This function solves the n-puzzle using the A* search algorithm"
        """
        self.startTime = time.time()
        self.frontier = StatePQ()
        self.explored = StateDeque()
        self.frontier.put(self.startState)
        self.nodesExpanded = 0
  
        while not self.frontier.isEmpty():
            s = self.frontier.get()
            self.explored.append(s)
   
            if s.isGoalState():
                self.outputResults(s)
                return s
   
            neighbors = s.getNeighbors()
            self.nodesExpanded += 1
            #if self.nodesExpanded % 1000 == 0:
            #  print("nodes expanded: " + str(self.nodesExpanded))

            for neighbor in neighbors:
                if not self.explored.isExists(neighbor) and not self.frontier.isExists(neighbor):
                    self.frontier.put(neighbor)
                    if neighbor.depth > self.maxDepth:
                        self.maxDepth = neighbor.depth
                elif self.frontier.isExists(neighbor):
                    self.frontier.updatePriority(neighbor)

        return None
   
board=stateString.split(",")
board = map(int, board)
s=State(board)

solver=Solver(s)

if method == "bfs":
    solver.bfs()
elif method == "dfs": 
    solver.dfs()
elif method == "ast":
    solver.ast()
else:
    print("Error: Invalid method " + method)
    printUsage()
    exit(-1)
