#! /usr/bin/python
import sys
import resource
import collections
import itertools
import time
import heapq
import pdb

def print_usage():
 print("Usage: " + sys.argv[0] + " <method> <board>")
 print("     : where method is bfs or dfs or ast")
 print("     :       board is a comma separated list of integers with no spaces")

if len(sys.argv) != 3:
 print("Error: Invalid number of arguments")
 print_usage()
 exit(-1)

method=sys.argv[1]
state_string=sys.argv[2]

class StatePQ:
 """
 This class implements a priority queue for state objects. This class
 uses heapq module to implement the priority queue.
 """

 def __init__(self):
  self.pq = []
  self.entry_finder = {}
  self.REMOVED = '<removed-task>'
  self.counter = itertools.count()

 def put(self, s):
  """
  This function inserts a state object into the priority queue. This function
  doesn't check if the an object with same state already exists. The caller
  must ensure that an object with same state doesn't already exist. This
  function uses the get_estimated_cost() function of the state object to
  determine the priority.
  
  Arguments:
  s -- an object of the class State
  """
  count = next(self.counter)
  entry = [s.get_estimated_cost(), count, s]
  self.entry_finder[s.board_string] = entry
  heapq.heappush(self.pq, entry)

 def get(self):
  """
  This function dequeues and returns an object with the lowest priority
  in the queue. If the queue is empty it returns None.
  """
  while self.pq:
   p, count, s = heapq.heappop(self.pq)
   if s is not self.REMOVED:
    del self.entry_finder[s.board_string]
    return s
  return None

 def remove(self, s):
  """
  This function removes a state object from the queue.

  Arguments:
  s -- an object of the class State
  """
  entry = self.entry_finder.pop(s.board_string, None)
  if entry is not None:
   entry[-1] = self.REMOVED

 def update_priority(self, n):
  """
  This function updates the priority of a state object.

  Arguments:
  n -- an object of the class State
  """
  entry = self.entry_finder[n.board_string]
  if entry[2].get_estimated_cost() > n.get_estimated_cost():
   self.remove(entry[2])
   self.put(n)

 def clear(self):
  while self.pq:
   self.pq.get()
  self.entry_finder.clear()

 def is_exists(self, s):
  """
  This function checks if a state object exists in the queue.

  Arguments:
  n -- an object of the class State
  """

  if s.board_string in self.entry_finder:
    return True
  else:
    return False

 def is_empty(self):
  """
  This function checks if the queue is empty.
  """
  if self.entry_finder:
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
  self.q_set = set()

 def append(self, s):
  self.q.append(s)
  self.q_set.add(s.board_string)

 def appendleft(self, s):
  self.q.appendleft(s)
  self.q_set.add(s.board_string)

 def pop(self):
  s = self.q.pop()
  self.q_set.remove(s.board_string)
  return s
 
 def popleft(self):
  s = self.q.popleft()
  self.q_set.remove(s.board_string)
  return s

 def clear(self):
  self.q.clear()
  self.q_set.clear()

 def is_exists(self, s):
  if s.board_string in self.q_set:
    return True
  else:
    return False

 def is_empty(self):
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
  self.board_string = ''.join(map(str, board))
  self.parent = parent
  self.path = ""
  if parent is None:
   self.depth = 0
  else:
   self.depth = parent.depth + 1

 def get_path_to_goal(self):
  path = []
  p = self

  while p.parent is not None:
   path.append(p.path)
   p = p.parent

  path.reverse()
  return path

 def is_goal_state(self):
  if self.board == State.GOAL:
   return True
  else:
   return False

 def get_manhattan_distance(self):
  """
  This function returns the manhatten distance for this node.
  """
  return sum(abs(val%3 - i%3) + abs(val//3 - i//3) for i, val in enumerate(self.board) if val)
  
 def get_estimated_cost(self):
  """
  This function retuns the estimated cost of the cheapest 
  solution through this node. If uses node depth to estimate the cost to reach
  this node and manhanttan distance to estimate the cost to from this node to
  the goal.
  """
  return self.depth + self.get_manhattan_distance()

 def get_neighbors(self):
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
  self.start_state=state
  self.frontier = None
  self.explored = None
  self.nodes_expanded = 0
  self.max_depth = 0
  self.start_time = time.time()

 def display_results(self, s):
  path = s.get_path_to_goal()
  elapsed_time = time.time() - self.start_time
  maxrss = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
  maxrss /= 1024
  print("path_to_goal: " + str(path))
  print("cost_of_path: " + str(len(path)))
  print("nodes_expanded: " + str(self.nodes_expanded))
  print("search_depth: " + str(len(path)))
  print("max_search_depth: " + str(self.max_depth))
  print("running_time: %.8f" % elapsed_time)
  print("max_ram_usage: %.8f" % maxrss)
  
 def bfs(self):
  """
  This function solves the n-puzzle using the breadth-first-search algorithm"
  """
  print("bfs")
  self.start_time = time.time()
  self.frontier = StateDeque()
  self.explored = StateDeque()
  self.frontier.append(self.start_state)
  self.nodes_expanded = 0
  
  while not self.frontier.is_empty():
   s = self.frontier.popleft()
   self.explored.append(s)
   
   if s.is_goal_state():
    self.display_results(s)
    return s
   
   neighbors = s.get_neighbors()
   self.nodes_expanded += 1
   #if self.nodes_expanded % 1000 == 0:
   #  print("nodes expanded: " + str(self.nodes_expanded))

   for neighbor in neighbors:
    if not self.explored.is_exists(neighbor) and not self.frontier.is_exists(neighbor):
      self.frontier.append(neighbor)
      if neighbor.depth > self.max_depth:
       self.max_depth = neighbor.depth
    
  return None
  
 def dfs(self):
  """
  This function solves the n-puzzle using the depth-first-search algorithm
  """
  print("dfs")
  self.start_time = time.time()
  self.frontier = StateDeque()
  self.explored = StateDeque()
  self.frontier.append(self.start_state)
  self.nodes_expanded = 0
  
  while not self.frontier.is_empty():
   s = self.frontier.pop()
   self.explored.append(s)
   
   if s.is_goal_state():
    self.display_results(s)
    return s
   
   neighbors = s.get_neighbors()
   self.nodes_expanded += 1
   #if self.nodes_expanded % 1000 == 0:
   #  print("nodes expanded: " + str(self.nodes_expanded))

   neighbors.reverse()
   for neighbor in neighbors:
    if not self.explored.is_exists(neighbor) and not self.frontier.is_exists(neighbor):
      self.frontier.append(neighbor)
      if neighbor.depth > self.max_depth:
       self.max_depth = neighbor.depth
    
  return None
   
 def ast(self):
  """
  This function solves the n-puzzle using the A* search algorithm"
  """
  print("ast")
  self.start_time = time.time()
  self.frontier = StatePQ()
  self.explored = StateDeque()
  self.frontier.put(self.start_state)
  self.nodes_expanded = 0
  
  while not self.frontier.is_empty():
   s = self.frontier.get()
   self.explored.append(s)
   
   if s.is_goal_state():
    self.display_results(s)
    return s
   
   neighbors = s.get_neighbors()
   self.nodes_expanded += 1
   #if self.nodes_expanded % 1000 == 0:
   #  print("nodes expanded: " + str(self.nodes_expanded))

   for neighbor in neighbors:
    if not self.explored.is_exists(neighbor) and not self.frontier.is_exists(neighbor):
     self.frontier.put(neighbor)
     if neighbor.depth > self.max_depth:
      self.max_depth = neighbor.depth
    elif self.frontier.is_exists(neighbor):
     self.frontier.update_priority(neighbor)

  return None
   
board=state_string.split(",")
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
 print_usage()
 exit(-1)
