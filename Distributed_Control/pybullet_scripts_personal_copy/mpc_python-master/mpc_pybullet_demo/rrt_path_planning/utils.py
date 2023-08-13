import numpy as np

class RRT_Params:
    def __init__(self):
        self.animate = 0 # show RRT construction, set 0 to reduce time of the RRT algorithm
        self.visualize = 1 # show constructed paths at the end of the RRT and path smoothing algorithms
        self.maxiters = 1000 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.minDistGoal = 0.2 # [m], min distance os samples from goal to add goal node to the RRT
        self.extension = 0.2 # [m], extension parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-1.5, 1.5] # [m], map size in X-direction
        self.world_bounds_y = [-1.5, 1.5] # [m], map size in Y-direction