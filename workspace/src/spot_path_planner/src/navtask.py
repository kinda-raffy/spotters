class NavTask:
    def __init__(self):
        self.curr_pos = None
        self.goal_pos = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.curr_map = None
        self.ultimate_goal_pos = None
        self.is_set_up = False
        self.is_goal_changed = False
        self.is_complete = False
        
    def is_set_up_needed(self):
        res = False
        if (self.curr_pos is not None and 
            self.goal_pos is not None and 
            self.map_width is not None and 
            self.map_height is not None and 
            self.map_resolution is not None and
            not self.is_set_up) or self.is_goal_changed:
            res = True
            self.is_set_up = True
            self.is_goal_changed = False
        return res
    
    def set_goal_pos(self, cartesian_goal_pos):
        if not self.map_width is None and not self.map_height is None:
            self.goal_pos = self.bound_goal_pos(cartesian_goal_pos)
        # if self.map_width and self.map_height:
        #     # Return the bounded_goal and store the original goal in ultimate_goal
        #     cartesian_goal_pos = self.bound_goal_pos(cartesian_goal_pos)
                
        #     # If the goal is set for the first time
        #     if self.goal_pos is None:
        #         self.goal_pos = cartesian_goal_pos
            
    #         # TODO: Persist the goal
    #         # If the new goal is not the same as the stored goal
    #         if self.goal_pos != goal_pos:
    #             self.is_goal_changed = True
    #             self.goal_pos = goal_pos
    
    # Returns the boudned goal position 
    def bound_goal_pos(self, goal_pos):
        # Remember the ultimate goal position
        self.ultimate_goal_pos = goal_pos
        
        goal_x_value, goal_y_value = goal_pos

        (gx, gy) = goal_pos
        temp_goal_x_value = gx
        temp_goal_y_value = gy
        # Assuming that all the map coordinates are positive,

        max_x_value = self.map_height - 1
        max_y_value = self.map_width - 1
        min_x_value = 0
        min_y_value = 0
        
        (x, y) = self.curr_pos

        if goal_x_value > max_x_value and goal_x_value < min_x_value:
            temp_goal_x_value = (x + goal_x_value) / 2
        elif goal_x_value > max_x_value:
            temp_goal_x_value = max_x_value
        elif goal_x_value < min_x_value:
            temp_goal_x_value = min_x_value

        if goal_y_value > max_y_value and goal_y_value < min_y_value:
            temp_goal_y_value = (y + goal_y_value) / 2
        elif goal_y_value > max_y_value:
            temp_goal_y_value = max_y_value
        elif goal_y_value < min_y_value:
            temp_goal_y_value = min_y_value

        return (temp_goal_x_value, temp_goal_y_value)
    
    def is_ultimate_goal_reached(self):
        res = False
        # If the current position is at the goal position
        if self.curr_pos == self.goal_pos:
            # If the goal_pos is equal to the ultimate_goal_pos
            if self.goal_pos == self.ultimate_goal_pos:
                self.is_complete = True
                res = True
            else:
                self.set_goal_pos(self.ultimate_goal_pos)
        return res 
        
        
        
        
        

        
            
            
    

    
     