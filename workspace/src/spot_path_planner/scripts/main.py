import rospy
from nav_msgs.msg import OccupancyGrid
from graph_wrapper import GraphWrapper
from utils import *
from d_star_lite import *

if __name__ == "__main__":
    rospy.init_node('map_subscriber')
     # Wait for a map message
    map_msg = rospy.wait_for_message('/map', OccupancyGrid)

    graph = GraphWrapper(map_msg)
    s_start = 'x1y2'
    s_goal = 'x5y8'
    graph.goal = s_goal
    goal_coords = stateNameToCoords(s_goal)
    graph.goal_coords = goal_coords
    graph.setStart(s_start)
    graph.setGoal(s_goal)
    k_m = 0
    s_last = s_start
    queue = []
    s_current = s_start
    pos_coords = stateNameToCoords(s_current)
    graph.pos_coords = pos_coords
    graph, queue, k_m = initDStarLite(graph, queue, s_start, s_goal, k_m)

    done = False
    # -------- Main Program Loop -----------
    while not done:
        # TODO: Convert the logic below to work with Spot 
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                # print('space bar! call next action')
                s_new, k_m = moveAndRescan(
                    graph, queue, s_current, VIEWING_RANGE, k_m)
                if s_new == 'goal':
                    print('Goal Reached!')
                    done = True
                else:
                    # print('setting s_current to ', s_new)
                    s_current = s_new
                    pos_coords = stateNameToCoords(s_current)
                    graph.pos_coords = pos_coords
                    # print('got pos coords: ', pos_coords)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # User clicks the mouse. Get the position
                pos = pygame.mouse.get_pos()
                print(pos)
                # Change the x/y screen coordinates to grid coordinates
                column = pos[0] // (WIDTH + MARGIN)
                print(column)
                row = pos[1] // (HEIGHT + MARGIN)
                print(row)
                # Set that location to one
                if(graph.cells[row][column] in [0,2,3]):
                    graph.cells[row][column] = -1
