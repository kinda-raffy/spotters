
import pygame
import time
#configuring variables

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY1 = (145, 145, 102)
GRAY2 = (77, 77, 51)
BLUE = (0, 191 , 255)
LIGHT_YELLOW = (255,255,153)
LIGHT_GREEN = (144,238,144)
colors = {
    0: WHITE, #non-visted
    1: GREEN, #Goal
    2: LIGHT_GREEN, #consistent
    3: LIGHT_YELLOW, #inconsistent
    -1: GRAY1,#blocks undiscovered
    -2: GRAY2 #blocks discovered
}
# This sets the WIDTH and HEIGHT of each grid location
WIDTH = 60
HEIGHT = 60

# This sets the margin between each cell
MARGIN = 5

# Create a 2 dimensional array. A two dimensional
# array is simply a list of lists.
grid = []
for row in range(10):
    # Add an empty array that will hold each cell
    # in this row
    grid.append([])
    for column in range(10):
        grid[row].append(0)  # Append a cell

# Set row 1, cell 5 to one. (Remember rows and
# column numbers start at zero.)
grid[1][5] = 1

# Initialize pygame
pygame.init()

X_DIM = 12
Y_DIM = 12
VIEWING_RANGE = 3


# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = [(WIDTH + MARGIN) * X_DIM + MARGIN,
               (HEIGHT + MARGIN) * Y_DIM + MARGIN + 30*MARGIN]
screen = pygame.display.set_mode(WINDOW_SIZE)

# Set title of screen
pygame.display.set_caption("D* Lite Path Planning")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()
basicfont = pygame.font.SysFont('Comic Sans MS', 20)

def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]


def render_all(graph, highlight=None, delay=0, status="D-Star-Lite"):
    global screen
    # Set the screen background
    screen.fill(BLACK)

    # Draw the grid
    for row in range(Y_DIM):
        for column in range(X_DIM):
            color = WHITE
            # if grid[row][column] == 1:
            #     color = GREEN

            color = colors[graph.cells[row][column]]
            pygame.draw.rect(screen, color,
                             [(MARGIN + WIDTH) * column + MARGIN,
                              (MARGIN + HEIGHT) * row + MARGIN, WIDTH, HEIGHT])
            node_name = 'x' + str(column) + 'y' + str(row)
            if(graph.cells[row][column] in [-1,2,3]):
                # text = basicfont.render(
                # str(graph.graph[node_name].g), True, (0, 0, 200), (255,
                # 255, 255))
                text = basicfont.render("{}:{}".format(graph.graph[node_name].g, graph.graph[node_name].rhs)
                    , True, (0, 0, 200))
                textrect = text.get_rect()
                textrect.centerx = int(
                    column * (WIDTH + MARGIN) + WIDTH / 2) + MARGIN
                textrect.centery = int(
                    row * (HEIGHT + MARGIN) + HEIGHT / 2) + MARGIN
                screen.blit(text, textrect)

    # fill in goal cell with GREEN
    pygame.draw.rect(screen, GREEN, [(MARGIN + WIDTH) * graph.goal_coords[0] + MARGIN,
                                     (MARGIN + HEIGHT) * graph.goal_coords[1] + MARGIN, WIDTH, HEIGHT])
    # print('drawing robot pos_coords: ', pos_coords)
    # draw moving robot, based on pos_coords
    robot_center = [int(graph.pos_coords[0] * (WIDTH + MARGIN) + WIDTH / 2) +
                    MARGIN, int(graph.pos_coords[1] * (HEIGHT + MARGIN) + HEIGHT / 2) + MARGIN]
    pygame.draw.circle(screen, RED, robot_center, int(WIDTH / 2) - 2)

    # draw robot viewing range
    pygame.draw.rect(
        screen, BLUE, [robot_center[0] - VIEWING_RANGE * (WIDTH + MARGIN), robot_center[1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 2)

    title = pygame.font.SysFont('Comic Sans MS', 80).render(status, True, (255, 255, 255))
    screen.blit(title, (WINDOW_SIZE[0]/2  -200, WINDOW_SIZE[1]-90))
    # Limit to 60 frames per second
    clock.tick(20)

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
    time.sleep(delay)

# Be IDLE friendly. If you forget this line, the program will 'hang'
# on exit.
