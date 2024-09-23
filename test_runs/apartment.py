import matplotlib.pyplot as plt
import numpy as np
from enum import Enum, auto
from math import sqrt

# Starting rotation
class starting_rotation(Enum):
    ZERO_DEGREES            = auto()
    NEGATIVE_NINETY_DEGREES = auto()

# The starting position of the robot
STARTING_X = 7.6
STARTING_Y = 3.5
STARTING_ROTATION = starting_rotation.ZERO_DEGREES

# Area 1 boundary points
AREA_ONE_TL = (2.6, 5.2)
AREA_ONE_BL = (2.6, 0)
AREA_ONE_TR = (8.4, 5.2)
AREA_ONE_BR = (8.4, 0)

# Area 2 boundary points
AREA_TWO_TL = (3.8, 7.2)
AREA_TWO_BL = (3.8, 5.2)
AREA_TWO_TR = (9.6, 7.2)
AREA_TWO_BR = (9.6, 5.2)

# Area 3 boundary points
AREA_THREE_TL = (0, 9.4)
AREA_THREE_BL = (0, 5.4)
AREA_THREE_TR = (3.6, 9.4)
AREA_THREE_BR = (3.6, 5.4)

# Area 4 boundary points
AREA_FOUR_TL = (0, 1.6)
AREA_FOUR_BL = (0, 0)
AREA_FOUR_TR = (2.6, 1.6)
AREA_FOUR_BR = (2.6, 0)

# Challenge Area 1 boundary points
C_AREA_ONE_TL = (0, 5.2)
C_AREA_ONE_BL = (0, 1.8)
C_AREA_ONE_TR = (2.4, 5.2)
C_AREA_ONE_BR = (2.4, 1.8)

# Challenge Area 2 boundary points
C_AREA_TWO_TL = (8.6, 4.0)
C_AREA_TWO_BL = (8.6, 0)
C_AREA_TWO_TR = (10.6, 4.0)
C_AREA_TWO_BR = (10.6, 0)

# Points forming the outside perimeter of the apartment
WALL_POINTS = [
    # Outside wall perimeter
    (-0.2, 9.6),  # 0
    (3.8, 9.6),   # 1
    (3.8, 7.4),   # 2
    (9.8, 7.4),   # 3
    (9.8, 4.2),   # 4
    (10.8, 4.2),  # 5
    (10.8, -0.2), # 6
    (-0.2, -0.2), # 7
    # Inside wall perimeter
    (0, 9.4),     # 8
    (3.6, 9.4),   # 9
    (3.6, 7.2),   # 10
    (9.6, 7.2),   # 11
    (9.6, 4.0),   # 12
    (10.6, 4.0),  # 13
    (10.6, 0),    # 14
    (0, 0),       # 15
    # Area 3 / Challenge area 1 wall
    (0, 5.4),     # 16
    (2.6, 5.4),   # 17
    (2.6, 3.8),   # 18
    (2.4, 3.8),   # 19
    (2.4, 5.2),   # 20
    (0, 5.2),     # 21
    # Challenge area 1 / Area 1
    (2.4, 2.8),   # 22
    (2.6, 2.8),   # 23
    (2.6, 1.4),   # 24
    (2.4, 1.4),   # 25
    # Challenge area 1 / Area 4
    (0, 1.8),     # 26
    (2.4, 1.8),   # 27
    (2.4, 1.6),   # 28
    (0, 1.6),     # 29
    # Area 4 block
    (2.4, 0),     # 30
    (2.4, 0.2),   # 31
    (2.6, 0.2),   # 32
    (2.6, 0),     # 33
    # Area 3 / Area 2
    (3.6, 7.2),   # 34
    (3.6, 4.2),   # 35
    (3.8, 4.2),   # 36
    (3.8, 7.2),   # 37
    # Area 1
    (3.6, 4.0),   # 38
    (4.2, 4.0),   # 39
    (4.2, 4.2),   # 40
    # Area 1 / Challenge area 2
    (8.4, 4.2),   # 41
    (8.4, 0.0),   # 42
    (8.6, 0.0),   # 43
    (8.6, 4.2)    # 44
]

# Define connections using indices of the points
WALL_CONNECTIONS = [
    #  Outside wall connections
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 0),
    # Inside wall connections
    (8, 9), (9, 10), (10, 11), (11, 12), (12, 13), (13, 14), (14, 15), (15, 8),
    # Area 3 / Challenge area 1 wall
    (16, 17), (17, 18), (18, 19), (19, 20), (20, 21),
    # Challenge area 1 / Area 1
    (22, 23), (23, 24), (24, 25), (25, 22),
    # Challenge area 1 / Area 4
    (26, 27), (27, 28), (28, 29),
    # Area 4 block
    (30, 31), (31, 32), (32, 33),
    # Area 3 / Area 2
    (34, 35), (35, 36), (36, 37),
    # Area 1
    (35, 38), (38, 39), (39, 40), (40, 35),
    # Area 1 / Challenge area 2
    (41, 42), (42, 43), (43, 44), (44, 41)
]


def is_point_in_perimeter(point_x, point_y, top_left, bottom_left, top_right, bottom_right):
    '''
    Determines if the given point is within the provided boundary.

    @note This function assumes all boundaries are a rectangular boundary.

    Parameters:
    -----------
        point_x:      The x coordinate of the point to check.
        point_y:      The y coordinate of the point to check.
        top_left:     The top left point of the rectangular boundary.
        bottom_left:  The bottom left point of the rectangular boundary.
        top_right:    THe top right point of the rectangular boundary.
        bottom_right: The bottom right point of the rectanfular boundary.

    Returns:
    --------
        bool: Whether or not the point is within the perimeter.
    '''
    x_min, x_max, y_min, y_max = bottom_left[0], bottom_right[0], bottom_left[1], top_left[1]

    return (point_x >= x_min and point_x <= x_max) and (point_y >= y_min and point_y <= y_max)


def distance_between_points(x1, y1, x2, y2):
    '''
    Computes the distance between two points on a 2D plane.

    Parameters:
    -----------
        x1: The first x value.
        y1: The first y value.
        x2: The second x value.
        y2: The second y value.

    Returns:
    --------
        float: The distance between the points.
    '''
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)


def get_furthest_point(x_list, y_list, top_left, bottom_left, top_right, bottom_right):
    '''
    Gets the furthest coordinate from the starting coordinates that is contained
    within the specified boundary.

    Parameters:
    -----------
        x_list:       A list of x coordinate values. Needs to be paired with y_list.
        y_list:       A list of y coordinate values. Needs to be paired with x_list.
        top_left:     The top left point of the rectangular boundary.
        bottom_left:  The bottom left point of the rectangular boundary.
        top_right:    THe top right point of the rectangular boundary.
        bottom_right: The bottom right point of the rectanfular boundary.

    Returns:
    --------
        str: A string representation of the furthest coordinate from start within
             the specified boundary.
    '''
    current_max_distance, current_max_distance_x, current_max_distance_y = 0, 0, 0

    # for each coordinate, check if the point is in parameter, then determine
    # if the coordinate is the furthest point encountered from the starting
    # position encountered so far
    for x, y in zip(x_list, y_list):

        if is_point_in_perimeter(x, y, top_left, bottom_left, top_right, bottom_right):

            new_distance = distance_between_points(x, y, STARTING_X, STARTING_Y)

            if new_distance > current_max_distance:
                current_max_distance_x = x
                current_max_distance_y = y
                current_max_distance = new_distance

    return f'({current_max_distance_x}, {current_max_distance_y})'

def get_path_length(x_list, y_list):
    '''
    Gets the length of the robots path by adding up all the distances between
    consequtive positions.

    Parameters:
    -----------
        x_list: The list of x coordinates the robot was at.
        y_list: The list of y coordinates the robot was at.

    Returns:
    --------
        float: The total length of the robots path
    '''
    path_length = 0
    points      = list(zip(x_list, y_list))

    for idx in range(len(points) - 1):

        path_length += distance_between_points(
            points[idx][0],
            points[idx][1],
            points[idx+1][0],
            points[idx+1][1]
        )

    return path_length

def plot_apartment(
    ax,
    points,
    connections,
    b_plot_points,
    line_color='blue',
    marker='',
    linestyle='-'
):
    '''
    Plots the apartment outline so that the points are appropriately scaled.

    Parameters:
    -----------
        fig:           The figure that the apartment should be plotted on.
        points:        The (x, y) points of the apartment borders.
        connections:   The connections of the points defined by the indexes of the points list.
        b_plot_points: Whether or not the points should be shown with coordinates.
        line_color:    The color of the apartment outline. Blue by default.
        marker:        What marker should be used for the points.
        linestyle:     The style of the line to add to the plot.
    '''
    # Plot each connection
    for start, end in connections:
        x_values = [points[start][0], points[end][0]]
        y_values = [points[start][1], points[end][1]]
        ax.plot(x_values, y_values, marker=marker, linestyle=linestyle, color=line_color)

    # Plot all points
    if b_plot_points:
        for idx, (x, y) in enumerate(points):
            ax.text(x, y, f'({x}, {y})', fontsize=5, ha='right')

    return

def plot_path(ax, line_color, marker, filename):
    '''
    Plots the path of the robot within the previously computed plot. Along with
    this, the total path length and furthest points in various zones will be
    computed.

    Parameters:
    -----------
        ax:         The plot to add the path to.
        line_color: The color of the path.
        marker:     The marker type for points in the path.
        filename:   The name of the file containing the coordinate poisitions.

    Returns:
    --------
        float: The total length of the path the robot reported.
        str:   The furthest point in zone 1 reported.
        str:   The furthest point in zone 2 reported.
        str:   The furthest point in zone 3 reported.
        str:   The furthest point in zone 4 reported.
        str:   The furthest point in challenge zone 1 reported.
        str:   The furthest point in challendge zone 2 reported.
    '''
    # Load coordinates from a file
    coordinates = []

    # Read all lines in the file
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            line = line.replace('(', '').replace(')', '')  # Remove parentheses
            coords = list(map(float, line.split(',')))  # Convert to float
            coordinates.append(coords)

    # Convert list of coordinates to x and y lists
    x = []
    y = []
    for coords in coordinates:
        x.extend(coords[::2])  # Odd indices for x
        y.extend(coords[1::2])  # Even indices for y

    # Translate the robots reported positions to the apartment coordinates
    x_translated = []
    y_translated = []
    for xi, yi in zip(x, y):
        x_new = 0
        y_new = 0
        if STARTING_ROTATION == starting_rotation.ZERO_DEGREES:
            x_new = STARTING_X + xi
            y_new = STARTING_Y + yi
        elif STARTING_ROTATION == starting_rotation.NEGATIVE_NINETY_DEGREES:
            x_new = STARTING_X + yi
            y_new = STARTING_Y - xi
        else:
            print('Invalid Starting Rotation Value')

        x_translated.append(x_new)
        y_translated.append(y_new)

    # Add the path to the plot
    ax.plot(x_translated, y_translated, marker=marker, markersize=5, color=line_color)

    # Compute the total length of the path according to the robot
    total_path_length = get_path_length(x_translated, y_translated)

    # Get the furthest point in zone one from the starting point according to
    # the robot.
    zone_one_furthest_point = get_furthest_point(
        x_translated,
        y_translated,
        AREA_ONE_TL,
        AREA_ONE_BL,
        AREA_ONE_TR,
        AREA_ONE_BR
    )

    # Get the furthest point in zone two from the starting point according to
    # the robot.
    zone_two_furthest_point = get_furthest_point(
        x_translated,
        y_translated,
        AREA_TWO_TL,
        AREA_TWO_BL,
        AREA_TWO_TR,
        AREA_TWO_BR
    )

    # Get the furthest point in zone three from the starting point according to
    # the robot.
    zone_three_furthest_point = get_furthest_point(
        x_translated,
        y_translated,
        AREA_THREE_TL,
        AREA_THREE_BL,
        AREA_THREE_TR,
        AREA_THREE_BR
    )

    # Get the furthest point in zone four from the starting point according to
    # the robot.
    zone_four_furthest_point = get_furthest_point(
        x_translated,
        y_translated,
        AREA_FOUR_TL,
        AREA_FOUR_BL,
        AREA_FOUR_TR,
        AREA_FOUR_BR
    )

    # Get the furthest point in c zone one from the starting point according to
    # the robot.
    c_zone_one_furthest_point = get_furthest_point(
        x_translated,
        y_translated,
        C_AREA_ONE_TL,
        C_AREA_ONE_BL,
        C_AREA_ONE_TR,
        C_AREA_ONE_BR
    )

    # Get the furthest point in c zone two from the starting point according to
    # the robot.
    c_zone_two_furthest_point = get_furthest_point(
        x_translated,
        y_translated,
        C_AREA_TWO_TL,
        C_AREA_TWO_BL,
        C_AREA_TWO_TR,
        C_AREA_TWO_BR
    )

    return (
        total_path_length,
        zone_one_furthest_point,
        zone_two_furthest_point,
        zone_three_furthest_point,
        zone_four_furthest_point,
        c_zone_one_furthest_point,
        c_zone_two_furthest_point
    )

# Function to plot lines connecting specified pairs of points
def plot_paths(points, connections):
    '''
    Generates a plot of the robots paths in the apartment.

    Parameters:
    -----------
        points:
        connections:
    '''
    # Create a new figure
    fig, ax = plt.subplots()

    # Add labels and a title
    ax.set_title("Apartment Plot")

    # Generate the borders of the apartment
    plot_apartment(ax, WALL_POINTS, WALL_CONNECTIONS, False)

    prefix_dir = 'position_1'
    filename_list = ['trial1.txt', 'trial2.txt', 'trial3.txt', 'trial4.txt', 'trial5.txt']
    line_colors   = ['green', 'purple', 'red', 'orange', 'yellow']

    for filename, line_color in zip(filename_list, line_colors):

        # Generate the paths that the robot took
        (
            total_path_length,
            zone_one_furthest_point,
            zone_two_furthest_point,
            zone_three_furthest_point,
            zone_four_furthest_point,
            c_zone_one_furthest_point,
            c_zone_two_furthest_point
        ) = plot_path(ax, line_color, '', f'{prefix_dir}/{filename}')

        print(
            f'{filename}\n'                                                     +
             '---------------------------------------------------------------\n'+
            f'Total Path Length:                 {total_path_length}\n'         +
            f'Area One Furthest Point:           {zone_one_furthest_point}\n'   +
            f'Area Two Furthest Point:           {zone_two_furthest_point}\n'   +
            f'Area Three Furthest Point:         {zone_three_furthest_point}\n' +
            f'Area Four Furthest Point:          {zone_four_furthest_point}\n'  +
            f'Challenge Area One Furthest Point: {c_zone_one_furthest_point}\n' +
            f'Challenge Area Two Furthest Point: {c_zone_two_furthest_point}\n'
        )

    # Show the grid
    ax.grid(True)

    # Show the plot
    plt.savefig("plot.png", transparent=True)

plot_paths(WALL_POINTS, WALL_CONNECTIONS)

