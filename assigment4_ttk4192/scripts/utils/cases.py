from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        #self.start_pos = [4.6, 2.4, 0]
        #self.end_pos = [1.6, 8, -pi/2]

        #self.start_pos2 = [4, 4, 0]
        #self.end_pos2 = [4, 8, 1.2*pi]

        #self.obs = [
        #    [2, 3, 6, 0.1],
        #    [2, 3, 0.1, 1.5],
        #    [4.3, 0, 0.1, 1.8],
        #    [6.5, 1.5, 0.1, 1.5],
        #    [0, 6, 3.5, 0.1],
        #    [5, 6, 5, 0.1]
        #]
        # Case Turtlebot3 - 4 rooms
        self.start_pos = [2, 2, 0]
        self.end_pos = [2, 17, 3*pi/4]

        self.start_pos2 = [4, 4, 0]
        self.end_pos2 = [4, 8, -pi]

        self.obs = [
            [0, 6, 6, 0.1],
            [6, 0, 0.1, 4],
            [0, 14, 4, 0.1],
            [6, 14, 0.1, 6],
            [14, 14, 6, 0.1],
            [14, 16, 0.1, 4],            
            [16, 6, 4, 0.1],
            [14, 0, 0.1, 6],
        ]


class map_grid_robplan:
    """ Here the obstacles are defined for a 20x20 map. """
    def __init__(self):

        self.start_pos2 = [4, 4, 0]  # default values
        self.end_pos2 = [4, 8, -pi]  # default
        self.obs = [
            # Wall boxes
            [3.3,0,0.01, 0.2],
            [3.3,0.2,5.21-3.3,0.01],
            [0,0.9,0.5,0.01],
            [0.5,0.9,0.01,0.3],
            [0.5,0.9+0.3,-0.5,0],

            # boxes
            [1.3-0.2/2,1.85-0.4/2,0.2,0.4],
            [2.4-0.4/2,1.85-0.4/2,0.4,0.4],

            #valves
            [1.77-0.5/2,0.73-0.2/2,0.5,0.2],
            [3.48-0.5/2,.79-0.2/2,0.5,0.2],

            #pump
            [3.75-0.4/2,1.77-0.2/2,0.4,0.2],
        ]
