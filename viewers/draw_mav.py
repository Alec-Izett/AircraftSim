"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        1/13/2021 - TWM
        7/13/2023 - RWB
        1/16/2024 - RWB
"""
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation
from tools.drawing import rotate_points, translate_points, points_to_mesh


class DrawMav:
    def __init__(self, state, window, scale=10):
        """
        Draw the Mav.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        self.unit_length = scale
        sc_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        # convert North-East Down to East-North-Up for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        # get points that define the non-rotated, non-translated Mav and the mesh colors
        self.sc_points, self.sc_index, self.sc_meshColors = self.get_sc_points()
        self.sc_body = self.add_object(
            self.sc_points,
            self.sc_index,
            self.sc_meshColors,
            R_bi,
            sc_position)
        window.addItem(self.sc_body)  # add Mav to plot     

    def update(self, state):
        sc_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        
        
        
        self.sc_body = self.update_object(
            self.sc_body,
            self.sc_points,
            self.sc_index,
            self.sc_meshColors,
            R_bi,
            sc_position)

    def add_object(self, points, index, colors, R, position):
        rotated_points = rotate_points(points, R)
        translated_points = translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points, index)
        object = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        return object

    def update_object(self, object, points, index, colors, R, position):
        rotated_points = rotate_points(points, R)
        translated_points = translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points, index)
        object.setMeshData(vertexes=mesh, vertexColors=colors)
        return object

    def get_sc_points(self):
        """"
            Points that define the Mav, and the colors of the triangular mesh
            Define the points on the Mav following information in Appendix C.3
        """
        # points are in XYZ coordinates
        #   define the points on the Mav according to Appendix C.3
        points = self.unit_length * np.array([
            [0,0,0], # shift all points by one index
            [0.5, 0, 0.05],  # nose point 1 [1]
            [0.25, 0.15, 0.15],  # nose point 2 [2]
            [0.25, -0.15, 0.15],  # nose point 3 [3]
            [0.25, -0.15, -0.15],  # nose point 4 [4]
            [0.25, 0.15, -0.15],  # nose point 5 [5]
            [-1.1, 0, 0],  # tail point 6
            [0, 0.75, 0],  # wing point 7 
            [-0.32, 0.75, 0],  # wing point 8 
            [-0.32, -0.75, 0],  # wing point 9 
            [0, -0.75, 0],  # wing point 10 
            [-0.9, 0.35, 0],  # rear wing point 11 
            [-1.1, 0.35, 0],  # rear wing point 12
            [-1.1, -0.35, 0],  # rear wing  
            [-0.9, -0.35, 0],  # rear wing
            [-0.9, 0, 0],  # rear Tri
            [-1.1, 0, -0.4]  # rear Tri    

            ]).T
        # point index that defines the mesh
        index = np.array([
            [1, 2, 3],  # nose
            [1, 3, 4],  # nose
            [1, 4, 5],  # nose
            [1, 2, 5],  # nose
            [2, 3, 6],  # tail 
            [3, 4, 6],  # tail
            [5, 4, 6],  # tail
            [5, 2, 6],  # tail
            [7, 8, 9],  # wing
            [7, 9, 10],  # wing
            [11, 12, 13],  # rear wing
            [14, 11, 13],  # rear wing 
            [15, 6, 16]  # rear wing 
            ])
        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((13, 3, 4), dtype=np.float32)
        meshColors[0] = yellow  # front 1
        meshColors[1] = yellow  # front 2
        meshColors[2] = yellow  # back 1
        meshColors[3] = yellow  # back 2
        meshColors[4] = blue  # right 1
        meshColors[5] = blue  # right 2
        meshColors[6] = blue  # left 1
        meshColors[7] = blue  # left 2
        meshColors[8] = red  # top 1
        meshColors[9] = red  # top 2
        meshColors[10] = green  # bottom 1
        meshColors[11] = green  # bottom 2
        meshColors[12] = red # wing Tri
        return points, index, meshColors

