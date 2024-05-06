#!/usr/bin/env python
# encoding: utf-8
"""
The HomogeneousMatrix class
@Authors: Arturo Gil
@Time: April 2023
"""
import numpy as np
# from artelib.euler import Euler
from artelib.tools import rot2quaternion, buildT
from artelib import quaternion, rotationmatrix, euler, vector
import matplotlib.pyplot as plt
# from artelib.quaternion import Quaternion


class HomogeneousMatrix():
    def __init__(self, *args):
        if len(args) == 0:
            self.array = np.eye(4)
        elif len(args) == 1:
            if isinstance(args[0], HomogeneousMatrix):
                self.array = args[0].toarray()
            elif isinstance(args[0], np.ndarray):
                self.array = args[0]
            elif isinstance(args[0], list):
                self.array = np.array(args[0])
            else:
                self.array = np.array(args[0])
        elif len(args) == 2:
            position = args[0]
            orientation = args[1]
            if isinstance(position, list):
                position = np.array(position)
            elif isinstance(position, vector.Vector):
                position = np.array(position.array)
            if isinstance(orientation, euler.Euler):
                array = buildT(position, orientation)
            elif isinstance(orientation, list):
                array = buildT(position, euler.Euler(orientation))
            elif isinstance(orientation, quaternion.Quaternion):
                array = buildT(position, orientation)
            elif isinstance(orientation, rotationmatrix.RotationMatrix):
                array = buildT(position, orientation)
            else:
                raise Exception
            self.array = array

    def __str__(self):
        return str(self.array)

    def toarray(self):
        return self.array

    def print_nice(self, precision=3):
        # temp_array = self.array
        # th = 0.01
        # idx = np.abs(temp_array) < th
        # temp_array[idx] = 0
        print(np.array_str(self.array, precision=precision, suppress_small=True))

    def inv(self):
        return HomogeneousMatrix(np.linalg.inv(self.array))

    def Q(self):
        return quaternion.Quaternion(rot2quaternion(self.array))

    def R(self):
        return rotationmatrix.RotationMatrix(self.array[0:3, 0:3])

    def euler(self):
        return self.R().euler()[0], self.R().euler()[1],

    def pos(self):
        return self.array[0:3, 3]

    def __mul__(self, other):
        if isinstance(other, HomogeneousMatrix):
            T = np.dot(self.array, other.array)
            return HomogeneousMatrix(T)
        elif isinstance(other, vector.Vector):
            u = np.dot(self.array, other.array)
            return vector.Vector(u)

    def __add__(self, other):
        T = self.array+other.array
        return HomogeneousMatrix(T)

    def __sub__(self, other):
        T = self.array-other.array
        return HomogeneousMatrix(T)

    def __getitem__(self, item):
        return self.array[item[0], item[1]]

    def t2v(self, n=2):
        # converting from SE(2)
        if n == 2:
            tx = self.array[0, 3]
            ty = self.array[1, 3]
            th = np.arctan2(self.array[1, 0], self.array[0, 0])
            return np.array([tx, ty, th])
        else:
            tx = self.array[0, 3]
            ty = self.array[1, 3]
            tz = self.array[2, 3]
            th = self.Q().Euler()[0].abg
            return np.array([tx, ty, tz, th[0], th[1], th[2]])

    def plot(self, title='Homogeneous transformation', block=True):
        """
        Plot a rotation and translation using matplotlib's quiver method
        """
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # first drawing the "-" . Next drawing two lines for each head ">"
        # colors = ['red', 'green', 'blue', 'red', 'red', 'green', 'green', 'blue', 'blue']
        ax.view_init(15, 35)
        # plot identity axes at (0, 0, 0)
        # identity = np.eye(3)
        pos = self.pos()
        # plot identity axis
        ax.quiver(0, 0, 0, 1, 0, 0, linestyle='dashed', color='red', linewidth=3)
        ax.quiver(0, 0, 0, 0, 1, 0, linestyle='dashed', color='green', linewidth=3)
        ax.quiver(0, 0, 0, 0, 0, 1, linestyle='dashed', color='blue', linewidth=3)

        # plot rotated axes
        # axis X
        ax.quiver(pos[0], pos[1], pos[2], self.array[0, 0], self.array[1, 0], self.array[2, 0], color='red',
                  linewidth=3)
        # axis y
        ax.quiver(pos[0], pos[1], pos[2], self.array[0, 1], self.array[1, 1], self.array[2, 1], color='green',
                  linewidth=3)
        # axis Z
        ax.quiver(pos[0], pos[1], pos[2], self.array[0, 2], self.array[1, 2], self.array[2, 2], color='blue',
                  linewidth=3)

        ax.set_xlim([min(-pos[0], -1)-1, max(pos[0], 1)+1])
        ax.set_ylim([min(-pos[1], -1)-1, max(pos[1], 1)+1])
        ax.set_zlim([min(-pos[2], 1)-1, max(pos[2], 1)+1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.title(title)
        plt.show(block=block)


def compute_homogeneous_transforms(df_data):
    """
    Compute homogeneous transforms from global panda.
    """
    transforms = []
    for i in df_data.index:
        # CAUTION: THE ORDER IN THE QUATERNION class IS [qw, qx qy qz]
        # the order in ROS is [qx qy qz qw]
        qw = df_data['qw'][i]
        qx = df_data['qx'][i]
        qy = df_data['qy'][i]
        qz = df_data['qz'][i]
        q = [qw, qx, qy, qz]
        x = df_data['x'][i]
        y = df_data['y'][i]
        z = df_data['z'][i]
        pos = [x, y, z]
        Q = quaternion.Quaternion(q)
        Ti = HomogeneousMatrix(pos, Q)
        transforms.append(Ti)
    return transforms


def compute_relative_transformations(global_transforms):
    """
    Given a list of global transforms, obtain n-1 relative transforms.
    """
    transforms_relative = []
    # compute relative transformations
    for i in range(len(global_transforms) - 1):
        Ti = global_transforms[i]
        Tj = global_transforms[i + 1]
        Tij = Ti.inv() * Tj
        transforms_relative.append(Tij)
    return transforms_relative


def compute_global_transformations(transforms_relative, T0, Trobot_gps):
    """
    Compute global transformations from relative transformations, starting at T0.
    """
    if T0 is None:
        T = HomogeneousMatrix()
    else:
        T = T0
    if Trobot_gps is None:
        Trobot_gps = HomogeneousMatrix()
    transforms_global = []
    transforms_global.append(T0)
    # compute global transformations from relative
    for i in range(len(transforms_relative)):
        Tij = transforms_relative[i]
        # Tij.print_nice()
        T = T*Tij
        transforms_global.append(T)
    # given that the global coordinates are computed, now compute a relative transform
    for i in range(len(transforms_global)):
        transforms_global[i] = transforms_global[i]*Trobot_gps
    return transforms_global


def multiply_by_transform(transforms, Trel):
    transforms_result = []
    for i in range(len(transforms)):
        transforms_result.append(transforms[i]*Trel)
    return transforms_result