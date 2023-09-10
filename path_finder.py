import numpy as np
from typing import Any, List
import matplotlib.pyplot as plt
import itertools
from collections import deque
from mpl_toolkits.mplot3d import Axes3D


class PathFinder3D:
    def __init__(self, diagonal=False):
        if diagonal:
            possible_moves = [-1, 0, 1]
            self.steps = [p for p in itertools.product(possible_moves, repeat=3)]
        else:
            self.steps = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
        self.cax = None
        self.colors = None
    class Node:
        def __init__(self, parent=None, position=None):
            self.parent = parent
            self.position = position
            self.g = 0
            self.h = 0
            self.f = 0


        def __eq__(self, other):
            return np.array_equal(self.position, other.position)
        

    def add_children(self, matrix, current_node, shape, closed_set):
        children = []
        new_positions = current_node.position + self.steps
        valid_mask = np.all(new_positions >= 0, axis=1) & np.all(new_positions < shape, axis=1)
        valid_positions = new_positions[valid_mask]
        
        for new_pos in valid_positions:
            if matrix[tuple(new_pos)] == 1 or closed_set[tuple(new_pos)]:
                continue
            child_node = self.Node(current_node, new_pos)
            children.append(child_node)
        
        return children
    

    def astar(self, matrix, start, end):
        start_node = self.Node(None, start)
        end_node = self.Node(None, end)
        
        open_list = []
        closed_set = np.zeros(matrix.shape, dtype=bool)
        open_list.append(start_node)
        shape = np.array(matrix.shape)
        while open_list:
            current_node = min(open_list, key=lambda node: node.f)
            matrix[current_node.position[0], current_node.position[1], current_node.position[2]] = 2  # Mark current node as explored (2 represents explored)
            self.colors[current_node.position[0], current_node.position[1], current_node.position[2]] = [0,1,1, 0.5]  # Mark current node as explored (2 represents explored)

            open_list.remove(current_node)
            closed_set[tuple(current_node.position)] = True

            if current_node == end_node:
                path = deque()
                current = current_node
                while current is not None:
                    path.appendleft(current.position)
                    current = current.parent
                for node in path:
                    matrix[node[0],node[1],node[2]] = 3  # Mark path nodes as 3
                    self.colors[node[0],node[1],node[2]] = [1,1,1, 0.9]  # Mark path nodes as 3

                self.update_visualization(matrix)
                return list(path)
            children = self.add_children(matrix, current_node, shape, closed_set)
            for child in children:
                child.g = current_node.g + 1
                child.h = 2 * np.linalg.norm(child.position - end_node.position)
                child.f = child.g + child.h
                if child in open_list and child.g > open_list[open_list.index(child)].g:
                    continue
                open_list.append(child) 
                matrix[child.position[0],child.position[1],child.position[2]] = 5  # Mark neighbor as in open list (1 represents open list)
                self.colors[child.position[0],child.position[1],child.position[2]] = [1,1,0,0.1]  # Mark neighbor as in open list (1 represents open list)
                
                self.update_visualization(matrix)

        return None

    def update_visualization(self, grid):
        self.cax.clear()  # Clear the previous plot
        self.cax.voxels(grid, facecolors=self.colors, edgecolors='k')  # Display the 3D grid
        plt.pause(0.1)  # Introduce a delay to slow down the visualization
            
    def plot_matrix(self, matrix, start, end):
        # fig = plt.figure()
        # self.cax = fig.add_subplot(111, projection='3d')
        fig = plt.figure()
        axes = list(matrix.shape)
        self.colors = np.empty(axes + [4], dtype=np.float32)
        alpha = 0.1
        self.colors[:] = [1, 0, 0, alpha + 0.5]  # red
        self.colors[start[0],start[1],start[2]] = [0,0,1,alpha + 0.5]
        matrix[start[0],start[1],start[2]] = 3
        matrix[end[0],end[1],end[2]] = 3
        self.colors[end[0],end[1],end[2]] = [0,1,0, alpha +0.5]
        self.cax = fig.add_subplot(111, projection='3d')
        # if path is not None:
        #     for step in path[1:-1]:
        #         matrix[step[0], step[1], step[2]] = 2
        #         colors[step[0], step[1], step[2]] = [1,0,1, alpha + 0.5]
        self.cax.voxels(matrix, facecolors=self.colors, edgecolors=None)
        # plt.show()
        
    def find_path(
            self,
            matrix: Any,
            start: List[int],
            stop: List[int],
            verbose: bool = False
    ) -> List[List[int]]:
        """Finds shortest path in 3d matrix

        Args:
          matrix: 3 dimensional numpy array filled with 0 and 1
          start: starting coordinates in the matrix
          end: goal coordinates in the matrix

          Returns:
            List of 3d coordinates representing path from start to the end,
              empty if path is not possible

          Raises:
            TypeError: matrix is not numpy array
            ValueError: matrix is wrong dimension

        """
        self.plot_matrix(matrix, start, stop)
        assert type(matrix) == np.ndarray
        assert len(matrix.shape) == 3
        assert len(start) == 3
        assert len(stop) == 3
        path = self.astar(matrix, np.array(start), np.array(stop))
        if verbose:
            self.plot_matrix(matrix, start, stop, path)
        plt.show()
        return path
