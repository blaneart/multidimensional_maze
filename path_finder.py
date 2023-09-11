import numpy as np
from typing import Any, List
import matplotlib.pyplot as plt
import itertools
from collections import deque


class PathFinder3D:
    """PathFinder in 3D maze
    Attributes:
        steps: possible directions of moves

    """
    def __init__(self, diagonal=False):
        if diagonal:
            possible_moves = [-1, 0, 1]
            self.steps = [p for p in itertools.product(possible_moves, repeat=3)]
            self.steps.remove((0, 0, 0))
        else:
            self.steps = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]

    class Node:
        """Node representation of a position in matrix
        Attributes:
            parent: parent node
            position: coordinates in maze
            g: cost of the path
            h: heuristic function
            f: g + h
        """
        def __init__(self, parent=None, position=None):
            self.parent = parent
            self.position = position
            self.g = 0
            self.h = 0
            self.f = 0

        def __eq__(self, other):
            return np.array_equal(self.position, other.position)

    def add_children(self,
                     matrix,
                     current_node,
                     shape,
                     closed_set) -> List[List[int]]:
        """Finds shortest path in 3d matrix

        Args:
          matrix: 3 dimensional numpy array filled with 0 and 1
          current_node: node from which we explore next steps
          shape: matrix shape
          closed_set: set of already visited nodes

          Returns:
            List of possible position to search path
        """
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

    def astar(self, matrix, start, end, w) -> List[List[int]]:
        """Implementation of weighted A* algorithm

        Args:
          matrix: 3 dimensional numpy array filled with 0 and 1
          start: node from which we explore next steps
          end: matrix shape
          w: set of already visited nodes

          Returns:
            List of moves from start to end or None if path
            could not be built
        """
        start_node = self.Node(None, start)
        end_node = self.Node(None, end)
        open_list = []
        closed_set = np.zeros(matrix.shape, dtype=bool)
        open_list.append(start_node)
        shape = np.array(matrix.shape)
        while open_list:
            current_node = min(open_list, key=lambda node: node.f)
            open_list.remove(current_node)
            closed_set[tuple(current_node.position)] = True
            if current_node == end_node:
                path = deque()
                current = current_node
                while current is not None:
                    path.appendleft(list(current.position))
                    current = current.parent
                return list(path)
            children = self.add_children(matrix,
                                         current_node,
                                         shape,
                                         closed_set)
            for child in children:
                child.g = current_node.g + 1
                child.h = w * np.linalg.norm(child.position - end_node.position)
                child.f = child.g + child.h
                if child in open_list and child.g > open_list[open_list.index(child)].g:
                    continue
                open_list.append(child)
        return None

    def update_visualisation(self, matrix, colors, node, color):
        matrix[node[0], node[1], node[2]] = 2
        colors[node[0], node[1], node[2]] = color
        return matrix, colors

    def plot_matrix(self, matrix, start, end, path) -> None:
        """Function to plot matrix, start, end and path
            coordinates

        Args:
          matrix: 3 dimensional numpy array filled with 0 and 1
          start: starting coordinates in the matrix
          end: goal coordinates in the matrix
          path: list of nodes from start to end
        """
        fig = plt.figure()
        axes = list(matrix.shape)
        colors = np.empty(axes + [4], dtype=np.float32)
        colors[:] = [1, 0, 0, 0.5]  # red
        matrix, colors = self.update_visualisation(matrix, colors, start, [0, 0, 1, 0.7])
        matrix, colors = self.update_visualisation(matrix, colors, end, [0, 1, 0, 0.7])
        if path and len(path) > 2:
            for node in path[1:-1]:
                matrix, colors = self.update_visualisation(matrix,
                                                           colors,
                                                           node,
                                                           [1, 1, 1, 0.7])
        cax = fig.add_subplot(111, projection='3d')
        cax.voxels(matrix, facecolors=colors, edgecolors=None)
        plt.show()

    def find_path(
            self,
            matrix: Any,
            start: List[int],
            stop: List[int],
            w: float = 2,
            verbose: bool = False
    ) -> List[List[int]]:
        """Finds shortest path in 3d matrix

        Args:
          matrix: 3 dimensional numpy array filled with 0 and 1
          start: starting coordinates in the matrix
          end: goal coordinates in the matrix
          w: weights for A*
          verbose: if true shows maze in matplotlib
          Returns:
            List of 3d coordinates representing path from start to the end,
              empty if path is not possible
        """

        assert type(matrix) == np.ndarray, (
            'Maze should be numpy array'
        )
        assert len(matrix.shape) == 3, (
            f'Expected 3 dimensional maze, not {len(matrix.shape)}'
        )

        assert len(start) == 3, (
            'Start coordinates should be (x, y, z)'
        )
        assert len(stop) == 3, (
            'Stop coordinates should be (x, y, z)'
        )
        path = self.astar(matrix, np.array(start), np.array(stop), w)
        if verbose:
            self.plot_matrix(matrix, start, stop, path)
        return path
