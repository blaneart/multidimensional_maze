import unittest
import numpy as np
from path_finder import PathFinder3D  


class TestPathFinder3D(unittest.TestCase):

    def test_add_children(self):
        pf = PathFinder3D()
        matrix = np.zeros((3, 3, 3), dtype=int)
        start_node = pf.Node(None, np.array([1, 1, 1]))
        shape = np.array(matrix.shape)
        closed_set = np.zeros(matrix.shape, dtype=bool)

        children = pf.add_children(matrix, start_node, shape, closed_set)
        self.assertEqual(len(children), 6)

    def test_astar(self):
        pf = PathFinder3D()
        matrix = np.zeros((3, 3, 3), dtype=int)
        start = [0, 0, 0]
        end = [2, 2, 2]
        path = pf.astar(matrix, np.array(start), np.array(end), w=2)
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 7) 

    def test_find_path(self):
        pf = PathFinder3D()
        matrix = np.array([[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                           [[0, 0, 0], [0, 1, 0], [0, 0, 0]],
                           [[0, 0, 0], [0, 0, 0], [0, 0, 0]]], dtype=int)
        start = [0, 0, 0]
        end = [2, 2, 2]
        path = pf.find_path(matrix, start, end)
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 7)


if __name__ == '__main__':
    unittest.main()
