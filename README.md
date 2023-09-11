# PathFinder3D

## Introduction

`PathFinder3D` is a Python class designed to find the shortest path in a 3D maze using the weighted A* (A-star) algorithm. This class is particularly useful for solving pathfinding problems in a three-dimensional space.


## Usage

To use the `PathFinder3D`:


```python
from path_finder import PathFinder3D

path_finder = PathFinder3D()
start = [0, 0, 0]
end = [2, 2, 2]
path = path_finder.find_path(matrix, start, end)
```

If you want to visualize the results add:
```
path_finder.plot_matrix(matrix, start, end, path)
```

Or simply add verbose=True

```
path_finder.find_path(matrix, start, end, verbose=True)
```

By default only moves in one dimension per step are possible, you can add diagonal moves by initializing class

```
path_finder = PathFinder3D(diagonal=True)
``````