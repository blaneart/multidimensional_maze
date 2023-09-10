from path_finder import PathFinder3D
import numpy as np
import cProfile, pstats, io
from pstats import SortKey


with cProfile.Profile() as pr:
    np.random.seed(23)
    my_path_finder = PathFinder3D()    
    # maze_3d = np.random.randint(low=0, high=2, )

    maze_3d = np.random.choice([0, 1], size=(3, 5, 9), p=[0.7, 0.3])
    stop = [2, 4, 8]
    start = [1, 2, 0]
    maze_3d[stop[0], stop[1], stop[2]] = 0
    print(maze_3d)
    print(maze_3d[stop[0], stop[1], stop[2]], maze_3d[start[0], start[1], start[2]])
    path = my_path_finder.find_path(maze_3d, start=start, stop=stop, verbose=False)
    print(path)

    pr.disable()
    s = io.StringIO()
    sortby = SortKey.TIME
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats(10)
    print(s.getvalue())

