from path_finder import PathFinder3D
import numpy as np
import cProfile, pstats, io
from pstats import SortKey


for w in np.arange(2, 3.0, 0.25):
    with cProfile.Profile() as pr:
        np.random.seed(23)
        my_path_finder = PathFinder3D(diagonal=False)    
        maze_3d = np.random.choice([0, 1], size=(3, 5, 9), p=[0.7, 0.3])
        stop = [2, 4, 8]
        start = [0, 0, 0]
        maze_3d[stop[0], stop[1], stop[2]] = 0
        path = my_path_finder.find_path(maze_3d, start=start, stop=stop, w=w)
        pr.disable()
        s = io.StringIO()
        sortby = SortKey.TIME
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        ps.print_stats(10)
        print(s.getvalue())
