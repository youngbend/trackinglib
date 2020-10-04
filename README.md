Multithreaded Python Library written in C++ for tracking targets in EECS 467

Author: Benjamin Young (youngben@umich.edu)

Class object:
```
Tracker(int scan_offset_row, int scan_offset column, int target offset, int threshold, int tracking_offset, int tracking_timeout)
```

Class functions:
```
Tracker.scan(gray_image, num_threads)
Tracker.update_targets(gray_image)
Tracker.get_target_centers()
```

Tracker.get_target_centers() returns a point type where point has member variables:
```
point.row
point.col
```
