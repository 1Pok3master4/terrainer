## Terrain Mapping Drone Package

This package contains all the necessary code required to run the terrain mapping drone.

### Relevant Scripts

- `test_controller.py`
- `live_feed.py`
- `map.py` (located in the `maps` directory)
- `controller.py`

**Note:**  
All scripts reference absolute file paths. Make sure to change the file paths before running the code.

### Configuration

- The grid size and drone height can be altered based on the terrain being mapped.
- The origin of the GPS coordinate system must be set so that the entire terrain to be surveilled fits in the first quadrant.
- The number of waypoints will be calculated automatically.

### Features

- `live_feed.py` allows you to monitor the location of the drone in real time.
- `controller.py` allows manual control if there is any deviation.
- In the `maps` directory, `maps.py` allows for plotting after the mapping is done.

**Requirement:**  
`live_feed.py` requires NumPy 1.X in order to run.
