# Final Project of Ubiquitous Robot (JCCX0017) @ SJTU

## TODO
- [x] 1. Basic implementation of 2D Pixel Space -> Robot Space (using `cv2`)
- [x] 2. Robotic arm simulation, IK, and motion planning (using `pybullet`)
- [x] 3. Robotic arm control (`sdk 2.1.14`): Note that some 402 robotic arms cannot connect via Ethernet—be cautious!
- [ ] 4. Integration with ModelArts inference
- [ ] 5. Optimization of 2D Pixel Space -> Robot Space conversion

## Integration with ModelArts Inference
For invoking ModelArts APIs in Python, refer to `APIGW-python-sdk-2.0.5/main.py`.  
Implement the `get_pixel_point()` function in the `RobotController` class in `test_planning_real.py`.

## Environment Setup
```
# create a conda env if you want

pip install pybullet_planning
pip install pybullet
```

## Project Entry Point
Run `export LD_LIBRARY_PATH=./GeneralRobot/SDK` before running.

The `test_planning_real.py` script contains functions for both simulation and real-world testing.  
