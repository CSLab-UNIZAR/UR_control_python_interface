"""Building blocks of example 4, model-free shape servoing in image space.

    settings.py   reads and checks examples/04_shape_servoing.yaml
    vision.py     camera (RealSense or webcam), ArUco detection, Kalman tracking of the markers
    features.py   shape features of the markers: positions, edges, curvature
    jacobian.py   least-squares fit, Broyden update, damped pseudoinverse
    session.py    the control thread: teleoperation, target, probing, servoing, log
    display.py    the live window (matplotlib with blitting): image space and time plots

The program itself is examples/04_shape_servoing.py.
"""
