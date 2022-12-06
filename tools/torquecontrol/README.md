# Torque Control

This script can be used to visualize the `steer_torque` -> `lateral_accelration` response from logs.


## Usage
```
$ usage: visualize_torque.py [-h] [--qlog]
                           [route_or_segment_name]
                           [segment_count]

A helper to visualize torque response on openpilot routes

positional arguments:
  route_or_segment_name
                        The route or segment name to plot
                        (cabana share URL accepted) (default:
                        None)
  segment_count         The number of segments to plot
                        (default: None)

optional arguments:
  -h, --help            show this help message and exit
  --qlog                Use qlogs (default: False)

```

**Note:**  Check out our blog posts for more information on [Torque Control](https://blog.comma.ai/0815release/#torque-controller) or [torqued](https://blog.comma.ai/090release/#torqued-an-auto-tuner-for-lateral-control) (the live torque parameter estimator). 