import argparse
from collections import defaultdict
import matplotlib.pyplot as plt
import multiprocessing
import numpy as np
import seaborn as sns
import sys

from selfdrive.locationd.torqued import ACCELERATION_DUE_TO_GRAVITY, slope2rot, FRICTION_FACTOR, MIN_VEL, LAT_ACC_THRESHOLD
from tools.plotjuggler.juggle import route_or_segment_to_log_paths, load_segment

sns.set_theme()
MIN_POINTS = 1000
STEER_MAX_THRESHOLD = 0.5


def tls(x, y):
  points = np.column_stack((x, np.ones_like(x), y))
  _, _, v = np.linalg.svd(points, full_matrices=False)
  slope, offset = -v.T[0:2, 2] / v.T[2, 2]
  _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
  friction_coeff = np.std(spread) * FRICTION_FACTOR
  return slope, offset, friction_coeff


def get_data(log_paths):
  logs = []
  with multiprocessing.Pool(24) as pool:
    for d in pool.map(load_segment, log_paths):
      logs += d

  fields = {
    'active': ['carControl/latActive', None],
    'steer_torque': ['carControl/actuatorsOutput/steer', None],
    'yaw_rate': ['liveLocationKalman/angularVelocityCalibrated/value', 2],
    'roll': ['liveLocationKalman/orientationNED/value', 0],
    'v_ego': ['carState/vEgo', None]
  }
  raw_data = defaultdict(dict)
  for key, [path, index] in fields.items():
    path = path.split("/")
    raw_data[key]['t'] = np.array([x.logMonoTime * 1e-9 for x in logs if x.which() == path[0]])
    vals = [x for x in logs if x.which() == path[0]]
    for p in path:
      vals = [x._get(p) for x in vals]
    if index is not None:
      vals = [x[index] for x in vals]
    raw_data[key]['vals'] = np.array(vals)

  # interpolate all values to a reference time
  processed = {}
  processed['t'] = np.arange(raw_data['v_ego']['t'][0], raw_data['v_ego']['t'][-1], 0.1)
  for key in raw_data:
    processed[key] = np.interp(processed['t'], raw_data[key]['t'], raw_data[key]['vals'])
  processed['active'] = processed['active'].astype(bool)
  processed['steer_torque'] *= -1
  processed['lat_accel_g_adjusted'] = (processed['yaw_rate'] * processed['v_ego']) - (np.sin(processed['roll']) * ACCELERATION_DUE_TO_GRAVITY)

  processed['filters'] = (processed['active']) & (processed['v_ego'] > MIN_VEL) & (abs(processed['lat_accel_g_adjusted']) <= LAT_ACC_THRESHOLD) \
    & (abs(processed['steer_torque']) <= STEER_MAX_THRESHOLD)
  processed['steer_input'] = processed['steer_torque'][processed['filters']]
  processed['lat_accel_output'] = processed['lat_accel_g_adjusted'][processed['filters']]

  return processed


def visualize(processed):
  plt.figure(figsize=(20, 10))
  plt.subplot(1, 2, 2)
  plt.scatter(processed['steer_torque'], processed['lat_accel_g_adjusted'], alpha=0.05, label="all points")
  plt.scatter(processed['steer_input'], processed['lat_accel_output'], alpha=0.3, label="filtered points", c="r")
  plt.xlabel("steer_torque [-1, 1]")
  plt.ylabel("lateral_acceleration (m/s^2)")
  if np.sum(processed['filters']) > MIN_POINTS:
    slope, offset, friction = tls(processed['steer_input'], processed['lat_accel_output'])
    plt.plot(np.arange(-1, 1, 0.1), np.arange(-1, 1, 0.1) * slope + offset, label="TLS line", c="g")
    plt.text(-1, 1.6, f"Approx. estimate of slope for this route/segment ~ {np.round(slope, 2)}")
    plt.text(-1, 1.4, f"Approx. estimate of friction for this route/segment ~ {np.round(friction, 2)}")
    plt.text(-1, -3, "Note: These estimates do not account for actuator lag, which is essential, and used in the live learner")
    plt.legend()
    print(f"\n\nApproximate offline estimates for this route/segment: \nslope: {slope};\noffset: {offset};\nfriction: {friction}")
  else:
    print(f"Not enough points ({np.sum(processed['active'])}) to fit a curve!")

  plt.subplot(3, 2, 1)
  plt.plot(processed['t'], processed['v_ego'])
  plt.xlabel("t (s)")
  plt.ylabel("v_ego (m/s)")

  plt.subplot(3, 2, 3)
  plt.plot(processed['t'], processed['lat_accel_g_adjusted'])
  plt.xlabel("t (s)")
  plt.ylabel("lateral_acceleration (m/s^2)")

  plt.subplot(3, 2, 5)
  plt.plot(processed['t'], processed['active'])
  plt.xlabel("t (s)")
  plt.ylabel("active")

  plt.suptitle("Visualization of Lateral Control Parameters")
  plt.tight_layout()
  plt.show()


def process_data(route_or_segment_name, segment_count, qlog):
  log_paths = route_or_segment_to_log_paths(route_or_segment_name, segment_count, qlog, ci=False)
  processed = get_data(log_paths)
  visualize(processed)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="A helper to visualize torque response on openpilot routes",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--qlog", action="store_true", help="Use qlogs")
  parser.add_argument("route_or_segment_name", nargs='?', help="The route or segment name to plot (cabana share URL accepted)")
  parser.add_argument("segment_count", type=int, nargs='?', help="The number of segments to plot")

  if len(sys.argv) == 1:
    parser.print_help()
    sys.exit()
  args = parser.parse_args()
  process_data(args.route_or_segment_name.strip(), args.segment_count, args.qlog)
