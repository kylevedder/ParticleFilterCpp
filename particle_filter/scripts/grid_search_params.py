#!/usr/bin/env python3

import subprocess
import pandas
import numpy as np
import os.path

def run_pf():
  subprocess.run("rosrun particle_filter particle_filter_bag_reader", shell=True)

def get_errors_centroid(filename):
  assert(os.path.isfile(filename))
  df = pandas.read_csv(filename)
  col = df["cent_error_norm"]
  return (col.quantile(0.05), col.quantile(0.5), col.quantile(0.95))

def get_errors_max(filename):
  assert(os.path.isfile(filename))
  df = pandas.read_csv(filename)
  col = df["max_error_norm"]
  return (col.quantile(0.05), col.quantile(0.5), col.quantile(0.95))

def remove_file(filename):
  subprocess.run("rm {}".format(filename), shell=True, stdout=subprocess.DEVNULL)

def gen_config_file(laser, arc, rot, consist):
  template = \
"""pf = {{
  kLaserStdDev = {};
  kArcStdDev = {};
  kRotateStdDev = {};
  kTemporalConsistencyWeight = {};
}};
""".format(laser, arc, rot, consist)
  return template

def write_config(filename, content):
  f = open(filename, 'w')
  f.write(content)
  f.close()

def result_header(filename):
  f = open(filename, 'w')
  f.write("laser,arc,rot,consist,centroid_5th,centroid_50th,centroid_95th,max_5th,max_50th,max_95th\n")
  f.close()

def result_append(filename, laser, arc, rot, consist, centroid_5th, centroid_50th, centroid_95th, max_5th, max_50th, max_95th):
  f = open(filename, 'a')
  f.write("{},{},{},{},{},{},{},{},{},{}\n".format(laser, arc, rot, consist, centroid_5th, centroid_50th, centroid_95th, max_5th, max_50th, max_95th))
  f.close()

laser_vals = np.arange(0.001, 0.5, 0.05)
arc_vals = np.arange(0.001, 0.5, 0.05)
rot_vals = np.arange(0.001, 0.2, 0.02)
consist_vals = [0]

error_filename = "error.csv"
result_filename = "results.csv"
result_header(result_filename)

for laser in laser_vals:
  for arc in arc_vals:
    for rot in rot_vals:
      for consist in consist_vals:
        print("Laser {} Arc {} Rot {} Consist {}".format(laser, arc, rot, consist))
        content = gen_config_file(laser, arc, rot, consist)
        write_config("src/particle_filter/config/pf_config.lua", content)
        remove_file(error_filename)
        run_pf()
        centroid_5th, centroid_50th, centroid_95th = get_errors_centroid(error_filename)
        max_5th, max_50th, max_95th = get_errors_max(error_filename)
        result_append(result_filename, laser, arc, rot, consist, centroid_5th, centroid_50th, centroid_95th, max_5th, max_50th, max_95th)