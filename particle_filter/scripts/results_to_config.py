#!/usr/bin/env python3

import subprocess
import pandas
import numpy as np

def get_error():
  df = pandas.read_csv("./results.csv")
  return df.loc[df['max_percentile'].idxmin()], df.loc[df['cent_percentile'].idxmin()]

max_percentile, cent_percentile = get_error()
print(max_percentile)
print(cent_percentile)