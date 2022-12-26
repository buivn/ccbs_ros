
from shapely.geometry import Point
import re
import numpy
import numpy as np
import random
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon
# find shortest path
# save into xml file
from lxml import etree as ET

# define a list of functions
def does_line_collide(line, obstacles):
    sline = LineString([line[0], line[1]])
    return any(sline.intersects(obstacle) for obstacle in obstacles)

def does_contain_point(point, obstacles):
    spoint = Point(point)
    return any(obstacle.contains(spoint) for obstacle in obstacles)

def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                            count=int(width) * int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

def plot_env(room, obstacles):
  for poly in obstacles:
    plt.plot(*poly.exterior.xy, 'k')
  x,y = room.exterior.xy
  plt.plot(x,y)

env_multi = [
    Point(179,161).buffer(7),
    # Point(200, 173).buffer(5, cap_style=3),
    Point(200, 162).buffer(7),
    Point(223, 162).buffer(7), 
    Point(178, 183).buffer(7),
    Point(200, 184).buffer(7),
    Point(222, 184).buffer(7), 
    Point(178, 205).buffer(7),
    Point(200, 206).buffer(7),
    Point(222, 206).buffer(7), ]

room = Polygon([(223, 135), (250, 182), (223, 235),(180,235), (142,183),
          (180,135),(223, 135)])
room_buffer = room.buffer(-3)

def get_nodes_edges(filename):
  tree = ET.parse(filename)
  root = tree.getroot()
  print(root[2][0].attrib)

def get_paths(filename):
  x1=[]
  y1=[]
  x2=[]
  y2=[]
  x3=[] 
  y3=[]

  with open(filename,"r") as file:
    data1 = False
    data2 = False
    data3 = False
    while True:
      line = file.readline()
      if not line:
        break
      
      if line == 'Trajectory: 0\n':
        data1 = True
        data2 = False
        data3 = False

      if line == 'Trajectory: 1\n':
        data1 = False
        data2 = True
        data3 = False

      if line == 'Trajectory: 2\n':
        data1 = False
        data2 = False
        data3 = True
      
      if data1 and (not (line=='Trajectory: 0\n')): 
        d12 = line.strip()
        d12_split = d12.split()
        x1.append(float(d12_split[0]))
        y1.append(float(d12_split[1]))

      if data2 and (not (line=='Trajectory: 1\n')): 
        d12 = line.strip()
        d12_split = d12.split()
        x2.append(float(d12_split[0]))
        y2.append(float(d12_split[1]))

      if data3 and (not (line=='Trajectory: 2\n')): 
        d12 = line.strip()
        d12_split = d12.split()
        x3.append(float(d12_split[0]))
        y3.append(float(d12_split[1]))
  return x1, y1, x2, y2, x3, y3

if __name__ == "__main__":
  num_sampled_points = 3000
  max_distance = 4
  region_x = [145, 255]
  region_y = [128, 238]

  filename = 'example.txt'
  x1, y1, x2,y2, x3,y3 = get_paths(filename)

  plt.figure(figsize=(8, 8), dpi=150)

  image = read_pgm('./map.pgm', byteorder='<')
  plt.imshow(image, plt.cm.gray)
  plt.axis([140, 255, 128, 238])
  
  # Plot the environment
  plot_env(room_buffer, env_multi)
  
  # show the graph
  plt.plot(x1, y1, 'r', alpha=0.5)
  plt.plot(x2, y2, 'r', alpha=0.5)
  plt.plot(x3, y3, 'r', alpha=0.5)
  
  plt.show()

