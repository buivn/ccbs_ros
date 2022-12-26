
from shapely.geometry import Point
import re
import numpy
import numpy as np
import random
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon
# find shortest path
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path
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

def find_shortest_path(points, edge_lengths):
  # Find the path:
  dist_matrix, predecessors = shortest_path(csgraph=edge_lengths, method='D', directed=False, indices=0,
                                          return_predecessors=True)
  prev_ind = 1
  counter = 0
  distance = 0
  did_reach_goal = True
  while counter < num_sampled_points:
      point_ind = predecessors[prev_ind]
      if point_ind < 0:
          # print(point_ind)
          did_reach_goal = False
          break
      if point_ind == 0:
          did_reach_goal = True
          break
      point_a = points[prev_ind]
      point_b = points[point_ind]
      # distance += dist_matrix[prev_ind, point_ind]
      plt.plot([point_a[0], point_b[0]], [point_a[1], point_b[1]],
               'b', alpha=0.8)

      prev_ind = point_ind
      counter += 1

  if not did_reach_goal:
    raise ValueError("Did not reach goal")
  return did_reach_goal

def build_prm_graph(egion_x, region_y, room, obstacles, max_distance, num_sampled_points):
  
  # start1 = [160,190]
  # goal1 = [236,200]
  # start2 = [210,220]
  # goal2 = [200,170]

  start1 = [190,160]
  goal1 = [190,210]
  start2 = [160,170]
  goal2 = [236,190]
  start3 = [230,170]
  goal3 = [170,190]
  points = [start1, goal1, start2,goal2, start3, goal3]


  for _ in range(num_sampled_points):
    px = random.uniform(region_x[0], region_x[1])
    py = random.uniform(region_y[0], region_y[1])
    point = [px, py]

    if (not does_contain_point(point, obstacles)) and (room.contains(Point(point))):
      points.append(point)

  edge_length_matrix = np.zeros((len(points), len(points)))
  for ii, point_a in enumerate(points):
    for jj, point_b in enumerate(points[ii + 1:]):
      # reject (continue) if the points are within the max distance
      distance = np.linalg.norm(np.array(point_a) - np.array(point_b))
      if distance > max_distance:
        continue

      # Collision check: if the line between the points collides with the geometry, continue.
      if does_line_collide([point_a, point_b], obstacles):
        continue

      # Update the edge_length_matrix
      edge_length_matrix[ii, jj + ii + 1] = distance
      edge_length_matrix[jj + ii + 1, ii] = distance

  return points, edge_length_matrix

# handle the new lines in the xml file
def indent(elem, level=0):
  i = "\n" + level*"  "
  if len(elem):
    if not elem.text or not elem.text.strip():
      elem.text = i + "  "
    if not elem.tail or not elem.tail.strip():
      elem.tail = i
    for elem in elem:
      indent(elem, level+1)
    if not elem.tail or not elem.tail.strip():
      elem.tail = i
  else:
    if level and (not elem.tail or not elem.tail.strip()):
      elem.tail = i




if __name__ == "__main__":
  num_sampled_points = 3000
  max_distance = 4
  region_x = [145, 255]
  region_y = [128, 238]

  #plt.figure(figsize=(8, 8), dpi=150)

  image = read_pgm('./map.pgm', byteorder='<')
  # print("check ------------------------------------")
  plt.imshow(image, plt.cm.gray)
  plt.axis([140, 255, 128, 238])
  
  # Plot the environment
  points, edge_lengths = build_prm_graph(region_x, region_y, room_buffer,
                                         env_multi, max_distance, num_sampled_points)
  plot_env(room_buffer, env_multi)
  
  
  # show the graph
  np_points = np.array(points)
  plt.plot(np_points[:, 0], np_points[:, 1], '.')
  #Plot the edges between the points
  for ii, point_a in enumerate(points):
    for jj, point_b in enumerate(points[ii + 1:]):
      if edge_lengths[ii, jj + ii + 1] > 0:

        plt.plot([point_a[0], point_b[0]],
                 [point_a[1], point_b[1]],
                 'y', alpha=0.2)
  

  # check = find_shortest_path(points, edge_lengths)
  plt.show()

  # find the edge matrix to save to xml file
  edges=[]
  for ii, point_a in enumerate(points):
    for jj, point_b in enumerate(points[ii + 1:]):
      if edge_lengths[ii, jj + ii + 1] > 0:
        edges.append([ii, jj + ii + 1])

  # This is the parent (root) tag onto which other tags would be created
  data = ET.Element('graphml')
  data.set('xmlns', 'http://graphml.graphdrawing.org/xmlns')
  # data.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
  # data.set('xsi:schemaLocation', 'http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd')

  # Adding a subtag named `key` inside our root tag
  element1 = ET.SubElement(data, 'key')
  element1.set('id', 'key0')
  element1.set('for', 'node')
  element1.set('attr.name', 'coords')
  element1.set('attr.type', 'string')

  element2 = ET.SubElement(data, 'key')
  element2.set('id', 'key1')
  element2.set('for', 'edge')
  element2.set('attr.name', 'weight')
  element2.set('attr.type', 'double')

  # adding a subtag name "graph"
  graph = ET.SubElement(data, 'graph')
  graph.set('id', 'G')
  graph.set('edgedefault', 'directed')
  graph.set('parse.nodeids', 'free')
  graph.set('parse.edgeids', 'canonical')
  graph.set('parse.order', 'nodesfirst')
  

  # Adding subtags name "node" under the `graph` subtag
  for ii, point in enumerate(points):
    node = ET.SubElement(graph, 'node')
    v = "n"+str(ii)
    node.set('id', v)

    data_node = ET.SubElement(node, 'data')
    data_node.set('key', 'key0')
    x = str(round(point[0],3))
    y = str(round(point[1],3))
    data_node.text = x+","+y


  for ii, e in enumerate(edges):
    edge = ET.SubElement(graph, 'edge')
    edge_id = "e"+str(2*ii)
    edge.set('id', edge_id)
    n1 = "n"+str(e[0])
    n2 = "n"+str(e[1])
    edge.set('source', n1)
    edge.set('target', n2)
    data_edge = ET.SubElement(edge, 'data')
    data_edge.set('key', 'key1')
    data_edge.text="1"

    edge = ET.SubElement(graph, 'edge')
    edge_id = "e"+str(2*ii+1)
    edge.set('id', edge_id)
    n1 = "n"+str(e[1])
    n2 = "n"+str(e[0])
    edge.set('source', n1)
    edge.set('target', n2)
    data_edge = ET.SubElement(edge, 'data')
    data_edge.set('key', 'key1')
    data_edge.text="1"
  
  tree = ET.ElementTree(data)

  indent(data)
  tree.write("graph.xml", encoding="utf-8", xml_declaration=True)

  plt.show() # show to see
