import sys
import os.path
DIR_PATH = os.path.dirname(__file__)
sys.path.append(os.path.join(DIR_PATH, '..'))
import csv
from math import sqrt

from search import Graph

def map_parser(filename):
    edges = dict()
    nodes = dict()

    def add_edge(corr1, corr2):
        dist = int(sqrt((corr2[0] - corr1[0]) ** 2 + (corr2[1] - corr1[1]) ** 2))
        label1 = nameify(corr1)
        label2 = nameify(corr2)
        d = edges.get(label1)
        if d:
            d[label2] = dist
        else:
            edges[label1] = {label2: dist}
  
    def add_node(corr):
        nodes[nameify(corr)] = corr

    def nameify(corr):
        return '(' + str(corr[0]) + ',' + str(corr[1]) + ')'

    with open(os.path.join(DIR_PATH, filename), 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=' ')

        for row in reader:
            corr1 = int(row[0]), int(row[1])
            corr2 = int(row[3]), int(row[4])
            add_node(corr1)
            add_node(corr2)
            add_edge(corr1, corr2)

    graph = Graph(edges, directed=True)
    graph.locations = nodes
    return graph

# romania_map = map_parser('data/manhattan.txt')
# print(romania_map)
