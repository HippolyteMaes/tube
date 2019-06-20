import os
from xml.etree import ElementTree

class TubeGraph:

    class Node:

        def __init__(self, ref):
            self.ref = ref
            self.links = dict()

        def add_link(self, destination, distance):
            '''
            Add a link starting from this node. since we don't take into
            account line change time, we can just ignore the case where several
            lines lead from a station to another directly.

            destination: reference of the destination node
            distance: distance between the two nodes
            '''
            if destination not in self.links.keys():
                try:
                    distance = str(int(distance))
                    self.links[destination] = distance
                except:
                    # if the distance is not a number, ignore
                    pass

    def __init__(self, path=os.path.dirname(os.path.realpath(__file__))):
        self.name_to_ref = dict()
        self.ref_to_name = dict()
        self.ref_to_node = dict()
        self.stoppoint_to_ref = dict()
        self.build(path)

    def build(self, path):
        '''
        Build graph based on all files starting with tfl_1- in path. this
        correspond to the files related to the tube lines

        path: directory containing tube files
        '''
        files = filter(lambda f: 'tfl_1-' in f, os.listdir(path))

        # Fill reference list with all stations
        trees = []
        for f in files:
            tree = ElementTree.parse(f)
            root = tree.getroot()

            self.fill_references(root[0])
            self.fill_stoppoints(root[1])
            trees.append(tree)

        # Connect graph with routes
        for tree in trees:
            root = tree.getroot()
            self.connect_graph(root[2])

    def fill_references(self, localities):
        '''
        Fill references of each stations

        localities: list of xml localities
        '''
        for locality in localities:
            ref = locality[0].text
            name = locality[1].text

            # Replacing special caracters for later dot representation
            name = name.replace('&', 'and')
            name = name.replace('(', '')
            name = name.replace(')', '')
            name = name.replace('/', ' ')
            name = name.replace('\'', ' ')
            name = name.replace('.', '')
            name = name.replace(' ', '_')

            self.name_to_ref[name] = ref
            self.ref_to_name[ref] = name
            self.ref_to_node[ref] = TubeGraph.Node(ref)

    def fill_stoppoints(self, stoppoints):
        '''
        Connect stoppoints to station of reference

        stoppoints: list of xml stoppoints
        '''
        for stoppoint in stoppoints:
            stoppoint_ref = stoppoint[0].text
            station_ref = stoppoint[2][0].text
            self.stoppoint_to_ref[stoppoint_ref] = station_ref

    def connect_graph(self, routes):
        '''
        Connect nodes by routes described in xml files

        routes: routes to add to the graph
        '''
        for route_section in routes:
            for link in route_section:
                start = self.stoppoint_to_ref[link[0][0].text]
                destination = self.stoppoint_to_ref[link[1][0].text]
                distance = link[2].text

                # Making sure the route points to metro stations on both sides
                if (start in self.ref_to_node.keys()
                    and destination in self.ref_to_node.keys()):

                    self.ref_to_node[start].add_link(destination, distance)

    def write_dot(self, path):
        '''
        Write DOT representation to a file

        path: path of the ouput file
        '''
        with open(path, 'w') as f:
            f.write('digraph tube {\n')
            for name, ref in self.name_to_ref.items():
                for dest, distance in self.ref_to_node[ref].links.items():
                    dest_name = self.ref_to_name[dest]
                    link = '\t{} -> {} [label={}];\n'.format(name,
                                                             dest_name,
                                                             distance)
                    f.write(link)
            f.write('}')

    def shortest_path(self, start, destination, use_distance=True):
        '''
        Returns a list of stations forming the shortest path from start to
        destination. Dijkstra algorithm.

        start: name of the starting station
        destination: name of the destination station
        use_distance: if True use the distance attribute to compute the
                      shortest path, else count the shortest number of station
        '''

        nodes = dict(self.ref_to_node)
        dist = dict.fromkeys(self.ref_to_node, float('inf'))
        prev = dict.fromkeys(self.ref_to_node, float('inf'))

        start = self.name_to_ref[start]
        destination = self.name_to_ref[destination]
        dist[start] = 0

        while len(nodes) > 0:
            curr_ref = min(
                dist,
                key=lambda x: dist[x] if x in nodes.keys() else float('inf')
            )
            nodes.pop(curr_ref)

            if curr_ref == destination:
                break

            for other_ref, distance in self.ref_to_node[curr_ref].links.items():
                if other_ref in nodes.keys():
                    new_dist = dist[curr_ref] + (int(distance) if use_distance else 1)
                    if new_dist < dist[other_ref]:
                        dist[other_ref] = new_dist
                        prev[other_ref] = curr_ref

        path = [destination]
        while destination != start:
            destination = prev[destination]
            path.append(destination)

        path.reverse()
        path = list(map(lambda x: self.ref_to_name[x], path))
        return path
