import tube_graph

if __name__ == '__main__':
    graph = tube_graph.TubeGraph()
    graph.write_dot('graph.dot')
    print(graph.shortest_path('Embankment', 'Temple_London'))
    print(graph.shortest_path('City_of_London', 'King_s_Cross'))


    # Issue: Wembley_Central should be linked to Stonebridge_London
