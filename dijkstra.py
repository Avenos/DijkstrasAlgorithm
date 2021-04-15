
import numpy as np
import copy

class Vertex(object):
    def __init__(self, ID, name):
        self.ID = ID
        self.name = name

class Edge(object):
    def __init__(self, vertexID1, vertexID2, weight):
        self.vertexID1 = vertexID1
        self.vertexID2 = vertexID2
        self.weight = weight

#A weighted connected graph, weights must be > 0
class Graph(object):
    def __init__(self, vertices, edges):
        self.vertices = vertices
        self.edges = edges
        self.vertexCount = len(vertices)
        self.edgeCount = len(edges)

        self.adjacencyMatrix = np.zeros((self.vertexCount, self.vertexCount))
        for edge in self.edges:
            self.adjacencyMatrix[edge.vertexID1][edge.vertexID2] = edge.weight
            self.adjacencyMatrix[edge.vertexID2][edge.vertexID1] = edge.weight

    def dijkstra(self, sourceVertexId):
        paths = []
        for i in range(0, self.vertexCount):
            path = [sourceVertexId]
            paths.append(path)

        inf = 99999999999999999999
        #Create a list of all visited vertices (will be empty at first)
        visitedVertices = []

        #Assign to every vertex a tentative distance value: set it to zero for our initial vertex and to 
        #infinity for all other vertices. Set the source vertex as current.
        tentativeDistances = []
        for i in range(self.vertexCount):
            tentativeDistances.append(inf)
        tentativeDistances[sourceVertexId] = 0
        currentVertex = sourceVertexId

        #For the current vertex, consider all of its unvisited neighbors and calculate their tentative
        #distances through the current vertex. Compare the newly calculated tentative distance to the 
        #current assigned value and assign the smaller one.
        while(len(visitedVertices) < self.vertexCount):
            neighbors = []
            for i in range(self.vertexCount):
                weight = self.adjacencyMatrix[currentVertex, i]
                if weight > 0:
                    neighbors.append(i)

            unvisitedNeighbors = []
            for neighbor in neighbors:
                if neighbor not in visitedVertices:
                    unvisitedNeighbors.append(neighbor)

            for neighbor in unvisitedNeighbors:
                weight = self.adjacencyMatrix[currentVertex][neighbor]
                tentativeDistance = tentativeDistances[currentVertex] + weight
                if tentativeDistance < tentativeDistances[neighbor]:
                    tentativeDistances[neighbor] = tentativeDistance
                    
                    paths[neighbor] = copy.deepcopy(paths[currentVertex])
                    paths[neighbor].append(neighbor)

            #When we are done considering all of the unvisited neighbors of the current vertex,
            #mark the current vertex as visited. A visited vertex will never be checked again.
            visitedVertices.append(currentVertex)

            #Select the unvisited vertex that is marked with the smallest tentative distance, 
            #set it as the new "current vertex", and loop.   
            smallestTentativeDistance = inf
            for i in range(self.vertexCount):
                if i not in visitedVertices:
                    if tentativeDistances[i] < smallestTentativeDistance:
                        smallestTentativeDistance = tentativeDistances[i]
                        currentVertex = i

        print("Shortest distances to each vertex from source vertex %i" % (sourceVertexId))
        print(tentativeDistances)
        print()
        print(paths)

if __name__ == '__main__':
    #start vertex IDs at 0!
    vertices = []
    vertices.append(Vertex(0, 'Tree'))
    vertices.append(Vertex(1, 'Rock'))
    vertices.append(Vertex(2, 'River'))
    vertices.append(Vertex(3, 'Home'))
    vertices.append(Vertex(4, 'Cliff'))
    vertices.append(Vertex(5, 'Hill'))

    edges = []
    edges.append(Edge(0, 1, 3))
    edges.append(Edge(1, 2, 7))
    edges.append(Edge(1, 3, 6))
    edges.append(Edge(0, 3, 2))
    edges.append(Edge(2, 3, 5))
    edges.append(Edge(3, 4, 4))
    edges.append(Edge(2, 5, 6))
    edges.append(Edge(3, 5, 1))

    g = Graph(vertices, edges)

    print("Adjacency Matrix:")
    print(g.adjacencyMatrix, '\n')

    sourceVertexId = 0
    g.dijkstra(sourceVertexId)
