using Microsoft.Data.SqlClient;
using NetTopologySuite;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using NetTopologySuite.LinearReferencing;
using static E_Water_Test.PointModel;
using static E_Water_Test.RoadModel;
using static E_Water_Test.RouteModel;
using static System.Runtime.InteropServices.JavaScript.JSType;

namespace E_Water_Test;
public class Route
{
    private static string _connectionString = "Server=.;Database=EmergencyWaterPOC;ConnectRetryCount=0;Trusted_Connection=True;MultipleActiveResultSets=true;TrustServerCertificate=True";

    public async Task CreateRoute(SupplyPointDbModel spData, DistributionPointDbModel dpData, List<RouteStep> roadData)
    {
        Geometry mergedGeometry = null;
        foreach (var r in roadData)
        {
            if (mergedGeometry == null)
                mergedGeometry = r.Edge.Coordinate;
            else
                mergedGeometry = mergedGeometry.Union(r.Edge.Coordinate);
        }

        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();
        using var cmd = conn.CreateCommand();

        var wktWriter = new WKTWriter();
        string wkt = wktWriter.Write(mergedGeometry);

        cmd.CommandText = @"
        INSERT INTO Route (
            SupplyPointID,
            DestinationPointID,
            Roads,
            Distance,
            Coordinate
        )
        VALUES (
            @spId,
            @dpId,
            @roads,
            @distance,
            geometry::STGeomFromText(@wkt, 3006)
        );";
        cmd.Parameters.AddWithValue("@spId", spData.ID);
        cmd.Parameters.AddWithValue("@dpId", dpData.ID);
        cmd.Parameters.AddWithValue("@roads", String.Join(", ", roadData.Select(r => r.Edge.ID).Distinct()));
        cmd.Parameters.AddWithValue("@distance", roadData.Sum(r => r.Distance));
        cmd.Parameters.AddWithValue("@wkt", wkt);

        await cmd.ExecuteNonQueryAsync();
    }

    public async Task<NetTopologySuite.Geometries.Point> GetIntersectTwoRoads(RoadDbModel roadA, RoadDbModel roadB)
    {
        var geometry = roadA.Coordinate.Intersection(roadB.Coordinate);
        var geometryFactory = NtsGeometryServices.Instance.CreateGeometryFactory(srid: 3006);

        if (geometry.IsEmpty)
            return null;

        if (geometry is NetTopologySuite.Geometries.Point point)
            return point;


        if (geometry is MultiPoint multiPoint && multiPoint.NumGeometries > 0)
            return (NetTopologySuite.Geometries.Point)multiPoint.GetGeometryN(0);

        return null;
    }

    public async Task<LineString> GetRemainRoadFromPoint(NetTopologySuite.Geometries.Point startPoint, NetTopologySuite.Geometries.Point endPoint, RoadDbModel road)
    {
        var line = road.Coordinate as LineString;
        if (line == null)
            return null;

        var indexedLine = new LocationIndexedLine(line);
        var startIndex = indexedLine.Project(startPoint.Coordinate);
        var endIndex = indexedLine.Project(endPoint.Coordinate);

        if (startIndex.CompareTo(endIndex) > 0)
        {
            var temp = startIndex;
            startIndex = endIndex;
            endIndex = temp;
        }

        // Extract the sub-line between the two indices
        var subLine = indexedLine.ExtractLine(startIndex, endIndex) as LineString;
        return subLine;

    }

    public async Task<List<PopulatedRoadModel>> PopulateRoadModels(List<RoadDbModel> roads)
    {
        var geometryFactory = NtsGeometryServices.Instance.CreateGeometryFactory(srid: 4326);
        var populatedRoads = new List<PopulatedRoadModel>();

        // Find all intersection points between roads
        var intersectionPoints = new List<NetTopologySuite.Geometries.Point>();
        var spatialIndex = new NetTopologySuite.Index.Strtree.STRtree<int>();
        for (int i = 0; i < roads.Count; i++)
        {
            spatialIndex.Insert(roads[i].Coordinate.EnvelopeInternal, i);
        }

        // Find intersections using spatial index
        var processedPairs = new HashSet<(int, int)>();
        for (int i = 0; i < roads.Count; i++)
        {
            var roadA = roads[i];
            var envelope = roadA.Coordinate.EnvelopeInternal;

            // Query spatial index for nearby roads
            var nearbyIndices = spatialIndex.Query(envelope).Cast<int>();

            foreach (var j in nearbyIndices)
            {
                if (i != j && i < j && !processedPairs.Contains((i, j)))
                {
                    processedPairs.Add((i, j));

                    var intersection = await GetIntersectTwoRoads(roadA, roads[j]);
                    if (intersection != null)
                    {
                        intersectionPoints.Add(intersection);
                    }
                }
            }
        }

        foreach (var road in roads)
        {
            LineString line = road.Coordinate as LineString;
            var indexedLine = new LocationIndexedLine(line);

            // Find intersection points on this specific road
            var splitPoints = new List<NetTopologySuite.Geometries.Point> { line.StartPoint, line.EndPoint };

            foreach (var intersection in intersectionPoints)
            {
                if (line.Distance(intersection) < 0.01) // 1cm tolerance
                {
                    splitPoints.Add(intersection);
                }
            }

            // Sort points by their position along the road
            splitPoints = splitPoints.OrderBy(p =>
            {
                var location = indexedLine.Project(p.Coordinate);
                return location.SegmentIndex + location.SegmentFraction;
            }).ToList();

            // Create segments between consecutive points
            for (int i = 0; i < splitPoints.Count - 1; i++)
            {
                var startPoint = splitPoints[i];
                var endPoint = splitPoints[i + 1];

                var segment = GetRemainRoadFromPoint(startPoint, endPoint, road).Result;
                if (segment != null && segment.Length > 0.01) // Skip very small segments
                {
                    var startNodeGeometry = geometryFactory.CreatePoint(segment.StartPoint.Coordinate);
                    var endNodeGeometry = geometryFactory.CreatePoint(segment.EndPoint.Coordinate);

                    var populatedRoad = new PopulatedRoadModel
                    {
                        StartNode = new NodeDbModel
                        {
                            Coordinate = startNodeGeometry
                        },
                        EndNode = new NodeDbModel
                        {
                            Coordinate = endNodeGeometry
                        },
                        Edge = new EdgeDbModel
                        {
                            RoadID = road.RoadID,
                            Length = (float)segment.Length,
                            SpeedLimit = road.SpeedLimit,
                            BearingClass = road.BearingClass,
                            Type = road.Type,
                            Coordinate = segment
                        }
                    };

                    populatedRoads.Add(populatedRoad);
                }
            }
        }

        return populatedRoads;
    }

    public async Task CleanDuplicateNodesAndUpdateEdges()
    {
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        // Step 1: Find duplicate nodes by geometry (WKT)
        var nodeWktToIds = new Dictionary<string, List<int>>();
        using (var cmd = conn.CreateCommand())
        {
            cmd.CommandText = "SELECT ID, Coordinate.STAsText() AS Wkt FROM Node";
            using var reader = await cmd.ExecuteReaderAsync();
            while (await reader.ReadAsync())
            {
                int id = reader.GetInt32(reader.GetOrdinal("ID"));
                string wkt = reader.GetString(reader.GetOrdinal("Wkt"));
                if (!nodeWktToIds.ContainsKey(wkt))
                    nodeWktToIds[wkt] = new List<int>();
                nodeWktToIds[wkt].Add(id);
            }
        }

        // Step 2: For each duplicate, update Edge references and delete extra nodes
        foreach (var kvp in nodeWktToIds.Where(x => x.Value.Count > 1))
        {
            var ids = kvp.Value;
            int keepId = ids[0];
            var removeIds = ids.Skip(1).ToList();

            foreach (var removeId in removeIds)
            {
                // Update Edge table: replace FromNodeID and ToNodeID
                using var updateCmd = conn.CreateCommand();
                updateCmd.CommandText = @"
                UPDATE Edge SET FromNodeID = @keepId WHERE FromNodeID = @removeId;
                UPDATE Edge SET ToNodeID = @keepId WHERE ToNodeID = @removeId;";
                updateCmd.Parameters.AddWithValue("@keepId", keepId);
                updateCmd.Parameters.AddWithValue("@removeId", removeId);
                await updateCmd.ExecuteNonQueryAsync();
            }

            // Delete duplicate nodes
            using var deleteCmd = conn.CreateCommand();
            deleteCmd.CommandText = $"DELETE FROM Node WHERE ID IN ({string.Join(",", removeIds)})";
            await deleteCmd.ExecuteNonQueryAsync();
        }
    }


    public async Task AddPopulatedRoadsToDb(List<PopulatedRoadModel> populatedRoads)
    {
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();
        var wktWriter = new WKTWriter();

        foreach (var populatedRoad in populatedRoads)
        {
            // Insert start node and get its ID
            int startNodeId = await InsertNodeAndGetId(conn, populatedRoad.StartNode, wktWriter);

            // Insert end node and get its ID
            int endNodeId = await InsertNodeAndGetId(conn, populatedRoad.EndNode, wktWriter);

            // Set the node IDs for the edge
            populatedRoad.Edge.FromNodeID = startNodeId;
            populatedRoad.Edge.ToNodeID = endNodeId;

            // Insert the edge
            await InsertEdge(conn, populatedRoad.Edge, wktWriter);
        }
    }

    private async Task<int> InsertNodeAndGetId(SqlConnection conn, NodeDbModel node, WKTWriter wktWriter)
    {
        using var cmd = conn.CreateCommand();

        string nodeWkt = wktWriter.Write(node.Coordinate);

        cmd.CommandText = @"
        INSERT INTO Node (Coordinate)
        OUTPUT INSERTED.ID
        VALUES (geometry::STGeomFromText(@wkt, 3006));";
        cmd.Parameters.AddWithValue("@wkt", nodeWkt);

        var result = await cmd.ExecuteScalarAsync();
        return (int)result;
    }

    private async Task InsertEdge(SqlConnection conn, EdgeDbModel edge, WKTWriter wktWriter)
    {
        using var cmd = conn.CreateCommand();

        string edgeWkt = wktWriter.Write(edge.Coordinate);

        cmd.CommandText = @"
        INSERT INTO Edge (RoadID, FromNodeID, ToNodeID, Length, SpeedLimit, BearingClass, Type, Coordinate)
        VALUES (@roadId, @fromNodeId, @toNodeId, @length, @speedLimit, @bearingClass, @type, geometry::STGeomFromText(@wkt, 3006));";
        cmd.Parameters.AddWithValue("@roadId", edge.RoadID);
        cmd.Parameters.AddWithValue("@fromNodeId", edge.FromNodeID);
        cmd.Parameters.AddWithValue("@toNodeId", edge.ToNodeID);
        cmd.Parameters.AddWithValue("@length", edge.Length);
        cmd.Parameters.AddWithValue("@speedLimit", edge.SpeedLimit ?? (object)DBNull.Value);
        cmd.Parameters.AddWithValue("@bearingClass", edge.BearingClass ?? (object)DBNull.Value);
        cmd.Parameters.AddWithValue("@type", edge.Type ?? (object)DBNull.Value);
        cmd.Parameters.AddWithValue("@wkt", edgeWkt);

        await cmd.ExecuteNonQueryAsync();
    }

    public async Task<List<NodeDbModel>> GetAllNodes()
    {
        var nodes = new List<NodeDbModel>();
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        using var cmd = conn.CreateCommand();
        cmd.CommandText = @"SELECT 
                                ID, 
                                Coordinate.STAsText() AS Wkt
                            FROM Node";

        using var reader = await cmd.ExecuteReaderAsync();
        var wktReader = new WKTReader();

        while (await reader.ReadAsync())
        {
            var node = new NodeDbModel
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                Coordinate = wktReader.Read(reader.GetString(reader.GetOrdinal("Wkt")))
            };
            nodes.Add(node);
        }

        return nodes;
    }

    public async Task<List<EdgeDbModel>> GetAllEdges()
    {
        var edges = new List<EdgeDbModel>();
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        using var cmd = conn.CreateCommand();
        cmd.CommandText = @"SELECT 
                                ID, 
                                RoadID, 
                                FromNodeID, 
                                ToNodeID, 
                                Length, 
                                SpeedLimit, 
                                BearingClass, 
                                Type, 
                                Coordinate.STAsText() AS Wkt
                            FROM Edge";

        using var reader = await cmd.ExecuteReaderAsync();
        var wktReader = new WKTReader();

        while (await reader.ReadAsync())
        {
            var edge = new EdgeDbModel
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                RoadID = reader.GetString(reader.GetOrdinal("RoadID")),
                FromNodeID = reader.GetInt32(reader.GetOrdinal("FromNodeID")),
                ToNodeID = reader.GetInt32(reader.GetOrdinal("ToNodeID")),
                Length = (float)reader.GetDouble(reader.GetOrdinal("Length")),
                SpeedLimit = reader.IsDBNull(reader.GetOrdinal("SpeedLimit")) ? null : reader.GetString(reader.GetOrdinal("SpeedLimit")),
                BearingClass = reader.IsDBNull(reader.GetOrdinal("BearingClass")) ? null : reader.GetString(reader.GetOrdinal("BearingClass")),
                Type = reader.IsDBNull(reader.GetOrdinal("Type")) ? null : reader.GetString(reader.GetOrdinal("Type")),
                Coordinate = wktReader.Read(reader.GetString(reader.GetOrdinal("Wkt")))
            };
            edges.Add(edge);
        }

        return edges;
    }


    public async Task<List<RouteStep>> CalculateRouteBetweenPoints(
                                        SupplyPointDbModel startPoint,
                                        DistributionPointDbModel endPoint,
                                        List<NodeDbModel> nodes,
                                        List<EdgeDbModel> edges)
    {
        // Find nearest nodes to start and end points
        var startNode = FindNearestNode(startPoint.Coordinate, nodes);
        var endNode = FindNearestNode(endPoint.Coordinate, nodes);

        if (startNode == null || endNode == null)
            return new List<RouteStep>();

        // Build adjacency list for graph traversal
        var adjacencyList = new Dictionary<int, List<AdjacentNode>>();

        foreach (var edge in edges)
        {
            if (!adjacencyList.ContainsKey(edge.FromNodeID))
                adjacencyList[edge.FromNodeID] = new List<AdjacentNode>();

            if (!adjacencyList.ContainsKey(edge.ToNodeID))
                adjacencyList[edge.ToNodeID] = new List<AdjacentNode>();

            // Add bidirectional edges (roads can be traveled in both directions)
            adjacencyList[edge.FromNodeID].Add(new AdjacentNode() { NodeId = edge.ToNodeID, Edge = edge, Weight = edge.Length });
            adjacencyList[edge.ToNodeID].Add(new AdjacentNode() { NodeId = edge.FromNodeID, Edge = edge, Weight = edge.Length });
        }

        return FindShortestPath(startNode.ID, endNode.ID, adjacencyList, nodes);
    }

    private NodeDbModel FindNearestNode(Geometry point, List<NodeDbModel> nodes)
    {
        NodeDbModel nearestNode = null;
        double minDistance = double.MaxValue;

        foreach (var node in nodes)
        {
            double distance = point.Distance(node.Coordinate);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearestNode = node;
            }
        }

        return nearestNode;
    }

    private List<RouteStep> FindShortestPath(int startNodeId, int endNodeId,
        Dictionary<int, List<AdjacentNode>> adjacencyList,
        List<NodeDbModel> nodes)
    {
        var distances = new Dictionary<int, double>();
        var previous = new Dictionary<int, (int nodeId, RouteStep step)>();
        var visited = new HashSet<int>();
        var priorityQueue = new SortedSet<(double distance, int nodeId)>();

        // Initialize distances
        foreach (var node in nodes)
        {
            distances[node.ID] = double.MaxValue;
        }
        distances[startNodeId] = 0;
        priorityQueue.Add((0, startNodeId));

        while (priorityQueue.Count > 0)
        {
            var (currentDistance, currentNodeId) = priorityQueue.Min;
            priorityQueue.Remove(priorityQueue.Min);

            if (visited.Contains(currentNodeId))
                continue;

            visited.Add(currentNodeId);

            if (currentNodeId == endNodeId)
                break;

            if (!adjacencyList.ContainsKey(currentNodeId))
                continue;

            foreach (var adjacent in adjacencyList[currentNodeId])
            {
                if (visited.Contains(adjacent.NodeId))
                    continue;

                var newDistance = currentDistance + adjacent.Weight;

                if (newDistance < distances[adjacent.NodeId])
                {
                    distances[adjacent.NodeId] = newDistance;
                    previous[adjacent.NodeId] = (currentNodeId, new RouteStep { Edge = adjacent.Edge, Distance = adjacent.Weight });
                    priorityQueue.Add((newDistance, adjacent.NodeId));
                }
            }
        }

        // Reconstruct path, ensuring no cycles
        var path = new List<RouteStep>();
        var current = endNodeId;
        while (previous.ContainsKey(current))
        {
            var (previousNodeId, step) = previous[current];
            path.Add(step);
            current = previousNodeId;
        }

        path.Reverse();
        return path;
    }

    public List<RouteStep> FindAStarPath(SupplyPointDbModel startPoint,
                                         DistributionPointDbModel endPoint,
                                         List<EdgeDbModel> edges,
                                         List<NodeDbModel> nodes
         )
    {
        var adjacencyList = new Dictionary<int, List<AdjacentNode>>();
        var startNode = FindNearestNode(startPoint.Coordinate, nodes);
        var endNode = FindNearestNode(endPoint.Coordinate, nodes);

        foreach (var edge in edges)
        {
            if (!adjacencyList.ContainsKey(edge.FromNodeID))
                adjacencyList[edge.FromNodeID] = new List<AdjacentNode>();

            if (!adjacencyList.ContainsKey(edge.ToNodeID))
                adjacencyList[edge.ToNodeID] = new List<AdjacentNode>();

            // Add bidirectional edges (roads can be traveled in both directions)
            adjacencyList[edge.FromNodeID].Add(new AdjacentNode() { NodeId = edge.ToNodeID, Edge = edge, Weight = edge.Length });
            adjacencyList[edge.ToNodeID].Add(new AdjacentNode() { NodeId = edge.FromNodeID, Edge = edge, Weight = edge.Length });
        }

        var nodeDict = nodes.ToDictionary(n => n.ID, n => n);

        var openSet = new SortedSet<(double fScore, int nodeId)>();
        var gScore = new Dictionary<int, double>();
        var fScore = new Dictionary<int, double>();
        var cameFrom = new Dictionary<int, (int nodeId, RouteStep step)>();

        foreach (var node in nodes)
        {
            gScore[node.ID] = double.MaxValue;
            fScore[node.ID] = double.MaxValue;
        }
        var edgeForHeuristic = FindEdge(startNode.ID, endNode.ID, edges);
        gScore[startNode.ID] = 0;
        fScore[startNode.ID] = edgeForHeuristic != null ? Heuristic(nodeDict[startNode.ID], nodeDict[endNode.ID], edgeForHeuristic!) : Heuristic(nodeDict[startNode.ID], nodeDict[endNode.ID]);
        openSet.Add((fScore[startNode.ID], startNode.ID));

        var closedSet = new HashSet<int>();

        while (openSet.Count > 0)
        {
            var (currentF, currentNodeId) = openSet.Min;
            openSet.Remove(openSet.Min);

            if (currentNodeId == endNode.ID)
                break;

            closedSet.Add(currentNodeId);

            if (!adjacencyList.ContainsKey(currentNodeId))
                continue;

            foreach (var adjacent in adjacencyList[currentNodeId])
            {
                if (closedSet.Contains(adjacent.NodeId))
                    continue;

                double tentativeG = gScore[currentNodeId] + adjacent.Weight;
                if (tentativeG < gScore[adjacent.NodeId])
                {
                    cameFrom[adjacent.NodeId] = (currentNodeId, new RouteStep { Edge = adjacent.Edge, Distance = adjacent.Weight });
                    gScore[adjacent.NodeId] = tentativeG;
                    fScore[adjacent.NodeId] = tentativeG + Heuristic(nodeDict[adjacent.NodeId], nodeDict[endNode.ID]);
                    openSet.Add((fScore[adjacent.NodeId], adjacent.NodeId));
                }
            }
        }

        // Reconstruct path
        var path = new List<RouteStep>();
        var current = endNode.ID;
        while (cameFrom.ContainsKey(current))
        {
            var (prevNodeId, step) = cameFrom[current];
            path.Add(step);
            current = prevNodeId;
        }
        path.Reverse();
        return path;
    }

    // Heuristic: Euclidean distance between nodes
    private double Heuristic(NodeDbModel a, NodeDbModel b)
    {
        return a.Coordinate.Distance(b.Coordinate);
    }

    private double Heuristic(NodeDbModel a, NodeDbModel b, EdgeDbModel edge)
    {
        double speedVal;
        if (!double.TryParse(edge.SpeedLimit, out speedVal))
        {
            speedVal = 80;
        }
        return edge.Length + speedVal;
    }

    private EdgeDbModel? FindEdge(int fromNodeId, int toNodeId, List<EdgeDbModel> edges)
    {
        return edges.FirstOrDefault(e => e.FromNodeID == fromNodeId && e.ToNodeID == toNodeId);
    }

    public async Task<List<List<RouteStep>>> CalculateTopKRoutesBetweenPoints(
                                                SupplyPointDbModel startPoint,
                                                DistributionPointDbModel endPoint,
                                                List<NodeDbModel> nodes,
                                                List<EdgeDbModel> edges,
                                                int k)
    {
        var startNode = FindNearestNode(startPoint.Coordinate, nodes);
        var endNode = FindNearestNode(endPoint.Coordinate, nodes);

        if (startNode == null || endNode == null)
            return new List<List<RouteStep>>();

        var adjacencyList = new Dictionary<int, List<AdjacentNode>>();
        foreach (var edge in edges)
        {
            if (!adjacencyList.ContainsKey(edge.FromNodeID))
                adjacencyList[edge.FromNodeID] = new List<AdjacentNode>();
            if (!adjacencyList.ContainsKey(edge.ToNodeID))
                adjacencyList[edge.ToNodeID] = new List<AdjacentNode>();

            adjacencyList[edge.FromNodeID].Add(new AdjacentNode() { NodeId = edge.ToNodeID, Edge = edge, Weight = edge.Length });
            adjacencyList[edge.ToNodeID].Add(new AdjacentNode() { NodeId = edge.FromNodeID, Edge = edge, Weight = edge.Length });
        }

        // Helper to calculate path distance
        double PathDistance(List<RouteStep> path) => path.Sum(s => s.Distance);

        // Store unique paths (by edge sequence)
        var pathSet = new HashSet<string>();
        var pathList = new List<List<RouteStep>>();

        // Find initial shortest path
        var shortestPath = FindShortestPath(startNode.ID, endNode.ID, adjacencyList, nodes);
        if (shortestPath.Count == 0)
            return pathList;

        string PathKey(List<RouteStep> path) => string.Join("-", path.Select(s => s.Edge.ID));

        pathSet.Add(PathKey(shortestPath));
        pathList.Add(shortestPath);

        // Try to find up to 20 unique paths
        int maxPaths = 20;
        int idx = 0;
        while (pathList.Count < maxPaths && idx < pathList.Count)
        {
            var currentPath = pathList[idx];
            for (int i = 0; i < currentPath.Count; i++)
            {
                // Remove one edge at a time
                var edgeToRemove = currentPath[i].Edge;
                var filteredEdges = edges.Where(e => e.ID != edgeToRemove.ID).ToList();

                // Build new adjacency list
                var newAdjacencyList = new Dictionary<int, List<AdjacentNode>>();
                foreach (var edge in filteredEdges)
                {
                    if (!newAdjacencyList.ContainsKey(edge.FromNodeID))
                        newAdjacencyList[edge.FromNodeID] = new List<AdjacentNode>();
                    if (!newAdjacencyList.ContainsKey(edge.ToNodeID))
                        newAdjacencyList[edge.ToNodeID] = new List<AdjacentNode>();

                    newAdjacencyList[edge.FromNodeID].Add(new AdjacentNode() { NodeId = edge.ToNodeID, Edge = edge, Weight = edge.Length });
                    newAdjacencyList[edge.ToNodeID].Add(new AdjacentNode() { NodeId = edge.FromNodeID, Edge = edge, Weight = edge.Length });
                }

                var newPath = FindShortestPath(startNode.ID, endNode.ID, newAdjacencyList, nodes);
                var newKey = PathKey(newPath);

                if (newPath.Count > 0 && !pathSet.Contains(newKey))
                {
                    pathSet.Add(newKey);
                    pathList.Add(newPath);
                    if (pathList.Count >= maxPaths)
                        break;
                }
            }
            idx++;
        }

        // Sort by total distance and return top k
        var sortedPaths = pathList.Take(k).ToList();
        return sortedPaths;
    }

    public List<List<List<RouteStep>>> ClusterRoutes(List<List<RouteStep>> data)
    {
        var routeEdgeSets = data
                            .Select(routeSteps => routeSteps.Select(rs => rs.Edge.ID).ToHashSet())
                            .ToList();

        int n = routeEdgeSets.Count;
        double[,] distanceMatrix = new double[n, n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                distanceMatrix[i, j] = JaccardDistance(routeEdgeSets[i], routeEdgeSets[j]);

        // Each cluster is a list of route indices
        List<List<int>> clusters = routeEdgeSets.Select((_, idx) => new List<int> { idx }).ToList();

        while (clusters.Count > 3)
        {
            // Find closest pair of clusters
            double minDist = double.MaxValue;
            int minI = -1, minJ = -1;
            for (int i = 0; i < clusters.Count; i++)
            {
                for (int j = i + 1; j < clusters.Count; j++)
                {
                    // Average linkage
                    double dist = clusters[i].SelectMany(a => clusters[j], (a, b) => distanceMatrix[a, b]).Average();
                    if (dist < minDist)
                    {
                        minDist = dist;
                        minI = i;
                        minJ = j;
                    }
                }
            }
            // Merge clusters
            clusters[minI].AddRange(clusters[minJ]);
            clusters.RemoveAt(minJ);
        }

        return clusters.Select(cluster => cluster.Select(idx => data[idx]).ToList()).ToList();
    }

    public double JaccardDistance(HashSet<int> a, HashSet<int> b)
    {
        var intersection = a.Intersect(b).Count();
        var union = a.Union(b).Count();
        return 1.0 - (double)intersection / union;
    }
}