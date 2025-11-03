using Microsoft.Data.SqlClient;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;

namespace E_Water_Test;

public class Route2
{
    private static string _connectionString = "Server=.;Database=EmergencyWaterPOC;ConnectRetryCount=0;Trusted_Connection=True;MultipleActiveResultSets=true;TrustServerCertificate=True";

    public class NodeDbModel
    {
        public int ID { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
    }

    public class EdgeDbModel
    {
        public int ID { get; set; }
        public string RoadID { get; set; }
        public int FromNodeID { get; set; }
        public int ToNodeID { get; set; }
        public float Length { get; set; }
        public Geometry Geometry { get; set; }
    }

    public async Task<List<NodeDbModel>> GetAllNodes()
    {
        var nodes = new List<NodeDbModel>();
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        using var cmd = conn.CreateCommand();
        cmd.CommandText = @"SELECT 
                                ID, 
                                Coordinate.STTransform(4326).STX AS X,
                                Coordinate.STTransform(4326).STY AS Y
                            FROM Node";

        using var reader = await cmd.ExecuteReaderAsync();

        while (await reader.ReadAsync())
        {
            var node = new NodeDbModel
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                X = reader.GetDouble(reader.GetOrdinal("X")),
                Y = reader.GetDouble(reader.GetOrdinal("Y"))
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
                                Coordinate.STTransform(4326).STAsText() AS Wkt
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
                Geometry = wktReader.Read(reader.GetString(reader.GetOrdinal("Wkt")))
            };
            edges.Add(edge);
        }

        return edges;
    }

    public async Task<Geometry> GeometryTransforer(Geometry geometrySweref)
    {
        var sourceProjection = DotSpatial.Projections.ProjectionInfo.FromEpsgCode(3006); // SWEREF99 TM
        var targetProjection = DotSpatial.Projections.ProjectionInfo.FromEpsgCode(4326); // WGS84

        var coordinates = geometrySweref.Coordinates;
        var transformedCoords = new List<Coordinate>();

        foreach (var coord in coordinates)
        {
            double[] xy = { coord.X, coord.Y };
            double[] z = { coord.Z };

            DotSpatial.Projections.Reproject.ReprojectPoints(xy, z, sourceProjection, targetProjection, 0, 1);

            transformedCoords.Add(new Coordinate(xy[0], xy[1]));
        }

        var factory = new GeometryFactory(new PrecisionModel(), 4326);

        return geometrySweref switch
        {
            NetTopologySuite.Geometries.Point => factory.CreatePoint(transformedCoords[0]),
            LineString => factory.CreateLineString(transformedCoords.ToArray()),
            Polygon polygon => factory.CreatePolygon(transformedCoords.ToArray()),
            _ => geometrySweref
        };
    }

}
