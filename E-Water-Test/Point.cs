using Microsoft.Data.SqlClient;
using NetTopologySuite;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using NetTopologySuite.Operation.Distance;
using static E_Water_Test.MunicipalModel;
using static E_Water_Test.PointModel;
using static E_Water_Test.RoadModel;

namespace E_Water_Test;
public class Point
{
    private static string _connectionString = "Server=.;Database=EmergencyWaterPOC;ConnectRetryCount=0;Trusted_Connection=True;MultipleActiveResultSets=true;TrustServerCertificate=True";
    public static string dpTableName = "DistributionPoint";
    public static string spTableName = "SupplyPoint";

    public async Task CreatePoint(
                     MunicipalDbModel municipal,
                     double y, // Northing (first value, e.g. 6579702.8)
                     double x, // Easting (second value, e.g. 583762.5)
                     string name,
                     string address,
                     string tableName
                 )
    {
        var geometryFactory = GeometryFactory.Default;
        var point = geometryFactory.CreatePoint(new Coordinate(x, y)); // X=East, Y=North

        if (!municipal.Coordinate.Contains(point))
        {
            Console.WriteLine("Point is not inside the municipal geometry.");
            return;
        }

        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();
        using var cmd = conn.CreateCommand();

        var wktWriter = new WKTWriter();
        string wkt = wktWriter.Write(point);

        cmd.CommandText = $@"
                            INSERT INTO {tableName} (Name, Address, MunicipalID, Coordinate)
                            VALUES (@name, @address, @municipalId, geometry::STGeomFromText(@wkt, 3006))";
        cmd.Parameters.AddWithValue("@name", name);
        cmd.Parameters.AddWithValue("@address", address);
        cmd.Parameters.AddWithValue("@municipalId", municipal.ID);
        cmd.Parameters.AddWithValue("@wkt", wkt);

        await cmd.ExecuteNonQueryAsync();
    }

    public async Task<T> GetPoint<T>(string tableName, int id) where T : PointAbstract, new()
    {
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();
        using var cmd = conn.CreateCommand();
        cmd.CommandText = $@"
            SELECT TOP 1 
                ID, 
                Name, 
                MunicipalID, 
                Coordinate.STAsText() AS WKT, 
                EntranceRoadID, 
                EntranceRoadCoordinate.STAsText() AS EntranceWKT 
            FROM {tableName} Where ID = {id}";
        using var reader = await cmd.ExecuteReaderAsync();
        if (await reader.ReadAsync())
        {
            var model = new T()
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                Name = reader.GetString(reader.GetOrdinal("Name")),
                MunicipalID = reader.GetInt32(reader.GetOrdinal("MunicipalID"))
            };
            var wktReader = new WKTReader();
            string wkt = reader.GetString(reader.GetOrdinal("WKT"));
            model.Coordinate = wktReader.Read(wkt);

            var entranceRoadIdOrdinal = reader.GetOrdinal("EntranceRoadID");
            if (!reader.IsDBNull(entranceRoadIdOrdinal))
                model.EntranceRoadID = reader.GetString(entranceRoadIdOrdinal);

            var entranceWktOrdinal = reader.GetOrdinal("EntranceWKT");
            if (!reader.IsDBNull(entranceWktOrdinal))
                model.EntranceRoadCoordinate = wktReader.Read(reader.GetString(entranceWktOrdinal));

            return model;
        }
        return null;
    }


    public async Task<T> FindNearestRoadToPoint<T>(T point, List<RoadDbModel> roads) where T : PointAbstract, new()
    {

        var nearestRoad = roads.OrderBy(road => road.Coordinate.Distance(point.Coordinate))
                    .FirstOrDefault();
        point.EntranceRoadID = nearestRoad?.RoadID;
        var distOp = new DistanceOp(nearestRoad.Coordinate, point.Coordinate);
        var closestPoints = distOp.NearestPoints();
        var geometryFactory = NtsGeometryServices.Instance.CreateGeometryFactory(srid: 3006);
        point.EntranceRoadCoordinate = geometryFactory.CreatePoint(closestPoints[0]);
        return point;   
    }

    public async Task UpdatePointEntrance<T>(T point) where T : PointAbstract, new()
    {
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();
        using var cmd = conn.CreateCommand();
        var wktWriter = new WKTWriter();
        string entranceWkt = wktWriter.Write(point.EntranceRoadCoordinate);
        cmd.CommandText = $@"
                            UPDATE {(point is SupplyPointDbModel ? spTableName : dpTableName)}
                            SET EntranceRoadID = @entranceRoadId,
                                EntranceRoadCoordinate = geometry::STGeomFromText(@entranceWkt, 3006)
                            WHERE ID = @id";
        cmd.Parameters.AddWithValue("@entranceRoadId", point.EntranceRoadID);
        cmd.Parameters.AddWithValue("@entranceWkt", entranceWkt);
        cmd.Parameters.AddWithValue("@id", point.ID);
        await cmd.ExecuteNonQueryAsync();
    }
}

