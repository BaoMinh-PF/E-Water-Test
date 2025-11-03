using Microsoft.Data.SqlClient;
using NetTopologySuite.Features;
using NetTopologySuite.IO;
using System.Text.Json.Serialization;
using System.Text.Json;
using static E_Water_Test.RoadModel;

namespace E_Water_Test;

public class Road
{
    private static string _connectionString = "Server=.;Database=EmergencyWaterPOC;ConnectRetryCount=0;Trusted_Connection=True;MultipleActiveResultSets=true;TrustServerCertificate=True";

    public async Task<List<RoadGeoJsonModel>> ReadRoadGeoJsonFile(string filePath)
    {
        string geoJsonContent = await File.ReadAllTextAsync(filePath);
        var reader = new GeoJsonReader();
        FeatureCollection featureCollection = reader.Read<FeatureCollection>(geoJsonContent);
        var models = new List<RoadGeoJsonModel>();
        var mapper = (new RoadModel()).RoadGeoJsonPropertyMapper;

        foreach (var feature in featureCollection)
        {
            var model = new RoadGeoJsonModel();
            model.Geometry = feature.Geometry;
            foreach (var prop in mapper)
            {
                if (feature.Attributes.Exists(prop.Key))
                {
                    if (prop.Key == "TYPE" && feature.Attributes[prop.Key] != null)
                    {
                        var jsonValue = feature.Attributes[prop.Key].ToString();
                        if (jsonValue != null && !(new RoadModel()).TrafficTypeMapper.ContainsKey(jsonValue))
                            continue; // Skip this feature if TYPE is not in the mapper

                        var mappedValue = (new RoadModel()).TrafficTypeMapper[jsonValue!];
                        typeof(RoadGeoJsonModel).GetProperty(prop.Value)?.SetValue(model, mappedValue);
                        continue;
                    }

                    var value = feature.Attributes[prop.Key]?.ToString();
                    if (prop.Value == nameof(RoadGeoJsonModel.Length))
                    {
                        if (float.TryParse(value, out float floatValue))
                        {
                            typeof(RoadGeoJsonModel).GetProperty(prop.Value)?.SetValue(model, floatValue);
                        }
                        continue;
                    }
                    else
                    {
                        typeof(RoadGeoJsonModel).GetProperty(prop.Value)?.SetValue(model, value);
                    }
                }
            }
            models.Add(model);
        }

        return models;
    }

    public async Task HandleUpsertRoadGeometry(List<RoadGeoJsonModel> models)
    {
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        foreach (var model in models)
        {
            using var cmd = conn.CreateCommand();
            var wktWriter = new WKTWriter();
            string wkt = wktWriter.Write(model.Geometry);
            cmd.CommandText = @"INSERT INTO Road (
                                    RoadID,
                                    Type,
                                    Length,
                                    SpeedLimit,
                                    BearingClass,
                                    Coordinate
                                )
                                VALUES (
                                    @roadId,
                                    @type,
                                    @length,
                                    @speedLimit,
                                    @bearingClass,
                                    geometry::STGeomFromText(@wkt, 3006)
                                );";
            cmd.Parameters.AddWithValue("@wkt", wkt);
            cmd.Parameters.AddWithValue("@roadId", model.ElementID);
            cmd.Parameters.AddWithValue("@type", model.Type);
            cmd.Parameters.AddWithValue("@length", model.Length);
            cmd.Parameters.AddWithValue("@bearingClass", model.BearingClass ?? (object)DBNull.Value);
            cmd.Parameters.AddWithValue("@speedLimit", model.SpeedLimit ?? (object)DBNull.Value);

            await cmd.ExecuteNonQueryAsync();
        }
    }

    public async Task<List<RoadDbModel>> ReadRoads()
    {
        var roads = new List<RoadDbModel>();
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        using var cmd = conn.CreateCommand();
        cmd.CommandText = @"SELECT 
                            ID, 
                            RoadID, 
                            Type, 
                            Length, 
                            SpeedLimit, 
                            BearingClass, 
                            Coordinate.STAsText() AS Wkt
                        FROM Road";

        using var reader = await cmd.ExecuteReaderAsync();
        var wktReader = new WKTReader();

        while (await reader.ReadAsync())
        {
            var road = new RoadDbModel
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                RoadID = reader.GetString(reader.GetOrdinal("RoadID")),
                Type = reader.GetString(reader.GetOrdinal("Type")),
                Length = (float)reader.GetDouble(reader.GetOrdinal("Length")),
                SpeedLimit = reader.IsDBNull(reader.GetOrdinal("SpeedLimit")) ? null : reader.GetString(reader.GetOrdinal("SpeedLimit")),
                BearingClass = reader.IsDBNull(reader.GetOrdinal("BearingClass")) ? null : reader.GetString(reader.GetOrdinal("BearingClass")),
                Coordinate = wktReader.Read(reader.GetString(reader.GetOrdinal("Wkt")))
            };
            roads.Add(road);
        }

        return roads;
    }

    public string ConvertRoadsToGeoJsonNetTopologySuite(IEnumerable<RoadDbModel> roads)
    {
        var featureCollection = new FeatureCollection();

        foreach (var road in roads)
        {
            var attributes = new AttributesTable
        {
            { "id", road.ID },
            { "roadId", road.RoadID },
            { "type", road.Type },
            { "length", road.Length },
            { "speedLimit", road.SpeedLimit },
            { "bearingClass", road.BearingClass }
        };

            var feature = new Feature(road.Coordinate, attributes);
            featureCollection.Add(feature);
        }

        var geoJsonWriter = new GeoJsonWriter();
        return geoJsonWriter.Write(featureCollection);
    }

    public async Task<List<RoadDbModel>> ReadRoads(List<string> roadIDs)
    {
        var roads = new List<RoadDbModel>();
        if (roadIDs == null || roadIDs.Count == 0)
            return roads;

        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        // Build parameterized IN clause
        var parameters = roadIDs.Select((id, idx) => $"@id{idx}").ToList();
        var inClause = string.Join(", ", parameters);

        using var cmd = conn.CreateCommand();
        cmd.CommandText = $@"
                            SELECT 
                                ID, 
                                RoadID, 
                                Type, 
                                Length, 
                                SpeedLimit, 
                                BearingClass, 
                                Coordinate.STAsText() AS Wkt
                            FROM Road
                            WHERE RoadID IN ({inClause})";

        for (int i = 0; i < roadIDs.Count; i++)
            cmd.Parameters.AddWithValue(parameters[i], roadIDs[i]);

        using var reader = await cmd.ExecuteReaderAsync();
        var wktReader = new WKTReader();

        while (await reader.ReadAsync())
        {
            var road = new RoadDbModel
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                RoadID = reader.GetString(reader.GetOrdinal("RoadID")),
                Type = reader.GetString(reader.GetOrdinal("Type")),
                Length = (float)reader.GetDouble(reader.GetOrdinal("Length")),
                SpeedLimit = reader.IsDBNull(reader.GetOrdinal("SpeedLimit")) ? null : reader.GetString(reader.GetOrdinal("SpeedLimit")),
                BearingClass = reader.IsDBNull(reader.GetOrdinal("BearingClass")) ? null : reader.GetString(reader.GetOrdinal("BearingClass")),
                Coordinate = wktReader.Read(reader.GetString(reader.GetOrdinal("Wkt")))
            };
            roads.Add(road);
        }

        return roads;
    }

    public async Task<List<RoadDbModel>> ReadRoads(List<int> IDs)
    {
        var roads = new List<RoadDbModel>();
        if (IDs == null || IDs.Count == 0)
            return roads;

        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();

        // Build parameterized IN clause
        var parameters = IDs.Select((id, idx) => $"@id{idx}").ToList();
        var inClause = string.Join(", ", parameters);

        using var cmd = conn.CreateCommand();
        cmd.CommandText = $@"
                            SELECT 
                                ID, 
                                RoadID, 
                                Type, 
                                Length, 
                                SpeedLimit, 
                                BearingClass, 
                                Coordinate.STAsText() AS Wkt
                            FROM Road
                            WHERE ID IN ({inClause})";

        for (int i = 0; i < IDs.Count; i++)
            cmd.Parameters.AddWithValue(parameters[i], IDs[i]);

        using var reader = await cmd.ExecuteReaderAsync();
        var wktReader = new WKTReader();

        while (await reader.ReadAsync())
        {
            var road = new RoadDbModel
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                RoadID = reader.GetString(reader.GetOrdinal("RoadID")),
                Type = reader.GetString(reader.GetOrdinal("Type")),
                Length = (float)reader.GetDouble(reader.GetOrdinal("Length")),
                SpeedLimit = reader.IsDBNull(reader.GetOrdinal("SpeedLimit")) ? null : reader.GetString(reader.GetOrdinal("SpeedLimit")),
                BearingClass = reader.IsDBNull(reader.GetOrdinal("BearingClass")) ? null : reader.GetString(reader.GetOrdinal("BearingClass")),
                Coordinate = wktReader.Read(reader.GetString(reader.GetOrdinal("Wkt")))
            };
            roads.Add(road);
        }

        return roads;
    }
}
