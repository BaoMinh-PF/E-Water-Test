using Microsoft.Data.SqlClient;
using NetTopologySuite.Features;
using NetTopologySuite.IO;
using static E_Water_Test.DistrictModel;
using static E_Water_Test.MunicipalModel;

namespace E_Water_Test;

public class Municipal
{
    private static string _connectionString = "Server=.;Database=EmergencyWaterPOC;ConnectRetryCount=0;Trusted_Connection=True;MultipleActiveResultSets=true;TrustServerCertificate=True";

    public async Task<List<MunicipalGeoJsonModel>> ReadMunicipalGeoJsonFile(string filePath)
    {
        string geoJsonContent = await File.ReadAllTextAsync(filePath);
        var reader = new GeoJsonReader();
        FeatureCollection featureCollection = reader.Read<FeatureCollection>(geoJsonContent);
        var models = new List<MunicipalGeoJsonModel>();
        var mapper = (new MunicipalModel()).MunicipalGeoJsonPropertyMapper;

        foreach (var feature in featureCollection)
        {
            var model = new MunicipalGeoJsonModel();
            model.Geometry = feature.Geometry;
            foreach (var prop in mapper)
            {
                if (feature.Attributes.Exists(prop.Key))
                {
                    var value = feature.Attributes[prop.Key]?.ToString();
                    typeof(MunicipalGeoJsonModel).GetProperty(prop.Value)?.SetValue(model, value);
                }
            }
            models.Add(model);
        }

        return models;
    }

    public async Task HandleUpsertMunicipalGeometry(List<MunicipalGeoJsonModel> models)
    {
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();
        using var cmd = conn.CreateCommand();

        foreach (var model in models)
        {
            var wktWriter = new WKTWriter();
            string wkt = wktWriter.Write(model.Geometry);
            cmd.CommandText = @"
                UPDATE Municipal
                SET Coordinate = geometry::STGeomFromText(@wkt, 0)
                WHERE Code = @municipalcode";
            cmd.Parameters.AddWithValue("@wkt", wkt);
            cmd.Parameters.AddWithValue("@municipalcode", model.MunicipalCode);

            await cmd.ExecuteNonQueryAsync();
        }
    }

    public async Task<MunicipalDbModel> GetMunicipalData(string municipalCode)
    {
        using var conn = new SqlConnection(_connectionString);
        await conn.OpenAsync();
        using var cmd = conn.CreateCommand();
        cmd.CommandText = "SELECT ID, Code, Name, Coordinate.STAsText() AS WKT FROM Municipal WHERE Code = @municipalcode";
        cmd.Parameters.AddWithValue("@municipalcode", municipalCode);
        using var reader = await cmd.ExecuteReaderAsync();
        if (await reader.ReadAsync())
        {
            var model = new MunicipalDbModel
            {
                ID = reader.GetInt32(reader.GetOrdinal("ID")),
                MunicipalCode = reader.GetString(reader.GetOrdinal("Code")),
                MunicipalName = reader.GetString(reader.GetOrdinal("Name"))

            };
            string wkt = reader.GetString(reader.GetOrdinal("WKT"));
            var wktReader = new WKTReader();
            model.Coordinate = wktReader.Read(wkt);
            return model;
        }
        return null;
    }

    public async Task ReadUpdateDistrictGeometry(string geoJsonFilePath)
    {
        string geoJsonContent = await File.ReadAllTextAsync(geoJsonFilePath);
        var reader = new GeoJsonReader();
        FeatureCollection featureCollection = reader.Read<FeatureCollection>(geoJsonContent);
        var models = new List<DistrictGeoJsonModel>();
        var mapper = (new DistrictModel()).DistrictGeoJsonPropertyMapper;

        foreach (var feature in featureCollection)
        {
            var model = new DistrictGeoJsonModel();
            model.Geometry = feature.Geometry;
            foreach (var prop in mapper)
            {
                if (feature.Attributes.Exists(prop.Key))
                {
                    var value = feature.Attributes[prop.Key]?.ToString();
                    typeof(DistrictGeoJsonModel).GetProperty(prop.Value)?.SetValue(model, value);
                }
            }
            models.Add(model);
        }

        foreach (var model in models)
        {
            using var conn = new SqlConnection(_connectionString);
            await conn.OpenAsync();
            using var cmd = conn.CreateCommand();
            var wktWriter = new WKTWriter();
            string wkt = wktWriter.Write(model.Geometry);
            cmd.CommandText = @"
                UPDATE District
                SET Coordinate = geometry::STGeomFromText(@wkt, 0)
                WHERE Code = @code";
            cmd.Parameters.AddWithValue("@wkt", wkt);
            cmd.Parameters.AddWithValue("@code", model.DistrictCode);

            await cmd.ExecuteNonQueryAsync();
            await conn.CloseAsync();
        }
    }
}

