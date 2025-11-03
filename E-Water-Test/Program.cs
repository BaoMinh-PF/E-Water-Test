using E_Water_Test;
using Microsoft.Data.SqlClient;
using System.Net.Http.Headers;
using static E_Water_Test.PointModel;


class Program
{
    // Placeholder for your DB connection string
    private static string _connectionString = "Server=.;Database=EmergencyWaterPOC;ConnectRetryCount=0;Trusted_Connection=True;MultipleActiveResultSets=true;TrustServerCertificate=True";
    private static string _bearerToken = "";
    private static string _eskilstunaMunicipalGeoJsonFilePath = @"D:\Emergency Water Distribution\JSON\Eskilstuna_Municipal_Map.geojson";
    private static string _eskilstunaRoadsGeoJsonFilePath = @"D:\Emergency Water Distribution\JSON\Road_Network_Test_1.geojson";

    static async Task Main(string[] args)
    {
        var route = new Route();
        var point = new Point();
        var road = new Road();
        var municipal = new Municipal();
        var route2 = new Route2();
        var municipalData = await municipal.GetMunicipalData("0484");
        var nodes = await route.GetAllNodes();
        var edges = await route.GetAllEdges();
        var start = await point.GetPoint<SupplyPointDbModel>(Point.spTableName, 2);
        var end = await point.GetPoint<DistributionPointDbModel>(Point.dpTableName, 5);
        var data = await route.CalculateTopKRoutesBetweenPoints(start, end, nodes, edges, 20);

        var clusteredRoutes = route.ClusterRoutes(data);

        foreach (var cluster in clusteredRoutes)
        {
            await route.CreateRoute(start, end, cluster[0]);
        }
    }

}
