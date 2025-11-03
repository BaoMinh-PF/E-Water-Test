using NetTopologySuite.Geometries;

namespace E_Water_Test;
public class MunicipalModel
{
    public class MunicipalGeoJsonModel
    {
        public string CountyCode { get; set; }
        public string MunicipalCode { get; set; }
        public string MunicipalName { get; set; }
        public Geometry Geometry { get; set; }
    }

    public Dictionary<string, string> MunicipalGeoJsonPropertyMapper =new Dictionary<string, string>
    {
        { "lanskod", "CountyCode" },
        { "kommunkod", "MunicipalCode" },
        { "namnkortform", "MunicipalName" }
    };

    public class MunicipalDbModel
    {
        public int ID { get; set; }
        public string MunicipalCode { get; set; }
        public string MunicipalName { get; set; }
        public Geometry Coordinate { get; set; }
    }
}
