using NetTopologySuite.Geometries;

namespace E_Water_Test;

public class DistrictModel
{
    public class DistrictGeoJsonModel
    {
        public string DistrictCode { get; set; }
        public Geometry Geometry { get; set; }
    }

    public Dictionary<string, string> DistrictGeoJsonPropertyMapper = new Dictionary<string, string>
    {
        { "distriktskod", "DistrictCode" }
    };
}