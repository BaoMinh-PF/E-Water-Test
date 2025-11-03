using NetTopologySuite.Geometries;

namespace E_Water_Test;
public class RoadModel
{
    public class RoadGeoJsonModel
    {
        public string ElementID { get; set; }
        public string Type { get; set; }
        public float Length { get; set; }
        public string SpeedLimit { get; set; }
        public string BearingClass { get; set; }
        public Geometry Geometry { get; set; }
    }

    public Dictionary<string, string> RoadGeoJsonPropertyMapper = new Dictionary<string, string>
    {
        { "ELEMENT_ID", "ElementID" },
        { "TYPE", "Type" },
        { "EXTENT_LENGTH", "Length" },
        { "SPEED_LIMIT", "SpeedLimit" },
        { "BEARING_CLASS", "BearingClass" },
    };


    public class RoadDbModel
    {
        public int ID { get; set; }
        public string RoadID { get; set; }
        public string Type { get; set; }
        public float Length { get; set; }
        public string SpeedLimit { get; set; }
        public string BearingClass { get; set; }
        public Geometry Coordinate { get; set; }
    }

    public Dictionary<string, string> TrafficTypeMapper = new Dictionary<string, string>
    {
        { "bilnät", "car" },
        { "gångnät", "walking" },
        { "cykelnät", "bicycle" }
    };
}
