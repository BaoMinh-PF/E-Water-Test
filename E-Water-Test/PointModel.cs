using NetTopologySuite.Geometries;

namespace E_Water_Test;
public class PointModel
{
    public abstract class PointAbstract
    {
        public int ID { get; set; }
        public string Name { get; set; }
        public string Address { get; set; }
        public int MunicipalID { get; set; }
        public string EntranceRoadID { get; set; }
        public Geometry EntranceRoadCoordinate { get; set; }
        public Geometry Coordinate { get; set; }
    }

    public class SupplyPointDbModel : PointAbstract
    {

    }

    public class DistributionPointDbModel : PointAbstract
    {

    }
}
