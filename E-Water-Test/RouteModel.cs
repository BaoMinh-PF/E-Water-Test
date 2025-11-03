using NetTopologySuite.Geometries;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace E_Water_Test
{
    public class RouteModel
    {
        public class RouteDbModel
        {
            public int ID { get; set; }
            public float Distance { get; set; }
            public int SupplyPointID { get; set; }
            public int DestiantionPointID { get; set; }
            public string Roads { get; set; }
            public Geometry Coordinate { get; set; }
        }

        public class NodeDbModel
        {
            public int ID { get; set; }
            public Geometry Coordinate { get; set; }
        }

        public class EdgeDbModel
        {
            public int ID { get; set; }
            public string RoadID { get; set; }
            public int FromNodeID { get; set; }
            public int ToNodeID { get; set; }
            public float Length { get; set; }
            public string SpeedLimit { get; set; }
            public string BearingClass { get; set; }
            public string Type { get; set; }
            public Geometry Coordinate { get; set; }
        }

        public class PopulatedRoadModel
        {
            public NodeDbModel StartNode { get; set; }
            public NodeDbModel EndNode { get; set; }
            public EdgeDbModel Edge { get; set; }
        }


        public class RouteStep
        {
            public EdgeDbModel Edge { get; set; }
            public double Distance { get; set; }
        }

        public class AdjacentNode
        {
            public int NodeId { get; set; }
            public EdgeDbModel Edge { get; set; }
            public double Weight { get; set; }
        }
    }
}
