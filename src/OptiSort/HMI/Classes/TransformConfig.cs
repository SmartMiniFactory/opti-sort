using Ace.Core.Server;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptiSort.Classes
{
    public class TransformConfig
    {
        // (X, Y, Z, Yaw, Pitch, Roll)
        public Transform3DSimple GridPick { get; set; } = new Transform3DSimple(516.00, -80.00, 320.00, -130.0, 180.00, 0);
        public Transform3DSimple GridPlace { get; set; } = new Transform3DSimple(432.924, 224.126, 330.00, -130.0, 180.00, 0);
        public Transform3DSimple SafeFlexi { get; set; } = new Transform3DSimple(375.00, 15.00, 385.00, -130.0, 180.00, 0);
        public Transform3DSimple BoxPlaceA { get; set; } = new Transform3DSimple(160.00, -450.00, 180.00, 50.0, 180.00, 0);
        public Transform3DSimple BoxPlaceB { get; set; } = new Transform3DSimple(310.00, -450.00, 180.00, 50.0, 180.00, 0);
        public Transform3DSimple SafeBoxes { get; set; } = new Transform3DSimple(200.00, -450.00, 360.00, 50.0, 180.00, 0);
    }
}
