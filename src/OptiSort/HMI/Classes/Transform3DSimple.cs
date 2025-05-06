using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptiSort.Classes
{
    public class Transform3DSimple
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Roll { get; set; }
        public double Pitch { get; set; }
        public double Yaw { get; set; }

        // Parameterless constructor for JSON deserialization
        public Transform3DSimple() { }

        public Transform3DSimple(double x, double y, double z, double yaw, double pitch, double roll)
        {
            X = x;
            Y = y;
            Z = z;
            Roll = roll;
            Pitch = pitch;
            Yaw = yaw;
        }
    }
}
