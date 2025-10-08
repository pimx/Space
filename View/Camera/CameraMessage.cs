using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace First
{
    // Data structure for camera control
    public struct CameraControl
    {
        public float Angle;      // Angle from north in radians
        public float Zoom;       // Zoom factor (0.1 to 10.0)
        public float Elevation;  // Optional: elevation angle in radians
    }
}
