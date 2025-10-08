using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using OpenTK.Mathematics;

namespace First
{
    // Data structure for rocket telemetry
    public struct RocketTelemetry
    {
        public double Timestamp;      // Timestamp in seconds (double precision)
        public Vector3 Position;      // Position in local frame (meters)
        public Vector3 Angles;        // Roll, Pitch, Yaw (radians)
        public float ThrustMagnitude; // Thrust magnitude (0-1 normalized)
        public Vector3 ThrustVector;  // Thrust direction in body frame

        public unsafe void Unmarshal(byte[] data)
        {
            fixed (void* p = &data[0])
            {
                double* d = (double*)p;
                Timestamp = d[0];
                Position.X = (float)d[1];      // Position in local frame (meters)
                Position.Y = (float)d[3];      // Position in local frame (meters)
                Position.Z = (float)d[2];      // Position in local frame (meters)
                Angles.X = (float)d[4];        // Roll, Pitch, Yaw (radians)
                Angles.Y = (float)d[5];        // Roll, Pitch, Yaw (radians)
                Angles.Z = (float)d[6];        // Roll, Pitch, Yaw (radians)
                ThrustMagnitude = (float)d[7] / 100000.0f; // Thrust magnitude (0-1 normalized)
                ThrustVector.X = (float)d[8];  // Thrust direction in body frame
                ThrustVector.Y = (float)d[9];  // Thrust direction in body frame
                ThrustVector.Z = (float)d[10];  // Thrust direction in body frame
                double t = d[11];
            }
        }

        public unsafe void Marshal(byte[] data)
        {

            fixed (byte* p = &data[0])
            {
                *(RocketTelemetry*)p = this;
            }
        }
    }
}
