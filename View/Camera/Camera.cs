using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace First
{
    // Camera controller
    public class Camera
    {
        public Vector3 Position { get; set; }
        public Vector3 Target { get; set; }
        public Vector3 Up { get; set; }
        public float FOV { get; set; } = MathHelper.PiOver4;
        public float AspectRatio { get; set; } = 1.0f;

        public Camera(Vector3 position, Vector3 target)
        {
            Position = position;
            Target = target;
            Up = Vector3.UnitY;
        }

        public Matrix4 GetViewMatrix()
        {
            return Matrix4.LookAt(Position, Target, Up);
        }

        public Matrix4 GetProjectionMatrix()
        {
            return Matrix4.CreatePerspectiveFieldOfView(FOV, AspectRatio, 0.1f, 1000f);
        }
    }
}
