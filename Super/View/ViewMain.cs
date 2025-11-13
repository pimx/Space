using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using System;
using System.Drawing;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace View
{
    public partial class RocketWindow : GameWindow
    {
        //private int shaderProgram;
        //private int VAO, VBO, EBO;
        //private int flameVAO, flameVBO;
        //private int padVAO, padVBO, padEBO;
        //private int thrustVAO, thrustVBO;

        private Vector3 cameraPos = new Vector3(0, 50, 150);
        private Vector3 cameraTarget = new Vector3(0, 10, 0);


        public RocketWindow(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings)
            : base(gameWindowSettings, nativeWindowSettings)
        {
            VSync = VSyncMode.On;
        }

        protected override void OnLoad()
        {
            base.OnLoad();

            OnLoadPicture();

            StartUDPReceiver();

            Console.WriteLine("=== Rocket Visualizer Started ===");
            Console.WriteLine("Earth-Fixed Frame: X=North, Y=Up, Z=East");
            Console.WriteLine("Listening on UDP port 5000");
            Console.WriteLine("Message format: PosX,PosY,PosZ,DirX,DirY,DirZ,ThrustMag,ThrustDirX,ThrustDirY,ThrustDirZ");
            Console.WriteLine("Example: 0,50,0,0,1,0,980000,0,1,0");
            //Console.WriteLine("\nControls:");
            //Console.WriteLine("  SPACE - Start/Stop embedded landing simulation");
            //Console.WriteLine("  W/A/S/D/Q/E - Camera movement");
            //Console.WriteLine("  ESC - Exit\n");
        }

        protected override void OnUnload()
        {
            base.OnUnload();

            isReceiving = false;
            isSimulating = false;
            udpClient?.Close();
            senderClient?.Close();

            OnUnloadPicture();
        }

        static private Matrix4 CreateRotationFromDirection(Vector3 direction)
        {
            // Create rotation matrix to orient rocket along direction vector
            // Rocket model default points up (+Y), we want it to point along 'direction'

            direction = Vector3.Normalize(direction);

            // If direction is already up, return identity
            if (Math.Abs(direction.Y - 1.0f) < 0.001f)
                return Matrix4.Identity;

            // If direction is down, rotate 180 degrees
            if (Math.Abs(direction.Y + 1.0f) < 0.001f)
                return Matrix4.CreateRotationZ(MathF.PI);

            // Create orthonormal basis
            Vector3 up = direction; // Rocket's longitudinal axis
            Vector3 worldUp = Vector3.UnitY;

            // Find a right vector perpendicular to 'up'
            Vector3 right = Vector3.Normalize(Vector3.Cross(worldUp, up));

            // Recompute forward to ensure orthogonality
            Vector3 forward = Vector3.Normalize(Vector3.Cross(up, right));

            // Build rotation matrix (column-major for OpenGL)
            Matrix4 rotation = new Matrix4(
                new Vector4(right.X, right.Y, right.Z, 0),
                new Vector4(up.X, up.Y, up.Z, 0),
                new Vector4(forward.X, forward.Y, forward.Z, 0),
                new Vector4(0, 0, 0, 1)
            );

            return rotation;
        }

        protected override void OnUpdateFrame(FrameEventArgs args)
        {
            base.OnUpdateFrame(args);

            if (KeyboardState.IsKeyDown(Keys.Escape))
                Close();

            if (KeyboardState.IsKeyPressed(Keys.Space))
            {
                //if (!isSimulating)
                    //StartLandingSimulation();
                //else
                    //StopLandingSimulation();
            }

            /*
            float cameraSpeed = 50.0f * (float)args.Time;

            if (KeyboardState.IsKeyDown(Keys.W)) cameraPos.Z -= cameraSpeed;
            if (KeyboardState.IsKeyDown(Keys.S)) cameraPos.Z += cameraSpeed;
            if (KeyboardState.IsKeyDown(Keys.A)) cameraPos.X -= cameraSpeed;
            if (KeyboardState.IsKeyDown(Keys.D)) cameraPos.X += cameraSpeed;
            if (KeyboardState.IsKeyDown(Keys.Q)) cameraPos.Y -= cameraSpeed;
            if (KeyboardState.IsKeyDown(Keys.E)) cameraPos.Y += cameraSpeed;
            */
        }

    }

    class Program
    {
        static void Main(string[] args)
        {
            var nativeWindowSettings = new NativeWindowSettings()
            {
                ClientSize = new Vector2i(1280, 720),
                Title = "SpaceX Rocket Visualizer - Vector Directions (X=North, Y=Up, Z=East)",
                Flags = ContextFlags.ForwardCompatible,
            };

            using (var window = new RocketWindow(GameWindowSettings.Default, nativeWindowSettings))
            {
                window.Run();
            }
        }
    }
}