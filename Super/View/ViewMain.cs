using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using System;
using System.Drawing;
using System.Net;
using System.Net.Sockets;
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

        // Rocket state in Earth-fixed frame (X=North, Y=Up, Z=East)
        private Vector3 rocketPosition = new Vector3(0, 100, 0);
        private Vector3 rocketDirection = new Vector3(0, 1, 0); // Nose direction (unit vector)

        // Thrust state
        private float thrustMagnitude = 0;
        private Vector3 thrustDirection = new Vector3(0, 1, 0); // Thrust direction (unit vector)

        // UDP receiver
        private UdpClient udpClient;
        private Thread receiveThread;
        private bool isReceiving = true;

        // Embedded UDP sender for simulation
        private UdpClient senderClient;
        private Thread simulationThread;
        private bool isSimulating = false;


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

        private void StartUDPReceiver()
        {
            udpClient = new UdpClient(5000);
            receiveThread = new Thread(() =>
            {
                IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 5000);

                while (isReceiving)
                {
                    try
                    {
                        byte[] data = udpClient.Receive(ref remoteEP);
                        string message = Encoding.ASCII.GetString(data);
                        ParseRocketData(message);
                    }
                    catch (Exception ex)
                    {
                        if (isReceiving)
                            Console.WriteLine($"UDP Error: {ex.Message}");
                    }
                }
            });

            receiveThread.IsBackground = true;
            receiveThread.Start();
        }

        private void ParseRocketData(string message)
        {
            try
            {
                string[] parts = message.Split(',');
                if (parts.Length >= 10)
                {
                    // Position
                    rocketPosition.X = float.Parse(parts[0].Trim());
                    rocketPosition.Y = float.Parse(parts[1].Trim());
                    rocketPosition.Z = float.Parse(parts[2].Trim());

                    // Direction (nose pointing direction)
                    rocketDirection.X = float.Parse(parts[3].Trim());
                    rocketDirection.Y = float.Parse(parts[4].Trim());
                    rocketDirection.Z = float.Parse(parts[5].Trim());
                    rocketDirection = Vector3.Normalize(rocketDirection);

                    // Thrust
                    thrustMagnitude = float.Parse(parts[6].Trim());
                    thrustDirection.X = float.Parse(parts[7].Trim());
                    thrustDirection.Y = float.Parse(parts[8].Trim());
                    thrustDirection.Z = float.Parse(parts[9].Trim());
                    thrustDirection = Vector3.Normalize(thrustDirection);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Parse Error: {ex.Message}");
            }
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

        private void StartLandingSimulation()
        {
            if (isSimulating) return;

            isSimulating = true;
            senderClient = new UdpClient();

            simulationThread = new Thread(() =>
            {
                IPEndPoint endpoint = new IPEndPoint(IPAddress.Loopback, 5000);

                // Physics simulation state
                Vector3 position = new Vector3(0, 100, 0);
                Vector3 velocity = new Vector3(0, 0, 0);
                Vector3 direction = new Vector3(0, 1, 0); // Pointing up initially
                Vector3 angularVelocity = new Vector3(0, 0, 0);

                const float gravity = 9.81f;
                const float mass = 100000f; // 100 tons
                const float dt = 0.02f; // 20ms timestep (50 Hz)
                const float maxTilt = 0.3f; // Max 0.3 radians from vertical


                Console.WriteLine("\n=== Landing Simulation Started ===");
                Console.WriteLine("Initial altitude: 100m, pointing UP");

                long start = DateTime.UtcNow.ToFileTimeUtc();
                float simTime = 0;

                while (isSimulating && position.Y > 0.1f)
                {
                    simTime += dt;
                    int sleep = (int)(simTime * 1000) + (int)(start / 10000 - DateTime.UtcNow.ToFileTimeUtc() / 10000);
                    //Console.WriteLine($"sleep {sleep}");
                    if (sleep > 0)
                    {
                        Thread.Sleep(sleep);
                    }

                    // Calculate target direction (mostly up with small corrections)
                    Vector3 targetDir = new Vector3(0, 1, 0);

                    // Add small random perturbations
                    float perturbX = ((float)random.NextDouble() - 0.5f) * 0.2f;
                    float perturbZ = ((float)random.NextDouble() - 0.5f) * 0.2f;
                    targetDir.X += perturbX;
                    targetDir.Z += perturbZ;
                    targetDir = Vector3.Normalize(targetDir);

                    // Smoothly interpolate direction (simulates attitude control)
                    direction = Vector3.Lerp(direction, targetDir, 0.1f);
                    direction = Vector3.Normalize(direction);

                    // Clamp direction to max tilt angle
                    float currentTilt = MathF.Acos(Vector3.Dot(direction, Vector3.UnitY));
                    if (currentTilt > maxTilt)
                    {
                        // Project back to max tilt
                        Vector3 tiltAxis = Vector3.Cross(Vector3.UnitY, direction);
                        if (tiltAxis.Length > 0.001f)
                        {
                            tiltAxis = Vector3.Normalize(tiltAxis);
                            Quaternion rotation = Quaternion.FromAxisAngle(tiltAxis, maxTilt);
                            direction = Vector3.Transform(Vector3.UnitY, rotation);
                        }
                    }

                    // Calculate required thrust for controlled descent
                    float targetThrust = mass * gravity;
                    float altitudeError = position.Y;
                    float velocityError = velocity.Y + 1.0f; // Target: -5 m/s descent

                    // PD controller
                    float thrustMag = targetThrust - altitudeError * 200.0f - velocityError * 3000.0f;
                    thrustMag = MathF.Max(0, MathF.Min(thrustMag, mass * gravity * 1.5f));

                    // Thrust direction (opposite to rocket direction for main engines)
                    // Add small gimbal for stabilization
                    /// Vector3 thrustDir = -direction;
                    Vector3 thrustDir = direction;

                    // Apply small corrections based on horizontal velocity
                    Vector3 horizontalVel = new Vector3(velocity.X, 0, velocity.Z);
                    if (horizontalVel.Length > 0.1f)
                    {
                        Vector3 correction = -Vector3.Normalize(horizontalVel) * 0.05f;
                        thrustDir += correction;
                        thrustDir = Vector3.Normalize(thrustDir);
                    }

                    // Physics integration
                    Vector3 thrustForce = thrustDir * thrustMag;
                    Vector3 gravityForce = new Vector3(0, -mass * gravity, 0);
                    Vector3 acceleration = (thrustForce + gravityForce) / mass;

                    velocity += acceleration * dt;
                    position += velocity * dt;

                    // Prevent ground penetration
                    if (position.Y < 0)
                    {
                        position.Y = 0;
                        velocity = Vector3.Zero;
                    }

                    // Send UDP message
                    string message = $"{position.X:F3},{position.Y:F3},{position.Z:F3}," +
                                   $"{direction.X:F6},{direction.Y:F6},{direction.Z:F6}," +
                                   $"{thrustMag:F1},{thrustDir.X:F6},{thrustDir.Y:F6},{thrustDir.Z:F6}";

                    byte[] data = Encoding.ASCII.GetBytes(message);
                    senderClient.Send(data, data.Length, endpoint);

                    // Log progress
                    //if ((int)(simTime * 10) % 10 == 0)
                    {
                        float tiltDeg = MathF.Acos(direction.Y) * 180.0f / MathF.PI;
                        Console.WriteLine($"T+{simTime:F1}s | Alt: {position.Y:F1}m | Vel: {velocity.Y:F2}m/s | " +
                                        $"Thrust: {thrustMag / 1000:F0}kN | Tilt: {tiltDeg:F1}°");
                    }

                    //Thread.Sleep((int)(dt * 1000));
                }

                Console.WriteLine($"=== Landing Complete! ===");
                Console.WriteLine($"Final velocity: {velocity.Y:F2} m/s");
                Console.WriteLine($"Final tilt: {MathF.Acos(direction.Y) * 180.0f / MathF.PI:F2}°");
                Console.WriteLine($"Landing time: {simTime:F2}s\n");

                isSimulating = false;
            });

            simulationThread.IsBackground = true;
            simulationThread.Start();
        }

        private void StopLandingSimulation()
        {
            isSimulating = false;
            senderClient?.Close();
        }


        protected override void OnUpdateFrame(FrameEventArgs args)
        {
            base.OnUpdateFrame(args);

            if (KeyboardState.IsKeyDown(Keys.Escape))
                Close();

            if (KeyboardState.IsKeyPressed(Keys.Space))
            {
                if (!isSimulating)
                    StartLandingSimulation();
                else
                    StopLandingSimulation();
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