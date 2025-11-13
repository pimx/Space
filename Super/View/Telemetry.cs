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
        // UDP receiver
        private UdpClient udpClient;
        private Thread receiveThread;
        private bool isReceiving = true;

        // Embedded UDP sender for simulation
        private UdpClient senderClient;
        private Thread simulationThread;
        private bool isSimulating = false;

        // Rocket state in Earth-fixed frame (X=North, Y=Up, Z=East)
        private double rocketTime = 0;
        private Vector3 rocketPosition = new Vector3(0, 100, 0);
        private Vector3 rocketVelocity = new Vector3(0, -500, 0);
        private Vector3 rocketDirection = new Vector3(0, 1, 0); // Nose direction (unit vector)

        // Thrust state
        private float thrustMagnitude = 0;
        private Vector3 thrustDirection = new Vector3(0, 1, 0); // Thrust direction (unit vector)

        private void StartUDPReceiver()
        {
            udpClient = new UdpClient(5002);
            receiveThread = new Thread(() =>
            {
                IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 5002);

                while (isReceiving)
                {
                    try
                    {
                        byte[] message = udpClient.Receive(ref remoteEP);
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

        private void ParseRocketData(byte[] message)
        {
            /*
            time,
            rocket.worldPosition[1],
            rocket.worldPosition[2],
            rocket.worldPosition[3],
            rocket.roll,
            rocket.pitch,
            rocket.yaw,
            rocket.requiredThrustMagnitude,
            rocket.requiredThrustDirection[1],
            rocket.requiredThrustDirection[2],
            rocket.requiredThrustDirection[3],
            time
            */
            try
            {
                ReadOnlySpan<double> doubles = MemoryMarshal.Cast<byte, double>(message);
                rocketTime = doubles[0];
                // Position
                rocketPosition.Y = (float)doubles[1];   // alt
                rocketPosition.Z = (float)doubles[2];
                rocketPosition.X = (float)doubles[3];

                // Velocity
                rocketVelocity.Y = (float)doubles[4];   // alt
                rocketVelocity.Z = (float)doubles[5];
                rocketVelocity.X = (float)doubles[6];

                // Direction (nose pointing direction)
                rocketDirection.Y = (float)doubles[7];
                rocketDirection.Z = (float)doubles[8];
                rocketDirection.X = (float)doubles[9];
                rocketDirection = Vector3.Normalize(rocketDirection);

                // Thrust
                thrustMagnitude = (float)doubles[10];
                thrustDirection.Y = (float)doubles[11];
                thrustDirection.Z = (float)doubles[12];
                thrustDirection.X = (float)doubles[13];
                thrustDirection = Vector3.Normalize(thrustDirection);
                
                Console.WriteLine("Time {0,6:F3}:  Pos {1,8:F2} {2,8:F2} {3,8:F2}  Vel {4,8:F2} {5,8:F2} {6,8:F2}  Dir {7,8:F2} {8,8:F2} {9,8:F2}  Mag {10,8:F2}  Vec {11,8:F2} {12,8:F2} {13,8:F2}",
                    doubles[0],
                    doubles[1], doubles[2], doubles[3],
                    doubles[4], doubles[5], doubles[6],
                    doubles[7], doubles[8], doubles[9],
                    doubles[10],
                    doubles[11], doubles[12], doubles[13]
                );
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Parse Error: {ex.Message}");
            }
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


    }
}
