using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;

using OpenTK;
using OpenTK.Graphics;
using OpenTK.Input;
using System.Drawing;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;

namespace First
{
    // Main visualization window
    public partial class RocketVisualizationWindow : GameWindow
    {
        private Cameras cameras;
        private Camera CurrentCamera => cameras.CurrentCamera;
        private const float ROCKET_LENGTH = 10.0f;  // meters
        private const float ROCKET_RADIUS = 0.5f;   // meters
        private const int UDP_PORT = 5002;         // UDP listening port for telemetry
        private const int CAMERA_UDP_PORT = 12346;  // UDP listening port for camera control

        private TelemetryReceiver telemetryReceiver;
        //private CameraControlReceiver cameraControlReceiver;
        //private RocketTelemetry currentTelemetry;
        private readonly object telemetryLock = new object();

        private List<Vector3> trajectoryHistory;
        private const int MAX_TRAJECTORY_POINTS = 500;

        private int rocketVAO, rocketVBO, rocketEBO;
        private int groundVAO, groundVBO;
        private int gridVAO, gridVBO;
        private int thrustVAO, thrustVBO;
        private int trajectoryVAO, trajectoryVBO;

        private int shaderProgram;
        private int vertexShader, fragmentShader;

        private Matrix4 rocketModelMatrix;
        private DateTime lastFrameTime;
        private double targetFrameTime = 1.0 / 30.0; // 30 FPS

        public RocketVisualizationWindow() : base(GameWindowSettings.Default, new NativeWindowSettings() { ClientSize = (1000, 800), Title = "SpaceXaXa" })
        {
            VSync = VSyncMode.Off;
            trajectoryHistory = new List<Vector3>();
        }

        protected override void OnLoad()
        {
            base.OnLoad();

            GL.ClearColor(0.1f, 0.2f, 0.2f, 1.0f);
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

            // Initialize cameras
            cameras = new Cameras();
            cameras.InitializeCameras(CAMERA_UDP_PORT, FloatAspectRatio);

            // Create shaders
            CreateShaders();

            // Create geometry
            CreateRocketGeometry();
            CreateGroundGeometry();
            CreateGridGeometry();
            CreateThrustGeometry();
            CreateTrajectoryGeometry();

            // Start UDP receivers
            telemetryReceiver = new TelemetryReceiver(UDP_PORT);
            telemetryReceiver.OnDataReceived += OnTelemetryReceived;

            //cameraControlReceiver = new CameraControlReceiver(CAMERA_UDP_PORT);
            //cameraControlReceiver.OnControlReceived += OnCameraControlReceived;

            Console.WriteLine($"Telemetry UDP Receiver started on port {UDP_PORT}");
            //Console.WriteLine($"Camera Control UDP Receiver started on port {CAMERA_UDP_PORT}");
            Console.WriteLine("Use mouse (rigth button, weel) to camera control");

            lastFrameTime = DateTime.Now;
        }

        /*
        private void InitializeCameras()
        {
            cameras = new List<Camera>
            {
                // Side view
                new Camera(new Vector3(50, 20, 0), Vector3.Zero),
                // Rocket view (from rocket looking down)
                new Camera(new Vector3(0, 15, 0), Vector3.Zero),
                // Diagonal view
                new Camera(new Vector3(30, 30, 30), Vector3.Zero),
                // Top-down view
                new Camera(new Vector3(0, 50, 1), Vector3.Zero),
                // UDP-controlled camera (rotates around landing point)
                new Camera(new Vector3(30, 20, 0), Vector3.Zero)
            };

            foreach (var camera in cameras)
            {
                camera.AspectRatio = FloatAspectRatio; // Width / Height;
            }
        }

        private void OnCameraControlReceived(CameraControl control)
        {
            lock (telemetryLock)
            {
                udpCameraAngle = control.Angle;
                udpCameraZoom = control.Zoom;
                udpCameraElevation = control.Elevation;
            }
        }

        private void UpdateUDPCamera()
        {
            // Smooth camera transitions
            smoothCameraAngle = MathHelper.Lerp(smoothCameraAngle, udpCameraAngle, CAMERA_SMOOTH_FACTOR);
            smoothCameraZoom = MathHelper.Lerp(smoothCameraZoom, udpCameraZoom, CAMERA_SMOOTH_FACTOR);
            smoothCameraElevation = MathHelper.Lerp(smoothCameraElevation, udpCameraElevation, CAMERA_SMOOTH_FACTOR);

            // Calculate camera position based on angle and zoom
            // Angle is from north (positive Z axis), rotating clockwise when viewed from above
            float baseDistance = 30.0f / smoothCameraZoom; // Base distance modified by zoom
            float height = baseDistance * (float)Math.Sin(smoothCameraElevation);
            float horizontalDistance = baseDistance * (float)Math.Cos(smoothCameraElevation);

            // Convert angle from north to standard math angle (from east)
            // North = 0, East = π/2, South = π, West = 3π/2
            float mathAngle = MathHelper.PiOver2 - smoothCameraAngle;

            float x = horizontalDistance * (float)Math.Cos(mathAngle);
            float z = horizontalDistance * (float)Math.Sin(mathAngle);

            // Update UDP-controlled camera (index 4)
            cameras[4].Position = new Vector3(x, height, z);
            cameras[4].Target = Vector3.Zero; // Always look at landing point
            cameras[4].FOV = MathHelper.PiOver4 / (float)Math.Sqrt(smoothCameraZoom); // Adjust FOV with zoom
        }
        */

        private void OnTelemetryReceived(RocketTelemetry telemetry)
        {
            lock (telemetryLock)
            {
                //currentTelemetry = telemetry;

                // Add to trajectory history
                if (trajectoryHistory.Count >= MAX_TRAJECTORY_POINTS)
                {
                    trajectoryHistory.RemoveAt(0);
                }
                trajectoryHistory.Add(telemetry.Position);
            }
        }

        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            base.OnMouseDown(e);
            cameras.OnMouseDown(e);
        }

        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            base.OnMouseUp(e);
            cameras.OnMouseUp(e);
        }

        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            base.OnMouseMove(e);
            cameras.OnMouseMove(e);
        }

        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            base.OnMouseWheel(e);
            cameras.OnMouseWheel(e);
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);
            cameras.OnUpdateFrame(e, KeyboardState);

            rocketModelMatrix = telemetryReceiver.ModelMatrix();

            UpdateTrajectoryBuffer();
        }

        private void UpdateTrajectoryBuffer()
        {
            if (trajectoryHistory.Count < 2)
                return;

            List<float> trajectoryData = new List<float>();

            for (int i = 0; i < trajectoryHistory.Count; i++)
            {
                Vector3 point = trajectoryHistory[i];
                float intensity = (float)i / trajectoryHistory.Count;

                trajectoryData.AddRange(new[] { point.X, point.Y, point.Z });
                trajectoryData.AddRange(new[] { 0.2f, intensity, 0.2f }); // Green gradient
            }

            GL.BindBuffer(BufferTarget.ArrayBuffer, trajectoryVBO);
            GL.BufferSubData(BufferTarget.ArrayBuffer, IntPtr.Zero,
                           trajectoryData.Count * sizeof(float), trajectoryData.ToArray());
        }

        private void UpdateThrustGeometry()
        {
            lock (telemetryLock)
            {
                if (telemetryReceiver.currentTelemetry.ThrustMagnitude > 0.01f)
                {
                    List<float> thrustVertices = new List<float>();

                    // Create multi-layered flame effect
                    float flameLength = telemetryReceiver.currentTelemetry.ThrustMagnitude * 8.0f;

                    // Transform thrust vector from body frame to world frame
                    //Vector3 thrustDir = Vector3.Transform(currentTelemetry.ThrustVector, Matrix4.CreateFromQuaternion(Quaternion.FromEulerAngles(currentTelemetry.Angles)));
                    Vector3 thrustDir = Vector3.Transform(telemetryReceiver.currentTelemetry.ThrustVector, Quaternion.FromEulerAngles(telemetryReceiver.currentTelemetry.Angles));
                    thrustDir.Normalize();

                    // Inner hot core (white-blue)
                    CreateFlameCone(thrustVertices, flameLength * 0.3f, 0.3f,
                                   new Vector3(0.9f, 0.9f, 1.0f), new Vector3(0.7f, 0.8f, 1.0f), 8);

                    // Middle flame (yellow-orange)
                    CreateFlameCone(thrustVertices, flameLength * 0.6f, 0.6f,
                                   new Vector3(1.0f, 0.9f, 0.3f), new Vector3(1.0f, 0.6f, 0.1f), 12);

                    // Outer flame (red-orange)
                    CreateFlameCone(thrustVertices, flameLength, 1.0f,
                                   new Vector3(1.0f, 0.4f, 0.1f), new Vector3(0.8f, 0.2f, 0.0f), 16);

                    GL.BindBuffer(BufferTarget.ArrayBuffer, thrustVBO);
                    GL.BufferData(BufferTarget.ArrayBuffer, thrustVertices.Count * sizeof(float),
                                thrustVertices.ToArray(), BufferUsageHint.DynamicDraw);
                }
            }
        }

        private void CreateFlameCone(List<float> vertices, float length, float radiusScale,
                                    Vector3 tipColor, Vector3 baseColor, int segments)
        {
            float baseRadius = ROCKET_RADIUS * radiusScale * telemetryReceiver.currentTelemetry.ThrustMagnitude;
            float angleStep = MathHelper.TwoPi / segments;

            // Add random flicker effect
            Random rand = new Random();
            float flicker = 0.9f + (float)rand.NextDouble() * 0.2f;
            length *= flicker;

            // Cone tip (at rocket nozzle)
            vertices.AddRange(new[] { 0f, -ROCKET_LENGTH / 2 - 0.1f, 0f });
            vertices.AddRange(new[] { tipColor.X, tipColor.Y, tipColor.Z });

            // Cone base with wavy effect
            for (int i = 0; i <= segments; i++)
            {
                float angle = i * angleStep;
                float waveOffset = (float)Math.Sin(angle * 3 + DateTime.Now.Millisecond / 100.0) * 0.1f;
                float radius = baseRadius * (1.0f + waveOffset);

                float x = (float)Math.Cos(angle) * radius;
                float z = (float)Math.Sin(angle) * radius;

                vertices.AddRange(new[] { x, -ROCKET_LENGTH / 2 - length, z });
                vertices.AddRange(new[] { baseColor.X * flicker, baseColor.Y * flicker, baseColor.Z });
            }
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            // Frame rate limiting
            var currentTime = DateTime.Now;
            var elapsed = (currentTime - lastFrameTime).TotalSeconds;

            if (elapsed < targetFrameTime)
            {
                Thread.Sleep((int)((targetFrameTime - elapsed) * 1000));
            }
            lastFrameTime = DateTime.Now;

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);


            GL.UseProgram(shaderProgram);

            // Set view and projection matrices
            Matrix4 view = cameras.GetViewMatrix();
            Matrix4 projection = cameras.GetProjectionMatrix();

            int viewLoc = GL.GetUniformLocation(shaderProgram, "view");
            int projLoc = GL.GetUniformLocation(shaderProgram, "projection");

            GL.UniformMatrix4(viewLoc, false, ref view);
            GL.UniformMatrix4(projLoc, false, ref projection);

            // Render ground
            Matrix4 groundModel = Matrix4.Identity;
            int modelLoc = GL.GetUniformLocation(shaderProgram, "model");
            GL.UniformMatrix4(modelLoc, false, ref groundModel);

            GL.BindVertexArray(groundVAO);
            GL.DrawArrays(PrimitiveType.TriangleFan, 0, 4); // Ground plane
            GL.DrawArrays(PrimitiveType.TriangleFan, 4, 4); // Landing pad outer
            GL.DrawArrays(PrimitiveType.TriangleFan, 8, 4); // Landing pad inner

            // Render grid
            GL.BindVertexArray(gridVAO);
            GL.LineWidth(0.0f);
            int gridLineCount = 41 * 4 + 4; // 41 lines in each direction * 2 points each + axes
            GL.DrawArrays(PrimitiveType.Lines, 0, gridLineCount);

            // Render rocket
            GL.UniformMatrix4(modelLoc, false, ref rocketModelMatrix);
            GL.BindVertexArray(rocketVAO);
            GL.DrawElements(PrimitiveType.Triangles, 20 * 6, DrawElementsType.UnsignedInt, 0);

            // Render thrust with blending for flame effect
            lock (telemetryLock)
            {
                //currentTelemetry.ThrustMagnitude = 0.5f;
                //currentTelemetry.Position.X = 0;
                //currentTelemetry.Position.Y = 0;
                //currentTelemetry.Position.Z = 20;

                if (telemetryReceiver.currentTelemetry.ThrustMagnitude > 0.01f)
                {
                    UpdateThrustGeometry();

                    // Enable additive blending for flame effect
                    GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.One);
                    GL.DepthMask(false); // Disable depth writing for translucent flame

                    GL.UniformMatrix4(modelLoc, false, ref rocketModelMatrix);
                    GL.BindVertexArray(thrustVAO);

                    // Draw multiple flame layers
                    int vertexCount = 0;
                    GL.DrawArrays(PrimitiveType.TriangleFan, vertexCount, 9);  // Inner core
                    vertexCount += 9;
                    GL.DrawArrays(PrimitiveType.TriangleFan, vertexCount, 13); // Middle flame
                    vertexCount += 13;
                    GL.DrawArrays(PrimitiveType.TriangleFan, vertexCount, 17); // Outer flame

                    // Restore normal blending
                    GL.DepthMask(true);
                    GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
                }
            }

            // Render trajectory
            if (trajectoryHistory.Count > 1)
            {
                Matrix4 trajectoryModel = Matrix4.Identity;
                GL.UniformMatrix4(modelLoc, false, ref trajectoryModel);
                GL.BindVertexArray(trajectoryVAO);
                GL.LineWidth(2.0f);
                GL.DrawArrays(PrimitiveType.LineStrip, 0, trajectoryHistory.Count);
                GL.LineWidth(1.0f);
            }

            SwapBuffers();

            Title = $"Rocket Landing Visualization - Side View Camera";
        }

        protected int Width
        {
            get
            {
                return this.ClientSize.X;
            }
        }
        protected int Height
        {
            get
            {
                return this.ClientSize.Y;
            }
        }
        protected float FloatAspectRatio
        {
            get
            {
                (int x, int y)? ar = base.AspectRatio;
                if (ar == null) return 1.0f;
                return (float)ar.Value.x / (float)ar.Value.y;
            }
        }
        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);

            GL.Viewport(0, 0, Width, Height);

            cameras.OnResize(e, FloatAspectRatio);
        }

        protected override void OnUnload()
        {
            telemetryReceiver?.Stop();
            cameras?.OnUnload();

            GL.DeleteBuffer(rocketVBO);
            GL.DeleteBuffer(rocketEBO);
            GL.DeleteVertexArray(rocketVAO);

            GL.DeleteBuffer(groundVBO);
            GL.DeleteVertexArray(groundVAO);

            GL.DeleteBuffer(gridVBO);
            GL.DeleteVertexArray(gridVAO);

            GL.DeleteBuffer(thrustVBO);
            GL.DeleteVertexArray(thrustVAO);

            GL.DeleteBuffer(trajectoryVBO);
            GL.DeleteVertexArray(trajectoryVAO);

            GL.DeleteProgram(shaderProgram);
            GL.DeleteShader(vertexShader);
            GL.DeleteShader(fragmentShader);

            base.OnUnload();
        }
    }
}