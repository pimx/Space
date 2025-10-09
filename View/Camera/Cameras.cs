using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Windowing.Desktop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace First
{
    public class Cameras
    {
        public Camera CurrentCamera => cameras[currentCameraIndex];

        //private const int CAMERA_UDP_PORT = 12346;  // UDP listening port for camera control
        private CameraControlReceiver cameraControlReceiver;

        private List<Camera> cameras;
        private int currentCameraIndex = 0;

        private float smoothCameraAngle = 0.0f;                     // Smoothed angle
        private float smoothCameraZoom = 1.0f;                      // Smoothed zoom
        private float smoothCameraElevation = MathHelper.PiOver6;   // 30 degrees default
        private const float CAMERA_SMOOTH_FACTOR = 0.5f; // Smoothing factor

        private bool mouseRotating = false;
        private Vector2 lastMousePos;
        private float manualCameraAngle = 0.0f;
        private float manualCameraElevation = MathHelper.PiOver6;
        private float manualCameraZoom = 1.0f;

        public void InitializeCameras(int port, float aspectRatio)
        {
            cameras = new List<Camera>
            {
                new Camera(new Vector3(50, 20, 0), Vector3.Zero),   // Side view
            };

            foreach (var camera in cameras)
            {
                camera.AspectRatio = aspectRatio; // Width / Height;
                camera.FOV = 30;
            }
            cameraControlReceiver = new CameraControlReceiver(port);
            cameraControlReceiver.OnControlReceived += OnCameraControlReceived;
        }

        private void OnCameraControlReceived(CameraControl control)
        {
            //lock (telemetryLock)
            {
                //udpCameraAngle = control.Angle;
                //udpCameraZoom = control.Zoom;
                //udpCameraElevation = control.Elevation;
            }
        }

        private void UpdateUDPCamera()
        {
            // Smooth camera transitions
            smoothCameraAngle = MathHelper.Lerp(smoothCameraAngle, manualCameraAngle, CAMERA_SMOOTH_FACTOR);
            smoothCameraZoom = MathHelper.Lerp(smoothCameraZoom, manualCameraZoom, CAMERA_SMOOTH_FACTOR);
            smoothCameraElevation = MathHelper.Lerp(smoothCameraElevation, manualCameraElevation, CAMERA_SMOOTH_FACTOR);

            // Calculate camera position based on angle and zoom
            // Angle is from north (positive Z axis), rotating clockwise when viewed from above
            float baseDistance = 50.0f / smoothCameraZoom; // Base distance modified by zoom
            float height = baseDistance * (float)Math.Sin(smoothCameraElevation);
            float horizontalDistance = baseDistance * (float)Math.Cos(smoothCameraElevation);

            // Convert angle from north to standard math angle (from east)
            // North = 0, East = π/2, South = π, West = 3π/2
            float mathAngle = -smoothCameraAngle; // MathHelper.PiOver2 - smoothCameraAngle;

            float x = horizontalDistance * (float)Math.Cos(mathAngle);
            float z = horizontalDistance * (float)Math.Sin(mathAngle);

            // Update UDP-controlled camera (index 4)
            CurrentCamera.Position = new Vector3(x, height, z);
            CurrentCamera.Target = Vector3.Zero; // Always look at landing point
            CurrentCamera.FOV = 30.0f / (float)Math.Sqrt(smoothCameraZoom); // MathHelper.PiOver4 / (float)Math.Sqrt(smoothCameraZoom); // Adjust FOV with zoom
        }

        public Matrix4 GetViewMatrix() => CurrentCamera.GetViewMatrix();
        public Matrix4 GetProjectionMatrix()
        {
            float fov = CurrentCamera.FOV;
            float asr = CurrentCamera.AspectRatio;
            Matrix4 projection = Matrix4.CreatePerspectiveFieldOfView(MathHelper.DegreesToRadians(fov), asr, 0.01f, 1000f);
            return projection;
        }

        public void OnMouseDown(MouseButtonEventArgs e)
        {
            //Console.WriteLine($"Down {e.Button}");
            //base.OnMouseDown(e);

            if (e.Button == MouseButton.Right)
            {
                mouseRotating = true;
                //lastMousePos = new Vector2(e.X, e.Y);
            }
        }

        public void OnMouseUp(MouseButtonEventArgs e)
        {
            //Console.WriteLine($"Up   {e.Button}");
            //base.OnMouseUp(e);

            if (e.Button == MouseButton.Right)
            {
                mouseRotating = false;
            }
        }

        public void OnMouseMove(MouseMoveEventArgs e)
        {
            if (mouseRotating) // Only for UDP/manual camera
            {
                float deltaX = e.X - lastMousePos.X;
                float deltaY = e.Y - lastMousePos.Y;

                manualCameraAngle -= deltaX * 0.01f;
                manualCameraElevation = MathHelper.Clamp(
                    manualCameraElevation + deltaY * 0.01f,
                    0.1f, MathHelper.PiOver2 - 0.1f);
            }
            lastMousePos = new Vector2(e.X, e.Y);
        }

        public void OnMouseWheel(MouseWheelEventArgs e)
        {
                manualCameraZoom *= (float)Math.Pow(1.05, e.OffsetY);
                manualCameraZoom = MathHelper.Clamp(manualCameraZoom, 0.1f, 10.0f);
        }

        public void OnResize(ResizeEventArgs e, float aspectRatio)
        {
            //base.OnResize(e);

            foreach (var camera in cameras)
            {
                camera.AspectRatio = aspectRatio;
            }
        }
        public void OnUpdateFrame(FrameEventArgs e, KeyboardState keyboardState)
        {
            if (keyboardState.IsKeyDown(Keys.Escape))
            {
                //Exit();
            }

            if (keyboardState.IsKeyDown(Keys.Equal))
            {
                CurrentCamera.FOV /= 1.01f;
            }
            if (keyboardState.IsKeyDown(Keys.KeyPadAdd))
            {
                CurrentCamera.FOV /= 1.01f;
            }
            if (keyboardState.IsKeyDown(Keys.Minus))
            {
                CurrentCamera.FOV *= 1.01f;
            }
            if (keyboardState.IsKeyDown(Keys.KeyPadSubtract))
            {
                CurrentCamera.FOV *= 1.01f;
            }
            
            UpdateUDPCamera();

        }
        public void OnUnload()
        {
            cameraControlReceiver?.Stop();
        }
    }
}
