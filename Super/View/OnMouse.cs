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
        bool mouseRotating = false;
        Vector2 lastMousePos = new Vector2(0, 0);
        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            base.OnMouseDown(e);
            if (e.Button == MouseButton.Right)
            {
                mouseRotating = true;
            }
        }

        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            base.OnMouseUp(e);
            if (e.Button == MouseButton.Right)
            {
                mouseRotating = false;
            }
        }

        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            base.OnMouseMove(e);
            if (mouseRotating) // Only for UDP/manual camera
            {
                float deltaX = e.X - lastMousePos.X;
                float deltaY = e.Y - lastMousePos.Y;

                float angle = MathF.Atan2(cameraPos.X, cameraPos.Z);
                angle -= deltaX * 0.005f;
                float dist = MathF.Sqrt(cameraPos.X * cameraPos.X + cameraPos.Z * cameraPos.Z);
                cameraPos.X = MathF.Sin(angle) * dist;
                cameraPos.Z = MathF.Cos(angle) * dist;

                cameraPos.Y = cameraPos.Y + deltaY * 0.5f;
            }
            lastMousePos = new Vector2(e.X, e.Y);
        }

        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            base.OnMouseWheel(e);
            float angle = MathF.Atan2(cameraPos.X, cameraPos.Z);
            float dist = MathF.Sqrt(cameraPos.X * cameraPos.X + cameraPos.Z * cameraPos.Z);
            cameraPos.X = MathF.Sin(angle) * dist;
            cameraPos.Z = MathF.Cos(angle) * dist;

            float mult = (float)Math.Pow(1.05, e.OffsetY);
            cameraPos.X = MathF.Sin(angle) * dist * mult;
            cameraPos.Z = MathF.Cos(angle) * dist * mult;
            cameraPos.Y = cameraPos.Y * mult;
        }
        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);
            GL.Viewport(0, 0, ClientSize.X, ClientSize.Y);
        }
    }

}
