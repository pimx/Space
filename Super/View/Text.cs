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

        private int textVAO, textVBO;
        private int textTexture;
        private int textShaderProgram;

        private void CreateTextShader()
        {
            string vertexShaderSource = @"
                #version 330 core
                layout (location = 0) in vec2 aPosition;
                layout (location = 1) in vec2 aTexCoord;
                
                out vec2 TexCoord;
                
                uniform mat4 projection;
                
                void main()
                {
                    gl_Position = projection * vec4(aPosition, 0.0, 1.0);
                    TexCoord = aTexCoord;
                }
            ";

            string fragmentShaderSource = @"
                #version 330 core
                in vec2 TexCoord;
                out vec4 FragColor;
                
                uniform sampler2D textTexture;
                uniform vec3 textColor;
                
                void main()
                {
                    vec4 sampled = texture(textTexture, TexCoord);
                    FragColor = vec4(textColor, 1.0) * sampled;
                }
            ";

            int vertexShader = GL.CreateShader(ShaderType.VertexShader);
            GL.ShaderSource(vertexShader, vertexShaderSource);
            GL.CompileShader(vertexShader);

            int fragmentShader = GL.CreateShader(ShaderType.FragmentShader);
            GL.ShaderSource(fragmentShader, fragmentShaderSource);
            GL.CompileShader(fragmentShader);

            textShaderProgram = GL.CreateProgram();
            GL.AttachShader(textShaderProgram, vertexShader);
            GL.AttachShader(textShaderProgram, fragmentShader);
            GL.LinkProgram(textShaderProgram);

            GL.DeleteShader(vertexShader);
            GL.DeleteShader(fragmentShader);
        }

        private void CreateTextQuad()
        {
            float[] vertices = new float[]
            {
                // Positions    // TexCoords
                0.0f, 0.0f,     0.0f, 1.0f,
                1.0f, 0.0f,     1.0f, 1.0f,
                1.0f, 1.0f,     1.0f, 0.0f,
                0.0f, 0.0f,     0.0f, 1.0f,
                1.0f, 1.0f,     1.0f, 0.0f,
                0.0f, 1.0f,     0.0f, 0.0f
            };

            textVAO = GL.GenVertexArray();
            textVBO = GL.GenBuffer();

            GL.BindVertexArray(textVAO);
            GL.BindBuffer(BufferTarget.ArrayBuffer, textVBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.StaticDraw);

            GL.VertexAttribPointer(0, 2, VertexAttribPointerType.Float, false, 4 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(1, 2, VertexAttribPointerType.Float, false, 4 * sizeof(float), 2 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);

            // Create initial texture
            textTexture = GL.GenTexture();
        }

        private void RenderTelemetryText()
        {
            // Calculate telemetry values
            //float altitude = rocketPosition.Y;
            //float speed = 1000.0f; // velocity.Length;
            //float verticalSpeed = rocketVelocity.Y; // velocity.Y;
            //float thrustKN = thrustMagnitude / 1000.0f;
            float tiltRad = MathF.Abs(MathF.Atan2(MathF.Sqrt(rocketDirection.X * rocketDirection.X + rocketDirection.Z * rocketDirection.Z), rocketDirection.Y));
            float tiltAngle = tiltRad * 180.0f / MathF.PI;

            // Create telemetry text
            string telemetry = $"телеметрия\n" +
                             $"Alt:  {rocketPosition.Y,6:F2} m\n" +
                             $"Vel:  {rocketVelocity.Y,6:F2} m/s\n" +
                             $"Tilt: {tiltAngle,7:F1}\n" +
                             $"Thr:  {thrustMagnitude,7:F0} kN\n" +
                             $"Pos:  {rocketPosition.X,6:F2} {rocketPosition.Y,6:F2} {rocketPosition.Z,6:F2})\n" +
                             $"Dir:  {rocketDirection.X,6:F2} {rocketDirection.Y,6:F2} {rocketDirection.Z,6:F2})\n" +
                             $"Time: {rocketTime,6:F3} \n";

            // Render text to texture using System.Drawing
            var bitmap = new Bitmap(400, 300);
            var graphics = Graphics.FromImage(bitmap);

            // Background with transparency
            graphics.Clear(System.Drawing.Color.FromArgb(200, 0, 0, 0));

            // Draw text
            var font = new System.Drawing.Font("Consolas", 10, System.Drawing.FontStyle.Bold);
            var brush = new System.Drawing.SolidBrush(System.Drawing.Color.FromArgb(255, 0, 255, 0));
            graphics.DrawString(telemetry, font, brush, new System.Drawing.PointF(10, 10));

            // Convert bitmap to OpenGL texture
            var bitmapData = bitmap.LockBits(
                new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height),
                System.Drawing.Imaging.ImageLockMode.ReadOnly,
                System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            GL.BindTexture(TextureTarget.Texture2D, textTexture);
            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba,
                bitmap.Width, bitmap.Height, 0,
                PixelFormat.Bgra, PixelType.UnsignedByte, bitmapData.Scan0);

            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);

            bitmap.UnlockBits(bitmapData);

            graphics.Dispose();
            bitmap.Dispose();
            font.Dispose();
            brush.Dispose();
        }

        private void DrawTelemetryOverlay()
        {
            // Disable depth test for 2D overlay
            GL.Disable(EnableCap.DepthTest);

            GL.UseProgram(textShaderProgram);

            // Orthographic projection for 2D overlay
            Matrix4 projection = Matrix4.CreateOrthographicOffCenter(0, Size.X, Size.Y, 0, -1, 1);
            int projLoc = GL.GetUniformLocation(textShaderProgram, "projection");
            GL.UniformMatrix4(projLoc, false, ref projection);

            // Set text color
            int colorLoc = GL.GetUniformLocation(textShaderProgram, "textColor");
            GL.Uniform3(colorLoc, new Vector3(1.0f, 1.0f, 1.0f));

            // Bind texture
            GL.ActiveTexture(TextureUnit.Texture0);
            GL.BindTexture(TextureTarget.Texture2D, textTexture);

            // Draw quad in top-left corner
            GL.BindVertexArray(textVAO);

            // Scale and position the text quad
            float width = 400;
            float height = 300;
            float x = 10;
            float y = 10;

            // Update quad vertices for position and size
            float[] vertices = new float[]
            {
                x,         y,          0.0f, 0.0f,
                x + width, y,          1.0f, 0.0f,
                x + width, y + height, 1.0f, 1.0f,
                x,         y,          0.0f, 0.0f,
                x + width, y + height, 1.0f, 1.0f,
                x,         y + height, 0.0f, 1.0f
            };

            GL.BindBuffer(BufferTarget.ArrayBuffer, textVBO);
            GL.BufferSubData(BufferTarget.ArrayBuffer, IntPtr.Zero, vertices.Length * sizeof(float), vertices);

            GL.DrawArrays(PrimitiveType.Triangles, 0, 6);

            GL.BindVertexArray(0);
            GL.BindTexture(TextureTarget.Texture2D, 0);

            // Re-enable depth test
            GL.Enable(EnableCap.DepthTest);
        }

    }
}
