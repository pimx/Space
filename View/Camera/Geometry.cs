using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;

namespace First
{
    public partial class RocketVisualizationWindow
    {
        private void CreateShaders()
        {
            string vertexShaderSource = @"
                #version 330 core
                layout(location = 0) in vec3 aPosition;
                layout(location = 1) in vec3 aColor;
                
                out vec3 fragColor;
                
                uniform mat4 model;
                uniform mat4 view;
                uniform mat4 projection;
                
                void main()
                {
                    gl_Position = projection * view * model * vec4(aPosition, 1.0);
                    fragColor = aColor;
                }
            ";

            string fragmentShaderSource = @"
                #version 330 core
                in vec3 fragColor;
                out vec4 FragColor;
                
                void main()
                {
                    FragColor = vec4(fragColor, 1.0);
                }
            ";

            vertexShader = GL.CreateShader(ShaderType.VertexShader);
            GL.ShaderSource(vertexShader, vertexShaderSource);
            GL.CompileShader(vertexShader);

            fragmentShader = GL.CreateShader(ShaderType.FragmentShader);
            GL.ShaderSource(fragmentShader, fragmentShaderSource);
            GL.CompileShader(fragmentShader);

            shaderProgram = GL.CreateProgram();
            GL.AttachShader(shaderProgram, vertexShader);
            GL.AttachShader(shaderProgram, fragmentShader);
            GL.LinkProgram(shaderProgram);

            GL.UseProgram(shaderProgram);
        }

        private void CreateRocketGeometry()
        {
            List<float> vertices = new List<float>();
            List<uint> indices = new List<uint>();

            // Create cylinder for rocket body
            int segments = 20;
            float angleStep = MathHelper.TwoPi / segments;

            // Bottom circle
            for (int i = 0; i < segments; i++)
            {
                float angle = i * angleStep;
                float x = (float)Math.Cos(angle) * ROCKET_RADIUS;
                float z = (float)Math.Sin(angle) * ROCKET_RADIUS;

                // Position
                vertices.AddRange(new[] { x, -ROCKET_LENGTH / 2, z });
                // Color (metallic gray)
                vertices.AddRange(new[] { 0.5f, 0.5f, 0.6f });
            }

            // Top circle
            for (int i = 0; i < segments; i++)
            {
                float angle = i * angleStep;
                float x = (float)Math.Cos(angle) * ROCKET_RADIUS;
                float z = (float)Math.Sin(angle) * ROCKET_RADIUS;

                // Position
                vertices.AddRange(new[] { x, ROCKET_LENGTH / 2, z });
                // Color (metallic gray)
                vertices.AddRange(new[] { 0.6f, 0.6f, 0.7f });
            }

            // Create indices for cylinder sides
            for (uint i = 0; i < segments; i++)
            {
                uint next = (i + 1) % (uint)segments;

                // Bottom triangle
                indices.AddRange(new[] { i, next, i + (uint)segments });
                // Top triangle
                indices.AddRange(new[] { next, next + (uint)segments, i + (uint)segments });
            }

            // Create and bind VAO/VBO/EBO
            rocketVAO = GL.GenVertexArray();
            rocketVBO = GL.GenBuffer();
            rocketEBO = GL.GenBuffer();

            GL.BindVertexArray(rocketVAO);

            GL.BindBuffer(BufferTarget.ArrayBuffer, rocketVBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Count * sizeof(float),
                         vertices.ToArray(), BufferUsageHint.StaticDraw);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, rocketEBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Count * sizeof(uint),
                         indices.ToArray(), BufferUsageHint.StaticDraw);

            // Position attribute
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            // Color attribute
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);
        }

        private void CreateGroundGeometry()
        {
            float[] vertices = {
                // Ground plane (large quad for background)
                -100, -0.01f, -100,  0.15f, 0.15f, 0.15f,
                 100, -0.01f, -100,  0.15f, 0.15f, 0.15f,
                 100, -0.01f,  100,  0.15f, 0.15f, 0.15f,
                -100, -0.01f,  100,  0.15f, 0.15f, 0.15f,
                
                // Landing pad center (X marks the spot)
                -5, 0.02f, -5,  0.9f, 0.1f, 0.1f,
                 5, 0.02f, -5,  0.9f, 0.1f, 0.1f,
                 5, 0.02f,  5,  0.9f, 0.1f, 0.1f,
                -5, 0.02f,  5,  0.9f, 0.1f, 0.1f,
                
                // Landing pad inner circle area
                -3, 0.03f, -3,  1.0f, 0.3f, 0.3f,
                 3, 0.03f, -3,  1.0f, 0.3f, 0.3f,
                 3, 0.03f,  3,  1.0f, 0.3f, 0.3f,
                -3, 0.03f,  3,  1.0f, 0.3f, 0.3f,
            };

            groundVAO = GL.GenVertexArray();
            groundVBO = GL.GenBuffer();

            GL.BindVertexArray(groundVAO);

            GL.BindBuffer(BufferTarget.ArrayBuffer, groundVBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float),
                         vertices, BufferUsageHint.StaticDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);
        }

        private void CreateGridGeometry()
        {
            List<float> vertices = new List<float>();
            float gridSize = 100.0f;
            float gridStep = 5.0f;
            int gridLines = (int)(gridSize * 2 / gridStep) + 1;

            // Create grid lines parallel to X axis
            for (int i = 0; i < gridLines; i++)
            {
                float z = -gridSize + i * gridStep;

                // Start point
                vertices.AddRange(new[] { -gridSize, 0.01f, z });
                vertices.AddRange(new[] { 0.2f, 0.4f, 0.2f }); // Gray color

                // End point
                vertices.AddRange(new[] { gridSize, 0.01f, z });
                vertices.AddRange(new[] { 0.2f, 0.4f, 0.2f });
            }

            // Create grid lines parallel to Z axis
            for (int i = 0; i < gridLines; i++)
            {
                float x = -gridSize + i * gridStep;

                // Start point
                vertices.AddRange(new[] { x, 0.01f, -gridSize });
                vertices.AddRange(new[] { 0.2f, 0.4f, 0.2f }); // Gray color

                // End point
                vertices.AddRange(new[] { x, 0.01f, gridSize });
                vertices.AddRange(new[] { 0.2f, 0.4f, 0.2f });
            }

            // Highlight main axes
            // X axis (red)
            vertices.AddRange(new[] { -gridSize, 0.015f, 0f });
            vertices.AddRange(new[] { 0.8f, 0.2f, 0.2f });
            vertices.AddRange(new[] { gridSize, 0.015f, 0f });
            vertices.AddRange(new[] { 0.8f, 0.2f, 0.2f });

            // Z axis (blue)
            vertices.AddRange(new[] { 0f, 0.015f, -gridSize });
            vertices.AddRange(new[] { 0.2f, 0.2f, 0.8f });
            vertices.AddRange(new[] { 0f, 0.015f, gridSize });
            vertices.AddRange(new[] { 0.2f, 0.2f, 0.8f });

            gridVAO = GL.GenVertexArray();
            gridVBO = GL.GenBuffer();

            GL.BindVertexArray(gridVAO);

            GL.BindBuffer(BufferTarget.ArrayBuffer, gridVBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Count * sizeof(float),
                         vertices.ToArray(), BufferUsageHint.StaticDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);
        }

        private void CreateThrustGeometry()
        {
            // Create cone for thrust visualization - allocate more space for multi-layer flame
            thrustVAO = GL.GenVertexArray();
            thrustVBO = GL.GenBuffer();

            GL.BindVertexArray(thrustVAO);
            GL.BindBuffer(BufferTarget.ArrayBuffer, thrustVBO);

            // Reserve space for dynamic thrust geometry (3 layers with more segments)
            GL.BufferData(BufferTarget.ArrayBuffer, 3000 * sizeof(float), IntPtr.Zero, BufferUsageHint.DynamicDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);
        }

        private void CreateTrajectoryGeometry()
        {
            trajectoryVAO = GL.GenVertexArray();
            trajectoryVBO = GL.GenBuffer();

            GL.BindVertexArray(trajectoryVAO);
            GL.BindBuffer(BufferTarget.ArrayBuffer, trajectoryVBO);

            // Reserve space for trajectory points
            GL.BufferData(BufferTarget.ArrayBuffer, MAX_TRAJECTORY_POINTS * 6 * sizeof(float),
                         IntPtr.Zero, BufferUsageHint.DynamicDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);
        }

    }
}
