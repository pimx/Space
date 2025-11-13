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
        private int shaderProgram;
        private int VAO, VBO, EBO;
        private int flameVAO, flameVBO;
        private int padVAO, padVBO, padEBO;
        private int thrustVAO, thrustVBO;

        // Flame particles
        private Random random = new Random();
        private float[] flameVertices;
        private int particleCount = 6000;
        private float time = 0;

        private Matrix4 view, projection;

        protected void OnLoadPicture()
        {
            GL.ClearColor(0.05f, 0.05f, 0.15f, 1.0f);
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

            CreateShaders();
            CreateTextShader();
            CreateRocketModel();
            CreateFlameParticles();
            CreateLandingPad();
            CreateThrustVisualization();
            CreateTextQuad();
        }

        protected void OnUnloadPicture()
        {
            GL.DeleteBuffer(VBO);
            GL.DeleteBuffer(EBO);
            GL.DeleteVertexArray(VAO);
            GL.DeleteBuffer(flameVBO);
            GL.DeleteVertexArray(flameVAO);
            GL.DeleteBuffer(thrustVBO);
            GL.DeleteVertexArray(thrustVAO);
            GL.DeleteBuffer(padVBO);
            GL.DeleteBuffer(padEBO);
            GL.DeleteVertexArray(padVAO);
            GL.DeleteBuffer(textVBO);
            GL.DeleteVertexArray(textVAO);
            GL.DeleteTexture(textTexture);
            GL.DeleteProgram(shaderProgram);
            GL.DeleteProgram(textShaderProgram);
        }

        private void CreateShaders()
        {
            string vertexShaderSource = @"
                #version 330 core
                layout (location = 0) in vec3 aPosition;
                layout (location = 1) in vec3 aNormal;
                layout (location = 2) in vec3 aColor;
                
                out vec3 FragPos;
                out vec3 Normal;
                out vec3 Color;
                
                uniform mat4 model;
                uniform mat4 view;
                uniform mat4 projection;
                
                void main()
                {
                    FragPos = vec3(model * vec4(aPosition, 1.0));
                    Normal = mat3(transpose(inverse(model))) * aNormal;
                    Color = aColor;
                    gl_Position = projection * view * vec4(FragPos, 1.0);
                }
            ";

            string fragmentShaderSource = @"
                #version 330 core
                in vec3 FragPos;
                in vec3 Normal;
                in vec3 Color;
                
                out vec4 FragColor;
                
                uniform vec3 lightPos;
                uniform vec3 viewPos;
                uniform vec3 lightColor;
                uniform float ambientStrength;
                
                void main()
                {
                    vec3 ambient = ambientStrength * lightColor;
                    vec3 norm = normalize(Normal);
                    vec3 lightDir = normalize(lightPos - FragPos);
                    float diff = max(dot(norm, lightDir), 0.0);
                    vec3 diffuse = diff * lightColor;
                    
                    float specularStrength = 0.5;
                    vec3 viewDir = normalize(viewPos - FragPos);
                    vec3 reflectDir = reflect(-lightDir, norm);
                    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
                    vec3 specular = specularStrength * spec * lightColor;
                    
                    vec3 result = (ambient + diffuse + specular) * Color;
                    FragColor = vec4(result, 1.0);
                }
            ";

            int vertexShader = GL.CreateShader(ShaderType.VertexShader);
            GL.ShaderSource(vertexShader, vertexShaderSource);
            GL.CompileShader(vertexShader);

            int fragmentShader = GL.CreateShader(ShaderType.FragmentShader);
            GL.ShaderSource(fragmentShader, fragmentShaderSource);
            GL.CompileShader(fragmentShader);

            shaderProgram = GL.CreateProgram();
            GL.AttachShader(shaderProgram, vertexShader);
            GL.AttachShader(shaderProgram, fragmentShader);
            GL.LinkProgram(shaderProgram);

            GL.DeleteShader(vertexShader);
            GL.DeleteShader(fragmentShader);
        }

        private void CreateRocketModel()
        {
            float[] vertices = GenerateRocketVertices();
            int[] indices = GenerateRocketIndices();

            VAO = GL.GenVertexArray();
            VBO = GL.GenBuffer();
            EBO = GL.GenBuffer();

            GL.BindVertexArray(VAO);

            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.StaticDraw);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(int), indices, BufferUsageHint.StaticDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.VertexAttribPointer(2, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 6 * sizeof(float));
            GL.EnableVertexAttribArray(2);

            GL.BindVertexArray(0);
        }

        private float[] GenerateRocketVertices()
        {
            int segments = 16;
            float radius = 5.0f;
            float height = 70.0f;
            var verticesList = new System.Collections.Generic.List<float>();

            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)i / segments * MathF.PI * 2;
                float x = MathF.Cos(angle) * radius;
                float z = MathF.Sin(angle) * radius;

                verticesList.AddRange(new float[] { x, 0, z, x, 0, z, 0.9f, 0.9f, 0.9f });
                verticesList.AddRange(new float[] { x, height, z, x, 0, z, 0.8f, 0.8f, 0.85f });
            }

            float noseHeight = 10.0f;
            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)i / segments * MathF.PI * 2;
                float x = MathF.Cos(angle) * radius;
                float z = MathF.Sin(angle) * radius;

                verticesList.AddRange(new float[] { x, height, z, 0, 1, 0, 0.1f, 0.1f, 0.1f });
                verticesList.AddRange(new float[] { 0, height + noseHeight, 0, 0, 1, 0, 0.1f, 0.1f, 0.1f });
            }

            float finHeight = height * 0.7f;
            float finSize = 8.0f;
            for (int f = 0; f < 4; f++)
            {
                float angle = f * MathF.PI / 2;
                float x = MathF.Cos(angle) * radius;
                float z = MathF.Sin(angle) * radius;
                float nx = MathF.Cos(angle);
                float nz = MathF.Sin(angle);

                verticesList.AddRange(new float[] { x, finHeight, z, nx, 0, nz, 0.2f, 0.2f, 0.2f });
                verticesList.AddRange(new float[] { x + nx * finSize, finHeight, z + nz * finSize, nx, 0, nz, 0.2f, 0.2f, 0.2f });
                verticesList.AddRange(new float[] { x + nx * finSize, finHeight + 5, z + nz * finSize, nx, 0, nz, 0.2f, 0.2f, 0.2f });
                verticesList.AddRange(new float[] { x, finHeight + 5, z, nx, 0, nz, 0.2f, 0.2f, 0.2f });
            }

            return verticesList.ToArray();
        }

        private int[] GenerateRocketIndices()
        {
            int segments = 16;
            var indicesList = new System.Collections.Generic.List<int>();

            for (int i = 0; i < segments; i++)
            {
                int bottom1 = i * 2;
                int top1 = i * 2 + 1;
                int bottom2 = (i + 1) * 2;
                int top2 = (i + 1) * 2 + 1;

                indicesList.AddRange(new int[] { bottom1, bottom2, top1 });
                indicesList.AddRange(new int[] { top1, bottom2, top2 });
            }

            int noseStart = (segments + 1) * 2;
            for (int i = 0; i < segments; i++)
            {
                int base1 = noseStart + i * 2;
                int tip = noseStart + i * 2 + 1;
                int base2 = noseStart + ((i + 1) % segments) * 2;

                indicesList.AddRange(new int[] { base1, base2, tip });
            }

            int finStart = noseStart + (segments + 1) * 2;
            for (int f = 0; f < 4; f++)
            {
                int offset = finStart + f * 4;
                indicesList.AddRange(new int[] { offset, offset + 1, offset + 2 });
                indicesList.AddRange(new int[] { offset, offset + 2, offset + 3 });
            }

            return indicesList.ToArray();
        }

        private void CreateFlameParticles()
        {
            flameVertices = new float[particleCount * 9];
            UpdateFlameParticles();

            flameVAO = GL.GenVertexArray();
            flameVBO = GL.GenBuffer();

            GL.BindVertexArray(flameVAO);
            GL.BindBuffer(BufferTarget.ArrayBuffer, flameVBO);
            GL.BufferData(BufferTarget.ArrayBuffer, flameVertices.Length * sizeof(float), flameVertices, BufferUsageHint.DynamicDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(2, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 6 * sizeof(float));
            GL.EnableVertexAttribArray(2);

            GL.BindVertexArray(0);
        }

        private void UpdateFlameParticles()
        {
            Vector3 thrustDir = thrustMagnitude > 0.01f ?
                Vector3.Normalize(thrustDirection) : new Vector3(0, 1, 0);

            const float gravity = 9.81f;
            float meanMagnitude = gravity * 27000.0f;
            float intensity = MathF.Min(thrustMagnitude / meanMagnitude, 2.0f);

            for (int i = 0; i < particleCount; i++)
            {
                float t = (float)i / particleCount;
                float radius = (2.0f + (float)random.NextDouble() * 2.0f); // * intensity;
                float angle = (float)random.NextDouble() * MathF.PI * 2 + time * 3;
                float length = t * 50.0f * intensity;
                float diff = length - (rocketPosition.Y - 0.5f);
                if (diff >= 0)
                {
                    radius += diff * 0.75f;
                    length = length - diff - intensity * 5.0f * (float)random.NextDouble();
                }

                // Create perpendicular vectors to thrust direction
                Vector3 perpendicular1 = Vector3.Cross(thrustDir, Vector3.UnitY);
                if (perpendicular1.Length < 0.1f)
                    perpendicular1 = Vector3.Cross(thrustDir, Vector3.UnitX);
                perpendicular1 = Vector3.Normalize(perpendicular1);
                Vector3 perpendicular2 = Vector3.Cross(thrustDir, perpendicular1);

                Vector3 offset = (MathF.Cos(angle) * perpendicular1 + MathF.Sin(angle) * perpendicular2) * radius * (1.0f - t * 0.25f);
                Vector3 particlePos = -thrustDir * length + offset;

                float r = 1.0f;
                float b = 1.0f - t * 0.1f;
                float g = 0.4f - t * 0.35f;
                float alpha = (1.05f - t) * intensity * 2;

                int idx = i * 9;
                flameVertices[idx] = particlePos.X;
                flameVertices[idx + 1] = particlePos.Y;
                flameVertices[idx + 2] = particlePos.Z;
                flameVertices[idx + 3] = thrustDir.X;
                flameVertices[idx + 4] = thrustDir.Y;
                flameVertices[idx + 5] = thrustDir.Z;
                flameVertices[idx + 6] = r * alpha;
                flameVertices[idx + 7] = g * alpha;
                flameVertices[idx + 8] = b * alpha;
            }
        }

        private void CreateThrustVisualization()
        {
            float[] vertices = new float[]
            {
                0, 0, 0,  0, 1, 0,  1, 1, 0,
                0, 20, 0,  0, 1, 0,  1, 0.5f, 0
            };

            thrustVAO = GL.GenVertexArray();
            thrustVBO = GL.GenBuffer();

            GL.BindVertexArray(thrustVAO);
            GL.BindBuffer(BufferTarget.ArrayBuffer, thrustVBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.DynamicDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(2, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 6 * sizeof(float));
            GL.EnableVertexAttribArray(2);

            GL.BindVertexArray(0);
        }

        private void CreateLandingPad()
        {
            int segments = 64;
            float radius = 40.0f;
            var verticesList = new System.Collections.Generic.List<float>();
            var indicesList = new System.Collections.Generic.List<int>();

            verticesList.AddRange(new float[] { 0, 0, 0, 0, 1, 0, 0.3f, 0.3f, 0.3f });

            for (int i = 0; i <= segments; i++)
            {
                float angle = (float)i / segments * MathF.PI * 2;
                float x = MathF.Cos(angle) * radius;
                float z = MathF.Sin(angle) * radius;
                float gray = (i % 4 < 2) ? 0.1f : 0.1f;
                float green = (i % 4 < 2) ? 0.4f : 0.3f;
                float blue = (i % 4 < 2) ? 0.1f : 0.1f;
                verticesList.AddRange(new float[] { x, 0, z, 0, 1, 0, gray, green, blue });
            }

            for (int i = 1; i <= segments; i++)
            {
                indicesList.AddRange(new int[] { 0, i, i + 1 });
            }

            float[] vertices = verticesList.ToArray();
            int[] indices = indicesList.ToArray();

            padVAO = GL.GenVertexArray();
            padVBO = GL.GenBuffer();
            padEBO = GL.GenBuffer();

            GL.BindVertexArray(padVAO);
            GL.BindBuffer(BufferTarget.ArrayBuffer, padVBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.StaticDraw);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, padEBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(int), indices, BufferUsageHint.StaticDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(2, 3, VertexAttribPointerType.Float, false, 9 * sizeof(float), 6 * sizeof(float));
            GL.EnableVertexAttribArray(2);

            GL.BindVertexArray(0);
        }
        protected override void OnRenderFrame(FrameEventArgs args)
        {
            base.OnRenderFrame(args);

            time += (float)args.Time;

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            view = Matrix4.LookAt(cameraPos, cameraTarget, Vector3.UnitY);
            projection = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.DegreesToRadians(30.0f),
                Size.X / (float)Size.Y,
                0.01f,
                1000.0f
            );

            GL.UseProgram(shaderProgram);

            int viewLoc = GL.GetUniformLocation(shaderProgram, "view");
            int projLoc = GL.GetUniformLocation(shaderProgram, "projection");
            int modelLoc = GL.GetUniformLocation(shaderProgram, "model");
            int lightPosLoc = GL.GetUniformLocation(shaderProgram, "lightPos");
            int viewPosLoc = GL.GetUniformLocation(shaderProgram, "viewPos");
            int lightColorLoc = GL.GetUniformLocation(shaderProgram, "lightColor");
            int ambientLoc = GL.GetUniformLocation(shaderProgram, "ambientStrength");

            GL.UniformMatrix4(viewLoc, false, ref view);
            GL.UniformMatrix4(projLoc, false, ref projection);

            Vector3 lightPos = new Vector3(100, 200, 100);
            GL.Uniform3(lightPosLoc, lightPos);
            GL.Uniform3(viewPosLoc, cameraPos);
            GL.Uniform3(lightColorLoc, new Vector3(1.0f, 0.95f, 0.9f));
            GL.Uniform1(ambientLoc, 0.3f);

            // Draw landing pad
            Matrix4 padModel = Matrix4.Identity;
            GL.UniformMatrix4(modelLoc, false, ref padModel);
            GL.BindVertexArray(padVAO);
            GL.DrawElements(PrimitiveType.Triangles, 64 * 3, DrawElementsType.UnsignedInt, 0);

            // Draw rocket with direction-based rotation
            Matrix4 rocketRotation = CreateRotationFromDirection(rocketDirection);
            Matrix4 rocketModel = rocketRotation * Matrix4.CreateTranslation(rocketPosition);

            GL.UniformMatrix4(modelLoc, false, ref rocketModel);
            GL.BindVertexArray(VAO);
            GL.DrawElements(PrimitiveType.Triangles, 300, DrawElementsType.UnsignedInt, 0);

            // Draw flame with thrust direction
            if (thrustMagnitude > 1000.0f)
            {
                UpdateFlameParticles();
                GL.BindBuffer(BufferTarget.ArrayBuffer, flameVBO);
                GL.BufferSubData(BufferTarget.ArrayBuffer, IntPtr.Zero, flameVertices.Length * sizeof(float), flameVertices);

                Matrix4 thrustRotation = CreateRotationFromDirection(thrustDirection);
                Matrix4 thrustModel = thrustRotation * Matrix4.CreateTranslation(rocketPosition);

                GL.UniformMatrix4(modelLoc, false, ref thrustModel);
                GL.BindVertexArray(flameVAO);
                GL.PointSize(3.0f);
                GL.DrawArrays(PrimitiveType.Points, 0, particleCount);
            }

            // Render and draw telemetry overlay
            RenderTelemetryText();
            DrawTelemetryOverlay();

            SwapBuffers();
        }
    }
}
