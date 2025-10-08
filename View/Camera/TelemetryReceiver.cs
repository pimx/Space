using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace First
{
    // UDP Receiver class
    public class TelemetryReceiver
    {
        public RocketTelemetry currentTelemetry;

        private UdpClient udpClient;
        private Thread simulateThread;
        private Thread receiveThread;
        private bool isRunning;
        public event Action<RocketTelemetry> OnDataReceived;
        int udpPort = 0;

        public TelemetryReceiver(int port)
        {
            udpPort = port;
            udpClient = new UdpClient(port);
            isRunning = true;
            receiveThread = new Thread(ReceiveData);
            receiveThread.Start();
            simulateThread = new Thread(SimulateData);
            //simulateThread.Start();
        }

        private void SimulateData()
        {
            IPEndPoint remoteEP = new IPEndPoint(IPAddress.Loopback, udpPort);
            float alt = 10.0f;
            float thr = 1.0f;
            while (isRunning)
            {
                try
                {
                    RocketTelemetry rt = new RocketTelemetry();
                    rt.ThrustMagnitude = thr;
                    rt.Position.X = alt;
                    rt.Position.Y = alt;
                    rt.Position.Z = alt;
                    byte[] data = new byte[256];
                    rt.Marshal(data);
                    int n = udpClient.Send(data, 256, remoteEP);
                    alt -= 0.01f;
                    thr -= 0.001f;
                    Thread.Sleep(100);
                    //RocketTelemetry telemetry = ParseData(data);
                    //OnDataReceived?.Invoke(telemetry);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"UDP Receive error: {ex.Message}");
                }
            }
        }

        public Matrix4 ModelMatrix()
        {
            Matrix4 rocketModelMatrix;
            //lock (telemetryLock)
            {
                rocketModelMatrix = Matrix4.CreateRotationX(currentTelemetry.Angles.X) *
                                   Matrix4.CreateRotationY(currentTelemetry.Angles.Y) *
                                   Matrix4.CreateRotationZ(currentTelemetry.Angles.Z) *
                                   Matrix4.CreateTranslation(currentTelemetry.Position);
            }
            return rocketModelMatrix;
        }
        private void ReceiveData()
        {
            IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);

            while (isRunning)
            {
                try
                {
                    byte[] data = udpClient.Receive(ref remoteEP);
                    RocketTelemetry rt = new RocketTelemetry();
                    rt.Unmarshal(data);
                    currentTelemetry = rt;
                    OnDataReceived?.Invoke(rt);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"UDP Receive error: {ex.Message}");
                }
            }
        }

        private RocketTelemetry ParseData(byte[] data)
        {
            // Parse UDP packet - adjust format as needed
            // Expected format: x,y,z,roll,pitch,yaw,thrust_mag,thrust_x,thrust_y,thrust_z,timestamp
            RocketTelemetry telemetry = new RocketTelemetry();

            if (data.Length >= 48) // 10 floats * 4 bytes + 1 double * 8 bytes
            {
                int offset = 0;
                telemetry.Position.X = BitConverter.ToSingle(data, offset); offset += 4;
                telemetry.Position.Y = BitConverter.ToSingle(data, offset); offset += 4;
                telemetry.Position.Z = BitConverter.ToSingle(data, offset); offset += 4;
                telemetry.Angles.X = BitConverter.ToSingle(data, offset); offset += 4;  // Roll
                telemetry.Angles.Y = BitConverter.ToSingle(data, offset); offset += 4;  // Pitch
                telemetry.Angles.Z = BitConverter.ToSingle(data, offset); offset += 4;  // Yaw
                telemetry.ThrustMagnitude = BitConverter.ToSingle(data, offset); offset += 4;
                telemetry.ThrustVector.X = BitConverter.ToSingle(data, offset); offset += 4;
                telemetry.ThrustVector.Y = BitConverter.ToSingle(data, offset); offset += 4;
                telemetry.ThrustVector.Z = BitConverter.ToSingle(data, offset); offset += 4;
                telemetry.Timestamp = BitConverter.ToDouble(data, offset); // Double precision timestamp
            }

            return telemetry;
        }

        public void Stop()
        {
            isRunning = false;
            udpClient?.Close();
            receiveThread?.Join(1000);
        }
    }
}
