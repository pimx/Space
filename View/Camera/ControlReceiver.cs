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
    // Camera Control UDP Receiver
    public class CameraControlReceiver
    {
        private UdpClient udpClient;
        private Thread receiveThread;
        private bool isRunning;
        public event Action<CameraControl>? OnControlReceived;

        public CameraControlReceiver(int port)
        {
            udpClient = new UdpClient(port);
            isRunning = true;
            receiveThread = new Thread(ReceiveData);
            receiveThread.Start();
        }

        private void ReceiveData()
        {
            IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);

            while (isRunning)
            {
                try
                {
                    byte[] data = udpClient.Receive(ref remoteEP);
                    CameraControl control = ParseData(data);
                    OnControlReceived?.Invoke(control);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Camera UDP Receive error: {ex.Message}");
                }
            }
        }

        private CameraControl ParseData(byte[] data)
        {
            CameraControl control = new CameraControl();

            if (data.Length >= 8) // 2 floats minimum
            {
                int offset = 0;
                control.Angle = BitConverter.ToSingle(data, offset); offset += 4;
                control.Zoom = BitConverter.ToSingle(data, offset); offset += 4;

                // Optional elevation angle
                if (data.Length >= 12)
                {
                    control.Elevation = BitConverter.ToSingle(data, offset);
                }
                else
                {
                    control.Elevation = MathHelper.PiOver6; // Default 30 degrees elevation
                }

                // Clamp zoom to valid range
                control.Zoom = Math.Max(0.1f, Math.Min(10.0f, control.Zoom));
            }

            return control;
        }

        public void Stop()
        {
            isRunning = false;
            udpClient?.Close();
            receiveThread?.Join(1000);
        }
    }
}
