using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace Sender
{
    class SenderMain
    {
        static void Main(string[] args)
        {
            UdpClient client = new UdpClient();
            IPEndPoint endpoint = new IPEndPoint(IPAddress.Loopback, 5000);

            Console.WriteLine("Rocket UDP Test Client");
            Console.WriteLine("Sending data to localhost:5000");
            Console.WriteLine("Press Ctrl+C to exit\n");

            // Simulate a landing sequence
            float time = 0;

            while (true)
            {
                time += 0.1f;

                // Simulate descent and landing
                float altitude = Math.Max(0, 100 - time * 2);
                float x = (float)Math.Sin(time * 0.3) * 10; // Slight drift
                float z = (float)Math.Cos(time * 0.3) * 10;

                // Tilt correction during descent
                float pitch = altitude > 10 ? (float)Math.Sin(time) * 5 : 0;
                float yaw = (float)(time * 10) % 360;
                float roll = altitude > 10 ? (float)Math.Cos(time * 1.5) * 3 : 0;

                string message = $"{x:F2},{altitude:F2},{z:F2},{pitch:F2},{yaw:F2},{roll:F2}";
                byte[] data = Encoding.ASCII.GetBytes(message);

                try
                {
                    client.Send(data, data.Length, endpoint);
                    Console.WriteLine($"Sent: {message}");
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error: {ex.Message}");
                }

                Thread.Sleep(50); // 20 Hz update rate

                // Reset after landing
                if (altitude <= 0 && time > 60)
                    time = 0;
            }
        }
    }
}