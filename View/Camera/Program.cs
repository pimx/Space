namespace First
{
    internal class Program
    {
        static void Main(string[] args)
        {
            using (var window = new RocketVisualizationWindow())
            {
                window.Run(); // Run(30.0); // Run at 30 FPS
            }
        }
    }
}
