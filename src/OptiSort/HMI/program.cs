using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{
    internal static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            // Show the splash screen
            using (Splashscreen splash = new Splashscreen())
            {
                splash.Show();
                splash.Update();

                // Simulate a loading process
                System.Threading.Thread.Sleep(3000); // Replace with actual loading code
            }

            Application.Run(new frmMain());
        }        
    }
}
