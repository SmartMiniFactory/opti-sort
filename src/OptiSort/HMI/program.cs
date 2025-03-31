using System;
using System.Collections.Generic;
using System.Diagnostics;
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

            frmMain mainForm = new frmMain();
            AppDomain.CurrentDomain.ProcessExit += (s, e) => mainForm.manager.KillAllProcesses(); // Ensure cleanup if the application crashes
            
            Application.Run(mainForm);
        }

    }
}
