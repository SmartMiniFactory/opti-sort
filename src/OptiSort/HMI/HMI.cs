using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace HMI
{
    internal static class HMI
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new ucOptiSort());
        }

        internal class Cameras
        {
            public int ID { get; set; }
            public string Text { get; set; }

            public override string ToString() // Overriding ToString to display Text in the ComboBox
            {
                return Text;
            }
        }
    }
}
