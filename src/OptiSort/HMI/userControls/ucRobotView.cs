using Ace.Adept.Server.Motion;
using Ace.Core.Client.Sim3d.Controls;
using Ace.Core.Client;
using Ace.Core.Server;
using CobraLibrary;
using OptiSort;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Net.Security;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{
    public partial class ucRobotView : UserControl
    {
        public Cobra Cobra600 { get; set; }
        private frmMain _frmMain;
        private SimulationContainerControl _simulationControl;

        public ucRobotView(frmMain frmMain)
        {
            InitializeComponent();
        }

        private void ucRobotView_Load(object sender, EventArgs e)
        {
            using (Graphics g = pnl3D.CreateGraphics())
            {
                g.Clear(Color.Black);
                g.DrawLine(new Pen(Color.Red, 10), pnl3D.Width / 3, pnl3D.Height / 3, pnl3D.Width * 2 / 3, pnl3D.Height * 2 / 3);
                g.DrawLine(new Pen(Color.Red, 10), pnl3D.Width / 3, pnl3D.Height * 2 / 3, pnl3D.Width * 2 / 3, pnl3D.Height / 3);
            }
        }

        public void Create3DDisplay(IAceClient aceClient, IAdeptRobot robot)
        {

            _simulationControl = new SimulationContainerControl();
            _simulationControl.Dock = DockStyle.Fill;
            pnl3D.Controls.Add(_simulationControl);

            _simulationControl.Client = aceClient;
            _simulationControl.Visible = false;
            _simulationControl.Visible = true;
            _simulationControl.AddToScene(robot);
            _simulationControl.CameraPositions = new Transform3D[] { _simulationControl.DefaultIsometricViewPosition };
        }
    }
}
