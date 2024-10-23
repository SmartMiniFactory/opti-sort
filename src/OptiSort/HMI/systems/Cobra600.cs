using Ace.Adept.Server.Controls;
using Ace.Adept.Server.Device;
using Ace.Adept.Server.Motion;
using Ace.Adept.Server.Motion.Robots;
using Ace.Core.Client;
using Ace.Core.Client.Sim3d.Controls;
using Ace.Core.Server;
using Ace.Core.Server.Device;
using Ace.Core.Server.Motion;
using Ace.Core.Util;
using ActiproSoftware.SyntaxEditor;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{
    public class Cobra600
    {
        public IAceServer Server { get; private set; }
        public IAceClient Client { get; private set; }
        public IAdeptController Controller { get; private set; }
        public IAdeptRobot Robot { get; private set; }
        public IAbstractEndEffector EndEffector { get; private set; }
        public SimulationContainerControl SimulationControl { get; private set; }

        // TODO: useful??
        private string _remotingName;
        private string _address;
        private int _remotingPort;

        private RemoteAceObjectEventHandler _generalEventHandler;
        private RemoteApplicationEventHandler _applicationEventHandler;
        private ControlPanelManager _pendantManager;

        public Cobra600(string remotingName, string address, int remotingPort)
        {
            _remotingName = remotingName;
            _address = address;
            _remotingPort = remotingPort;
        }


        public bool Connect(bool emulation, string controllerName, string robotName, string endEffectorName)
        {
            try
            {
                // Connect to ACE
                Server = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), _remotingName, _address, _remotingPort);
                Client = new AceClient(Server);
                Client.InitializePlugins(null);

                // Clear workspace
                Server.Clear();

                Server.EmulationMode = emulation;


                // Get the available controllers
                IList<IAceObject> availableControllers = Server.Root.Filter(new ObjectTypeFilter(typeof(IAdeptController)), true);
                if (availableControllers.Count == 0)
                {
                    Console.WriteLine("No controllers available.\nCreating a new one.");
                    Controller = Server.Root.AddObjectOfType(typeof(IAdeptController), controllerName) as IAdeptController; // create and connect
                }
                else
                {
                    Console.WriteLine("Connecting to {0}", availableControllers.FirstOrDefault());
                    Controller = availableControllers.FirstOrDefault() as IAdeptController; // connect to first available
                }

                Controller.Address = _address;

                // Get the available robots
                IList<IAceObject> availableRobots = Server.Root.Filter(new ObjectTypeFilter(typeof(IAdeptRobot)), true);
                if (availableRobots.Count == 0)
                {
                    Console.WriteLine("No robots available.\nCreating a new one.");
                    Robot = Server.Root.AddObjectOfType(typeof(ICobra600), robotName) as IAdeptRobot; // create and connect
                }
                else
                {
                    Console.WriteLine("Connecting to {0}", availableRobots.FirstOrDefault());
                    Robot = availableRobots.FirstOrDefault() as IAdeptRobot; // connect to first available
                }

                Robot.Controller = Controller;

                // Get the available end-effectors
                IList<IAceObject> availableEndEffectors = Server.Root.Filter(new ObjectTypeFilter(typeof(IIODrivenEndEffector)), true);
                if (availableEndEffectors.Count == 0)
                {
                    Console.WriteLine("No End-effectors available.\nCreating a new one.");
                    EndEffector = Server.Root.AddObjectOfType(typeof(IIODrivenEndEffector), endEffectorName) as IAbstractEndEffector; // create object
                }
                else
                {
                    Console.WriteLine("Connecting to {0}", availableEndEffectors.FirstOrDefault());
                    EndEffector = availableEndEffectors.FirstOrDefault() as IIODrivenEndEffector; // use first available
                }

                Robot.SelectedEndEffector = EndEffector;
                Robot.EndEffectorGripSignal = 98;
                Robot.EndEffectorReleaseSignal = 97;


                Controller.Enabled = true;


                // If not calibrated, calibrate the robot

                if (Robot.IsCalibrated == false)
                {
                    Robot.Power = true;
                    Controller.HighPower = true;
                    Console.WriteLine("Enabling power to {0}. Please press the button on the front panel.", Robot);
                    Robot.Calibrate();
                    Controller.Calibrate();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return false;
            }
            return true;
        }



        public bool Disconnect()
        {
            try
            {
                // Release all event handlers before we clear the workspace.				
                _generalEventHandler.Dispose();
                _generalEventHandler = null;
                _applicationEventHandler.Dispose();
                _applicationEventHandler = null;

                // Clear the workspace
                Controller.Enabled = false;
                Server.Clear();
                Server = null;
                Robot = null;
                Controller = null;
                EndEffector = null;
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                //GuiUtil.ShowExceptionDialog(this, ex);
                return false;
            }
        }



        public bool Create3DDisplay()
        {
            try
            {
                SimulationControl = new SimulationContainerControl();
                SimulationControl.Dock = DockStyle.Fill;
                SimulationControl.Client = Client;
                SimulationControl.Visible = false;
                SimulationControl.Visible = true;
                SimulationControl.AddToScene(Robot);
                SimulationControl.CameraPositions = new Transform3D[] { SimulationControl.DefaultIsometricViewPosition };
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return false;

            }
        }



    }
}
