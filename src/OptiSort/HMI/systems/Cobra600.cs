using Ace.Adept.Server.Controls;
using Ace.Adept.Server.Device;
using Ace.Adept.Server.Motion;
using Ace.Adept.Server.Motion.Robots;
using Ace.Core.Client;
using Ace.Core.Client.Sim3d;
using Ace.Core.Client.Sim3d.Controls;
using Ace.Core.Server;
using Ace.Core.Server.Device;
using Ace.Core.Server.Motion;
using Ace.Core.Util;
using ActiproSoftware.SyntaxEditor;
using HSCLASSLIBRARYLib;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{
    public class Cobra600
    {

        public string ServerIP {  get; set; }
        public string RobotIP { get; set; }

        public IAceServer Server { get; private set; }
        public IAceClient Client { get; private set; }
        public IAdeptController Controller { get; private set; }
        public IAdeptRobot Robot { get; private set; }
        public IAbstractEndEffector EndEffector { get; private set; }
        public SimulationContainerControl SimulationContainerControl { get; private set; }


        // TODO: should think about using singletones or properties. For the MQTT class properties are useful because config may change. But the ace server in theroy cannot change easily...Must standardize. Then think about how to handle property changes in general
        private string _remotingName;
        private int _remotingPort;

        private RemoteAceObjectEventHandler generalEventHandler;
        private RemoteApplicationEventHandler applicationEventHandler;
        private ControlPanelManager pendantManager;

        public Cobra600(string remotingName, string serverAddress, string remotingPort)
        {
            _remotingName = remotingName;
            ServerIP = serverAddress;
            _remotingPort = int.Parse(remotingPort);
        }


        public bool Connect(bool emulation, string controllerName, string robotName, string endEffectorName)
        {
            try
            {
                // Connect to ACE
                //Server = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), _remotingName, ServerIP, _remotingPort);
                Server = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), _remotingName, "127.0.0.1", _remotingPort);
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

                //Controller.Address = ServerIP;
                Controller.Address = RobotIP;

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

                Create3DDisplay();
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
                //generalEventHandler.Dispose();
                //generalEventHandler = null;
                //applicationEventHandler.Dispose();
                //applicationEventHandler = null;

                SimulationContainerControl.Dispose();
                SimulationContainerControl = null;

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
                GuiUtil.ShowExceptionDialog((IWin32Window)this, ex);
                return false;
            }
        }


        public void Create3DDisplay()
        {
            try
            {
                SimulationContainerControl = new SimulationContainerControl();
                SimulationContainerControl.Dock = DockStyle.Fill;
                SimulationContainerControl.Client = Client;
                SimulationContainerControl.Visible = false;
                SimulationContainerControl.Visible = true;
                var robotSimObject = SimulationContainerControl.AddToScene(Robot);
               
                Debug.Assert(robotSimObject != null, "Robot object was not added to the scene.");
                Debug.Assert(robotSimObject.Visible == true, "Robot not visible.");

                SimulationContainerControl.CameraPositions = new Transform3D[] { SimulationContainerControl.DefaultIsometricViewPosition };
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }


        public void OpenJobControl(IWin32Window form)
        {
            ControlPanelManager pendantManager = new ControlPanelManager();
            pendantManager.LaunchControlForm(form, Client, null);
        }

    }
}
