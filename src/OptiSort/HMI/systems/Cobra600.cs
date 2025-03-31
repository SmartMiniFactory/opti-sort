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
        public string RobotIP { get; set; }
        public IAceServer Server { get; private set; }
        public IAceClient Client { get; private set; }
        public IAdeptController Controller { get; private set; }
        public IAdeptRobot Robot { get; private set; }
        public IAbstractEndEffector EndEffector { get; private set; }
        public SimulationContainerControl SimulationContainerControl { get; private set; }
        //private ControlPanelManager PendantManager { get; set; }


        // TODO: should think about using singletones or properties. For the MQTT class properties are useful because config may change. But the ace server in theroy cannot change easily...Must standardize. Then think about how to handle property changes in general
        private string _remotingName;
        private int _remotingPort;
        private ControlPanelManager pendantManager;

        //private RemoteAceObjectEventHandler generalEventHandler;
        //private RemoteApplicationEventHandler applicationEventHandler;

        public Cobra600(string remotingName, string remotingPort)
        {
            _remotingName = remotingName;
            _remotingPort = int.Parse(remotingPort);
        }


        public Exception Connect(bool emulation, string controllerName, string robotName, string endEffectorName)
        {
            try
            {
                // Connect to ACE
                Server = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), _remotingName, "127.0.0.1", _remotingPort);
                Client = new AceClient(Server);
                Client.InitializePlugins(null); // WARNING: For some reason, this makes the connection to the robot fail at first. While at the second try it works. Cannot be removed otherwise manual control does not work.
                

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

                if (emulation)
                    Controller.Address = "127.0.0.1";
                else
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
                return ex;
            }
            return null;
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
            pendantManager = new ControlPanelManager();
            pendantManager.LaunchControlForm(form, Client, null);
        }


        public void ToggleDigitalOutput(int channel)
        {
            try
            {
                bool ioStatus = Controller.GetDigitalIO(channel);

                if(ioStatus)
                    Controller.SetDigitalIO(-channel);
                else
                    Controller.SetDigitalIO(channel);

                bool newStatus = Controller.GetDigitalIO(channel);

                Console.WriteLine($"Digital output channel {channel} set to {newStatus}");
            }
            catch (Exception)
            {
                Trace.WriteLine("Failed to access to the robot I/O.");
            }
        }

        public bool getDigitalOutput(int channel)
        {
            try
            {
                bool status = Controller.GetDigitalIO(channel);
                return status;
            }
            catch (Exception)
            {
                Trace.WriteLine("Failed to access to the robot I/O.");
                return false;
            }
        }

        public class Motion
        {
            public static double[] GetJointPositions(IAdeptRobot robot)
            {
                // Initialise an array to store the joint positions
                double[] jointPositions = robot.JointPosition;

                return jointPositions;
            }
            public static double[] GetRobotPosition(IAdeptRobot robot)
            {
                // Get the current joint positions
                double[] jointPositions = robot.JointPosition;

                // Convert them into world coordinates
                Transform3D loc = robot.JointToWorld(jointPositions);

                double[] robotPosition = { Math.Round(loc.DX, 3), Math.Round(loc.DY, 3), Math.Round(loc.DZ, 3) };

                return robotPosition;
            }
            public static void CartesianMove(IAceServer _ace, IAdeptRobot _robot, Transform3D _destinationLoc, bool _linear, int _speed = 50, int _accel = 100, int _decel = 100)
            {
                // Make sure the robot controller communications is enabled
                if (_robot.IsAlive == false)
                    throw new InvalidOperationException();

                // Make sure power is enabled
                if (_robot.Power == false)
                    _robot.Power = true;

                // Make sure the robot is calibrated
                if (_robot.IsCalibrated == false)
                    _robot.Calibrate();

                // Calculate a position close to the current position of the robot
                int inRange = _robot.InRange(_destinationLoc);

                if (inRange == 0)
                {
                    // Create a cartesian move command for the robot
                    CartesianMove cartesianMove = _ace.CreateObject(typeof(CartesianMove)) as CartesianMove;
                    cartesianMove.Robot = _robot;
                    cartesianMove.WorldLocation = CoordinateUtils.RobotToWorkspace(_destinationLoc, _robot);
                    cartesianMove.Param.Accel = _accel;
                    cartesianMove.Param.Decel = _decel;
                    cartesianMove.Param.Speed = _speed;
                    cartesianMove.Param.Straight = _linear;
                    cartesianMove.Param.MotionEnd = MotionEnd.Blend;
                    cartesianMove.Param.SCurveProfile = 0;

                    // Issue the motion and wait until the motion is done
                    _robot.Move(cartesianMove);
                    _robot.WaitMoveDone();
                    cartesianMove.Dispose();
                }
                else
                {
                    throw new InvalidOperationException("Destination location is out of range");
                }
            }
            public static void Approach(IAceServer _ace, IAdeptRobot _robot, Transform3D _destinationLoc, double _approDist, int _speed = 50, int _accel = 100, int _decel = 100)
            {
                // Make sure the robot controller communications is enabled
                if (_robot.IsAlive == false)
                    throw new InvalidOperationException();

                // Make sure power is enabled
                if (_robot.Power == false)
                    _robot.Power = true;

                // Make sure the robot is calibrated
                if (_robot.IsCalibrated == false)
                    _robot.Calibrate();

                // Shift the destination location up by the approach distance
                _destinationLoc = _destinationLoc.Shift(0, 0, _approDist);

                // Calculate a position close to the current position of the robot
                int inRange = _robot.InRange(_destinationLoc);

                if (inRange == 0)
                {
                    // Create a cartesian move command for the robot
                    CartesianMove cartesianMove = _ace.CreateObject(typeof(CartesianMove)) as CartesianMove;
                    cartesianMove.Robot = _robot;
                    cartesianMove.WorldLocation = CoordinateUtils.RobotToWorkspace(_destinationLoc, _robot);
                    cartesianMove.Param.Accel = _accel;
                    cartesianMove.Param.Decel = _decel;
                    cartesianMove.Param.Speed = _speed;
                    cartesianMove.Param.Straight = false;
                    cartesianMove.Param.MotionEnd = MotionEnd.Blend;
                    cartesianMove.Param.SCurveProfile = 0;

                    // Issue the motion and wait until the motion is done
                    _robot.Move(cartesianMove);
                    _robot.WaitMoveDone();
                    cartesianMove.Dispose();
                }
                else
                {
                    throw new InvalidOperationException("Destination location is out of range");
                }
            }
        }


    }
}
