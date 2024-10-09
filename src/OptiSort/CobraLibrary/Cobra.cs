using Ace.Adept.Server.Controls;
using Ace.Adept.Server.Device;
using Ace.Adept.Server.Motion.Robots;
using Ace.Adept.Server.Motion;
using Ace.Core.Client;
using Ace.Core.Server.Device;
using Ace.Core.Server;
using Ace.Core.Util;
using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using Ace.Adept.Server.Desktop.Connection;
using Ace.Core.Server.Motion;
using Ace.Core.Server.Event;
using Ace.Core.Client.Sim3d.Controls;
using System.Reflection;
using System.Threading.Tasks;

namespace CobraLibrary
{
    public partial class Cobra
    {

        const int _callbackPort = 43431; 
        const int _remotingPort = 43434;
        const string _endEffectorName = "Suction Cup";
        const string _remotingName = "ace";
        const string _controllerName = "OptiSort";

        const string _robotIP = "10.90.90.60";
        const string _controllerIP = "127.0.0.1";

        const bool _newWorkspace = true;

        private IAceServer aceServer;
        private IAceClient aceClient;
        static private IAdeptController controller;
        static private IAdeptRobot robot;
        static private IAbstractEndEffector endEffector;

        private RemoteAceObjectEventHandler generalEventHandler;
        private RemoteApplicationEventHandler applicationEventHandler;
        private SimulationContainerControl simulationControl;
        private ControlPanelManager pendantManager;


        public (IAdeptController controller, IAdeptRobot robot, IAceServer aceServer, IAceClient aceClient) Connect(bool emulate)
        {
            // Connect to ACE
            aceServer = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), _remotingName, "localhost", _remotingPort);
            aceClient = new AceClient(aceServer);
            aceClient.InitializePlugins(null);

            if (_newWorkspace)
                aceServer.Clear(); // Clear the workspace

            aceServer.EmulationMode = emulate;

            // Get the available controllers
            IList<IAceObject> availableControllers = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IAdeptController)), true);
            if (availableControllers.Count == 0)
            {
                Console.WriteLine("No controllers available.\nCreating a new one.");
                controller = aceServer.Root.AddObjectOfType(typeof(IAdeptController), _controllerName) as IAdeptController; // create and connect
            }
            else
            {
                Console.WriteLine("Connecting to {0}", availableControllers.FirstOrDefault());
                controller = availableControllers.FirstOrDefault() as IAdeptController; // connect to first available
            }

            // Get the available robots
            IList<IAceObject> availableRobots = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IAdeptRobot)), true);
            if (availableRobots.Count == 0)
            {
                Console.WriteLine("No robots available.\nCreating a new one.");
                robot = aceServer.Root.AddObjectOfType(typeof(ICobra600), "R1 Cobra600") as IAdeptRobot; // create and connect
            }
            else
            {
                Console.WriteLine("Connecting to {0}", availableRobots.FirstOrDefault());
                robot = availableRobots.FirstOrDefault() as IAdeptRobot; // connect to first available
            }

            // Configure the robot and controller IP addresses
            controller.Address = _robotIP;
            robot.Controller = controller;

            // Get the available end-effectors
            IList<IAceObject> availableEndEffectors = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IIODrivenEndEffector)), true);
            if (availableEndEffectors.Count == 0)
            {
                Console.WriteLine("No End-effectors available.\nCreating a new one.");
                endEffector = aceServer.Root.AddObjectOfType(typeof(IIODrivenEndEffector), _endEffectorName) as IAbstractEndEffector; // create object
            }
            else
            {
                Console.WriteLine("Connecting to {0}", availableEndEffectors.FirstOrDefault());
                endEffector = availableEndEffectors.FirstOrDefault() as IIODrivenEndEffector; // use first available
            }

            // Associate the end-effector with the robot
            robot.SelectedEndEffector = endEffector;
            robot.EndEffectorGripSignal = 98;
            robot.EndEffectorReleaseSignal = 97;


            controller.Enabled = true; // Enable the controller


            // If not calibrated, calibrate the robot
            try
            {
                if (robot.IsCalibrated == false)
                {
                    robot.Power = true;
                    controller.HighPower = true;
                    Console.WriteLine("Enabling power to {0}. Please press the button on the front panel.", robot);
                    robot.Calibrate();
                    controller.Calibrate();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return (null, null, null, null);
            }

            return (controller, robot, aceServer, aceClient);
        }


        public void Disconnect(IAdeptController controller, IAceServer server)
        {
            try
            {
                //ChangeControlState(false);
                //Remove3DDisplay();
                //ClosePendant();

                // Release all event handlers before we clear the workspace.				
                generalEventHandler.Dispose();
                generalEventHandler = null;
                applicationEventHandler.Dispose();
                applicationEventHandler = null;

                // Clear the workspace
                controller.Enabled = false;
                server.Clear();
                server = null;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                //GuiUtil.ShowExceptionDialog(this, ex);
            }
        }


        // Handle robot's motion
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

        // Handles the execution of external V+ programs
        public class Program
        {
            private ControlPanelManager pendantManager;

            bool _programRunning = false;

            public static IVPlusModuleProgram Load(IAceServer ace, IAdeptController controller, IAdeptRobot robot, string _programFile)
            {
                // Replace with the actual path to your text file
                Console.WriteLine("Loading < " + _programFile + " >");
                string filePath = @"C:\Users\DGalli\source\repos\CobraRemote\" + _programFile;

                // Read the contents of the file into a string
                string[] programLines = File.ReadAllLines(filePath);

                // Create a V+ program module
                Console.WriteLine("Creating the V+ module");
                IVPlusModule module = ace.CreateObject(typeof(IVPlusModule)) as IVPlusModule;
                module.ObjectLink = controller;

                // See if the program already exists in any module in the workspace
                bool alreadyExists = module.DoesProgramExist(_programFile);
                if (alreadyExists == true)
                    controller.Link.DeleteP(_programFile);

                // Create a V+ program
                IVPlusModuleProgram program = ace.CreateObject(typeof(IVPlusModuleProgram), new object[] { programLines }) as IVPlusModuleProgram;

                // Add to the module
                Console.WriteLine("Adding the V+ program ");
                module.AddProgram(program);

                // Make the program the module program for the module
                module.ModuleName = program.ProgramName;

                // Push the module to the controller
                controller.Memory.Synchronize(module);

                // The module is now in controller memory.             
                module.Dispose();
                module = controller.Memory.FindModule(program.ProgramName);

                return program;
            }
            public static bool Execute(IAceServer ace, IAdeptController controller, IAdeptRobot robot, IVPlusModuleProgram program, int VPlusRobotTask)
            {
                // Create a link between the controller and the program
                IVpLink link = controller.Link;

                // Execute the program on a task and do not wait until execution completes
                Console.WriteLine("Executing program < ", program.ProgramName, " > on task < ", VPlusRobotTask, " >");
                link.Execute(program.ProgramName, VPlusRobotTask, 0);

                return true;
            }
            public static bool Kill(IAdeptController controller, int VPlusRobotTask)
            {
                // Create a link between the controller and the program
                IVpLink link = controller.Link;

                // Execute the program on a task and do not wait until execution completes
                Console.WriteLine("Killing program task < ", VPlusRobotTask, " >");
                link.Kill(VPlusRobotTask);

                return false;
            }
        }
    }
}
