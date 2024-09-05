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

namespace CobraLibrary
{
    public class Cobra
    {
        // Hanlde robot's intialisation
        public class Init
        {
            const int _callbackPort = 43431;
            const int _remotingPort = 43434;

            const string RobotIP = "10.90.90.60";
            const string ControllerIP = "127.0.0.1";

            const bool _emulation = true;
            const bool _newWorkspace = true;

            static private IAceServer aceServer;
            static private IAceClient aceClient;
            static private IAdeptController controller;
            static private IAdeptRobot robot;
            static private IAbstractEndEffector endEffector;

            public static (IAdeptController, IAdeptRobot, IAceServer) Connect(string _controllerIP, string _controllerName, string _endEffectorName = "Suction Cup", string _remotingName = "ace")
            {
                // Initialize remoting infrastructure
                RemotingUtil.InitializeRemotingSubsystem(true, _callbackPort);

                // Connect to ACE
                IAceServer aceServer = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), _remotingName, _controllerIP, _remotingPort);
                Console.WriteLine("Connected to server");

                IAceClient aceClient = new AceClient(aceServer);
                Console.WriteLine("Connected to client");

                aceClient.InitializePlugins(null);

                if (_newWorkspace)
                {
                    // Clear the workspace
                    aceServer.Clear();
                }

                aceServer.EmulationMode = _emulation;

                // Get the available controllers
                IList<IAceObject> availableControllers = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IAdeptController)), true);
                if (availableControllers.Count == 0)
                {
                    // Handle the case when no controllers are available
                    Console.WriteLine("No controllers available.\nCreating a new one.");

                    // Create a controller and robot and establish the connection
                    controller = aceServer.Root.AddObjectOfType(typeof(IAdeptController), _controllerName) as IAdeptController;

                }
                else
                {
                    // Connect to the first available controller
                    Console.WriteLine("Connecting to {0}", availableControllers.FirstOrDefault());
                    controller = availableControllers.FirstOrDefault() as IAdeptController;
                }

                // Get the available robots
                IList<IAceObject> availableRobots = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IAdeptRobot)), true);
                if (availableRobots.Count == 0)
                {
                    // Handle the case when no controllers are available
                    Console.WriteLine("No robots available.\nCreating a new one.");

                    // Create a controller and robot and establish the connection
                    robot = aceServer.Root.AddObjectOfType(typeof(ICobra600), "R1 Cobra600") as IAdeptRobot;
                }
                else
                {
                    // Connect to the first available robot
                    Console.WriteLine("Connecting to {0}", availableRobots.FirstOrDefault());
                    robot = availableRobots.FirstOrDefault() as IAdeptRobot;
                }

                // Configure the robot and controller IP addresses
                controller.Address = RobotIP;
                robot.Controller = controller;

                // Get the available end-effectors
                IList<IAceObject> availableEndEffectors = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IIODrivenEndEffector)), true);
                if (availableEndEffectors.Count == 0)
                {
                    // Handle the case when no controllers are available
                    Console.WriteLine("No End-effectors available.\nCreating a new one.");

                    // Create an end-effector object
                    endEffector = aceServer.Root.AddObjectOfType(typeof(IIODrivenEndEffector), _endEffectorName) as IAbstractEndEffector;
                }
                else
                {
                    // Connect to the first available end-effector
                    Console.WriteLine("Connecting to {0}", availableEndEffectors.FirstOrDefault());
                    endEffector = availableEndEffectors.FirstOrDefault() as IIODrivenEndEffector;
                }

                // Associate the end-effector with the robot
                robot.SelectedEndEffector = endEffector;
                robot.EndEffectorGripSignal = 98;
                robot.EndEffectorReleaseSignal = 97;

                // Enable the controller
                controller.Enabled = true;

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
                }

                return (controller, robot, aceServer);
            }
            public static void Disconnet(IAdeptController controller, IAdeptRobot robot)
            {
                Console.WriteLine("Shutting down {0}", robot);
                robot.Power = false;
                Console.WriteLine("Disconnecting from {0}", controller);
                controller.Enabled = false;
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
