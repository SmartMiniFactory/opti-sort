using System;
using Ace.Adept.Server.Controls;
using Ace.Core.Server;
using Ace.Core.Util;
using Ace.Adept.Server.Motion;
using Ace.Adept.Server.Motion.Robots;
using Ace.Core.Client;
using MQTTnet.Client;
using System.Text.Json;
using System.Security.Cryptography;
using System.Collections.Generic;
using System.Text.Json.Serialization;
using System.Linq;
using System.Net.Mail;
using MQTTnet;


namespace JointReader
{
    internal class Program
    {
        static string topicName = "DT_BROADCAST";
        static string brokerAddress = "10.12.238.20";
        static string cliendId = "OptiSort";
        static int brokerPort = 1883;

        public static string TopicName { get => topicName; set => topicName = value; }
        public static string BrokerAddress { get => brokerAddress; set => brokerAddress = value; }
        public static string CliendId { get => cliendId; set => cliendId = value; }
        public static int BrokerPort { get => brokerPort; set => brokerPort = value; }

        public static string CreateMessage(List<double> values)
        {
            if (values == null || values.Count != 4)
            {
                throw new ArgumentException("Input must be a list with exactly 4 elements.");
            }

            // Payload creation
            var payload = new Dictionary<string, double>
            {
                { "q1", values[0] },
                { "q2", values[1] },
                { "q3", values[2] },
                { "q4", values[3] },
            };
            // Create the main message
            var messageDict = new Dictionary<string, object>
            {
                { "sender", "SCARA" },
                { "receiver", "IPHYSICS" },
                { "command", 10 },
                { "payload", payload } // Nested payload dictionary

            };
            return JsonSerializer.Serialize(messageDict);
        }


        static void Main()
        {
            Console.WriteLine("Connecting to MQTT broker");

            // Initialize the MQTT connection
            var mqttFactory = new MQTTnet.MqttFactory();
            var mqttClient = mqttFactory.CreateMqttClient();
            var options = new MqttClientOptionsBuilder()
            .WithClientId(cliendId)
                .WithTcpServer(brokerAddress, brokerPort)
                .WithProtocolVersion(MQTTnet.Formatter.MqttProtocolVersion.V500)
                .WithCleanSession()
                .Build();

            // Connect to the MQTT broker
            mqttClient.ConnectAsync(options).Wait();
            Console.WriteLine($"Connected to {brokerAddress}");

            // Subscribe to the topic
            mqttClient.SubscribeAsync(topicName).Wait();
            Console.WriteLine($"Subscribed to {topicName}");

            // Initialize remoting infrastructure
            RemotingUtil.InitializeRemotingSubsystem(true, 0);

            // Get a reference to the ACE server
            IAceServer aceServer = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), "ace" , "127.0.0.1", 43434);
            Console.WriteLine("Connected to the ACE server");

            IAceClient aceClient = new AceClient(aceServer);
            Console.WriteLine("Connected to ACE client");

            aceClient.InitializePlugins(null);
            Console.WriteLine("Initialized ACE plugins");

            Console.WriteLine("Creating controller and robot in emulation mode");
            // Clear the workspace and put in emulation mode
            aceServer.Clear();
            aceServer.EmulationMode = true;

            // Create a controller and robot and establish the connection
            IAdeptController controller = aceServer.Root.AddObjectOfType(typeof(IAdeptController), "Controller") as IAdeptController;
            IAdeptRobot robot = aceServer.Root.AddObjectOfType(typeof(ICobra600), "Robot") as IAdeptRobot;
            robot.Controller = controller;
            controller.Enabled = true;
            Console.WriteLine("Controller and robot added to the workspace.");

            // Force the power on and the robot to calibrated
            controller.HighPower = true;
            controller.Calibrate();
            Console.WriteLine("Controller powered on and robot calibrated.");

            Console.WriteLine("Streaming joint positions over MQTT");

            while (true)
            {
                // Create a message with the joint positions
                string jsonMessage = CreateMessage(robot.JointPosition.ToList());

                // Create the MQTT message
                var message = new MqttApplicationMessageBuilder()
                    .WithTopic(topicName) // Replace with your topic
                    .WithPayload(jsonMessage)
                    .WithRetainFlag(false)
                    .Build();

                // Publish the message
                mqttClient.PublishAsync(message);
            }

        }
    }
}
