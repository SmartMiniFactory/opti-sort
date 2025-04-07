using MQTTnet.Protocol;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;

namespace OptiSort.Classes
{
    internal class DigitalTwin
    {
        private optisort_mgr _manager;
        private Thread _digitalTwinThread;
        private MQTT _mqttClient;
        private string _clientId = "OptisortDT";
        private string _broker = "10.12.238.20";
        private string _mqttTopic = "DT_BROADCAST";
        private string _mqttPort;
        private Cobra600 _scaraRobot;
        private System.Timers.Timer _timer;

        public DigitalTwin(optisort_mgr manager, MQTT mqttClient, string mqttPort, Cobra600 scaraRobot)
        {
            _manager = manager;

            _mqttPort = mqttPort;
            _mqttClient = mqttClient ?? throw new ArgumentNullException(nameof(mqttClient));
            _scaraRobot = scaraRobot ?? throw new ArgumentNullException(nameof(scaraRobot));

            _timer = new System.Timers.Timer(100); // publish each 0.1 seconds
            _timer.Elapsed += OnTimerElapsed;
            _timer.AutoReset = true;
        }

        public void Start()
        {
            _digitalTwinThread = new Thread(new ThreadStart(RunDigitalTwin));
            _digitalTwinThread.Start();
            _manager.Log("Digital Twin: Thread started", false, true);
        }

        private async void RunDigitalTwin()
        {
            Task<bool> success = _mqttClient.CreateClient(_clientId, _broker, _mqttPort);
            if (!await success)
            {
                _manager.Log("Digital Twin: MQTT client creation failed", true, false);
                return;
            }
            _timer.Start();
        }

        private void OnTimerElapsed(object sender, ElapsedEventArgs e)
        {

            var joints = Cobra600.Motion.GetJointPositions(_scaraRobot.Robot);

            var message = new
            {
                sender = "SCARA",
                receiver = "IPHYSICS",
                command = 10,
                payload = new
                {
                    q1 = joints[0],
                    q2 = joints[1],
                    q3 = joints[2],
                    q4 = joints[3]
                }
            };

            _ = _mqttClient.PublishMessage(_clientId, _mqttTopic, message);
        }

        public void Stop()
        {
            // Stop the timer and the thread
            _timer.Stop();
            if (_digitalTwinThread.IsAlive)
            {
                _digitalTwinThread.Join();
                _ = _mqttClient.DestroyClient(_clientId);
                _manager.Log("Digital Twin: Thread stopped", false, true);
            }
        }
    }
}
