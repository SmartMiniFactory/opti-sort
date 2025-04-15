using Newtonsoft.Json;
using System;
using System.Net.Http;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Timers;
using WebSocketSharp;

namespace OptiSort.Classes
{
    internal class DigitalTwin
    {
        private optisort_mgr _manager;
        private Thread _digitalTwinThread;
        private Cobra600 _scaraRobot;
        private System.Timers.Timer _timer;

        private WebSocketSharp.WebSocket ws;

        public DigitalTwin(optisort_mgr manager, Cobra600 scaraRobot)
        {
            _manager = manager;
            _scaraRobot = scaraRobot ?? throw new ArgumentNullException(nameof(scaraRobot));

            _timer = new System.Timers.Timer(100); // publish each 0.1 seconds
            _timer.Elapsed += OnTimerElapsed;
            _timer.AutoReset = true;

            ConnectWebSocket();
        }


        private void ConnectWebSocket()
        {
            ws = new WebSocketSharp.WebSocket("ws://192.168.10.145:8180");

            ws.OnOpen += (s, e) =>
            {
                _manager.Log("✅ WebSocket connesso.", false, true);
            };

            ws.OnError += (s, e) =>
            {
                _manager.Log("❌ Errore WebSocket: " + e.Message, true, false);
            };

            ws.Connect();
        }

        public void Start()
        {
            _digitalTwinThread = new Thread(new ThreadStart(RunDigitalTwin));
            _digitalTwinThread.Start();
            _manager.Log("Digital Twin: Thread started", false, true);
        }

        private void RunDigitalTwin()
        {
            _timer.Start();
        }

        private async void OnTimerElapsed(object sender, ElapsedEventArgs e)
        {

            var joints = Cobra600.Motion.GetJointPositions(_scaraRobot.Robot);

            if (ws.ReadyState != WebSocketSharp.WebSocketState.Open)
            {
                _manager.Log("❌ WebSocket non connesso.", true, false);
                return;
            }

            for (int i = 0; i < 4; i++)
            {
                var request = new
                {
                    id = 100 + i, // ID univoco per ogni variabile
                    method = "PUT",
                    url = $"/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ{i + 1}",
                    body = JsonConvert.SerializeObject(new { value = joints[i] }),
                    headers = new { }
                };

                string json = JsonConvert.SerializeObject(request);
                ws.Send(json);
                _manager.Log($"DT inviato", false, true);
            }

        }



        public void Stop()
        {
            // Stop the timer and the thread
            _timer.Stop();
            if (_digitalTwinThread.IsAlive)
            {
                _digitalTwinThread.Join();
                _manager.Log("Digital Twin: Thread stopped", false, true);
            }
        }
    }
}
