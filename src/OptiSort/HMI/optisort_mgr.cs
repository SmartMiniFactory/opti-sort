using Ace.Core.Util;
using FlexibowlLibrary;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{

    internal class optisort_mgr : INotifyPropertyChanged
    {
        public MQTT MqttClient { get; set; }
        public Cobra600 Cobra600 { get; set; }
        public Flexibowl Flexibowl { get; set; }

        // status
        public event PropertyChangedEventHandler PropertyChanged; // declare propertyChanged event (required by the associated interface)
        private bool _statusScara = false;
        private bool _statusScaraEmulation = true;
        private bool _statusFelxibowl = false;
        private bool _statusMqttClient = false;
        private string _streamingTopic = null;
        private bool _requestScreenshots = false;

        // custom defined properties to trigger events on status changed
        public bool StatusScara
        {
            get { return _statusScara; }
            set
            {
                if (_statusScara != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusScara = value;
                    OnPropertyChanged(nameof(StatusScara));
                }
            }
        }
        public bool StatusFlexibowl
        {
            get { return _statusFelxibowl; }
            set
            {
                if (_statusFelxibowl != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusFelxibowl = value;
                    OnPropertyChanged(nameof(StatusFlexibowl));
                }
            }
        }
        public bool StatusMqttClient
        {
            get { return _statusMqttClient; }
            set
            {
                if (_statusMqttClient != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusMqttClient = value;
                    OnPropertyChanged(nameof(StatusMqttClient));
                }
            }
        }
        public bool StatusScaraEmulation
        {
            get { return _statusScaraEmulation; }
            set
            {
                if (_statusScaraEmulation != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusScaraEmulation = value;
                    OnPropertyChanged(nameof(StatusScaraEmulation));
                }
            }
        }
        public string StreamingTopic
        {
            get { return _streamingTopic; }
            set
            {
                if (_streamingTopic != value) // setting value different from actual value -> store new value and trigger event
                {
                    _streamingTopic = value;
                    OnPropertyChanged(nameof(StreamingTopic));
                }
            }
        }
        public bool RequestScreenshots
        {
            get { return _requestScreenshots; }
            set
            {
                if (_requestScreenshots != value) // setting value different from actual value -> store new value and trigger event
                {
                    _requestScreenshots = value;
                    OnPropertyChanged(nameof(RequestScreenshots));
                }
            }
        }
        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public event Action<Dictionary<string, Bitmap>> ScreenshotsReady; // Event to notify subscribers
        public event Action<Control> NewUserControlRequested; // Event to notify subscribers


        // Logging
        public event Action<LogEntry> LogEvent; // Event to notify subscribers
        public class LogEntry
        {
            public string Message { get; set; }
            public bool IsError { get; set; }
            public bool IsSuccess { get; set; }
            public override string ToString() => Message; // Fallback for ListBox's default behavior
        }

        public optisort_mgr()
        {
            // Instance MQTT class
            string mqttBroker = Properties.Settings.Default.mqtt_broker;
            string mqttPort = Properties.Settings.Default.mqtt_port;
            MqttClient = new MQTT(mqttBroker, mqttPort);

            // attach property changes to various useful methods
            PropertyChanged += NewStreamingTopic;

            // Initialize the remoting subsystem
            RemotingUtil.InitializeRemotingSubsystem(true, 0);


            // Instance cobra class
            string remotingPort = Properties.Settings.Default.scara_port;
            Cobra600 = new Cobra600("ace", remotingPort);

            // Instance flexibowl class
            string flexibowlIP = Properties.Settings.Default.flexibowl_IP;
            Flexibowl = new Flexibowl(flexibowlIP);
        }


        private void NewStreamingTopic(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(StreamingTopic))
            {
                Log($"Streaming topic updated to '{StreamingTopic}'", false, false);
            }
        }

        public void NotifyScreenshotsReady(Dictionary<string, Bitmap> screenshotBuffer)
        {
            ScreenshotsReady?.Invoke(new Dictionary<string, Bitmap>(screenshotBuffer));
        }

        public void RequestNewUcLoading(Control control)
        {
            NewUserControlRequested?.Invoke(control);
        }

        // -----------------------------------------------------------------------------------
        // ---------------------------------- CONNECTIONS ------------------------------------
        // -----------------------------------------------------------------------------------

        #region ACE scara

        // TODO: is possible to use async?

        public bool ConnectScara()
        {
            // check if ACE is running
            Process[] ProcessList = Process.GetProcessesByName("Ace");
            if (ProcessList.Length != 1)
            {
                MessageBox.Show("ACE is not running: please open the robot server");
                return false;
            }
            Log("Trying to connect to robot", false, false);


            string controllerName = Properties.Settings.Default.scara_controllerName;
            string robotName = Properties.Settings.Default.scara_robotName;
            string endEffectorName = Properties.Settings.Default.scara_endEffectorName;


            if (!StatusScaraEmulation)
                Cobra600.RobotIP = Properties.Settings.Default.scara_controllerIP;

            var ex = Cobra600.Connect(StatusScaraEmulation, controllerName, robotName, endEffectorName);
            if (ex == null)
            {
                StatusScara = true;
                Log("Robot successfully connected", false, true);
                MessageBox.Show("Scara connected, enabling power. Please press the physical button on the front panel!");
                return true;
            }
            else
            {
                MessageBox.Show($"{ex}");
                Log("Failed to connect to robot", true, false);
                return false;
            }
        }

        public bool DisconnectScara()
        {
            Log("Trying to disconnect from robot", false, false);

            if (Cobra600.Disconnect())
            {
                StatusScara = false;
                Log("Robot succesfully disconnected", false, true);
                return true;
            }
            else
            {
                Log("Failed to disconnect from Cobra", true, false);
                return false;
            }
        }

        public void ToggleScaraEmulationMode()
        {
            if (StatusScaraEmulation)
            {
                StatusScaraEmulation = false;
                Log("Scara emulation mode disabled", false, false);
            }
            else
            {
                StatusScaraEmulation = true;
                Log("Scara emulation mode enabled", false, false);
            }
        }

        #endregion

        #region flexibowl

        public bool ConnectFlexibowl()
        {
            Log("Trying to connect to flexibowl", false, false);

            Flexibowl.Connect();
            try
            {
                Flexibowl.Set.Servo(true);
                // TODO: should check for a connection by asking for a response
                StatusFlexibowl = true;
                Log("Flexibowl connected and servo ON", false, true);
                return true;
            }
            catch (Exception ex)
            {
                Log($"Failed to connect Flexibowl UDP client", true, false);
                MessageBox.Show($"{ex}");
                return false;
            }
        }

        public bool DisconnectFlexibowl()
        {
            Log("Trying to disconnect to flexibowl", false, false);

            if (Flexibowl.Disconnect())
            {
                // TODO: should check for a connection by asking for a response
                StatusFlexibowl = false;
                Log("Flexibowl UDP client disconnected", false, true);
                return true;
            }
            else
            {
                Log("Failed to disconnect from Flexibowl UDP Client", true, false);
                return false;
            }
        }

        #endregion

        #region MQTT

        public async Task<bool> ConnectMQTTClient()
        {
            if (StatusMqttClient)
            {
                Log("MQTT client already connected, cannot connect another", true, false);
                return false;
            }

            Log("Trying to create new MQTT client", false, false);

            string mqttClientName = Properties.Settings.Default.mqtt_client;
            Task<bool> createClient = MqttClient.CreateClient(mqttClientName);

            if (await createClient)
            {
                StatusMqttClient = true;

                Log($"MQTT client '{mqttClientName}' created", false, true);

                List<string> topics = new List<string>
                {
                    // Retreive topics from configuration file
                    Properties.Settings.Default.mqtt_topic_scaraTarget,
                    Properties.Settings.Default.mqtt_topic_idsStream,
                    Properties.Settings.Default.mqtt_topic_luxonisStream,
                    Properties.Settings.Default.mqtt_topic_baslerStream
                };

                foreach (string topic in topics)
                {
                    SubscribeMqttTopic(mqttClientName, topic);
                }

                return true;
            }
            else
            {
                Log($"Failed to create '{mqttClientName}' MQTT client", true, false);
                return false;
            }
        }

        public async Task<bool> DisconnectMqttClient(string clientName)
        {
            Log("Trying to destroy MQTT client", false, false);

            Task<bool> destroyClient = MqttClient.DestroyClient(clientName);
            if (await destroyClient)
            {
                StatusMqttClient = false;
                Log($"MQTT client '{clientName}' destroyed", false, true);
                return true;
            }
            else
            {
                Log($"Error destroying '{clientName}'", true, false);
                return false;
            }
        }

        public async void SubscribeMqttTopic(string client, string topic)
        {
            bool subscribed = await MqttClient.SubscribeClientToTopic(client, topic);
            if (subscribed)
                Log($"{client} subscribed to {topic}", false, true);
            else
                Log($"Unable subscribing {client} to {topic}", true, false);
        }

        public async void UnsubscribeMqttTopic(string client, string topic)
        {
            bool unsubscribed = await MqttClient.UnsubscribeClientFromTopic(client, topic);
            if (unsubscribed)
                Log($"{client} unsubscribed to {topic}", false, true);
            else
                Log($"Unable unsubscribing {client} to {topic}", true, false);
        }

        #endregion




        public void Log(string message, bool isError = false, bool isSuccess = false)
        {
            // Prepend timestamp to the message
            var logEntry = new LogEntry
            {
                Message = $"{DateTime.Now}: {message}",
                IsError = isError,
                IsSuccess = isSuccess
            };

            // Raise the log event
            LogEvent?.Invoke(logEntry);
        }




    }
}
