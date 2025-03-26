using Ace.Core.Util;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort.systems
{
    internal class CameraManager
    {
        private optisort_mgr _manager;
        private frmMain _frmMain;
        private int _scriptID;

        public enum Status
        {
            init, 
            webcam, 
            cameras,
            idle,
            config,
            ready,
            streaming,
            processing, 
            ended
        }


        public CameraManager(optisort_mgr manager, frmMain frmMain)
        {
            _manager = manager;
            _frmMain = frmMain;
        } 

        public void ConnectCameraManager()
        {
            // subscribe to mqtt topic to receive updates from python file
            _manager.SubscribeMqttTopic("optisort/camera_manager/output");
            _manager.MqttMessageReceived += MqttMessageReceived;

            // subscribe termination events to handler
            _manager.OnErrorReceived += PythonErrorHandler;
            _manager.OnExecutionTerminated += PythonTerminationHandler;

            string scriptPath = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\python\camera_manager\mqtt_manager_test.py"));
            _scriptID = _manager.ExecuteScript(scriptPath);
        }

        private void MqttMessageReceived(string topic, JsonElement message, int processID)
        {
            if (processID == _scriptID)
            {
                if (message.TryGetProperty("message", out JsonElement messageElement))
                {
                    string msg = messageElement.GetString();
                    _manager.Log("Camera manager over MQTT: " + msg, false, false);
                }
            }
        }


        private void PythonErrorHandler(int processID, string output)
        {
            if (processID == _scriptID)
            {
                _manager.Log("Camera manager file threw an error!", true, false);
                MessageBox.Show($"Camera manager file threw an error {output}", "Python error!");
            }
        }

        private void PythonTerminationHandler(int processID, bool executionTerminated)
        {
            if (processID == _scriptID)
            {
                _manager.Log("Camera manager file has closed!", false, false);
                _manager.MqttMessageReceived -= MqttMessageReceived;
                _manager.OnErrorReceived -= PythonErrorHandler;
                _manager.OnExecutionTerminated -= PythonTerminationHandler;
                _manager.UnsubscribeMqttTopic("optisort/camera_manager/output");
                _manager.StopExecution(_scriptID); // needed to reset active processes memory
            }
        }


    
    }
}
