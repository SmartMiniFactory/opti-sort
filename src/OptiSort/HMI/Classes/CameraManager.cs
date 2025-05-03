using Ace.Core.Util;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows.Forms;
using WebSocketSharp;

namespace OptiSort.systems
{
    internal class CameraManager
    {
        private optisort_mgr _manager;
        private int _scriptID;
        private string _mqttClient = Properties.Settings.Default.mqtt_client;
        private string _processingCamera = null;

        public event Action CamerasWorking;

        public Status CurrentState { get; private set; } = Status.ended;

        public enum Status
        {
            init, 
            idle,
            streaming,
            processing,
            stopped,
            ended
        }


        public CameraManager(optisort_mgr manager)
        {
            _manager = manager;
        } 

        public bool ConnectCameraManager()
        {
            // subscribe to mqtt topic to receive updates from python file
            _manager.SubscribeMqttTopic(_mqttClient, "optisort/camera_manager/output");
            _manager.MqttMessageReceived += MqttMessageReceived;

            // subscribe termination events to handler
            _manager.OnErrorReceived += PythonErrorHandler;
            _manager.OnExecutionTerminated += PythonTerminationHandler;

            string scriptPath = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\python\camera_manager\main_CameraManager.py"));
            _scriptID = _manager.ExecuteScript(scriptPath);

            if (_scriptID == 0) 
            {
                _manager.NonBlockingMessageBox("Last camera manager execution remained unkilled in background! Please proceed to kill manually before retrying connection", "WARNING!", MessageBoxIcon.Warning);
                _manager.UnsubscribeMqttTopic(_mqttClient, "optisort/camera_manager/output");
                _manager.MqttMessageReceived -= MqttMessageReceived;
                _manager.OnErrorReceived -= PythonErrorHandler;
                _manager.OnExecutionTerminated -= PythonTerminationHandler;
                return false;
            }
            else
            {
                _manager.Log($"Camera manager execution launched in background! (PID = {_scriptID})", false, false);
                return true;
            }
        }

        public void DisconnectCameraManager()
        {
            SendCommand("exit");
            _manager.Log($"Terminating camera manager background execution (PID = {_scriptID})", false, false);
        }

        private void SendCommand(string cmd)
        {
            var data = new
            {
                command = cmd
            };
            _manager.PublishMqttMessage(_mqttClient, "optisort/camera_manager/input", data);
            _manager.Log($"Command sent to camera manager: {cmd}", false, false);
        }


        private void MqttMessageReceived(string topic, JsonElement message, int processID)
        {
            if (processID == _scriptID)
            {
                if (message.TryGetProperty("message", out JsonElement messageElement))
                {
                    string msg = messageElement.GetString();
                    _manager.Log($"Camera manager over MQTT ({processID}): " + msg, false, false);
                }

                if (message.TryGetProperty("result", out JsonElement resultElement))
                {
                    int result = resultElement.GetInt16();
                    
                    if (Enum.IsDefined(typeof(Status), result))
                    {
                        Status newStatus = (Status)result;

                        if (CurrentState != newStatus)
                        {
                            switch (newStatus)
                            {
                                case Status.init:
                                    if (_manager.StatusCameraTesting)
                                        SendCommand("webcam");
                                    else
                                        SendCommand("cameras");
                                    break;

                                case Status.idle:
                                    if (_processingCamera != null)
                                    {
                                        var data = new
                                        {
                                            command = "process",
                                            camera = _processingCamera
                                        };
                                        _manager.PublishMqttMessage(_mqttClient, "optisort/camera_manager/input", data);
                                        _processingCamera = null;
                                        _manager.Log($"Command sent to camera manager: process", false, false);
                                    }
                                    else 
                                        SendCommand("stream");

                                    break;

                                case Status.streaming:
                                    CamerasWorking?.Invoke();
                                    break;

                                case Status.processing:
                                    CamerasWorking?.Invoke();
                                    break;
                            }
                        }

                        CurrentState = (Status)result;

                    }
                    else
                    {
                        Console.WriteLine($"Unknown state received: {result}");
                    }

                }
            }
        }


        public void SwitchToProcessing(string camera)
        {
            if (CurrentState == Status.idle || CurrentState == Status.streaming)
            {
                _processingCamera = camera;
                SendCommand("stop");
                _manager.Log($"Switching camera manager to processing mode for camera {camera}", false, false);
            }
            else
            {
                _manager.Log($"Unable to command processing mode", true, false);
            }
        }

        public void SwitchToStreaming()
        {
            if (CurrentState == Status.idle || CurrentState == Status.processing)
            {
                SendCommand("stop");
                _manager.Log($"Switching camera manager to streaming mode ", false, false);
            }
            else
            {
                _manager.Log($"Unable to command streaming mode", true, false);
            }
        }


        private void PythonErrorHandler(int processID, string output)
        {
            if (processID == _scriptID)
            {
                _manager.Log("Camera manager file threw an error!", true, false);
                _manager.NonBlockingMessageBox($"Camera manager file threw an error {output}", "Python error!", MessageBoxIcon.Error);
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
                
                _manager.UnsubscribeMqttTopic(_mqttClient, "optisort/camera_manager/output");
                _manager.StopExecution(_scriptID); // needed to reset active processes memory

                _manager.StatusCameraManager = false;
            }
        }

    }

}

