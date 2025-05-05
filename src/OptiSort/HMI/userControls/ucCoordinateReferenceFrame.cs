using Ace.Core.Server;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort.userControls
{
    public partial class ucCoordinateReferenceFrame : UserControl
    {
        private optisort_mgr _manager;
        private bool _idsShot = false;
        private bool _luxonisShot = false;
        private bool _baslerShot = false;
        private DateTime _elapsedTime;
        private int _pythonProcessId;
        private string _mqttClient = Properties.Settings.Default.mqtt_client;

        internal ucCoordinateReferenceFrame(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;
        }

        private void btn_StartCoordinateRefCalibration_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusScara)
            {
                _manager.NonBlockingMessageBox("Please connect to the SCARA first!", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            if (!_manager.StatusMqttClient)
            {
                _manager.NonBlockingMessageBox("Please connect to the MQTT Client first!", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            if (!_manager.StatusCameraManager)
            {
                _manager.NonBlockingMessageBox("Please connect to the Camera Manager first!", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            PlaceCalibrationGrid();

        }

        private void PlaceCalibrationGrid()
        {
            Transform3D safeFlexi = new Transform3D(375.0, 15.0, 385.0, 0.0, 180.0, -130.0);
            Transform3D storagePick = new Transform3D(516.0, -80.0, 320.0, 0.0, 180.0, -130.0);
            Transform3D flexiPlace = new Transform3D(432.924, 224.126, 330.0, 0.0, 180.0, -130.0);

            // Move to safe position
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeFlexi, true);

            // Pick tile
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, storagePick, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, storagePick, true);
            _manager.Cobra600.ToggleGripperAction(); // turn on suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, storagePick, 50);

            // Place tile
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, flexiPlace, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, flexiPlace, true);
            _manager.Cobra600.ToggleGripperAction(); // turn off suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, flexiPlace, 50);

            // Move robot out the camera sight
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, storagePick, 50);


            // Save screenshots
            _manager.RequestScreenshots = true;
            _manager.BitmapQueued += SaveShots; // subscribe to the event to save screenshots
            _elapsedTime = DateTime.Now;
        }


        private void SaveShots(string topic)
        {
            if (_manager.RequestScreenshots) // this check is needed because the event is not unsubscribed immediately and late coming triggers may generate additional screenshots
            {
                Invoke(new Action(() =>
                {
                    string prefix = string.Empty;
                    Bitmap image = null;

                    if (topic == Properties.Settings.Default.mqtt_topic_idsStream && _idsShot == false)
                    {
                        if (_manager._idsQueue.TryDequeue(out var item))
                        {
                            image = item.Frame;
                            _idsShot = true;
                        }
                        else return; // in case queue is empty, saving cannot continue because bitmaps cannot be null to be saved
                        prefix = "ids";
                    }
                    else if (topic == Properties.Settings.Default.mqtt_topic_baslerStream && _baslerShot == false)
                    {
                        if (_manager._baslerQueue.TryDequeue(out var item))
                        {
                            image = item.Frame;
                            _baslerShot = true;
                        }
                        else return;
                        prefix = "basler";
                    }
                    else if (topic == Properties.Settings.Default.mqtt_topic_luxonisStream && _luxonisShot == false)
                    {
                        if (_manager._luxonisQueue.TryDequeue(out var item))
                        {
                            image = item.Frame;
                            _luxonisShot = true;
                        }
                        else return;
                        prefix = "luxonis";
                    }

                    if (!string.IsNullOrEmpty(prefix))
                    {
                        string filename = $"{prefix}_RefPlaneCalibration";
                        _manager.SaveBitmapAsFile(image, filename);
                    }

                    if (_idsShot && _baslerShot && _luxonisShot)
                    {
                        _manager.BitmapQueued -= SaveShots;
                        _manager.RequestScreenshots = false;
                        _idsShot = false;
                        _luxonisShot = false;
                        _baslerShot = false;
                        _manager.Log("Images from all cameras acquired. Starting calibration script...", false, false);
                        StartCalibrationScript();
                    }
                    else if ((DateTime.Now - _elapsedTime).TotalSeconds > 5)
                    {
                        _manager.BitmapQueued -= SaveShots;
                        _manager.RequestScreenshots = false;
                        _idsShot = false;
                        _luxonisShot = false;
                        _baslerShot = false;
                        _manager.NonBlockingMessageBox("Timeout reached. MQTT streamings might have a problem: check all the topics or inform the developer.", "Timeout!", MessageBoxIcon.Exclamation);
                    }
                }));
            }
        }


        private void StartCalibrationScript()
        {
            // launch python file and memorize processId
            string scriptPath = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\python\other_scripts\cameras_calibration.py"));
            _pythonProcessId = _manager.ExecuteScript(scriptPath);
            _manager.OnExecutionTerminated -= PythonTerminationHandler;
            _manager.Log($"Coordinate reference frame calibration file launched in background (PID = {_pythonProcessId})", false, false);
            _manager.SubscribeMqttTopic(_mqttClient, "optisort/reference_calibration/output");
            _manager.MqttMessageReceived += CalibrationMqttMessageReceived;
            
        }

        private void CalibrationMqttMessageReceived(string topic, JsonElement message, int processID)
        {
            if (processID == _pythonProcessId)
            {
                if (message.TryGetProperty("message", out JsonElement messageElement))
                {
                    string msg = messageElement.GetString();
                    _manager.Log($"Calibration file over MQTT ({processID}): " + msg, false, false);

                    if (msg.Contains("started"))
                    {

                        var data = new
                        {
                            columns = 5,
                            rows = 5,
                            size = 12
                        };

                        _manager.PublishMqttMessage(_mqttClient, "optisort/reference_calibration/input", data);
                        _manager.Log($"Grid parameters sent to calibration file", false, false);
                    }
                }
            }
        }


        private void PythonTerminationHandler(int processID, bool executionTerminated)
        {
            if (processID == _pythonProcessId)
            {
                _manager.Log("Camera manager file has closed!", false, false);

                _manager.MqttMessageReceived -= CalibrationMqttMessageReceived;
                _manager.OnExecutionTerminated -= PythonTerminationHandler;

                _manager.UnsubscribeMqttTopic(_mqttClient, "optisort/reference_calibration/output");
                _manager.StopExecution(_pythonProcessId); // needed to reset active processes memory

                _manager.StatusCameraManager = false;

                RemoveCalibrationGrid();
            }
        }


        private void RemoveCalibrationGrid()
        {
            Transform3D safeFlexi = new Transform3D(375.0, 15.0, 385.0, 0.0, 180.0, -130.0);
            Transform3D storagePick = new Transform3D(516.0, -80.0, 320.0, 0.0, 180.0, -130.0);
            Transform3D flexiPlace = new Transform3D(432.924, 224.126, 330.0, 0.0, 180.0, -130.0);

            // Pick tile from flexibowl
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, flexiPlace, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, flexiPlace, true);
            _manager.Cobra600.ToggleGripperAction(); // turn on suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, flexiPlace, 50);


            // Place tile
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, storagePick, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, storagePick, true);
            _manager.Cobra600.ToggleGripperAction(); // turn off suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, storagePick, 50);

            _manager.Log("Reference plane calibration procedure completed!", false, true)
        }


    }
}
