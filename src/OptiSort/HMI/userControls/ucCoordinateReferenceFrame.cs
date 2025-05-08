using Ace.Core.Server;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading;
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
        private bool _busy = false;
        private DateTime _elapsedTime;
        private int _pythonProcessId;
        private string _mqttClient = Properties.Settings.Default.mqtt_client;

        internal ucCoordinateReferenceFrame(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            RefreshCalibrationTimestamp();
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

            _busy = true;
            _manager.SubscribeMqttTopic(_mqttClient, "optisort/reference_calibration/output"); // launch here or subscription is too slow to catch publishing
            PlaceCalibrationGrid();
        }


        private void RefreshCalibrationTimestamp()
        {

            // Ensure UI updates are thread-safe
            if (InvokeRequired)
            {
                Invoke(new Action(() => RefreshCalibrationTimestamp()));
                return;
            }

            // Generate file path
            string filePath = Path.Combine(_manager.ConfigFolder, "ReferenceFrameCalibration.yaml");

            // Update the label text
            if (!File.Exists(filePath))
            {
                lbl_lastCalibrationDateTime.Text = "Last calibration: NEVER!";
            }
            else
            {
                string timestamp = File.GetLastWriteTime(filePath).ToString("dd/MM/yyyy HH:mm");
                lbl_lastCalibrationDateTime.Text = $"Last calibration: {timestamp}";
            }
        }


        private void PlaceCalibrationGrid()
        {
            _manager.Log("Positioning calibration grid...", false, false);

            // Move to safe position
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.SafeFlexi, true);

            // Pick tile
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPick, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPick, true);
            _manager.Cobra600.ToggleGripperAction(); // turn on suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPick, 50);

            // Place tile
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPlace, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPlace, true);
            _manager.Cobra600.ToggleGripperAction(); // turn off suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPlace, 50);

            // Move robot out the camera sight
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPick, 50);

            Thread.Sleep(1500); // Streaming is too slow to update: scara results still in front of the grid when picture gets taken

             //Save screenshots
            _manager.RequestScreenshots = true;
            _manager.BitmapQueued += SaveShots; // subscribe to the event to save screenshots
            _elapsedTime = DateTime.Now;

            //StartCalibrationScript();
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
            _manager.MqttMessageReceived += CalibrationMqttMessageReceived;
            _manager.OnExecutionTerminated += PythonTerminationHandler;
            _manager.OnErrorReceived += PythonErrorHandler;

            string scriptPath = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\python\other_scripts\coordinate_reference_frame.py"));
            _pythonProcessId = _manager.ExecuteScript(scriptPath);
            _manager.Log($"Coordinate reference frame calibration file launched in background (PID = {_pythonProcessId})", false, false);
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
                            columns = num_columns.Value,
                            rows = num_rows.Value,
                            size = num_size.Value,
                            center_x = _manager.GridPlace.DX,
                            center_y = _manager.GridPlace.DY,
                            yaw = _manager.GridPlace.Yaw
                        };

                        _manager.PublishMqttMessage(_mqttClient, "optisort/reference_calibration/input", data);
                        _manager.Log($"Grid parameters sent to calibration file", false, false);
                    }
                }

                if (message.TryGetProperty("result", out JsonElement resultElement))
                {
                    int result = resultElement.GetInt16();
                    if (result == 1)
                    {
                        _busy = false;
                        _manager.Log("Reference plane calibration procedure completed!", false, true);
                    }
                }
            }
        }


        private void PythonErrorHandler(int processID, string output)
        {
            if (processID == _pythonProcessId)
            {
                _manager.Log($"Reference frame calibration file threw an error: {output}", true, false);
            }
        }

        private void PythonTerminationHandler(int processID, bool executionTerminated)
        {
            if (processID == _pythonProcessId)
            {
                _manager.Log("Reference frame calibration file has closed!", false, false);

                if (_busy)
                    _manager.Log("Calibration procedure failed", true, false);

                _manager.MqttMessageReceived -= CalibrationMqttMessageReceived;
                _manager.OnExecutionTerminated -= PythonTerminationHandler;
                _manager.OnErrorReceived -= PythonErrorHandler;

                _manager.UnsubscribeMqttTopic(_mqttClient, "optisort/reference_calibration/output");
                _manager.StopExecution(_pythonProcessId); // needed to reset active processes memory

                RemoveCalibrationGrid();
            }
        }


        private void RemoveCalibrationGrid()
        {

            // Pick tile from flexibowl
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPlace, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPlace, true);
            _manager.Cobra600.ToggleGripperAction(); // turn on suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPlace, 50);


            // Place tile
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPick, 50);
            Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPick, true);
            _manager.Cobra600.ToggleGripperAction(); // turn off suction
            Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.GridPick, 50);

            RefreshCalibrationTimestamp();
            _manager.Log("Scara movements ended", false, false);
        }


    }
}
