using Ace.Adept.Server.Controls;
using Ace.Adept.Server.Motion;
using Ace.Core.Client;
using Ace.Core.Server;
using Ace.Core.Util;
using System;
using System.ComponentModel;
using System.Threading;
using System.Windows.Forms;
using System.Diagnostics;
using System.Text.Json;
using System.Collections.Generic;
using static OptiSort.ucScaraTargets;


namespace OptiSort
{
    /// <summary>
    /// Collects picking coordinates calculated by the openCV library which elaborate the camera image (incoming over MQTT)
    /// Two buttons are used to connect/disconnect from the robot and open the manual commands
    /// </summary>
    public partial class ucScaraTargets : UserControl
    {
        private optisort_mgr _manager;

        internal IReadOnlyList<TargetRow> TargetQueueList => _targetQueueList;
        private BindingList<TargetRow> _targetQueueList = new BindingList<TargetRow>();

        private TargetRow _lastTarget = new TargetRow(null, new Transform3D(0, 0, 0, 0, 0, 0));

        public int Backlog { get; private set; } = 0;

        public event Action ObjectDetected;

        internal ucScaraTargets(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;
        }

        private void ucScara_Load(object sender, EventArgs e)
        {
            // init dgv
            _targetQueueList = new BindingList<TargetRow>();
            dgvTargetQueue.AutoGenerateColumns = false;

            DataGridViewTextBoxColumn cColumn = new DataGridViewTextBoxColumn
            { HeaderText = "Component", DataPropertyName = "Component", Width = 300 };
            DataGridViewTextBoxColumn xColumn = new DataGridViewTextBoxColumn
            { HeaderText = "DX", DataPropertyName = "DX", Width = 100 };
            DataGridViewTextBoxColumn yColumn = new DataGridViewTextBoxColumn
            { HeaderText = "DY", DataPropertyName = "DY", Width = 100 };
            DataGridViewTextBoxColumn zColumn = new DataGridViewTextBoxColumn
            { HeaderText = "DZ", DataPropertyName = "DZ", Width = 100 };
            DataGridViewTextBoxColumn yawColumn = new DataGridViewTextBoxColumn
            { HeaderText = "Yaw", DataPropertyName = "Yaw", Width = 100 };
            DataGridViewTextBoxColumn pitchColumn = new DataGridViewTextBoxColumn
            { HeaderText = "Pitch", DataPropertyName = "Pitch", Width = 100 };
            DataGridViewTextBoxColumn rollColumn = new DataGridViewTextBoxColumn
            { HeaderText = "Roll", DataPropertyName = "Roll", AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill };

            dgvTargetQueue.Columns.Add(cColumn);
            dgvTargetQueue.Columns.Add(xColumn);
            dgvTargetQueue.Columns.Add(yColumn);
            dgvTargetQueue.Columns.Add(zColumn);
            dgvTargetQueue.Columns.Add(yawColumn);
            dgvTargetQueue.Columns.Add(pitchColumn);
            dgvTargetQueue.Columns.Add(rollColumn);

            dgvTargetQueue.DataSource = _targetQueueList;
            dgvTargetQueue.Rows.Clear();

        }


        public void UpdateTargetTable(JsonElement content)
        {

            if (InvokeRequired)
            {
                Invoke(new Action<JsonElement>(UpdateTargetTable), content);
            }
            else
            {

                if (content.TryGetProperty("coordinate", out JsonElement coordinateElement))
                {
                    // coordinateElement is a string JsonElement → get its string value
                    string coordinateStr = coordinateElement.GetString();

                    // Replace single quotes with double quotes to make valid JSON
                    string fixedCoordinateStr = coordinateStr.Replace("'", "\"");

                    // Parse the fixed string into a JsonDocument
                    var coordDoc = JsonDocument.Parse(fixedCoordinateStr);
                    JsonElement coord = coordDoc.RootElement;

                    string component = coord.GetProperty("type").GetString();

                    if (component == "AI" || component == "BI")
                        return;

                    double x = coord.GetProperty("x").GetDouble();
                    double y = coord.GetProperty("y").GetDouble();
                    double z = coord.GetProperty("z").GetDouble();
                    double yaw = coord.GetProperty("rx").GetDouble();
                    double pitch = coord.GetProperty("ry").GetDouble();
                    double roll = coord.GetProperty("rz").GetDouble();

                    TargetRow targetRow = new TargetRow(component, new Transform3D(x, y, z, yaw, pitch, roll));



                    // Check if list is empty OR distance between new and last target is at least 1 mm (in x OR y)
                    bool shouldAdd = false;
                    if (_targetQueueList.Count == 0)
                    {
                        shouldAdd = true;
                    }
                    else
                    {
                        double lastX = _lastTarget.Transform.DX;
                        double lastY = _lastTarget.Transform.DY;

                        double deltaX = Math.Abs(x - lastX);
                        double deltaY = Math.Abs(y - lastY);

                        if (deltaX >= 1.0 || deltaY >= 1.0)
                        {
                            shouldAdd = true;
                        }
                    }

                    // Only update the list if it's empty or the row is different from the last target
                    if (shouldAdd)
                    {
                        try
                        {

                            // rewriting components strings to something understandable from the user
                            var componentMap = new Dictionary<string, string>
                            {
                                { "AE", "Component A, external surface" },
                                { "AI", "Component A, internal surface" },
                                { "BE", "Component B, external surface" },
                                { "BI", "Component B, internal surface" }
                            };

                            componentMap.TryGetValue(component, out string readableComponent);
                            component = readableComponent ?? component;  // fallback to original if not mapped

                            targetRow.Component = component;
                            _targetQueueList.Add(targetRow);
                            _lastTarget = targetRow;

                            Backlog++;

                            dgvTargetQueue.Refresh();
                            ObjectDetected?.Invoke();

                        }
                        catch (Exception ex)
                        {
                            _manager.NonBlockingMessageBox($"Error adding new entry: {ex.Message}", "Error!", MessageBoxIcon.Error);
                        }
                    }
                }

            }
        }

        public void PlacingCompleted()
        {
            if (InvokeRequired)
            {
                // Marshal to the UI thread (needed to avoid cross-thread error)
                Invoke(new Action(PlacingCompleted));
            }
            else
            {
                // removing the first entry from the list
                try
                {
                    if (_targetQueueList.Count > 0)
                    {
                        _targetQueueList.RemoveAt(0);
                        _manager.Log("test", false, false);
                        Backlog--;
                    }

                    if (Backlog > 0)
                        ObjectDetected?.Invoke(); // recall event to let parent user control pick next component in backlog

                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error removing an entry: " + ex.ToString());
                    _manager.Log($"Error removing an entry: " + ex.ToString(), true, false);
                }

            }
        }

        public void DropBacklog()
        {
            if (InvokeRequired)
            {
                // Marshal to the UI thread (needed to avoid cross-thread error)
                Invoke(new Action(DropBacklog));
            }
            else
            {
                // removing all entries from the list
                try
                {
                    _targetQueueList.Clear();
                    Backlog = 0;
                    dgvTargetQueue.Refresh();
                }
                catch (Exception ex)
                {
                    _manager.NonBlockingMessageBox($"Error clearing backlog: " + ex.ToString(), "Error!", MessageBoxIcon.Error);
                }
            }
        }


        /// <summary>
        /// Represents a single row in the target queue table, containing both component description and coordinates.
        /// (Compact internal class - only for ucScaraTargets)
        /// </summary>
        internal class TargetRow
        {
            public string Component { get; set; }
            public double DX { get; set; }
            public double DY { get; set; }
            public double DZ { get; set; }
            public double Yaw { get; set; }
            public double Pitch { get; set; }
            public double Roll { get; set; }

            // Convenience property to get the Transform3D directly
            public Transform3D Transform => new Transform3D(DX, DY, DZ, Yaw, Pitch, Roll);

            internal TargetRow(string component, Transform3D transform)
            {
                Component = component;
                DX = transform.DX;
                DY = transform.DY;
                DZ = transform.DZ;
                Yaw = transform.Yaw;
                Pitch = transform.Pitch;
                Roll = transform.Roll;
            }
        }

    }
}
