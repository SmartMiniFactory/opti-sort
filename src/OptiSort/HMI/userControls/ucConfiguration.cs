using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Configuration;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;

namespace OptiSort.userControls
{
    public partial class ucConfiguration : UserControl
    {

        private frmMain _frmMain;
        private string _oldValue; // storing value that are about to be changed

        public ucConfiguration(frmMain frmMain)
        {
            // NOTE: access the settings menu: right click on HMI > properties > settings tab
            InitializeComponent();
            LoadConfigToDgv(); 
            _frmMain = frmMain;
        }

        /// <summary>
        /// Initial load of the dgv
        /// </summary>
        private void LoadConfigToDgv()
        {
            // create columns
            dgvConfig.Columns.Add("SettingName", "Setting Name");
            dgvConfig.Columns.Add("SettingValue", "Value");

            // setup columns
            dgvConfig.Columns["SettingName"].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;
            dgvConfig.Columns["SettingValue"].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;
            dgvConfig.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;

            // create a list storing all configuration parameters
            var settingsList = new List<KeyValuePair<string, object>>();
            foreach (SettingsProperty setting in Properties.Settings.Default.Properties)
            {
                string settingName = setting.Name;
                object settingValue = Properties.Settings.Default[settingName];
                settingsList.Add(new KeyValuePair<string, object>(settingName, settingValue));
            }

            // Sort list by name (alphabetical order)
            settingsList.Sort((x, y) => string.Compare(x.Key, y.Key));

            // Add sorted list to the DataGridView
            foreach (var setting in settingsList)
            {
                dgvConfig.Rows.Add(setting.Key, setting.Value?.ToString());
            }

            dgvConfig.Columns[0].Width = dgvConfig.Width / 2;
            dgvConfig.Columns[1].Width = dgvConfig.Width / 2;
        }

        
        
        private void dataGridView_CellBeginEdit(object sender, DataGridViewCellCancelEventArgs e)
        {
            // cancel edit if someone tries to modify settings name column
            if (e.ColumnIndex == 0)
            {
                e.Cancel = true;
                return;
            }
            _oldValue = dgvConfig[e.ColumnIndex, e.RowIndex].Value.ToString(); // Store the old value before editing starts
        }


        private void dgvConfig_CellValueChanged(object sender, DataGridViewCellEventArgs e)
        {
            // Ensure that the edited cell is from the Value column (index 1)
            if (e.ColumnIndex == 1 && e.RowIndex >= 0)
            {
                string settingName = dgvConfig.Rows[e.RowIndex].Cells[0].Value.ToString();
                string newValue = dgvConfig.Rows[e.RowIndex].Cells[1].Value.ToString();

                // Check if the setting exists in Properties.Settings.Default and save
                if (Properties.Settings.Default.Properties[settingName] != null)
                {
                    Properties.Settings.Default[settingName] = newValue;
                    Properties.Settings.Default.Save();
                    _frmMain.Log($"Setting {settingName} updated");
                }

                // Reconnect services based on which setting changed
                // MQTT
                if (settingName.IndexOf("mqtt", StringComparison.OrdinalIgnoreCase) >= 0 && _frmMain.StatusMqttClient)
                {
                    string clientID = _frmMain.MqttClient.GetConnectedClientName();

                    if (settingName.IndexOf("client", StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        Cursor = Cursors.WaitCursor;
                        _frmMain.DisconnectMqttClient(clientID);
                        Cursor = Cursors.WaitCursor;
                        _frmMain.ConnectMQTTClient();
                    }

                    else if (settingName.IndexOf("topic", StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        _frmMain.MqttClient.UnsubscribeClientFromTopic(clientID, _oldValue);
                        _frmMain.Log($"Topic {_oldValue} unsubscribed");

                        _frmMain.MqttClient.SubscribeClientToTopic(clientID, newValue);
                        _frmMain.Log($"Topic {newValue} subscribed");
                    }
                }

                // SCARA
                else if (settingName.IndexOf("scara", StringComparison.OrdinalIgnoreCase) >= 0 && _frmMain.StatusScara)
                {
                    _frmMain.Log("Reconnection to cobra in progress...");
                    Cursor = Cursors.WaitCursor;
                    _frmMain.DisconnectScara();
                    Cursor = Cursors.WaitCursor;
                    _frmMain.ConnectScara();
                }

                // FLEXIBOWL
                else if (settingName.IndexOf("flexibowl", StringComparison.OrdinalIgnoreCase) >= 0 && _frmMain.StatusFlexibowl)
                {
                    _frmMain.Log("Reconnection to flexibowl in progress...");
                    Cursor = Cursors.WaitCursor;
                    _frmMain.DisconnectFlexibowl();
                    Cursor = Cursors.WaitCursor;
                    _frmMain.ConnectFlexibowl();
                }
            }
        }

        

        /// <summary>
        /// load predetermined values in case of messing up with settings during testing
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnLoadDefaults_Click(object sender, EventArgs e)
        {
            DialogResult result = MessageBox.Show("Are you sure you want to load default settings? You will lose all table contents", "Load defaults", MessageBoxButtons.YesNo, MessageBoxIcon.Question);
            if (result == DialogResult.Yes)
            {
                // FLEXIBOWL
                Properties.Settings.Default.flexibowl_IP = "10.90.90.20";
                
                // MQTT
                Properties.Settings.Default.mqtt_broker = "localhost";
                Properties.Settings.Default.mqtt_client = "OptiSort";
                Properties.Settings.Default.mqtt_port = "1883";
                Properties.Settings.Default.mqtt_topic_baslerStream = "optisort/basler/stream";
                Properties.Settings.Default.mqtt_topic_idsStream = "optisort/ids/stream";
                Properties.Settings.Default.mqtt_topic_luxonisStream = "optisort/luxonis/stream";
                Properties.Settings.Default.mqtt_topic_scaraTarget = "optisort/scara/target";

                // SCARA
                Properties.Settings.Default.scara_controllerIP = "10.90.90.60";
                Properties.Settings.Default.scara_controllerName = "OptiSort";
                Properties.Settings.Default.scara_endEffectorName = "Suction Cup";
                Properties.Settings.Default.scara_port = "43434";
                Properties.Settings.Default.scara_robotName = "R1 Cobra600";
                Properties.Settings.Default.scara_serverIP = "10.90.90.91";

                Properties.Settings.Default.Save();
                _frmMain.Log("Settings restored to development defaults");

                dgvConfig.Rows.Clear();
                dgvConfig.Columns.Clear();
                LoadConfigToDgv();
            }
        }
    }
}
