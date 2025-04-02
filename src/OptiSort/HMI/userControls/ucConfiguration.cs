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

        private optisort_mgr _manager;
        private string _oldValue; // storing value that are about to be changed
        private bool _mqttClientDisconnected;
        private string _mqttClient = Properties.Settings.Default.mqtt_client;

        internal ucConfiguration(optisort_mgr manager)
        {
            // NOTE: access the settings menu: right click on HMI > properties > settings tab
            InitializeComponent();
            LoadConfigToDgv(); 
            _manager = manager;
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
                    _manager.Log($"Setting {settingName} updated", false, true);
                }

                // Reconnect services based on which setting changed
                // MQTT
                if (settingName.IndexOf("mqtt", StringComparison.OrdinalIgnoreCase) >= 0 && _manager.StatusMqttClient)
                {

                    if (settingName.IndexOf("client", StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        _ = DisconnectAsync();
                        _ = ConnectAsync();
                    }

                    else if (settingName.IndexOf("topic", StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        _manager.UnsubscribeMqttTopic(_mqttClient, _oldValue);
                        _manager.SubscribeMqttTopic(_mqttClient, newValue);   
                    }
                }

                // SCARA
                else if (settingName.IndexOf("scara", StringComparison.OrdinalIgnoreCase) >= 0 && _manager.StatusScara)
                {
                    _manager.Log("Reconnection to cobra in progress...", false, false);
                    _manager.DisconnectScara();
                    _manager.ConnectScara();
                }

                // FLEXIBOWL
                else if (settingName.IndexOf("flexibowl", StringComparison.OrdinalIgnoreCase) >= 0 && _manager.StatusFlexibowl)
                {
                    _manager.Log("Reconnection to flexibowl in progress...", false, false);
                    _manager.DisconnectFlexibowl();
                    _manager.ConnectFlexibowl();
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
                dgvConfig.Rows.Clear();
                dgvConfig.Columns.Clear();

                // storing to reconnect after change
                bool wasMqttConnected = _manager.StatusMqttClient;
                bool wasFlexibowlConnected = _manager.StatusFlexibowl;
                bool wasScaraConnected = _manager.StatusScara;

                // Disconnect services before changing 
                if (_manager.StatusMqttClient)
                    _ = DisconnectAsync();

                if (_manager.StatusFlexibowl)
                    _manager.DisconnectFlexibowl();

                if(_manager.StatusScara)
                    _manager.DisconnectScara();

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

                Properties.Settings.Default.Save();

                // Reconnect to services after changing 
                if (wasFlexibowlConnected)
                    _manager.ConnectFlexibowl();

                if (wasScaraConnected)
                    _manager.ConnectScara();

                if (wasMqttConnected)
                    _ = ConnectAsync();
                

                LoadConfigToDgv();
                _manager.Log("Settings restored to development defaults", false, true);
            }
        }


        public async Task DisconnectAsync()
        {
            try
            {
                _mqttClientDisconnected = false; 
                await _manager.DisconnectMqttClient(_mqttClient);
                _mqttClientDisconnected = true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error during disconnection: {ex.Message}");
            }
        }

        public async Task ConnectAsync()
        {
            // Wait until disconnect is completed
            while (!_mqttClientDisconnected)
            {
                await Task.Delay(10); // Check periodically (10ms)
            }

            try
            {
                _ = _manager.ConnectMQTTClient(_mqttClient, Properties.Settings.Default.mqtt_broker);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error during connection: {ex.Message}");
            }
        }
    }
}
