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

namespace OptiSort.userControls
{
    public partial class ucConfiguration : UserControl
    {

        private frmMain _frmMain;
        private string _oldValue; // storing value that are about to be changed

        public ucConfiguration(frmMain frmMain)
        {
            InitializeComponent();
            LoadConfigToDgv();
            _frmMain = frmMain;

            // NOTE: access the configuration menu: right click on HMI > properties > settings tab
        }

        private void LoadConfigToDgv()
        {
            // create columns
            dgvConfig.Columns.Add("SettingName", "Setting Name");
            dgvConfig.Columns.Add("SettingValue", "Value");

            // setup columns
            dgvConfig.Columns["SettingName"].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;
            dgvConfig.Columns["SettingValue"].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;
            dgvConfig.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
            dgvConfig.AutoResizeColumns();

            // clear rows
            dgvConfig.Rows.Clear();

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
        }

        private void dataGridView_CellBeginEdit(object sender, DataGridViewCellCancelEventArgs e)
        {
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
                }

                HandleMqttConnections(settingName, newValue);  
            }
        }

        private async void HandleMqttConnections(string name, string newValue)
        {
            if (name.IndexOf("mqtt", StringComparison.OrdinalIgnoreCase) >= 0) // case-insensitive check if property name contains "mqtt"
            {
                string clientID = _frmMain.MqttClient.GetConnectedClientName();
                
                if (name.IndexOf("client", StringComparison.OrdinalIgnoreCase) >= 0) 
                {
                    await _frmMain.MqttClient.DestroyClient(clientID);
                    _frmMain.Log($"Previous MqttClient {clientID} destroyed");

                    await _frmMain.MqttClient.CreateClient(newValue);
                    _frmMain.Log($"New MqttClient {newValue} created");

                    _frmMain.SubscribeMqttTopics();
                }
                
                else if (name.IndexOf("topic", StringComparison.OrdinalIgnoreCase)>=0) 
                {
                    await _frmMain.MqttClient.UnsubscribeClientFromTopic(clientID, _oldValue);
                    _frmMain.Log($"Topic {_oldValue} unsubscribed");

                    await _frmMain.MqttClient.SubscribeClientToTopic(clientID, newValue);
                    _frmMain.Log($"Topic {newValue} subscribed");
                }
            }
        }
    }
}
