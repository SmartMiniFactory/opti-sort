using System;
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
        public ucConfiguration()
        {
            InitializeComponent();
            LoadConfigToDgv();

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
            }
        }
    }
}
