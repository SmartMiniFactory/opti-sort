namespace OptiSort.userControls
{
    partial class ucConfiguration
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.dgvConfig = new System.Windows.Forms.DataGridView();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.btnLoadDefaults = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.dgvConfig)).BeginInit();
            this.flowLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // dgvConfig
            // 
            this.dgvConfig.AllowUserToAddRows = false;
            this.dgvConfig.AllowUserToDeleteRows = false;
            this.dgvConfig.AllowUserToResizeRows = false;
            this.dgvConfig.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.dgvConfig.Dock = System.Windows.Forms.DockStyle.Fill;
            this.dgvConfig.Location = new System.Drawing.Point(0, 0);
            this.dgvConfig.MultiSelect = false;
            this.dgvConfig.Name = "dgvConfig";
            this.dgvConfig.RowHeadersWidth = 62;
            this.dgvConfig.RowTemplate.Height = 28;
            this.dgvConfig.Size = new System.Drawing.Size(974, 442);
            this.dgvConfig.TabIndex = 0;
            this.dgvConfig.CellBeginEdit += new System.Windows.Forms.DataGridViewCellCancelEventHandler(this.dataGridView_CellBeginEdit);
            this.dgvConfig.CellValueChanged += new System.Windows.Forms.DataGridViewCellEventHandler(this.dgvConfig_CellValueChanged);
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.btnLoadDefaults);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(0, 365);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(974, 77);
            this.flowLayoutPanel1.TabIndex = 1;
            // 
            // btnLoadDefaults
            // 
            this.btnLoadDefaults.Location = new System.Drawing.Point(3, 3);
            this.btnLoadDefaults.Name = "btnLoadDefaults";
            this.btnLoadDefaults.Size = new System.Drawing.Size(130, 71);
            this.btnLoadDefaults.TabIndex = 0;
            this.btnLoadDefaults.Text = "Load defaults";
            this.btnLoadDefaults.UseVisualStyleBackColor = true;
            this.btnLoadDefaults.Click += new System.EventHandler(this.btnLoadDefaults_Click);
            // 
            // ucConfiguration
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.flowLayoutPanel1);
            this.Controls.Add(this.dgvConfig);
            this.Name = "ucConfiguration";
            this.Size = new System.Drawing.Size(974, 442);
            ((System.ComponentModel.ISupportInitialize)(this.dgvConfig)).EndInit();
            this.flowLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.DataGridView dgvConfig;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Button btnLoadDefaults;
    }
}
