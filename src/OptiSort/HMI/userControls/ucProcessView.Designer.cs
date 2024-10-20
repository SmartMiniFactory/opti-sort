namespace OptiSort.userControls
{
    partial class ucProcessView
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
            // unsubscribe mqtt messages triggers
            _frmMain._mqttClient.MessageReceived -= _ucScaraTargets.OnMessageReceived;
            _frmMain._mqttClient.MessageReceived -= _ucCameraStream.OnMessageReceived;
            _frmMain.Log("Process view detached from being triggered by MQTT messages");

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
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.pnlCameraStream = new System.Windows.Forms.Panel();
            this.cmbCameras = new System.Windows.Forms.ComboBox();
            this.pnlRobot3D = new System.Windows.Forms.Panel();
            this.pnlScara = new System.Windows.Forms.Panel();
            this.pnlFlexibowl = new System.Windows.Forms.Panel();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 65F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 35F));
            this.tableLayoutPanel1.Controls.Add(this.pnlCameraStream, 1, 1);
            this.tableLayoutPanel1.Controls.Add(this.cmbCameras, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.pnlRobot3D, 1, 2);
            this.tableLayoutPanel1.Controls.Add(this.pnlScara, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.pnlFlexibowl, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 3;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 40.90909F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 59.09091F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1537, 1034);
            this.tableLayoutPanel1.TabIndex = 0;
            // 
            // pnlCameraStream
            // 
            this.pnlCameraStream.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlCameraStream.Location = new System.Drawing.Point(1002, 63);
            this.pnlCameraStream.Name = "pnlCameraStream";
            this.pnlCameraStream.Size = new System.Drawing.Size(532, 392);
            this.pnlCameraStream.TabIndex = 23;
            // 
            // cmbCameras
            // 
            this.cmbCameras.Dock = System.Windows.Forms.DockStyle.Fill;
            this.cmbCameras.Font = new System.Drawing.Font("Microsoft Sans Serif", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmbCameras.FormattingEnabled = true;
            this.cmbCameras.Location = new System.Drawing.Point(1002, 3);
            this.cmbCameras.Name = "cmbCameras";
            this.cmbCameras.Size = new System.Drawing.Size(532, 54);
            this.cmbCameras.TabIndex = 22;
            this.cmbCameras.SelectedIndexChanged += new System.EventHandler(this.cmbCameras_SelectedIndexChanged);
            // 
            // pnlRobot3D
            // 
            this.pnlRobot3D.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlRobot3D.Location = new System.Drawing.Point(1002, 461);
            this.pnlRobot3D.Name = "pnlRobot3D";
            this.pnlRobot3D.Size = new System.Drawing.Size(532, 570);
            this.pnlRobot3D.TabIndex = 21;
            // 
            // pnlScara
            // 
            this.pnlScara.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pnlScara.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlScara.Location = new System.Drawing.Point(3, 461);
            this.pnlScara.Name = "pnlScara";
            this.pnlScara.Size = new System.Drawing.Size(993, 570);
            this.pnlScara.TabIndex = 20;
            // 
            // pnlFlexibowl
            // 
            this.pnlFlexibowl.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pnlFlexibowl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlFlexibowl.Location = new System.Drawing.Point(3, 3);
            this.pnlFlexibowl.Name = "pnlFlexibowl";
            this.tableLayoutPanel1.SetRowSpan(this.pnlFlexibowl, 2);
            this.pnlFlexibowl.Size = new System.Drawing.Size(993, 452);
            this.pnlFlexibowl.TabIndex = 24;
            // 
            // ucProcessView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "ucProcessView";
            this.Size = new System.Drawing.Size(1537, 1034);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Panel pnlScara;
        private System.Windows.Forms.Panel pnlRobot3D;
        private System.Windows.Forms.ComboBox cmbCameras;
        private System.Windows.Forms.Panel pnlCameraStream;
        private System.Windows.Forms.Panel pnlFlexibowl;
    }
}
