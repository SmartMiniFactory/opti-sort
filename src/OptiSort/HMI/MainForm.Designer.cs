namespace HMI
{
    partial class ucOptiSort
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

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.textBoxX = new System.Windows.Forms.TextBox();
            this.labelX = new System.Windows.Forms.Label();
            this.labelY = new System.Windows.Forms.Label();
            this.textBoxY = new System.Windows.Forms.TextBox();
            this.labelZ = new System.Windows.Forms.Label();
            this.textBoxZ = new System.Windows.Forms.TextBox();
            this.labelYaw = new System.Windows.Forms.Label();
            this.textBoxYaw = new System.Windows.Forms.TextBox();
            this.labelPitch = new System.Windows.Forms.Label();
            this.textBoxPitch = new System.Windows.Forms.TextBox();
            this.labelRoll = new System.Windows.Forms.Label();
            this.textBoxRoll = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBoxTargetQueue = new System.Windows.Forms.GroupBox();
            this._targetQueue = new System.Windows.Forms.DataGridView();
            this.pnlCameraStream = new System.Windows.Forms.Panel();
            this.cmbCameras = new System.Windows.Forms.ComboBox();
            this.panel1 = new System.Windows.Forms.Panel();
            this.lstLog = new System.Windows.Forms.ListBox();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.pnlRobot3D = new System.Windows.Forms.Panel();
            this.lblRobotIP = new System.Windows.Forms.Label();
            this.txtRobotIP = new System.Windows.Forms.TextBox();
            this.lblControllerIP = new System.Windows.Forms.Label();
            this.txtControllerIP = new System.Windows.Forms.TextBox();
            this.btnConnect = new System.Windows.Forms.Button();
            this.btnDisconnect = new System.Windows.Forms.Button();
            this.groupBox1.SuspendLayout();
            this.groupBoxTargetQueue.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this._targetQueue)).BeginInit();
            this.panel1.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // textBoxX
            // 
            this.textBoxX.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxX.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxX.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxX.Location = new System.Drawing.Point(62, 42);
            this.textBoxX.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxX.Name = "textBoxX";
            this.textBoxX.ReadOnly = true;
            this.textBoxX.Size = new System.Drawing.Size(148, 35);
            this.textBoxX.TabIndex = 0;
            // 
            // labelX
            // 
            this.labelX.AutoSize = true;
            this.labelX.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelX.Location = new System.Drawing.Point(8, 51);
            this.labelX.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.labelX.Name = "labelX";
            this.labelX.Size = new System.Drawing.Size(36, 29);
            this.labelX.TabIndex = 1;
            this.labelX.Text = "X:";
            // 
            // labelY
            // 
            this.labelY.AutoSize = true;
            this.labelY.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelY.Location = new System.Drawing.Point(9, 98);
            this.labelY.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.labelY.Name = "labelY";
            this.labelY.Size = new System.Drawing.Size(35, 29);
            this.labelY.TabIndex = 3;
            this.labelY.Text = "Y:";
            // 
            // textBoxY
            // 
            this.textBoxY.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxY.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxY.Location = new System.Drawing.Point(62, 91);
            this.textBoxY.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxY.Name = "textBoxY";
            this.textBoxY.ReadOnly = true;
            this.textBoxY.Size = new System.Drawing.Size(148, 35);
            this.textBoxY.TabIndex = 2;
            // 
            // labelZ
            // 
            this.labelZ.AutoSize = true;
            this.labelZ.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelZ.Location = new System.Drawing.Point(9, 148);
            this.labelZ.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.labelZ.Name = "labelZ";
            this.labelZ.Size = new System.Drawing.Size(33, 29);
            this.labelZ.TabIndex = 5;
            this.labelZ.Text = "Z:";
            // 
            // textBoxZ
            // 
            this.textBoxZ.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxZ.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxZ.Location = new System.Drawing.Point(62, 138);
            this.textBoxZ.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxZ.Name = "textBoxZ";
            this.textBoxZ.ReadOnly = true;
            this.textBoxZ.Size = new System.Drawing.Size(148, 35);
            this.textBoxZ.TabIndex = 4;
            // 
            // labelYaw
            // 
            this.labelYaw.AutoSize = true;
            this.labelYaw.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelYaw.Location = new System.Drawing.Point(232, 143);
            this.labelYaw.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.labelYaw.Name = "labelYaw";
            this.labelYaw.Size = new System.Drawing.Size(66, 29);
            this.labelYaw.TabIndex = 12;
            this.labelYaw.Text = "Yaw:";
            // 
            // textBoxYaw
            // 
            this.textBoxYaw.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxYaw.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxYaw.Location = new System.Drawing.Point(310, 138);
            this.textBoxYaw.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxYaw.Name = "textBoxYaw";
            this.textBoxYaw.ReadOnly = true;
            this.textBoxYaw.Size = new System.Drawing.Size(148, 35);
            this.textBoxYaw.TabIndex = 11;
            // 
            // labelPitch
            // 
            this.labelPitch.AutoSize = true;
            this.labelPitch.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelPitch.Location = new System.Drawing.Point(232, 98);
            this.labelPitch.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.labelPitch.Name = "labelPitch";
            this.labelPitch.Size = new System.Drawing.Size(72, 29);
            this.labelPitch.TabIndex = 10;
            this.labelPitch.Text = "Pitch:";
            // 
            // textBoxPitch
            // 
            this.textBoxPitch.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxPitch.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxPitch.Location = new System.Drawing.Point(310, 89);
            this.textBoxPitch.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxPitch.Name = "textBoxPitch";
            this.textBoxPitch.ReadOnly = true;
            this.textBoxPitch.Size = new System.Drawing.Size(148, 35);
            this.textBoxPitch.TabIndex = 9;
            // 
            // labelRoll
            // 
            this.labelRoll.AutoSize = true;
            this.labelRoll.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelRoll.Location = new System.Drawing.Point(242, 51);
            this.labelRoll.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.labelRoll.Name = "labelRoll";
            this.labelRoll.Size = new System.Drawing.Size(62, 29);
            this.labelRoll.TabIndex = 8;
            this.labelRoll.Text = "Roll:";
            // 
            // textBoxRoll
            // 
            this.textBoxRoll.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxRoll.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxRoll.Location = new System.Drawing.Point(310, 42);
            this.textBoxRoll.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxRoll.Name = "textBoxRoll";
            this.textBoxRoll.ReadOnly = true;
            this.textBoxRoll.Size = new System.Drawing.Size(148, 35);
            this.textBoxRoll.TabIndex = 7;
            // 
            // groupBox1
            // 
            this.groupBox1.BackColor = System.Drawing.Color.Tomato;
            this.groupBox1.Controls.Add(this.labelZ);
            this.groupBox1.Controls.Add(this.textBoxX);
            this.groupBox1.Controls.Add(this.labelYaw);
            this.groupBox1.Controls.Add(this.labelX);
            this.groupBox1.Controls.Add(this.textBoxYaw);
            this.groupBox1.Controls.Add(this.textBoxY);
            this.groupBox1.Controls.Add(this.labelPitch);
            this.groupBox1.Controls.Add(this.labelY);
            this.groupBox1.Controls.Add(this.textBoxPitch);
            this.groupBox1.Controls.Add(this.textBoxZ);
            this.groupBox1.Controls.Add(this.labelRoll);
            this.groupBox1.Controls.Add(this.textBoxRoll);
            this.groupBox1.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.groupBox1.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBox1.Location = new System.Drawing.Point(30, 330);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(471, 189);
            this.groupBox1.TabIndex = 14;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Current Target";
            // 
            // groupBoxTargetQueue
            // 
            this.groupBoxTargetQueue.BackColor = System.Drawing.Color.GreenYellow;
            this.groupBoxTargetQueue.Controls.Add(this._targetQueue);
            this.groupBoxTargetQueue.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBoxTargetQueue.ImeMode = System.Windows.Forms.ImeMode.Off;
            this.groupBoxTargetQueue.Location = new System.Drawing.Point(27, 540);
            this.groupBoxTargetQueue.Name = "groupBoxTargetQueue";
            this.groupBoxTargetQueue.Size = new System.Drawing.Size(773, 377);
            this.groupBoxTargetQueue.TabIndex = 17;
            this.groupBoxTargetQueue.TabStop = false;
            this.groupBoxTargetQueue.Text = "Target queue";
            // 
            // _targetQueue
            // 
            this._targetQueue.AllowUserToAddRows = false;
            this._targetQueue.AllowUserToDeleteRows = false;
            this._targetQueue.AllowUserToResizeColumns = false;
            this._targetQueue.AllowUserToResizeRows = false;
            this._targetQueue.BackgroundColor = System.Drawing.Color.Snow;
            this._targetQueue.CellBorderStyle = System.Windows.Forms.DataGridViewCellBorderStyle.Sunken;
            this._targetQueue.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this._targetQueue.Dock = System.Windows.Forms.DockStyle.Fill;
            this._targetQueue.GridColor = System.Drawing.SystemColors.Info;
            this._targetQueue.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this._targetQueue.Location = new System.Drawing.Point(3, 31);
            this._targetQueue.MultiSelect = false;
            this._targetQueue.Name = "_targetQueue";
            this._targetQueue.RowHeadersVisible = false;
            this._targetQueue.RowHeadersWidth = 62;
            this._targetQueue.RowTemplate.Height = 24;
            this._targetQueue.Size = new System.Drawing.Size(767, 343);
            this._targetQueue.TabIndex = 0;
            this._targetQueue.UseWaitCursor = true;
            // 
            // pnlCameraStream
            // 
            this.pnlCameraStream.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlCameraStream.Location = new System.Drawing.Point(3, 71);
            this.pnlCameraStream.Name = "pnlCameraStream";
            this.pnlCameraStream.Size = new System.Drawing.Size(672, 448);
            this.pnlCameraStream.TabIndex = 18;
            // 
            // cmbCameras
            // 
            this.cmbCameras.Dock = System.Windows.Forms.DockStyle.Fill;
            this.cmbCameras.Font = new System.Drawing.Font("Microsoft Sans Serif", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmbCameras.FormattingEnabled = true;
            this.cmbCameras.Location = new System.Drawing.Point(3, 3);
            this.cmbCameras.Name = "cmbCameras";
            this.cmbCameras.Size = new System.Drawing.Size(672, 54);
            this.cmbCameras.TabIndex = 0;
            // 
            // panel1
            // 
            this.panel1.Controls.Add(this.lstLog);
            this.panel1.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.panel1.Location = new System.Drawing.Point(0, 977);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(1818, 100);
            this.panel1.TabIndex = 0;
            // 
            // lstLog
            // 
            this.lstLog.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.lstLog.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lstLog.FormattingEnabled = true;
            this.lstLog.ItemHeight = 20;
            this.lstLog.Location = new System.Drawing.Point(0, 0);
            this.lstLog.Name = "lstLog";
            this.lstLog.Size = new System.Drawing.Size(1818, 100);
            this.lstLog.TabIndex = 0;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 1;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel1.Controls.Add(this.pnlRobot3D, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.pnlCameraStream, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.cmbCameras, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Right;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(1140, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 3;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 6.976745F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 46.51163F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 46.51163F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(678, 977);
            this.tableLayoutPanel1.TabIndex = 19;
            // 
            // pnlRobot3D
            // 
            this.pnlRobot3D.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlRobot3D.Location = new System.Drawing.Point(3, 525);
            this.pnlRobot3D.Name = "pnlRobot3D";
            this.pnlRobot3D.Size = new System.Drawing.Size(672, 449);
            this.pnlRobot3D.TabIndex = 19;
            // 
            // lblRobotIP
            // 
            this.lblRobotIP.AutoSize = true;
            this.lblRobotIP.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblRobotIP.Location = new System.Drawing.Point(24, 21);
            this.lblRobotIP.Name = "lblRobotIP";
            this.lblRobotIP.Size = new System.Drawing.Size(132, 36);
            this.lblRobotIP.TabIndex = 20;
            this.lblRobotIP.Text = "Robot IP";
            // 
            // txtRobotIP
            // 
            this.txtRobotIP.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtRobotIP.Location = new System.Drawing.Point(216, 21);
            this.txtRobotIP.Name = "txtRobotIP";
            this.txtRobotIP.Size = new System.Drawing.Size(285, 41);
            this.txtRobotIP.TabIndex = 21;
            // 
            // lblControllerIP
            // 
            this.lblControllerIP.AutoSize = true;
            this.lblControllerIP.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblControllerIP.Location = new System.Drawing.Point(24, 82);
            this.lblControllerIP.Name = "lblControllerIP";
            this.lblControllerIP.Size = new System.Drawing.Size(181, 36);
            this.lblControllerIP.TabIndex = 22;
            this.lblControllerIP.Text = "Controller IP";
            // 
            // txtControllerIP
            // 
            this.txtControllerIP.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtControllerIP.Location = new System.Drawing.Point(216, 79);
            this.txtControllerIP.Name = "txtControllerIP";
            this.txtControllerIP.Size = new System.Drawing.Size(285, 41);
            this.txtControllerIP.TabIndex = 23;
            // 
            // btnConnect
            // 
            this.btnConnect.Location = new System.Drawing.Point(523, 21);
            this.btnConnect.Name = "btnConnect";
            this.btnConnect.Size = new System.Drawing.Size(118, 41);
            this.btnConnect.TabIndex = 24;
            this.btnConnect.Text = "Connect";
            this.btnConnect.UseVisualStyleBackColor = true;
            this.btnConnect.Click += new System.EventHandler(this.btnConnect_Click);
            // 
            // btnDisconnect
            // 
            this.btnDisconnect.Enabled = false;
            this.btnDisconnect.Location = new System.Drawing.Point(523, 79);
            this.btnDisconnect.Name = "btnDisconnect";
            this.btnDisconnect.Size = new System.Drawing.Size(118, 41);
            this.btnDisconnect.TabIndex = 25;
            this.btnDisconnect.Text = "Disconnect";
            this.btnDisconnect.UseVisualStyleBackColor = true;
            // 
            // ucOptiSort
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Gainsboro;
            this.ClientSize = new System.Drawing.Size(1818, 1077);
            this.Controls.Add(this.btnDisconnect);
            this.Controls.Add(this.btnConnect);
            this.Controls.Add(this.txtControllerIP);
            this.Controls.Add(this.lblControllerIP);
            this.Controls.Add(this.txtRobotIP);
            this.Controls.Add(this.lblRobotIP);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.groupBoxTargetQueue);
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "ucOptiSort";
            this.Text = "SCARA Remote Control";
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBoxTargetQueue.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this._targetQueue)).EndInit();
            this.panel1.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox textBoxX;
        private System.Windows.Forms.Label labelX;
        private System.Windows.Forms.Label labelY;
        private System.Windows.Forms.TextBox textBoxY;
        private System.Windows.Forms.Label labelZ;
        private System.Windows.Forms.TextBox textBoxZ;
        private System.Windows.Forms.Label labelYaw;
        private System.Windows.Forms.TextBox textBoxYaw;
        private System.Windows.Forms.Label labelPitch;
        private System.Windows.Forms.TextBox textBoxPitch;
        private System.Windows.Forms.Label labelRoll;
        private System.Windows.Forms.TextBox textBoxRoll;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBoxTargetQueue;
        private System.Windows.Forms.DataGridView _targetQueue;
        private System.Windows.Forms.Panel pnlCameraStream;
        private System.Windows.Forms.ComboBox cmbCameras;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.ListBox lstLog;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Panel pnlRobot3D;
        private System.Windows.Forms.Label lblRobotIP;
        private System.Windows.Forms.TextBox txtRobotIP;
        private System.Windows.Forms.Label lblControllerIP;
        private System.Windows.Forms.TextBox txtControllerIP;
        private System.Windows.Forms.Button btnConnect;
        private System.Windows.Forms.Button btnDisconnect;
    }
}

