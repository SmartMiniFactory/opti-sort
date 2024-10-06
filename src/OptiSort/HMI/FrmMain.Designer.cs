namespace OptiSort
{
    partial class frmMain
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
            this.tblRobots = new System.Windows.Forms.TableLayoutPanel();
            this.pnlConfig = new System.Windows.Forms.Panel();
            this.pnlScara = new System.Windows.Forms.Panel();
            this.panel1.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.tblRobots.SuspendLayout();
            this.pnlConfig.SuspendLayout();
            this.SuspendLayout();
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
            this.cmbCameras.SelectedIndexChanged += new System.EventHandler(this.cmbCameras_SelectedIndexChanged);
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
            this.lblRobotIP.Location = new System.Drawing.Point(27, 18);
            this.lblRobotIP.Name = "lblRobotIP";
            this.lblRobotIP.Size = new System.Drawing.Size(132, 36);
            this.lblRobotIP.TabIndex = 20;
            this.lblRobotIP.Text = "Robot IP";
            // 
            // txtRobotIP
            // 
            this.txtRobotIP.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtRobotIP.Location = new System.Drawing.Point(219, 18);
            this.txtRobotIP.Name = "txtRobotIP";
            this.txtRobotIP.Size = new System.Drawing.Size(285, 41);
            this.txtRobotIP.TabIndex = 21;
            // 
            // lblControllerIP
            // 
            this.lblControllerIP.AutoSize = true;
            this.lblControllerIP.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblControllerIP.Location = new System.Drawing.Point(27, 79);
            this.lblControllerIP.Name = "lblControllerIP";
            this.lblControllerIP.Size = new System.Drawing.Size(181, 36);
            this.lblControllerIP.TabIndex = 22;
            this.lblControllerIP.Text = "Controller IP";
            // 
            // txtControllerIP
            // 
            this.txtControllerIP.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtControllerIP.Location = new System.Drawing.Point(219, 76);
            this.txtControllerIP.Name = "txtControllerIP";
            this.txtControllerIP.Size = new System.Drawing.Size(285, 41);
            this.txtControllerIP.TabIndex = 23;
            // 
            // btnConnect
            // 
            this.btnConnect.Location = new System.Drawing.Point(526, 18);
            this.btnConnect.Name = "btnConnect";
            this.btnConnect.Size = new System.Drawing.Size(118, 41);
            this.btnConnect.TabIndex = 24;
            this.btnConnect.Text = "Connect";
            this.btnConnect.UseVisualStyleBackColor = true;
            // 
            // btnDisconnect
            // 
            this.btnDisconnect.Enabled = false;
            this.btnDisconnect.Location = new System.Drawing.Point(526, 76);
            this.btnDisconnect.Name = "btnDisconnect";
            this.btnDisconnect.Size = new System.Drawing.Size(118, 41);
            this.btnDisconnect.TabIndex = 25;
            this.btnDisconnect.Text = "Disconnect";
            this.btnDisconnect.UseVisualStyleBackColor = true;
            // 
            // tblRobots
            // 
            this.tblRobots.ColumnCount = 1;
            this.tblRobots.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tblRobots.Controls.Add(this.pnlConfig, 0, 0);
            this.tblRobots.Controls.Add(this.pnlScara, 0, 2);
            this.tblRobots.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tblRobots.Location = new System.Drawing.Point(0, 0);
            this.tblRobots.Name = "tblRobots";
            this.tblRobots.RowCount = 3;
            this.tblRobots.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 35F));
            this.tblRobots.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tblRobots.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 55F));
            this.tblRobots.Size = new System.Drawing.Size(1140, 977);
            this.tblRobots.TabIndex = 26;
            // 
            // pnlConfig
            // 
            this.pnlConfig.Controls.Add(this.lblRobotIP);
            this.pnlConfig.Controls.Add(this.btnDisconnect);
            this.pnlConfig.Controls.Add(this.txtRobotIP);
            this.pnlConfig.Controls.Add(this.btnConnect);
            this.pnlConfig.Controls.Add(this.lblControllerIP);
            this.pnlConfig.Controls.Add(this.txtControllerIP);
            this.pnlConfig.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlConfig.Location = new System.Drawing.Point(3, 3);
            this.pnlConfig.Name = "pnlConfig";
            this.pnlConfig.Size = new System.Drawing.Size(1134, 335);
            this.pnlConfig.TabIndex = 18;
            // 
            // pnlScara
            // 
            this.pnlScara.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlScara.Location = new System.Drawing.Point(3, 441);
            this.pnlScara.Name = "pnlScara";
            this.pnlScara.Size = new System.Drawing.Size(1134, 533);
            this.pnlScara.TabIndex = 19;
            // 
            // frmMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Gainsboro;
            this.ClientSize = new System.Drawing.Size(1818, 1077);
            this.Controls.Add(this.tblRobots);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Controls.Add(this.panel1);
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "frmMain";
            this.Text = "SCARA Remote Control";
            this.panel1.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tblRobots.ResumeLayout(false);
            this.pnlConfig.ResumeLayout(false);
            this.pnlConfig.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
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
        private System.Windows.Forms.TableLayoutPanel tblRobots;
        private System.Windows.Forms.Panel pnlConfig;
        private System.Windows.Forms.Panel pnlScara;
    }
}

