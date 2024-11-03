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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(frmMain));
            this.lstLog = new System.Windows.Forms.ListBox();
            this.tblPanel = new System.Windows.Forms.TableLayoutPanel();
            this.btnConfig = new System.Windows.Forms.Button();
            this.btnManual = new System.Windows.Forms.Button();
            this.btnProcess = new System.Windows.Forms.Button();
            this.pnlCurrentUc = new System.Windows.Forms.Panel();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.btnEmulateScara = new System.Windows.Forms.Button();
            this.btnMqttDisconnect = new System.Windows.Forms.Button();
            this.btnFlexibowlDisconnect = new System.Windows.Forms.Button();
            this.btnScaraDisconnect = new System.Windows.Forms.Button();
            this.btnMqttConnect = new System.Windows.Forms.Button();
            this.btnFlexibowlConnect = new System.Windows.Forms.Button();
            this.lblMqttStatusValue = new System.Windows.Forms.Label();
            this.lblMqttStatus = new System.Windows.Forms.Label();
            this.lblFlexibowlStatusValue = new System.Windows.Forms.Label();
            this.lblFlexibowlStatus = new System.Windows.Forms.Label();
            this.lblScaraStatusValue = new System.Windows.Forms.Label();
            this.lblScaraStatus = new System.Windows.Forms.Label();
            this.btnScaraConnect = new System.Windows.Forms.Button();
            this.tblMainStructure = new System.Windows.Forms.TableLayoutPanel();
            this.pnlRobotView = new System.Windows.Forms.Panel();
            this.cmbCameras = new System.Windows.Forms.ComboBox();
            this.pnlCameraStream = new System.Windows.Forms.Panel();
            this.tblPanel.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.tblMainStructure.SuspendLayout();
            this.SuspendLayout();
            // 
            // lstLog
            // 
            this.lstLog.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.lstLog.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lstLog.FormattingEnabled = true;
            this.lstLog.ItemHeight = 20;
            this.lstLog.Location = new System.Drawing.Point(3, 3);
            this.lstLog.Name = "lstLog";
            this.lstLog.Size = new System.Drawing.Size(1322, 107);
            this.lstLog.TabIndex = 0;
            // 
            // tblPanel
            // 
            this.tblPanel.ColumnCount = 4;
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 70F));
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tblPanel.Controls.Add(this.btnConfig, 3, 0);
            this.tblPanel.Controls.Add(this.btnManual, 2, 0);
            this.tblPanel.Controls.Add(this.lstLog, 0, 0);
            this.tblPanel.Controls.Add(this.btnProcess, 1, 0);
            this.tblPanel.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.tblPanel.Location = new System.Drawing.Point(0, 911);
            this.tblPanel.Name = "tblPanel";
            this.tblPanel.RowCount = 1;
            this.tblPanel.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tblPanel.Size = new System.Drawing.Size(1898, 113);
            this.tblPanel.TabIndex = 1;
            // 
            // btnConfig
            // 
            this.btnConfig.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnConfig.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnConfig.Location = new System.Drawing.Point(1709, 3);
            this.btnConfig.Name = "btnConfig";
            this.btnConfig.Size = new System.Drawing.Size(186, 107);
            this.btnConfig.TabIndex = 3;
            this.btnConfig.TabStop = false;
            this.btnConfig.Text = "CONFIG";
            this.btnConfig.UseVisualStyleBackColor = true;
            this.btnConfig.Click += new System.EventHandler(this.btnConfig_Click);
            // 
            // btnManual
            // 
            this.btnManual.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnManual.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnManual.Location = new System.Drawing.Point(1520, 3);
            this.btnManual.Name = "btnManual";
            this.btnManual.Size = new System.Drawing.Size(183, 107);
            this.btnManual.TabIndex = 2;
            this.btnManual.TabStop = false;
            this.btnManual.Text = "MANUAL";
            this.btnManual.UseVisualStyleBackColor = true;
            this.btnManual.Click += new System.EventHandler(this.btnManual_Click);
            // 
            // btnProcess
            // 
            this.btnProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnProcess.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnProcess.Location = new System.Drawing.Point(1331, 3);
            this.btnProcess.Name = "btnProcess";
            this.btnProcess.Size = new System.Drawing.Size(183, 107);
            this.btnProcess.TabIndex = 1;
            this.btnProcess.TabStop = false;
            this.btnProcess.Text = "AUTO";
            this.btnProcess.UseVisualStyleBackColor = true;
            this.btnProcess.Click += new System.EventHandler(this.btnProcess_Click);
            // 
            // pnlCurrentUc
            // 
            this.pnlCurrentUc.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pnlCurrentUc.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlCurrentUc.Location = new System.Drawing.Point(3, 3);
            this.pnlCurrentUc.Name = "pnlCurrentUc";
            this.tblMainStructure.SetRowSpan(this.pnlCurrentUc, 3);
            this.pnlCurrentUc.Size = new System.Drawing.Size(1322, 776);
            this.pnlCurrentUc.TabIndex = 4;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 10;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.33333F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 100F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 80F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.33333F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 80F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.33333F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 84F));
            this.tableLayoutPanel1.Controls.Add(this.btnEmulateScara, 2, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnMqttDisconnect, 9, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnFlexibowlDisconnect, 6, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnScaraDisconnect, 3, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnMqttConnect, 9, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnFlexibowlConnect, 6, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblMqttStatusValue, 8, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblMqttStatus, 7, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblFlexibowlStatusValue, 5, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblFlexibowlStatus, 4, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblScaraStatusValue, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblScaraStatus, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnScaraConnect, 3, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Top;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 2;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1898, 129);
            this.tableLayoutPanel1.TabIndex = 2;
            // 
            // btnEmulateScara
            // 
            this.btnEmulateScara.BackgroundImage = global::OptiSort.Properties.Resources.emulation_2x2_pptx;
            this.btnEmulateScara.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnEmulateScara.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnEmulateScara.Location = new System.Drawing.Point(520, 3);
            this.btnEmulateScara.Name = "btnEmulateScara";
            this.tableLayoutPanel1.SetRowSpan(this.btnEmulateScara, 2);
            this.btnEmulateScara.Size = new System.Drawing.Size(94, 123);
            this.btnEmulateScara.TabIndex = 12;
            this.btnEmulateScara.UseVisualStyleBackColor = true;
            this.btnEmulateScara.Click += new System.EventHandler(this.btnEmulateScara_Click);
            // 
            // btnMqttDisconnect
            // 
            this.btnMqttDisconnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnMqttDisconnect.BackgroundImage")));
            this.btnMqttDisconnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnMqttDisconnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnMqttDisconnect.Enabled = false;
            this.btnMqttDisconnect.Location = new System.Drawing.Point(1814, 67);
            this.btnMqttDisconnect.Name = "btnMqttDisconnect";
            this.btnMqttDisconnect.Size = new System.Drawing.Size(81, 59);
            this.btnMqttDisconnect.TabIndex = 11;
            this.btnMqttDisconnect.UseVisualStyleBackColor = true;
            this.btnMqttDisconnect.Click += new System.EventHandler(this.btnMqttDisconnect_Click);
            // 
            // btnFlexibowlDisconnect
            // 
            this.btnFlexibowlDisconnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnFlexibowlDisconnect.BackgroundImage")));
            this.btnFlexibowlDisconnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnFlexibowlDisconnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFlexibowlDisconnect.Enabled = false;
            this.btnFlexibowlDisconnect.Location = new System.Drawing.Point(1217, 67);
            this.btnFlexibowlDisconnect.Name = "btnFlexibowlDisconnect";
            this.btnFlexibowlDisconnect.Size = new System.Drawing.Size(74, 59);
            this.btnFlexibowlDisconnect.TabIndex = 10;
            this.btnFlexibowlDisconnect.UseVisualStyleBackColor = true;
            this.btnFlexibowlDisconnect.Click += new System.EventHandler(this.btnFlexibowlDisconnect_Click);
            // 
            // btnScaraDisconnect
            // 
            this.btnScaraDisconnect.BackgroundImage = global::OptiSort.Properties.Resources.disconnectedDisabled_2x2_pptx;
            this.btnScaraDisconnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnScaraDisconnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnScaraDisconnect.Enabled = false;
            this.btnScaraDisconnect.Location = new System.Drawing.Point(620, 67);
            this.btnScaraDisconnect.Name = "btnScaraDisconnect";
            this.btnScaraDisconnect.Size = new System.Drawing.Size(74, 59);
            this.btnScaraDisconnect.TabIndex = 9;
            this.btnScaraDisconnect.UseVisualStyleBackColor = true;
            this.btnScaraDisconnect.Click += new System.EventHandler(this.btnScaraDisconnect_Click);
            // 
            // btnMqttConnect
            // 
            this.btnMqttConnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnMqttConnect.BackgroundImage")));
            this.btnMqttConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnMqttConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnMqttConnect.Location = new System.Drawing.Point(1814, 3);
            this.btnMqttConnect.Name = "btnMqttConnect";
            this.btnMqttConnect.Size = new System.Drawing.Size(81, 58);
            this.btnMqttConnect.TabIndex = 8;
            this.btnMqttConnect.UseVisualStyleBackColor = true;
            this.btnMqttConnect.Click += new System.EventHandler(this.btnMqttConnect_Click);
            // 
            // btnFlexibowlConnect
            // 
            this.btnFlexibowlConnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnFlexibowlConnect.BackgroundImage")));
            this.btnFlexibowlConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnFlexibowlConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFlexibowlConnect.Location = new System.Drawing.Point(1217, 3);
            this.btnFlexibowlConnect.Name = "btnFlexibowlConnect";
            this.btnFlexibowlConnect.Size = new System.Drawing.Size(74, 58);
            this.btnFlexibowlConnect.TabIndex = 7;
            this.btnFlexibowlConnect.UseVisualStyleBackColor = true;
            this.btnFlexibowlConnect.Click += new System.EventHandler(this.btnFlexibowlConnect_Click);
            // 
            // lblMqttStatusValue
            // 
            this.lblMqttStatusValue.AutoSize = true;
            this.lblMqttStatusValue.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblMqttStatusValue.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblMqttStatusValue.Location = new System.Drawing.Point(1607, 0);
            this.lblMqttStatusValue.Name = "lblMqttStatusValue";
            this.tableLayoutPanel1.SetRowSpan(this.lblMqttStatusValue, 2);
            this.lblMqttStatusValue.Size = new System.Drawing.Size(201, 129);
            this.lblMqttStatusValue.TabIndex = 5;
            this.lblMqttStatusValue.Text = "Offline";
            this.lblMqttStatusValue.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lblMqttStatus
            // 
            this.lblMqttStatus.AutoSize = true;
            this.lblMqttStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblMqttStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblMqttStatus.Location = new System.Drawing.Point(1297, 0);
            this.lblMqttStatus.Name = "lblMqttStatus";
            this.tableLayoutPanel1.SetRowSpan(this.lblMqttStatus, 2);
            this.lblMqttStatus.Size = new System.Drawing.Size(304, 129);
            this.lblMqttStatus.TabIndex = 4;
            this.lblMqttStatus.Text = "MQTT Client:";
            this.lblMqttStatus.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lblFlexibowlStatusValue
            // 
            this.lblFlexibowlStatusValue.AutoSize = true;
            this.lblFlexibowlStatusValue.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblFlexibowlStatusValue.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFlexibowlStatusValue.Location = new System.Drawing.Point(1010, 0);
            this.lblFlexibowlStatusValue.Name = "lblFlexibowlStatusValue";
            this.tableLayoutPanel1.SetRowSpan(this.lblFlexibowlStatusValue, 2);
            this.lblFlexibowlStatusValue.Size = new System.Drawing.Size(201, 129);
            this.lblFlexibowlStatusValue.TabIndex = 3;
            this.lblFlexibowlStatusValue.Text = "Offline";
            this.lblFlexibowlStatusValue.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lblFlexibowlStatus
            // 
            this.lblFlexibowlStatus.AutoSize = true;
            this.lblFlexibowlStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblFlexibowlStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFlexibowlStatus.Location = new System.Drawing.Point(700, 0);
            this.lblFlexibowlStatus.Name = "lblFlexibowlStatus";
            this.tableLayoutPanel1.SetRowSpan(this.lblFlexibowlStatus, 2);
            this.lblFlexibowlStatus.Size = new System.Drawing.Size(304, 129);
            this.lblFlexibowlStatus.TabIndex = 2;
            this.lblFlexibowlStatus.Text = "Flexibowl:";
            this.lblFlexibowlStatus.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lblScaraStatusValue
            // 
            this.lblScaraStatusValue.AutoSize = true;
            this.lblScaraStatusValue.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblScaraStatusValue.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblScaraStatusValue.Location = new System.Drawing.Point(313, 0);
            this.lblScaraStatusValue.Name = "lblScaraStatusValue";
            this.tableLayoutPanel1.SetRowSpan(this.lblScaraStatusValue, 2);
            this.lblScaraStatusValue.Size = new System.Drawing.Size(201, 129);
            this.lblScaraStatusValue.TabIndex = 1;
            this.lblScaraStatusValue.Text = "Offline";
            this.lblScaraStatusValue.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lblScaraStatus
            // 
            this.lblScaraStatus.AutoSize = true;
            this.lblScaraStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblScaraStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblScaraStatus.Location = new System.Drawing.Point(3, 0);
            this.lblScaraStatus.Name = "lblScaraStatus";
            this.tableLayoutPanel1.SetRowSpan(this.lblScaraStatus, 2);
            this.lblScaraStatus.Size = new System.Drawing.Size(304, 129);
            this.lblScaraStatus.TabIndex = 0;
            this.lblScaraStatus.Text = "Scara:";
            this.lblScaraStatus.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // btnScaraConnect
            // 
            this.btnScaraConnect.BackgroundImage = global::OptiSort.Properties.Resources.connectedEnabled_2x2_pptx;
            this.btnScaraConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnScaraConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnScaraConnect.Location = new System.Drawing.Point(620, 3);
            this.btnScaraConnect.Name = "btnScaraConnect";
            this.btnScaraConnect.Size = new System.Drawing.Size(74, 58);
            this.btnScaraConnect.TabIndex = 6;
            this.btnScaraConnect.UseVisualStyleBackColor = true;
            this.btnScaraConnect.Click += new System.EventHandler(this.btnScaraConnect_Click);
            // 
            // tblMainStructure
            // 
            this.tblMainStructure.ColumnCount = 2;
            this.tblMainStructure.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 70F));
            this.tblMainStructure.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 30F));
            this.tblMainStructure.Controls.Add(this.pnlCurrentUc, 0, 0);
            this.tblMainStructure.Controls.Add(this.pnlRobotView, 1, 2);
            this.tblMainStructure.Controls.Add(this.cmbCameras, 1, 0);
            this.tblMainStructure.Controls.Add(this.pnlCameraStream, 1, 1);
            this.tblMainStructure.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tblMainStructure.Location = new System.Drawing.Point(0, 129);
            this.tblMainStructure.Name = "tblMainStructure";
            this.tblMainStructure.RowCount = 3;
            this.tblMainStructure.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tblMainStructure.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 45F));
            this.tblMainStructure.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 45F));
            this.tblMainStructure.Size = new System.Drawing.Size(1898, 782);
            this.tblMainStructure.TabIndex = 5;
            // 
            // pnlRobotView
            // 
            this.pnlRobotView.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlRobotView.Location = new System.Drawing.Point(1331, 432);
            this.pnlRobotView.Name = "pnlRobotView";
            this.pnlRobotView.Size = new System.Drawing.Size(564, 347);
            this.pnlRobotView.TabIndex = 6;
            this.pnlRobotView.Click += new System.EventHandler(this.pnlRobotView_MouseClick);
            // 
            // cmbCameras
            // 
            this.cmbCameras.Dock = System.Windows.Forms.DockStyle.Fill;
            this.cmbCameras.Font = new System.Drawing.Font("Microsoft Sans Serif", 25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmbCameras.FormattingEnabled = true;
            this.cmbCameras.Location = new System.Drawing.Point(1331, 3);
            this.cmbCameras.Name = "cmbCameras";
            this.cmbCameras.Size = new System.Drawing.Size(564, 66);
            this.cmbCameras.TabIndex = 7;
            this.cmbCameras.SelectedIndexChanged += new System.EventHandler(this.cmbCameras_SelectedIndexChanged);
            // 
            // pnlCameraStream
            // 
            this.pnlCameraStream.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlCameraStream.Location = new System.Drawing.Point(1331, 81);
            this.pnlCameraStream.Name = "pnlCameraStream";
            this.pnlCameraStream.Size = new System.Drawing.Size(564, 345);
            this.pnlCameraStream.TabIndex = 5;
            // 
            // frmMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Gainsboro;
            this.ClientSize = new System.Drawing.Size(1898, 1024);
            this.Controls.Add(this.tblMainStructure);
            this.Controls.Add(this.tblPanel);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "frmMain";
            this.Text = "OptiSort";
            this.tblPanel.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.tblMainStructure.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ListBox lstLog;
        private System.Windows.Forms.TableLayoutPanel tblPanel;
        private System.Windows.Forms.Button btnProcess;
        private System.Windows.Forms.Button btnConfig;
        private System.Windows.Forms.Button btnManual;
        private System.Windows.Forms.Panel pnlCurrentUc;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Label lblScaraStatusValue;
        private System.Windows.Forms.Label lblScaraStatus;
        private System.Windows.Forms.Label lblMqttStatusValue;
        private System.Windows.Forms.Label lblMqttStatus;
        private System.Windows.Forms.Label lblFlexibowlStatusValue;
        private System.Windows.Forms.Label lblFlexibowlStatus;
        private System.Windows.Forms.Button btnScaraConnect;
        private System.Windows.Forms.Button btnMqttDisconnect;
        private System.Windows.Forms.Button btnFlexibowlDisconnect;
        private System.Windows.Forms.Button btnScaraDisconnect;
        private System.Windows.Forms.Button btnMqttConnect;
        private System.Windows.Forms.Button btnFlexibowlConnect;
        private System.Windows.Forms.TableLayoutPanel tblMainStructure;
        private System.Windows.Forms.Panel pnlCameraStream;
        private System.Windows.Forms.Panel pnlRobotView;
        private System.Windows.Forms.ComboBox cmbCameras;
        private System.Windows.Forms.Button btnEmulateScara;
    }
}

