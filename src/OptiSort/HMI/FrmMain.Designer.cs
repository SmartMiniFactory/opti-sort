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
            this.pnlCurrentUc = new System.Windows.Forms.Panel();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.lblCamerasStatus = new System.Windows.Forms.Label();
            this.lblMqttStatus = new System.Windows.Forms.Label();
            this.lblFlexibowlStatus = new System.Windows.Forms.Label();
            this.lblScaraStatus = new System.Windows.Forms.Label();
            this.tblMainStructure = new System.Windows.Forms.TableLayoutPanel();
            this.pnlRobotView = new System.Windows.Forms.Panel();
            this.cmbCameras = new System.Windows.Forms.ComboBox();
            this.pnlCameraStream = new System.Windows.Forms.Panel();
            this.lblDtStatus = new System.Windows.Forms.Label();
            this.panel1 = new System.Windows.Forms.Panel();
            this.panel2 = new System.Windows.Forms.Panel();
            this.panel3 = new System.Windows.Forms.Panel();
            this.panel4 = new System.Windows.Forms.Panel();
            this.btnStop = new System.Windows.Forms.Button();
            this.btnRun = new System.Windows.Forms.Button();
            this.btnConfig = new System.Windows.Forms.Button();
            this.btnManual = new System.Windows.Forms.Button();
            this.btnAuto = new System.Windows.Forms.Button();
            this.btnDtDisconnect = new System.Windows.Forms.Button();
            this.btnDtConnect = new System.Windows.Forms.Button();
            this.btnCameraTesting = new System.Windows.Forms.Button();
            this.btnCamerasDisconnect = new System.Windows.Forms.Button();
            this.btnCamerasConnect = new System.Windows.Forms.Button();
            this.btnEmulateScara = new System.Windows.Forms.Button();
            this.btnMqttDisconnect = new System.Windows.Forms.Button();
            this.btnFlexibowlDisconnect = new System.Windows.Forms.Button();
            this.btnScaraDisconnect = new System.Windows.Forms.Button();
            this.btnMqttConnect = new System.Windows.Forms.Button();
            this.btnFlexibowlConnect = new System.Windows.Forms.Button();
            this.btnScaraConnect = new System.Windows.Forms.Button();
            this.pbDtStatus = new System.Windows.Forms.PictureBox();
            this.pbScaraStatus = new System.Windows.Forms.PictureBox();
            this.pbFlexibowlStatus = new System.Windows.Forms.PictureBox();
            this.pbMqttStatus = new System.Windows.Forms.PictureBox();
            this.pbCameraStatus = new System.Windows.Forms.PictureBox();
            this.tblPanel.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.tblMainStructure.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pbDtStatus)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbScaraStatus)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbFlexibowlStatus)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbMqttStatus)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbCameraStatus)).BeginInit();
            this.SuspendLayout();
            // 
            // lstLog
            // 
            this.lstLog.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.lstLog.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lstLog.DrawMode = System.Windows.Forms.DrawMode.OwnerDrawFixed;
            this.lstLog.FormattingEnabled = true;
            this.lstLog.ItemHeight = 20;
            this.lstLog.Location = new System.Drawing.Point(3, 3);
            this.lstLog.Name = "lstLog";
            this.tblPanel.SetRowSpan(this.lstLog, 2);
            this.lstLog.Size = new System.Drawing.Size(1417, 107);
            this.lstLog.TabIndex = 0;
            this.lstLog.DrawItem += new System.Windows.Forms.DrawItemEventHandler(this.LstLog_DrawItem);
            // 
            // tblPanel
            // 
            this.tblPanel.ColumnCount = 5;
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 75.0015F));
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 5.0001F));
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666133F));
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666133F));
            this.tblPanel.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666133F));
            this.tblPanel.Controls.Add(this.btnStop, 1, 1);
            this.tblPanel.Controls.Add(this.btnRun, 1, 0);
            this.tblPanel.Controls.Add(this.btnConfig, 4, 0);
            this.tblPanel.Controls.Add(this.btnManual, 3, 0);
            this.tblPanel.Controls.Add(this.lstLog, 0, 0);
            this.tblPanel.Controls.Add(this.btnAuto, 2, 0);
            this.tblPanel.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.tblPanel.Location = new System.Drawing.Point(0, 911);
            this.tblPanel.Name = "tblPanel";
            this.tblPanel.RowCount = 2;
            this.tblPanel.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tblPanel.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tblPanel.Size = new System.Drawing.Size(1898, 113);
            this.tblPanel.TabIndex = 1;
            // 
            // pnlCurrentUc
            // 
            this.pnlCurrentUc.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlCurrentUc.Location = new System.Drawing.Point(5, 5);
            this.pnlCurrentUc.Name = "pnlCurrentUc";
            this.tblMainStructure.SetRowSpan(this.pnlCurrentUc, 3);
            this.pnlCurrentUc.Size = new System.Drawing.Size(1413, 772);
            this.pnlCurrentUc.TabIndex = 4;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 19;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 6.666667F));
            this.tableLayoutPanel1.Controls.Add(this.pbCameraStatus, 14, 0);
            this.tableLayoutPanel1.Controls.Add(this.pbMqttStatus, 10, 0);
            this.tableLayoutPanel1.Controls.Add(this.pbFlexibowlStatus, 6, 0);
            this.tableLayoutPanel1.Controls.Add(this.pbScaraStatus, 2, 0);
            this.tableLayoutPanel1.Controls.Add(this.panel4, 15, 0);
            this.tableLayoutPanel1.Controls.Add(this.panel3, 11, 0);
            this.tableLayoutPanel1.Controls.Add(this.panel2, 7, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnDtDisconnect, 18, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnDtConnect, 17, 1);
            this.tableLayoutPanel1.Controls.Add(this.lblDtStatus, 16, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnCameraTesting, 12, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnCamerasDisconnect, 14, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnCamerasConnect, 13, 1);
            this.tableLayoutPanel1.Controls.Add(this.lblCamerasStatus, 12, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnEmulateScara, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnMqttDisconnect, 10, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnFlexibowlDisconnect, 6, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnScaraDisconnect, 2, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnMqttConnect, 9, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnFlexibowlConnect, 5, 1);
            this.tableLayoutPanel1.Controls.Add(this.lblMqttStatus, 8, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblFlexibowlStatus, 4, 0);
            this.tableLayoutPanel1.Controls.Add(this.lblScaraStatus, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnScaraConnect, 1, 1);
            this.tableLayoutPanel1.Controls.Add(this.panel1, 3, 0);
            this.tableLayoutPanel1.Controls.Add(this.pbDtStatus, 18, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Top;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 2;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1898, 129);
            this.tableLayoutPanel1.TabIndex = 2;
            // 
            // lblCamerasStatus
            // 
            this.lblCamerasStatus.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.lblCamerasStatus, 2);
            this.lblCamerasStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblCamerasStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblCamerasStatus.Location = new System.Drawing.Point(1152, 0);
            this.lblCamerasStatus.Name = "lblCamerasStatus";
            this.lblCamerasStatus.Size = new System.Drawing.Size(236, 64);
            this.lblCamerasStatus.TabIndex = 13;
            this.lblCamerasStatus.Text = "CAMERAS";
            this.lblCamerasStatus.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblMqttStatus
            // 
            this.lblMqttStatus.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.lblMqttStatus, 2);
            this.lblMqttStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblMqttStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblMqttStatus.Location = new System.Drawing.Point(769, 0);
            this.lblMqttStatus.Name = "lblMqttStatus";
            this.lblMqttStatus.Size = new System.Drawing.Size(236, 64);
            this.lblMqttStatus.TabIndex = 4;
            this.lblMqttStatus.Text = "MQTT";
            this.lblMqttStatus.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblFlexibowlStatus
            // 
            this.lblFlexibowlStatus.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.lblFlexibowlStatus, 2);
            this.lblFlexibowlStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblFlexibowlStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFlexibowlStatus.Location = new System.Drawing.Point(386, 0);
            this.lblFlexibowlStatus.Name = "lblFlexibowlStatus";
            this.lblFlexibowlStatus.Size = new System.Drawing.Size(236, 64);
            this.lblFlexibowlStatus.TabIndex = 2;
            this.lblFlexibowlStatus.Text = "FLEXIBOWL";
            this.lblFlexibowlStatus.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblScaraStatus
            // 
            this.lblScaraStatus.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.lblScaraStatus, 2);
            this.lblScaraStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblScaraStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblScaraStatus.Location = new System.Drawing.Point(3, 0);
            this.lblScaraStatus.Name = "lblScaraStatus";
            this.lblScaraStatus.Size = new System.Drawing.Size(236, 64);
            this.lblScaraStatus.TabIndex = 0;
            this.lblScaraStatus.Text = "SCARA";
            this.lblScaraStatus.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tblMainStructure
            // 
            this.tblMainStructure.CellBorderStyle = System.Windows.Forms.TableLayoutPanelCellBorderStyle.Inset;
            this.tblMainStructure.ColumnCount = 2;
            this.tblMainStructure.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 75F));
            this.tblMainStructure.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 25F));
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
            this.pnlRobotView.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pnlRobotView.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlRobotView.Location = new System.Drawing.Point(1426, 434);
            this.pnlRobotView.Name = "pnlRobotView";
            this.pnlRobotView.Size = new System.Drawing.Size(467, 343);
            this.pnlRobotView.TabIndex = 6;
            // 
            // cmbCameras
            // 
            this.cmbCameras.Dock = System.Windows.Forms.DockStyle.Fill;
            this.cmbCameras.DropDownHeight = 120;
            this.cmbCameras.Font = new System.Drawing.Font("Microsoft Sans Serif", 25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmbCameras.FormattingEnabled = true;
            this.cmbCameras.IntegralHeight = false;
            this.cmbCameras.Location = new System.Drawing.Point(1426, 5);
            this.cmbCameras.Name = "cmbCameras";
            this.cmbCameras.Size = new System.Drawing.Size(467, 66);
            this.cmbCameras.TabIndex = 7;
            this.cmbCameras.SelectedIndexChanged += new System.EventHandler(this.cmbCameras_SelectedIndexChanged);
            // 
            // pnlCameraStream
            // 
            this.pnlCameraStream.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pnlCameraStream.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlCameraStream.Location = new System.Drawing.Point(1426, 84);
            this.pnlCameraStream.Name = "pnlCameraStream";
            this.pnlCameraStream.Size = new System.Drawing.Size(467, 342);
            this.pnlCameraStream.TabIndex = 5;
            // 
            // lblDtStatus
            // 
            this.lblDtStatus.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.lblDtStatus, 2);
            this.lblDtStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblDtStatus.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblDtStatus.Location = new System.Drawing.Point(1535, 0);
            this.lblDtStatus.Name = "lblDtStatus";
            this.lblDtStatus.Size = new System.Drawing.Size(236, 64);
            this.lblDtStatus.TabIndex = 20;
            this.lblDtStatus.Text = "DIGITAL TWIN";
            this.lblDtStatus.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.Black;
            this.panel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel1.Location = new System.Drawing.Point(373, 0);
            this.panel1.Margin = new System.Windows.Forms.Padding(10, 0, 10, 0);
            this.panel1.Name = "panel1";
            this.tableLayoutPanel1.SetRowSpan(this.panel1, 2);
            this.panel1.Size = new System.Drawing.Size(1, 129);
            this.panel1.TabIndex = 24;
            // 
            // panel2
            // 
            this.panel2.BackColor = System.Drawing.Color.Black;
            this.panel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel2.Location = new System.Drawing.Point(756, 0);
            this.panel2.Margin = new System.Windows.Forms.Padding(10, 0, 10, 0);
            this.panel2.Name = "panel2";
            this.tableLayoutPanel1.SetRowSpan(this.panel2, 2);
            this.panel2.Size = new System.Drawing.Size(1, 129);
            this.panel2.TabIndex = 25;
            // 
            // panel3
            // 
            this.panel3.BackColor = System.Drawing.Color.Black;
            this.panel3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel3.Location = new System.Drawing.Point(1139, 0);
            this.panel3.Margin = new System.Windows.Forms.Padding(10, 0, 10, 0);
            this.panel3.Name = "panel3";
            this.tableLayoutPanel1.SetRowSpan(this.panel3, 2);
            this.panel3.Size = new System.Drawing.Size(1, 129);
            this.panel3.TabIndex = 26;
            // 
            // panel4
            // 
            this.panel4.BackColor = System.Drawing.Color.Black;
            this.panel4.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel4.Location = new System.Drawing.Point(1522, 0);
            this.panel4.Margin = new System.Windows.Forms.Padding(10, 0, 10, 0);
            this.panel4.Name = "panel4";
            this.tableLayoutPanel1.SetRowSpan(this.panel4, 2);
            this.panel4.Size = new System.Drawing.Size(1, 129);
            this.panel4.TabIndex = 27;
            // 
            // btnStop
            // 
            this.btnStop.BackgroundImage = global::OptiSort.Properties.Resources.stopDisabled_2x2_pptx;
            this.btnStop.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnStop.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnStop.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnStop.Location = new System.Drawing.Point(1426, 59);
            this.btnStop.Name = "btnStop";
            this.btnStop.Size = new System.Drawing.Size(88, 51);
            this.btnStop.TabIndex = 5;
            this.btnStop.TabStop = false;
            this.btnStop.UseVisualStyleBackColor = true;
            this.btnStop.Click += new System.EventHandler(this.btnStop_Click);
            // 
            // btnRun
            // 
            this.btnRun.BackgroundImage = global::OptiSort.Properties.Resources.playDisabled_2x2_pptx;
            this.btnRun.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnRun.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnRun.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnRun.Location = new System.Drawing.Point(1426, 3);
            this.btnRun.Name = "btnRun";
            this.btnRun.Size = new System.Drawing.Size(88, 50);
            this.btnRun.TabIndex = 4;
            this.btnRun.TabStop = false;
            this.btnRun.UseVisualStyleBackColor = true;
            this.btnRun.Click += new System.EventHandler(this.btnRun_Click);
            // 
            // btnConfig
            // 
            this.btnConfig.BackgroundImage = global::OptiSort.Properties.Resources.configDisabled_2x2_pptx;
            this.btnConfig.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnConfig.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnConfig.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnConfig.Location = new System.Drawing.Point(1772, 3);
            this.btnConfig.Name = "btnConfig";
            this.tblPanel.SetRowSpan(this.btnConfig, 2);
            this.btnConfig.Size = new System.Drawing.Size(123, 107);
            this.btnConfig.TabIndex = 3;
            this.btnConfig.TabStop = false;
            this.btnConfig.UseVisualStyleBackColor = true;
            this.btnConfig.Click += new System.EventHandler(this.btnConfig_Click);
            // 
            // btnManual
            // 
            this.btnManual.BackgroundImage = global::OptiSort.Properties.Resources.manualEnabled_2x2_pptx;
            this.btnManual.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnManual.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnManual.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnManual.Location = new System.Drawing.Point(1646, 3);
            this.btnManual.Name = "btnManual";
            this.tblPanel.SetRowSpan(this.btnManual, 2);
            this.btnManual.Size = new System.Drawing.Size(120, 107);
            this.btnManual.TabIndex = 2;
            this.btnManual.TabStop = false;
            this.btnManual.UseVisualStyleBackColor = true;
            this.btnManual.Click += new System.EventHandler(this.btnManual_Click);
            // 
            // btnAuto
            // 
            this.btnAuto.BackgroundImage = global::OptiSort.Properties.Resources.autoDisabled_2x2_pptx;
            this.btnAuto.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnAuto.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnAuto.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnAuto.Location = new System.Drawing.Point(1520, 3);
            this.btnAuto.Name = "btnAuto";
            this.tblPanel.SetRowSpan(this.btnAuto, 2);
            this.btnAuto.Size = new System.Drawing.Size(120, 107);
            this.btnAuto.TabIndex = 1;
            this.btnAuto.TabStop = false;
            this.btnAuto.UseVisualStyleBackColor = true;
            this.btnAuto.Click += new System.EventHandler(this.btnAuto_Click);
            // 
            // btnDtDisconnect
            // 
            this.btnDtDisconnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnDtDisconnect.BackgroundImage")));
            this.btnDtDisconnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnDtDisconnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnDtDisconnect.Enabled = false;
            this.btnDtDisconnect.Location = new System.Drawing.Point(1799, 67);
            this.btnDtDisconnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnDtDisconnect.Name = "btnDtDisconnect";
            this.btnDtDisconnect.Size = new System.Drawing.Size(74, 59);
            this.btnDtDisconnect.TabIndex = 23;
            this.btnDtDisconnect.UseVisualStyleBackColor = true;
            // 
            // btnDtConnect
            // 
            this.btnDtConnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnDtConnect.BackgroundImage")));
            this.btnDtConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnDtConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnDtConnect.Location = new System.Drawing.Point(1678, 67);
            this.btnDtConnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnDtConnect.Name = "btnDtConnect";
            this.btnDtConnect.Size = new System.Drawing.Size(71, 59);
            this.btnDtConnect.TabIndex = 22;
            this.btnDtConnect.UseVisualStyleBackColor = true;
            // 
            // btnCameraTesting
            // 
            this.btnCameraTesting.BackgroundImage = global::OptiSort.Properties.Resources.camerasEnabled_2x2_pptx;
            this.btnCameraTesting.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnCameraTesting.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnCameraTesting.Location = new System.Drawing.Point(1174, 67);
            this.btnCameraTesting.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnCameraTesting.Name = "btnCameraTesting";
            this.btnCameraTesting.Size = new System.Drawing.Size(71, 59);
            this.btnCameraTesting.TabIndex = 19;
            this.btnCameraTesting.UseVisualStyleBackColor = true;
            this.btnCameraTesting.Click += new System.EventHandler(this.btnCameraTesting_Click);
            // 
            // btnCamerasDisconnect
            // 
            this.btnCamerasDisconnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnCamerasDisconnect.BackgroundImage")));
            this.btnCamerasDisconnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnCamerasDisconnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnCamerasDisconnect.Enabled = false;
            this.btnCamerasDisconnect.Location = new System.Drawing.Point(1416, 67);
            this.btnCamerasDisconnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnCamerasDisconnect.Name = "btnCamerasDisconnect";
            this.btnCamerasDisconnect.Size = new System.Drawing.Size(71, 59);
            this.btnCamerasDisconnect.TabIndex = 18;
            this.btnCamerasDisconnect.UseVisualStyleBackColor = true;
            this.btnCamerasDisconnect.Click += new System.EventHandler(this.btnCamerasDisconnect_Click);
            // 
            // btnCamerasConnect
            // 
            this.btnCamerasConnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnCamerasConnect.BackgroundImage")));
            this.btnCamerasConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnCamerasConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnCamerasConnect.Location = new System.Drawing.Point(1295, 67);
            this.btnCamerasConnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnCamerasConnect.Name = "btnCamerasConnect";
            this.btnCamerasConnect.Size = new System.Drawing.Size(71, 59);
            this.btnCamerasConnect.TabIndex = 15;
            this.btnCamerasConnect.UseVisualStyleBackColor = true;
            this.btnCamerasConnect.Click += new System.EventHandler(this.btnCamerasConnect_Click);
            // 
            // btnEmulateScara
            // 
            this.btnEmulateScara.BackgroundImage = global::OptiSort.Properties.Resources.emulationEnabled_2x2_pptx;
            this.btnEmulateScara.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnEmulateScara.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnEmulateScara.Location = new System.Drawing.Point(25, 67);
            this.btnEmulateScara.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnEmulateScara.Name = "btnEmulateScara";
            this.btnEmulateScara.Size = new System.Drawing.Size(71, 59);
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
            this.btnMqttDisconnect.Location = new System.Drawing.Point(1033, 67);
            this.btnMqttDisconnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnMqttDisconnect.Name = "btnMqttDisconnect";
            this.btnMqttDisconnect.Size = new System.Drawing.Size(71, 59);
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
            this.btnFlexibowlDisconnect.Location = new System.Drawing.Point(650, 67);
            this.btnFlexibowlDisconnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnFlexibowlDisconnect.Name = "btnFlexibowlDisconnect";
            this.btnFlexibowlDisconnect.Size = new System.Drawing.Size(71, 59);
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
            this.btnScaraDisconnect.Location = new System.Drawing.Point(267, 67);
            this.btnScaraDisconnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnScaraDisconnect.Name = "btnScaraDisconnect";
            this.btnScaraDisconnect.Size = new System.Drawing.Size(71, 59);
            this.btnScaraDisconnect.TabIndex = 9;
            this.btnScaraDisconnect.UseVisualStyleBackColor = true;
            this.btnScaraDisconnect.Click += new System.EventHandler(this.btnScaraDisconnect_Click);
            // 
            // btnMqttConnect
            // 
            this.btnMqttConnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnMqttConnect.BackgroundImage")));
            this.btnMqttConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnMqttConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnMqttConnect.Location = new System.Drawing.Point(912, 67);
            this.btnMqttConnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnMqttConnect.Name = "btnMqttConnect";
            this.btnMqttConnect.Size = new System.Drawing.Size(71, 59);
            this.btnMqttConnect.TabIndex = 8;
            this.btnMqttConnect.UseVisualStyleBackColor = true;
            this.btnMqttConnect.Click += new System.EventHandler(this.btnMqttConnect_Click);
            // 
            // btnFlexibowlConnect
            // 
            this.btnFlexibowlConnect.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnFlexibowlConnect.BackgroundImage")));
            this.btnFlexibowlConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnFlexibowlConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFlexibowlConnect.Location = new System.Drawing.Point(529, 67);
            this.btnFlexibowlConnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnFlexibowlConnect.Name = "btnFlexibowlConnect";
            this.btnFlexibowlConnect.Size = new System.Drawing.Size(71, 59);
            this.btnFlexibowlConnect.TabIndex = 7;
            this.btnFlexibowlConnect.UseVisualStyleBackColor = true;
            this.btnFlexibowlConnect.Click += new System.EventHandler(this.btnFlexibowlConnect_Click);
            // 
            // btnScaraConnect
            // 
            this.btnScaraConnect.BackgroundImage = global::OptiSort.Properties.Resources.connectedEnabled_2x2_pptx;
            this.btnScaraConnect.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnScaraConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnScaraConnect.Location = new System.Drawing.Point(146, 67);
            this.btnScaraConnect.Margin = new System.Windows.Forms.Padding(25, 3, 25, 3);
            this.btnScaraConnect.Name = "btnScaraConnect";
            this.btnScaraConnect.Size = new System.Drawing.Size(71, 59);
            this.btnScaraConnect.TabIndex = 6;
            this.btnScaraConnect.UseVisualStyleBackColor = true;
            this.btnScaraConnect.Click += new System.EventHandler(this.btnScaraConnect_Click);
            // 
            // pbDtStatus
            // 
            this.pbDtStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pbDtStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pbDtStatus.Image = global::OptiSort.Properties.Resources.off_2x2_pptx;
            this.pbDtStatus.InitialImage = global::OptiSort.Properties.Resources.off_2x2_pptx1;
            this.pbDtStatus.Location = new System.Drawing.Point(1777, 3);
            this.pbDtStatus.Name = "pbDtStatus";
            this.pbDtStatus.Size = new System.Drawing.Size(118, 58);
            this.pbDtStatus.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pbDtStatus.TabIndex = 28;
            this.pbDtStatus.TabStop = false;
            // 
            // pbScaraStatus
            // 
            this.pbScaraStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pbScaraStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pbScaraStatus.Image = global::OptiSort.Properties.Resources.off_2x2_pptx;
            this.pbScaraStatus.InitialImage = global::OptiSort.Properties.Resources.off_2x2_pptx1;
            this.pbScaraStatus.Location = new System.Drawing.Point(245, 3);
            this.pbScaraStatus.Name = "pbScaraStatus";
            this.pbScaraStatus.Size = new System.Drawing.Size(115, 58);
            this.pbScaraStatus.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pbScaraStatus.TabIndex = 29;
            this.pbScaraStatus.TabStop = false;
            // 
            // pbFlexibowlStatus
            // 
            this.pbFlexibowlStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pbFlexibowlStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pbFlexibowlStatus.Image = global::OptiSort.Properties.Resources.off_2x2_pptx;
            this.pbFlexibowlStatus.InitialImage = global::OptiSort.Properties.Resources.off_2x2_pptx1;
            this.pbFlexibowlStatus.Location = new System.Drawing.Point(628, 3);
            this.pbFlexibowlStatus.Name = "pbFlexibowlStatus";
            this.pbFlexibowlStatus.Size = new System.Drawing.Size(115, 58);
            this.pbFlexibowlStatus.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pbFlexibowlStatus.TabIndex = 30;
            this.pbFlexibowlStatus.TabStop = false;
            // 
            // pbMqttStatus
            // 
            this.pbMqttStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pbMqttStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pbMqttStatus.Image = global::OptiSort.Properties.Resources.off_2x2_pptx;
            this.pbMqttStatus.InitialImage = global::OptiSort.Properties.Resources.off_2x2_pptx1;
            this.pbMqttStatus.Location = new System.Drawing.Point(1011, 3);
            this.pbMqttStatus.Name = "pbMqttStatus";
            this.pbMqttStatus.Size = new System.Drawing.Size(115, 58);
            this.pbMqttStatus.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pbMqttStatus.TabIndex = 31;
            this.pbMqttStatus.TabStop = false;
            // 
            // pbCameraStatus
            // 
            this.pbCameraStatus.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pbCameraStatus.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pbCameraStatus.Image = global::OptiSort.Properties.Resources.off_2x2_pptx;
            this.pbCameraStatus.InitialImage = global::OptiSort.Properties.Resources.off_2x2_pptx1;
            this.pbCameraStatus.Location = new System.Drawing.Point(1394, 3);
            this.pbCameraStatus.Name = "pbCameraStatus";
            this.pbCameraStatus.Size = new System.Drawing.Size(115, 58);
            this.pbCameraStatus.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pbCameraStatus.TabIndex = 32;
            this.pbCameraStatus.TabStop = false;
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
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.MinimumSize = new System.Drawing.Size(1920, 1080);
            this.Name = "frmMain";
            this.Text = "OptiSort";
            this.WindowState = System.Windows.Forms.FormWindowState.Maximized;
            this.tblPanel.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.tblMainStructure.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pbDtStatus)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbScaraStatus)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbFlexibowlStatus)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbMqttStatus)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbCameraStatus)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ListBox lstLog;
        private System.Windows.Forms.TableLayoutPanel tblPanel;
        private System.Windows.Forms.Button btnAuto;
        private System.Windows.Forms.Button btnConfig;
        private System.Windows.Forms.Button btnManual;
        private System.Windows.Forms.Panel pnlCurrentUc;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Label lblScaraStatus;
        private System.Windows.Forms.Label lblMqttStatus;
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
        private System.Windows.Forms.Button btnStop;
        private System.Windows.Forms.Button btnRun;
        private System.Windows.Forms.Button btnCamerasConnect;
        private System.Windows.Forms.Label lblCamerasStatus;
        private System.Windows.Forms.Button btnCamerasDisconnect;
        private System.Windows.Forms.Button btnCameraTesting;
        private System.Windows.Forms.Button btnDtDisconnect;
        private System.Windows.Forms.Button btnDtConnect;
        private System.Windows.Forms.Label lblDtStatus;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Panel panel4;
        private System.Windows.Forms.Panel panel3;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.PictureBox pbDtStatus;
        private System.Windows.Forms.PictureBox pbCameraStatus;
        private System.Windows.Forms.PictureBox pbMqttStatus;
        private System.Windows.Forms.PictureBox pbFlexibowlStatus;
        private System.Windows.Forms.PictureBox pbScaraStatus;
    }
}

