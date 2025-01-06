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
            _frmMain.MqttClient.MessageReceived -= _ucScaraTargets.OnMessageReceived;

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
            this.components = new System.ComponentModel.Container();
            this.pnlScara = new System.Windows.Forms.Panel();
            this.tbl_controls = new System.Windows.Forms.TableLayoutPanel();
            this.led_approachFlexibowl = new Bulb.LedBulb();
            this.lbl_approachFlexibowl = new System.Windows.Forms.Label();
            this.lbl_pick = new System.Windows.Forms.Label();
            this.lbl_approachShuttle = new System.Windows.Forms.Label();
            this.lbl_place = new System.Windows.Forms.Label();
            this.lbl_rotate = new System.Windows.Forms.Label();
            this.lbl_title_scara = new System.Windows.Forms.Label();
            this.lbl_title_flexibowl = new System.Windows.Forms.Label();
            this.lbl_shake = new System.Windows.Forms.Label();
            this.lbl_flip = new System.Windows.Forms.Label();
            this.led_pick = new Bulb.LedBulb();
            this.led_approachShuttle = new Bulb.LedBulb();
            this.led_place = new Bulb.LedBulb();
            this.led_rotate = new Bulb.LedBulb();
            this.led_shake = new Bulb.LedBulb();
            this.led_flip = new Bulb.LedBulb();
            this.lbl_title_control = new System.Windows.Forms.Label();
            this.lbl_title_coordinates = new System.Windows.Forms.Label();
            this.lbl_title_setup = new System.Windows.Forms.Label();
            this.lbl_selectedCamera = new System.Windows.Forms.Label();
            this.lbl_actualSelectedCamera = new System.Windows.Forms.Label();
            this.lbl_detectionAlgorithm = new System.Windows.Forms.Label();
            this.cmb_algorithm = new System.Windows.Forms.ComboBox();
            this.lbl_simplifiedProcess = new System.Windows.Forms.Label();
            this.btn_startSimplifiedProcess = new System.Windows.Forms.Button();
            this.lbl_optimizedProcess = new System.Windows.Forms.Label();
            this.btn_startOptimizedProcess = new System.Windows.Forms.Button();
            this.tmr_process = new System.Windows.Forms.Timer(this.components);
            this.lbl_timeSimple_placeholder = new System.Windows.Forms.Label();
            this.lbl_timeOpt_placeholder = new System.Windows.Forms.Label();
            this.lbl_lastTimeSimpleProcess = new System.Windows.Forms.Label();
            this.lbl_lastTimeOptimizedProcess = new System.Windows.Forms.Label();
            this.btn_stopProcess = new System.Windows.Forms.Button();
            this.tbl_controls.SuspendLayout();
            this.SuspendLayout();
            // 
            // pnlScara
            // 
            this.pnlScara.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.pnlScara.Location = new System.Drawing.Point(0, 513);
            this.pnlScara.Name = "pnlScara";
            this.pnlScara.Size = new System.Drawing.Size(1537, 407);
            this.pnlScara.TabIndex = 20;
            // 
            // tbl_controls
            // 
            this.tbl_controls.ColumnCount = 7;
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.28571F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.28572F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.28572F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.28572F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.28572F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.28572F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.28572F));
            this.tbl_controls.Controls.Add(this.btn_stopProcess, 6, 2);
            this.tbl_controls.Controls.Add(this.lbl_lastTimeOptimizedProcess, 5, 3);
            this.tbl_controls.Controls.Add(this.lbl_lastTimeSimpleProcess, 5, 2);
            this.tbl_controls.Controls.Add(this.lbl_timeOpt_placeholder, 3, 3);
            this.tbl_controls.Controls.Add(this.lbl_timeSimple_placeholder, 3, 2);
            this.tbl_controls.Controls.Add(this.btn_startOptimizedProcess, 2, 3);
            this.tbl_controls.Controls.Add(this.lbl_optimizedProcess, 0, 3);
            this.tbl_controls.Controls.Add(this.lbl_detectionAlgorithm, 3, 1);
            this.tbl_controls.Controls.Add(this.lbl_actualSelectedCamera, 2, 1);
            this.tbl_controls.Controls.Add(this.lbl_title_setup, 0, 0);
            this.tbl_controls.Controls.Add(this.lbl_title_coordinates, 0, 8);
            this.tbl_controls.Controls.Add(this.lbl_title_control, 0, 4);
            this.tbl_controls.Controls.Add(this.lbl_pick, 1, 6);
            this.tbl_controls.Controls.Add(this.lbl_approachShuttle, 2, 6);
            this.tbl_controls.Controls.Add(this.lbl_place, 3, 6);
            this.tbl_controls.Controls.Add(this.lbl_rotate, 4, 6);
            this.tbl_controls.Controls.Add(this.lbl_title_scara, 0, 5);
            this.tbl_controls.Controls.Add(this.lbl_title_flexibowl, 4, 5);
            this.tbl_controls.Controls.Add(this.lbl_shake, 5, 6);
            this.tbl_controls.Controls.Add(this.lbl_flip, 6, 6);
            this.tbl_controls.Controls.Add(this.led_pick, 1, 7);
            this.tbl_controls.Controls.Add(this.led_approachShuttle, 2, 7);
            this.tbl_controls.Controls.Add(this.led_place, 3, 7);
            this.tbl_controls.Controls.Add(this.led_rotate, 4, 7);
            this.tbl_controls.Controls.Add(this.led_shake, 5, 7);
            this.tbl_controls.Controls.Add(this.led_flip, 6, 7);
            this.tbl_controls.Controls.Add(this.lbl_selectedCamera, 0, 1);
            this.tbl_controls.Controls.Add(this.lbl_simplifiedProcess, 0, 2);
            this.tbl_controls.Controls.Add(this.btn_startSimplifiedProcess, 2, 2);
            this.tbl_controls.Controls.Add(this.lbl_approachFlexibowl, 0, 6);
            this.tbl_controls.Controls.Add(this.led_approachFlexibowl, 0, 7);
            this.tbl_controls.Controls.Add(this.cmb_algorithm, 5, 1);
            this.tbl_controls.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tbl_controls.Location = new System.Drawing.Point(0, 0);
            this.tbl_controls.Name = "tbl_controls";
            this.tbl_controls.RowCount = 9;
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10.004F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.66467F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.66467F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.66467F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10.004F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.66467F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.66467F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.66467F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10.004F));
            this.tbl_controls.Size = new System.Drawing.Size(1537, 513);
            this.tbl_controls.TabIndex = 21;
            // 
            // led_approachFlexibowl
            // 
            this.led_approachFlexibowl.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_approachFlexibowl.Location = new System.Drawing.Point(79, 400);
            this.led_approachFlexibowl.Name = "led_approachFlexibowl";
            this.led_approachFlexibowl.On = false;
            this.led_approachFlexibowl.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.led_approachFlexibowl.Size = new System.Drawing.Size(60, 53);
            this.led_approachFlexibowl.TabIndex = 0;
            // 
            // lbl_approachFlexibowl
            // 
            this.lbl_approachFlexibowl.AutoSize = true;
            this.lbl_approachFlexibowl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_approachFlexibowl.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_approachFlexibowl.Location = new System.Drawing.Point(3, 338);
            this.lbl_approachFlexibowl.Name = "lbl_approachFlexibowl";
            this.lbl_approachFlexibowl.Size = new System.Drawing.Size(213, 59);
            this.lbl_approachFlexibowl.TabIndex = 1;
            this.lbl_approachFlexibowl.Text = "Approach flexibowl";
            this.lbl_approachFlexibowl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_pick
            // 
            this.lbl_pick.AutoSize = true;
            this.lbl_pick.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_pick.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_pick.Location = new System.Drawing.Point(222, 338);
            this.lbl_pick.Name = "lbl_pick";
            this.lbl_pick.Size = new System.Drawing.Size(213, 59);
            this.lbl_pick.TabIndex = 2;
            this.lbl_pick.Text = "Pick";
            this.lbl_pick.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_approachShuttle
            // 
            this.lbl_approachShuttle.AutoSize = true;
            this.lbl_approachShuttle.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_approachShuttle.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_approachShuttle.Location = new System.Drawing.Point(441, 338);
            this.lbl_approachShuttle.Name = "lbl_approachShuttle";
            this.lbl_approachShuttle.Size = new System.Drawing.Size(213, 59);
            this.lbl_approachShuttle.TabIndex = 3;
            this.lbl_approachShuttle.Text = "Approach shuttle";
            this.lbl_approachShuttle.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_place
            // 
            this.lbl_place.AutoSize = true;
            this.lbl_place.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_place.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_place.Location = new System.Drawing.Point(660, 338);
            this.lbl_place.Name = "lbl_place";
            this.lbl_place.Size = new System.Drawing.Size(213, 59);
            this.lbl_place.TabIndex = 4;
            this.lbl_place.Text = "Place";
            this.lbl_place.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_rotate
            // 
            this.lbl_rotate.AutoSize = true;
            this.lbl_rotate.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_rotate.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_rotate.Location = new System.Drawing.Point(879, 338);
            this.lbl_rotate.Name = "lbl_rotate";
            this.lbl_rotate.Size = new System.Drawing.Size(213, 59);
            this.lbl_rotate.TabIndex = 5;
            this.lbl_rotate.Text = "Rotate";
            this.lbl_rotate.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_title_scara
            // 
            this.lbl_title_scara.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_title_scara, 4);
            this.lbl_title_scara.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_scara.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_title_scara.Location = new System.Drawing.Point(3, 279);
            this.lbl_title_scara.Name = "lbl_title_scara";
            this.lbl_title_scara.Size = new System.Drawing.Size(870, 59);
            this.lbl_title_scara.TabIndex = 6;
            this.lbl_title_scara.Text = "SCARA CONTROL";
            this.lbl_title_scara.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_title_flexibowl
            // 
            this.lbl_title_flexibowl.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_title_flexibowl, 3);
            this.lbl_title_flexibowl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_flexibowl.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_title_flexibowl.Location = new System.Drawing.Point(879, 279);
            this.lbl_title_flexibowl.Name = "lbl_title_flexibowl";
            this.lbl_title_flexibowl.Size = new System.Drawing.Size(655, 59);
            this.lbl_title_flexibowl.TabIndex = 7;
            this.lbl_title_flexibowl.Text = "FLEXIBOWL CONTROL";
            this.lbl_title_flexibowl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_shake
            // 
            this.lbl_shake.AutoSize = true;
            this.lbl_shake.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_shake.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_shake.Location = new System.Drawing.Point(1098, 338);
            this.lbl_shake.Name = "lbl_shake";
            this.lbl_shake.Size = new System.Drawing.Size(213, 59);
            this.lbl_shake.TabIndex = 8;
            this.lbl_shake.Text = "Shake";
            this.lbl_shake.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_flip
            // 
            this.lbl_flip.AutoSize = true;
            this.lbl_flip.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_flip.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_flip.Location = new System.Drawing.Point(1317, 338);
            this.lbl_flip.Name = "lbl_flip";
            this.lbl_flip.Size = new System.Drawing.Size(217, 59);
            this.lbl_flip.TabIndex = 9;
            this.lbl_flip.Text = "Flip";
            this.lbl_flip.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // led_pick
            // 
            this.led_pick.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_pick.Location = new System.Drawing.Point(298, 400);
            this.led_pick.Name = "led_pick";
            this.led_pick.On = false;
            this.led_pick.Size = new System.Drawing.Size(60, 53);
            this.led_pick.TabIndex = 10;
            this.led_pick.Text = "ledBulb2";
            // 
            // led_approachShuttle
            // 
            this.led_approachShuttle.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_approachShuttle.Location = new System.Drawing.Point(517, 400);
            this.led_approachShuttle.Name = "led_approachShuttle";
            this.led_approachShuttle.On = false;
            this.led_approachShuttle.Size = new System.Drawing.Size(60, 53);
            this.led_approachShuttle.TabIndex = 11;
            this.led_approachShuttle.Text = "ledBulb3";
            // 
            // led_place
            // 
            this.led_place.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_place.Location = new System.Drawing.Point(736, 400);
            this.led_place.Name = "led_place";
            this.led_place.On = false;
            this.led_place.Size = new System.Drawing.Size(60, 53);
            this.led_place.TabIndex = 12;
            this.led_place.Text = "ledBulb4";
            // 
            // led_rotate
            // 
            this.led_rotate.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_rotate.Location = new System.Drawing.Point(955, 400);
            this.led_rotate.Name = "led_rotate";
            this.led_rotate.On = false;
            this.led_rotate.Size = new System.Drawing.Size(60, 53);
            this.led_rotate.TabIndex = 13;
            this.led_rotate.Text = "ledBulb5";
            // 
            // led_shake
            // 
            this.led_shake.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_shake.Location = new System.Drawing.Point(1174, 400);
            this.led_shake.Name = "led_shake";
            this.led_shake.On = false;
            this.led_shake.Size = new System.Drawing.Size(60, 53);
            this.led_shake.TabIndex = 14;
            this.led_shake.Text = "ledBulb6";
            // 
            // led_flip
            // 
            this.led_flip.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_flip.Location = new System.Drawing.Point(1395, 400);
            this.led_flip.Name = "led_flip";
            this.led_flip.On = false;
            this.led_flip.Size = new System.Drawing.Size(60, 53);
            this.led_flip.TabIndex = 15;
            this.led_flip.Text = "ledBulb7";
            // 
            // lbl_title_control
            // 
            this.lbl_title_control.AutoSize = true;
            this.lbl_title_control.BackColor = System.Drawing.SystemColors.ControlLight;
            this.tbl_controls.SetColumnSpan(this.lbl_title_control, 7);
            this.lbl_title_control.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_control.Font = new System.Drawing.Font("Segoe UI", 14F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_title_control.Location = new System.Drawing.Point(3, 228);
            this.lbl_title_control.Name = "lbl_title_control";
            this.lbl_title_control.Size = new System.Drawing.Size(1531, 51);
            this.lbl_title_control.TabIndex = 16;
            this.lbl_title_control.Text = "Subsystems control";
            this.lbl_title_control.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_title_coordinates
            // 
            this.lbl_title_coordinates.AutoSize = true;
            this.lbl_title_coordinates.BackColor = System.Drawing.SystemColors.ControlLight;
            this.tbl_controls.SetColumnSpan(this.lbl_title_coordinates, 7);
            this.lbl_title_coordinates.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_coordinates.Font = new System.Drawing.Font("Segoe UI", 14F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_title_coordinates.Location = new System.Drawing.Point(3, 456);
            this.lbl_title_coordinates.Name = "lbl_title_coordinates";
            this.lbl_title_coordinates.Size = new System.Drawing.Size(1531, 57);
            this.lbl_title_coordinates.TabIndex = 17;
            this.lbl_title_coordinates.Text = "MQTT locations backlog";
            this.lbl_title_coordinates.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_title_setup
            // 
            this.lbl_title_setup.AutoSize = true;
            this.lbl_title_setup.BackColor = System.Drawing.SystemColors.ControlLight;
            this.tbl_controls.SetColumnSpan(this.lbl_title_setup, 7);
            this.lbl_title_setup.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_setup.Font = new System.Drawing.Font("Segoe UI", 14F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_title_setup.Location = new System.Drawing.Point(3, 0);
            this.lbl_title_setup.Name = "lbl_title_setup";
            this.lbl_title_setup.Size = new System.Drawing.Size(1531, 51);
            this.lbl_title_setup.TabIndex = 18;
            this.lbl_title_setup.Text = "Process setup";
            this.lbl_title_setup.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_selectedCamera
            // 
            this.lbl_selectedCamera.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_selectedCamera, 2);
            this.lbl_selectedCamera.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_selectedCamera.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_selectedCamera.Location = new System.Drawing.Point(3, 51);
            this.lbl_selectedCamera.Name = "lbl_selectedCamera";
            this.lbl_selectedCamera.Size = new System.Drawing.Size(432, 59);
            this.lbl_selectedCamera.TabIndex = 19;
            this.lbl_selectedCamera.Text = "Selected camera:";
            this.lbl_selectedCamera.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_actualSelectedCamera
            // 
            this.lbl_actualSelectedCamera.AutoSize = true;
            this.lbl_actualSelectedCamera.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_actualSelectedCamera.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_actualSelectedCamera.Location = new System.Drawing.Point(441, 51);
            this.lbl_actualSelectedCamera.Name = "lbl_actualSelectedCamera";
            this.lbl_actualSelectedCamera.Size = new System.Drawing.Size(213, 59);
            this.lbl_actualSelectedCamera.TabIndex = 20;
            this.lbl_actualSelectedCamera.Text = "XXX";
            this.lbl_actualSelectedCamera.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_detectionAlgorithm
            // 
            this.lbl_detectionAlgorithm.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_detectionAlgorithm, 2);
            this.lbl_detectionAlgorithm.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_detectionAlgorithm.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_detectionAlgorithm.Location = new System.Drawing.Point(660, 51);
            this.lbl_detectionAlgorithm.Name = "lbl_detectionAlgorithm";
            this.lbl_detectionAlgorithm.Size = new System.Drawing.Size(432, 59);
            this.lbl_detectionAlgorithm.TabIndex = 21;
            this.lbl_detectionAlgorithm.Text = "Detection algorithm:";
            this.lbl_detectionAlgorithm.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // cmb_algorithm
            // 
            this.cmb_algorithm.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.tbl_controls.SetColumnSpan(this.cmb_algorithm, 2);
            this.cmb_algorithm.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmb_algorithm.FormattingEnabled = true;
            this.cmb_algorithm.Location = new System.Drawing.Point(1098, 62);
            this.cmb_algorithm.Name = "cmb_algorithm";
            this.cmb_algorithm.Size = new System.Drawing.Size(436, 37);
            this.cmb_algorithm.TabIndex = 22;
            // 
            // lbl_simplifiedProcess
            // 
            this.lbl_simplifiedProcess.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_simplifiedProcess, 2);
            this.lbl_simplifiedProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_simplifiedProcess.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_simplifiedProcess.Location = new System.Drawing.Point(3, 110);
            this.lbl_simplifiedProcess.Name = "lbl_simplifiedProcess";
            this.lbl_simplifiedProcess.Size = new System.Drawing.Size(432, 59);
            this.lbl_simplifiedProcess.TabIndex = 23;
            this.lbl_simplifiedProcess.Text = "Simplified process";
            this.lbl_simplifiedProcess.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btn_startSimplifiedProcess
            // 
            this.btn_startSimplifiedProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_startSimplifiedProcess.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_startSimplifiedProcess.Location = new System.Drawing.Point(441, 113);
            this.btn_startSimplifiedProcess.Name = "btn_startSimplifiedProcess";
            this.btn_startSimplifiedProcess.Size = new System.Drawing.Size(213, 53);
            this.btn_startSimplifiedProcess.TabIndex = 24;
            this.btn_startSimplifiedProcess.Text = "START";
            this.btn_startSimplifiedProcess.UseVisualStyleBackColor = true;
            // 
            // lbl_optimizedProcess
            // 
            this.lbl_optimizedProcess.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_optimizedProcess, 2);
            this.lbl_optimizedProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_optimizedProcess.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_optimizedProcess.Location = new System.Drawing.Point(3, 169);
            this.lbl_optimizedProcess.Name = "lbl_optimizedProcess";
            this.lbl_optimizedProcess.Size = new System.Drawing.Size(432, 59);
            this.lbl_optimizedProcess.TabIndex = 25;
            this.lbl_optimizedProcess.Text = "Optimized process";
            this.lbl_optimizedProcess.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btn_startOptimizedProcess
            // 
            this.btn_startOptimizedProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_startOptimizedProcess.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_startOptimizedProcess.Location = new System.Drawing.Point(441, 172);
            this.btn_startOptimizedProcess.Name = "btn_startOptimizedProcess";
            this.btn_startOptimizedProcess.Size = new System.Drawing.Size(213, 53);
            this.btn_startOptimizedProcess.TabIndex = 26;
            this.btn_startOptimizedProcess.Text = "START";
            this.btn_startOptimizedProcess.UseVisualStyleBackColor = true;
            // 
            // lbl_timeSimple_placeholder
            // 
            this.lbl_timeSimple_placeholder.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_timeSimple_placeholder, 2);
            this.lbl_timeSimple_placeholder.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_timeSimple_placeholder.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_timeSimple_placeholder.Location = new System.Drawing.Point(660, 110);
            this.lbl_timeSimple_placeholder.Name = "lbl_timeSimple_placeholder";
            this.lbl_timeSimple_placeholder.Size = new System.Drawing.Size(432, 59);
            this.lbl_timeSimple_placeholder.TabIndex = 27;
            this.lbl_timeSimple_placeholder.Text = "Last process time";
            this.lbl_timeSimple_placeholder.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_timeOpt_placeholder
            // 
            this.lbl_timeOpt_placeholder.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_timeOpt_placeholder, 2);
            this.lbl_timeOpt_placeholder.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_timeOpt_placeholder.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_timeOpt_placeholder.Location = new System.Drawing.Point(660, 169);
            this.lbl_timeOpt_placeholder.Name = "lbl_timeOpt_placeholder";
            this.lbl_timeOpt_placeholder.Size = new System.Drawing.Size(432, 59);
            this.lbl_timeOpt_placeholder.TabIndex = 28;
            this.lbl_timeOpt_placeholder.Text = "Last process time";
            this.lbl_timeOpt_placeholder.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_lastTimeSimpleProcess
            // 
            this.lbl_lastTimeSimpleProcess.AutoSize = true;
            this.lbl_lastTimeSimpleProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_lastTimeSimpleProcess.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_lastTimeSimpleProcess.Location = new System.Drawing.Point(1098, 110);
            this.lbl_lastTimeSimpleProcess.Name = "lbl_lastTimeSimpleProcess";
            this.lbl_lastTimeSimpleProcess.Size = new System.Drawing.Size(213, 59);
            this.lbl_lastTimeSimpleProcess.TabIndex = 29;
            this.lbl_lastTimeSimpleProcess.Text = "hh:mm:ss";
            this.lbl_lastTimeSimpleProcess.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_lastTimeOptimizedProcess
            // 
            this.lbl_lastTimeOptimizedProcess.AutoSize = true;
            this.lbl_lastTimeOptimizedProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_lastTimeOptimizedProcess.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_lastTimeOptimizedProcess.Location = new System.Drawing.Point(1098, 169);
            this.lbl_lastTimeOptimizedProcess.Name = "lbl_lastTimeOptimizedProcess";
            this.lbl_lastTimeOptimizedProcess.Size = new System.Drawing.Size(213, 59);
            this.lbl_lastTimeOptimizedProcess.TabIndex = 30;
            this.lbl_lastTimeOptimizedProcess.Text = "hh:mm:ss";
            this.lbl_lastTimeOptimizedProcess.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btn_stopProcess
            // 
            this.btn_stopProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_stopProcess.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_stopProcess.Location = new System.Drawing.Point(1329, 125);
            this.btn_stopProcess.Margin = new System.Windows.Forms.Padding(15);
            this.btn_stopProcess.Name = "btn_stopProcess";
            this.tbl_controls.SetRowSpan(this.btn_stopProcess, 2);
            this.btn_stopProcess.Size = new System.Drawing.Size(193, 88);
            this.btn_stopProcess.TabIndex = 31;
            this.btn_stopProcess.Text = "STOP";
            this.btn_stopProcess.UseVisualStyleBackColor = true;
            // 
            // ucProcessView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tbl_controls);
            this.Controls.Add(this.pnlScara);
            this.Name = "ucProcessView";
            this.Size = new System.Drawing.Size(1537, 920);
            this.tbl_controls.ResumeLayout(false);
            this.tbl_controls.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel pnlScara;
        private System.Windows.Forms.TableLayoutPanel tbl_controls;
        private Bulb.LedBulb led_approachFlexibowl;
        private System.Windows.Forms.Label lbl_approachFlexibowl;
        private System.Windows.Forms.Label lbl_pick;
        private System.Windows.Forms.Label lbl_approachShuttle;
        private System.Windows.Forms.Label lbl_place;
        private System.Windows.Forms.Label lbl_rotate;
        private System.Windows.Forms.Label lbl_title_scara;
        private System.Windows.Forms.Label lbl_title_flexibowl;
        private System.Windows.Forms.Label lbl_shake;
        private System.Windows.Forms.Label lbl_flip;
        private Bulb.LedBulb led_pick;
        private Bulb.LedBulb led_approachShuttle;
        private Bulb.LedBulb led_place;
        private Bulb.LedBulb led_rotate;
        private Bulb.LedBulb led_shake;
        private Bulb.LedBulb led_flip;
        private System.Windows.Forms.Label lbl_title_coordinates;
        private System.Windows.Forms.Label lbl_title_control;
        private System.Windows.Forms.Label lbl_detectionAlgorithm;
        private System.Windows.Forms.Label lbl_actualSelectedCamera;
        private System.Windows.Forms.Label lbl_title_setup;
        private System.Windows.Forms.Label lbl_selectedCamera;
        private System.Windows.Forms.ComboBox cmb_algorithm;
        private System.Windows.Forms.Button btn_startOptimizedProcess;
        private System.Windows.Forms.Label lbl_optimizedProcess;
        private System.Windows.Forms.Label lbl_simplifiedProcess;
        private System.Windows.Forms.Button btn_startSimplifiedProcess;
        private System.Windows.Forms.Label lbl_timeOpt_placeholder;
        private System.Windows.Forms.Label lbl_timeSimple_placeholder;
        private System.Windows.Forms.Timer tmr_process;
        private System.Windows.Forms.Label lbl_lastTimeOptimizedProcess;
        private System.Windows.Forms.Label lbl_lastTimeSimpleProcess;
        private System.Windows.Forms.Button btn_stopProcess;
    }
}
