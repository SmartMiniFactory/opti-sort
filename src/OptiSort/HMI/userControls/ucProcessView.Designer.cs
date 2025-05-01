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
            this.btn_stop = new System.Windows.Forms.Button();
            this.lbl_actualSelectedCamera = new System.Windows.Forms.Label();
            this.lbl_title_setup = new System.Windows.Forms.Label();
            this.lbl_title_coordinates = new System.Windows.Forms.Label();
            this.lbl_title_control = new System.Windows.Forms.Label();
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
            this.lbl_camera = new System.Windows.Forms.Label();
            this.lbl_approachFlexibowl = new System.Windows.Forms.Label();
            this.led_approachFlexibowl = new Bulb.LedBulb();
            this.btn_start = new System.Windows.Forms.Button();
            this.tmr_process = new System.Windows.Forms.Timer(this.components);
            this.lbl_elapsedTime = new System.Windows.Forms.Label();
            this.lbl_cycleTimer = new System.Windows.Forms.Label();
            this.lbl_Adect = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.lbl_Adetected = new System.Windows.Forms.Label();
            this.lbl_Bdetected = new System.Windows.Forms.Label();
            this.lbl_picked = new System.Windows.Forms.Label();
            this.lbl_discarded = new System.Windows.Forms.Label();
            this.lbl_nrPicked = new System.Windows.Forms.Label();
            this.lbl_nrDiscarded = new System.Windows.Forms.Label();
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
            this.tbl_controls.Controls.Add(this.lbl_nrDiscarded, 6, 2);
            this.tbl_controls.Controls.Add(this.lbl_nrPicked, 6, 1);
            this.tbl_controls.Controls.Add(this.lbl_discarded, 5, 2);
            this.tbl_controls.Controls.Add(this.lbl_picked, 5, 1);
            this.tbl_controls.Controls.Add(this.lbl_Bdetected, 4, 2);
            this.tbl_controls.Controls.Add(this.lbl_Adetected, 4, 1);
            this.tbl_controls.Controls.Add(this.lbl_Adect, 2, 1);
            this.tbl_controls.Controls.Add(this.label3, 2, 2);
            this.tbl_controls.Controls.Add(this.lbl_cycleTimer, 1, 2);
            this.tbl_controls.Controls.Add(this.lbl_elapsedTime, 0, 2);
            this.tbl_controls.Controls.Add(this.btn_stop, 6, 0);
            this.tbl_controls.Controls.Add(this.lbl_actualSelectedCamera, 1, 1);
            this.tbl_controls.Controls.Add(this.lbl_title_setup, 1, 0);
            this.tbl_controls.Controls.Add(this.lbl_title_coordinates, 0, 7);
            this.tbl_controls.Controls.Add(this.lbl_title_control, 0, 3);
            this.tbl_controls.Controls.Add(this.lbl_pick, 1, 5);
            this.tbl_controls.Controls.Add(this.lbl_approachShuttle, 2, 5);
            this.tbl_controls.Controls.Add(this.lbl_place, 3, 5);
            this.tbl_controls.Controls.Add(this.lbl_rotate, 4, 5);
            this.tbl_controls.Controls.Add(this.lbl_title_scara, 0, 4);
            this.tbl_controls.Controls.Add(this.lbl_title_flexibowl, 4, 4);
            this.tbl_controls.Controls.Add(this.lbl_shake, 5, 5);
            this.tbl_controls.Controls.Add(this.lbl_flip, 6, 5);
            this.tbl_controls.Controls.Add(this.led_pick, 1, 6);
            this.tbl_controls.Controls.Add(this.led_approachShuttle, 2, 6);
            this.tbl_controls.Controls.Add(this.led_place, 3, 6);
            this.tbl_controls.Controls.Add(this.led_rotate, 4, 6);
            this.tbl_controls.Controls.Add(this.led_shake, 5, 6);
            this.tbl_controls.Controls.Add(this.led_flip, 6, 6);
            this.tbl_controls.Controls.Add(this.lbl_camera, 0, 1);
            this.tbl_controls.Controls.Add(this.lbl_approachFlexibowl, 0, 5);
            this.tbl_controls.Controls.Add(this.led_approachFlexibowl, 0, 6);
            this.tbl_controls.Controls.Add(this.btn_start, 0, 0);
            this.tbl_controls.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tbl_controls.Location = new System.Drawing.Point(0, 0);
            this.tbl_controls.Name = "tbl_controls";
            this.tbl_controls.RowCount = 8;
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.32502F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 13.20499F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 13.20499F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.32502F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 13.20499F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 13.20499F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 13.20499F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11.32502F));
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tbl_controls.Size = new System.Drawing.Size(1537, 513);
            this.tbl_controls.TabIndex = 21;
            // 
            // btn_stop
            // 
            this.btn_stop.BackgroundImage = global::OptiSort.Properties.Resources.stopDisabled_2x2_pptx;
            this.btn_stop.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_stop.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_stop.Location = new System.Drawing.Point(1317, 3);
            this.btn_stop.Name = "btn_stop";
            this.btn_stop.Size = new System.Drawing.Size(217, 52);
            this.btn_stop.TabIndex = 24;
            this.btn_stop.UseVisualStyleBackColor = true;
            // 
            // lbl_actualSelectedCamera
            // 
            this.lbl_actualSelectedCamera.AutoSize = true;
            this.lbl_actualSelectedCamera.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_actualSelectedCamera.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_actualSelectedCamera.Location = new System.Drawing.Point(222, 58);
            this.lbl_actualSelectedCamera.Name = "lbl_actualSelectedCamera";
            this.lbl_actualSelectedCamera.Size = new System.Drawing.Size(213, 67);
            this.lbl_actualSelectedCamera.TabIndex = 20;
            this.lbl_actualSelectedCamera.Text = "None";
            this.lbl_actualSelectedCamera.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_title_setup
            // 
            this.lbl_title_setup.AutoSize = true;
            this.lbl_title_setup.BackColor = System.Drawing.SystemColors.GradientInactiveCaption;
            this.tbl_controls.SetColumnSpan(this.lbl_title_setup, 5);
            this.lbl_title_setup.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_setup.Font = new System.Drawing.Font("Segoe UI", 15F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_title_setup.Location = new System.Drawing.Point(222, 0);
            this.lbl_title_setup.Name = "lbl_title_setup";
            this.lbl_title_setup.Size = new System.Drawing.Size(1089, 58);
            this.lbl_title_setup.TabIndex = 18;
            this.lbl_title_setup.Text = "Process Statistics";
            this.lbl_title_setup.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_title_coordinates
            // 
            this.lbl_title_coordinates.AutoSize = true;
            this.lbl_title_coordinates.BackColor = System.Drawing.SystemColors.GradientInactiveCaption;
            this.tbl_controls.SetColumnSpan(this.lbl_title_coordinates, 7);
            this.lbl_title_coordinates.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_coordinates.Font = new System.Drawing.Font("Segoe UI", 15F, System.Drawing.FontStyle.Bold);
            this.lbl_title_coordinates.Location = new System.Drawing.Point(3, 451);
            this.lbl_title_coordinates.Name = "lbl_title_coordinates";
            this.lbl_title_coordinates.Size = new System.Drawing.Size(1531, 62);
            this.lbl_title_coordinates.TabIndex = 17;
            this.lbl_title_coordinates.Text = "Components Detection Backlog";
            this.lbl_title_coordinates.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_title_control
            // 
            this.lbl_title_control.AutoSize = true;
            this.lbl_title_control.BackColor = System.Drawing.SystemColors.GradientInactiveCaption;
            this.tbl_controls.SetColumnSpan(this.lbl_title_control, 7);
            this.lbl_title_control.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_title_control.Font = new System.Drawing.Font("Segoe UI", 15F, System.Drawing.FontStyle.Bold);
            this.lbl_title_control.Location = new System.Drawing.Point(3, 192);
            this.lbl_title_control.Name = "lbl_title_control";
            this.lbl_title_control.Size = new System.Drawing.Size(1531, 58);
            this.lbl_title_control.TabIndex = 16;
            this.lbl_title_control.Text = "Subsystems Control";
            this.lbl_title_control.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_pick
            // 
            this.lbl_pick.AutoSize = true;
            this.lbl_pick.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_pick.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_pick.Location = new System.Drawing.Point(222, 317);
            this.lbl_pick.Name = "lbl_pick";
            this.lbl_pick.Size = new System.Drawing.Size(213, 67);
            this.lbl_pick.TabIndex = 2;
            this.lbl_pick.Text = "Pick";
            this.lbl_pick.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_approachShuttle
            // 
            this.lbl_approachShuttle.AutoSize = true;
            this.lbl_approachShuttle.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_approachShuttle.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_approachShuttle.Location = new System.Drawing.Point(441, 317);
            this.lbl_approachShuttle.Name = "lbl_approachShuttle";
            this.lbl_approachShuttle.Size = new System.Drawing.Size(213, 67);
            this.lbl_approachShuttle.TabIndex = 3;
            this.lbl_approachShuttle.Text = "Approach shuttle";
            this.lbl_approachShuttle.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_place
            // 
            this.lbl_place.AutoSize = true;
            this.lbl_place.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_place.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_place.Location = new System.Drawing.Point(660, 317);
            this.lbl_place.Name = "lbl_place";
            this.lbl_place.Size = new System.Drawing.Size(213, 67);
            this.lbl_place.TabIndex = 4;
            this.lbl_place.Text = "Place";
            this.lbl_place.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_rotate
            // 
            this.lbl_rotate.AutoSize = true;
            this.lbl_rotate.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_rotate.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_rotate.Location = new System.Drawing.Point(879, 317);
            this.lbl_rotate.Name = "lbl_rotate";
            this.lbl_rotate.Size = new System.Drawing.Size(213, 67);
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
            this.lbl_title_scara.Location = new System.Drawing.Point(3, 250);
            this.lbl_title_scara.Name = "lbl_title_scara";
            this.lbl_title_scara.Size = new System.Drawing.Size(870, 67);
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
            this.lbl_title_flexibowl.Location = new System.Drawing.Point(879, 250);
            this.lbl_title_flexibowl.Name = "lbl_title_flexibowl";
            this.lbl_title_flexibowl.Size = new System.Drawing.Size(655, 67);
            this.lbl_title_flexibowl.TabIndex = 7;
            this.lbl_title_flexibowl.Text = "FLEXIBOWL CONTROL";
            this.lbl_title_flexibowl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_shake
            // 
            this.lbl_shake.AutoSize = true;
            this.lbl_shake.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_shake.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_shake.Location = new System.Drawing.Point(1098, 317);
            this.lbl_shake.Name = "lbl_shake";
            this.lbl_shake.Size = new System.Drawing.Size(213, 67);
            this.lbl_shake.TabIndex = 8;
            this.lbl_shake.Text = "Shake";
            this.lbl_shake.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_flip
            // 
            this.lbl_flip.AutoSize = true;
            this.lbl_flip.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_flip.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_flip.Location = new System.Drawing.Point(1317, 317);
            this.lbl_flip.Name = "lbl_flip";
            this.lbl_flip.Size = new System.Drawing.Size(217, 67);
            this.lbl_flip.TabIndex = 9;
            this.lbl_flip.Text = "Flip";
            this.lbl_flip.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // led_pick
            // 
            this.led_pick.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_pick.Location = new System.Drawing.Point(298, 391);
            this.led_pick.Name = "led_pick";
            this.led_pick.On = false;
            this.led_pick.Size = new System.Drawing.Size(60, 53);
            this.led_pick.TabIndex = 10;
            this.led_pick.Text = "ledBulb2";
            // 
            // led_approachShuttle
            // 
            this.led_approachShuttle.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_approachShuttle.Location = new System.Drawing.Point(517, 391);
            this.led_approachShuttle.Name = "led_approachShuttle";
            this.led_approachShuttle.On = false;
            this.led_approachShuttle.Size = new System.Drawing.Size(60, 53);
            this.led_approachShuttle.TabIndex = 11;
            this.led_approachShuttle.Text = "ledBulb3";
            // 
            // led_place
            // 
            this.led_place.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_place.Location = new System.Drawing.Point(736, 391);
            this.led_place.Name = "led_place";
            this.led_place.On = false;
            this.led_place.Size = new System.Drawing.Size(60, 53);
            this.led_place.TabIndex = 12;
            this.led_place.Text = "ledBulb4";
            // 
            // led_rotate
            // 
            this.led_rotate.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_rotate.Location = new System.Drawing.Point(955, 391);
            this.led_rotate.Name = "led_rotate";
            this.led_rotate.On = false;
            this.led_rotate.Size = new System.Drawing.Size(60, 53);
            this.led_rotate.TabIndex = 13;
            this.led_rotate.Text = "ledBulb5";
            // 
            // led_shake
            // 
            this.led_shake.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_shake.Location = new System.Drawing.Point(1174, 391);
            this.led_shake.Name = "led_shake";
            this.led_shake.On = false;
            this.led_shake.Size = new System.Drawing.Size(60, 53);
            this.led_shake.TabIndex = 14;
            this.led_shake.Text = "ledBulb6";
            // 
            // led_flip
            // 
            this.led_flip.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_flip.Location = new System.Drawing.Point(1395, 391);
            this.led_flip.Name = "led_flip";
            this.led_flip.On = false;
            this.led_flip.Size = new System.Drawing.Size(60, 53);
            this.led_flip.TabIndex = 15;
            this.led_flip.Text = "ledBulb7";
            // 
            // lbl_camera
            // 
            this.lbl_camera.AutoSize = true;
            this.lbl_camera.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_camera.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_camera.ForeColor = System.Drawing.SystemColors.AppWorkspace;
            this.lbl_camera.Location = new System.Drawing.Point(3, 58);
            this.lbl_camera.Name = "lbl_camera";
            this.lbl_camera.Size = new System.Drawing.Size(213, 67);
            this.lbl_camera.TabIndex = 19;
            this.lbl_camera.Text = "Active streaming:";
            this.lbl_camera.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_approachFlexibowl
            // 
            this.lbl_approachFlexibowl.AutoSize = true;
            this.lbl_approachFlexibowl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_approachFlexibowl.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_approachFlexibowl.Location = new System.Drawing.Point(3, 317);
            this.lbl_approachFlexibowl.Name = "lbl_approachFlexibowl";
            this.lbl_approachFlexibowl.Size = new System.Drawing.Size(213, 67);
            this.lbl_approachFlexibowl.TabIndex = 1;
            this.lbl_approachFlexibowl.Text = "Approach flexibowl";
            this.lbl_approachFlexibowl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // led_approachFlexibowl
            // 
            this.led_approachFlexibowl.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.led_approachFlexibowl.Location = new System.Drawing.Point(79, 391);
            this.led_approachFlexibowl.Name = "led_approachFlexibowl";
            this.led_approachFlexibowl.On = false;
            this.led_approachFlexibowl.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.led_approachFlexibowl.Size = new System.Drawing.Size(60, 53);
            this.led_approachFlexibowl.TabIndex = 0;
            // 
            // btn_start
            // 
            this.btn_start.BackgroundImage = global::OptiSort.Properties.Resources.playDisabled_2x2_pptx;
            this.btn_start.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_start.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_start.Location = new System.Drawing.Point(3, 3);
            this.btn_start.Name = "btn_start";
            this.btn_start.Size = new System.Drawing.Size(213, 52);
            this.btn_start.TabIndex = 23;
            this.btn_start.UseVisualStyleBackColor = true;
            this.btn_start.Click += new System.EventHandler(this.btn_start_Click);
            // 
            // tmr_process
            // 
            this.tmr_process.Enabled = true;
            this.tmr_process.Interval = 1000;
            this.tmr_process.Tick += new System.EventHandler(this.tmr_process_Tick);
            // 
            // lbl_elapsedTime
            // 
            this.lbl_elapsedTime.AutoSize = true;
            this.lbl_elapsedTime.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_elapsedTime.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_elapsedTime.ForeColor = System.Drawing.SystemColors.AppWorkspace;
            this.lbl_elapsedTime.Location = new System.Drawing.Point(3, 125);
            this.lbl_elapsedTime.Name = "lbl_elapsedTime";
            this.lbl_elapsedTime.Size = new System.Drawing.Size(213, 67);
            this.lbl_elapsedTime.TabIndex = 25;
            this.lbl_elapsedTime.Text = "Elapsed Time:";
            this.lbl_elapsedTime.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_cycleTimer
            // 
            this.lbl_cycleTimer.AutoSize = true;
            this.lbl_cycleTimer.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_cycleTimer.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_cycleTimer.Location = new System.Drawing.Point(222, 125);
            this.lbl_cycleTimer.Name = "lbl_cycleTimer";
            this.lbl_cycleTimer.Size = new System.Drawing.Size(213, 67);
            this.lbl_cycleTimer.TabIndex = 26;
            this.lbl_cycleTimer.Text = "mm:ss";
            this.lbl_cycleTimer.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_Adect
            // 
            this.lbl_Adect.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.lbl_Adect, 2);
            this.lbl_Adect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_Adect.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_Adect.ForeColor = System.Drawing.SystemColors.AppWorkspace;
            this.lbl_Adect.Location = new System.Drawing.Point(441, 58);
            this.lbl_Adect.Name = "lbl_Adect";
            this.lbl_Adect.Size = new System.Drawing.Size(432, 67);
            this.lbl_Adect.TabIndex = 28;
            this.lbl_Adect.Text = "Components A detected:";
            this.lbl_Adect.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.tbl_controls.SetColumnSpan(this.label3, 2);
            this.label3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ForeColor = System.Drawing.SystemColors.AppWorkspace;
            this.label3.Location = new System.Drawing.Point(441, 125);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(432, 67);
            this.label3.TabIndex = 27;
            this.label3.Text = "Components B detected:";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_Adetected
            // 
            this.lbl_Adetected.AutoSize = true;
            this.lbl_Adetected.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_Adetected.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_Adetected.Location = new System.Drawing.Point(879, 58);
            this.lbl_Adetected.Name = "lbl_Adetected";
            this.lbl_Adetected.Size = new System.Drawing.Size(213, 67);
            this.lbl_Adetected.TabIndex = 29;
            this.lbl_Adetected.Text = "#";
            this.lbl_Adetected.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_Bdetected
            // 
            this.lbl_Bdetected.AutoSize = true;
            this.lbl_Bdetected.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_Bdetected.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_Bdetected.Location = new System.Drawing.Point(879, 125);
            this.lbl_Bdetected.Name = "lbl_Bdetected";
            this.lbl_Bdetected.Size = new System.Drawing.Size(213, 67);
            this.lbl_Bdetected.TabIndex = 30;
            this.lbl_Bdetected.Text = "#";
            this.lbl_Bdetected.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_picked
            // 
            this.lbl_picked.AutoSize = true;
            this.lbl_picked.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_picked.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_picked.ForeColor = System.Drawing.SystemColors.AppWorkspace;
            this.lbl_picked.Location = new System.Drawing.Point(1098, 58);
            this.lbl_picked.Name = "lbl_picked";
            this.lbl_picked.Size = new System.Drawing.Size(213, 67);
            this.lbl_picked.TabIndex = 31;
            this.lbl_picked.Text = "Number picked:";
            this.lbl_picked.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_discarded
            // 
            this.lbl_discarded.AutoSize = true;
            this.lbl_discarded.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_discarded.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_discarded.ForeColor = System.Drawing.SystemColors.AppWorkspace;
            this.lbl_discarded.Location = new System.Drawing.Point(1098, 125);
            this.lbl_discarded.Name = "lbl_discarded";
            this.lbl_discarded.Size = new System.Drawing.Size(213, 67);
            this.lbl_discarded.TabIndex = 32;
            this.lbl_discarded.Text = "Number ignored:";
            this.lbl_discarded.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_nrPicked
            // 
            this.lbl_nrPicked.AutoSize = true;
            this.lbl_nrPicked.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_nrPicked.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_nrPicked.Location = new System.Drawing.Point(1317, 58);
            this.lbl_nrPicked.Name = "lbl_nrPicked";
            this.lbl_nrPicked.Size = new System.Drawing.Size(217, 67);
            this.lbl_nrPicked.TabIndex = 33;
            this.lbl_nrPicked.Text = "#";
            this.lbl_nrPicked.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_nrDiscarded
            // 
            this.lbl_nrDiscarded.AutoSize = true;
            this.lbl_nrDiscarded.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_nrDiscarded.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_nrDiscarded.Location = new System.Drawing.Point(1317, 125);
            this.lbl_nrDiscarded.Name = "lbl_nrDiscarded";
            this.lbl_nrDiscarded.Size = new System.Drawing.Size(217, 67);
            this.lbl_nrDiscarded.TabIndex = 34;
            this.lbl_nrDiscarded.Text = "#";
            this.lbl_nrDiscarded.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
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
        private System.Windows.Forms.Label lbl_title_setup;
        private System.Windows.Forms.Timer tmr_process;
        private System.Windows.Forms.Button btn_stop;
        private System.Windows.Forms.Label lbl_actualSelectedCamera;
        private System.Windows.Forms.Label lbl_camera;
        private System.Windows.Forms.Button btn_start;
        private System.Windows.Forms.Label lbl_Adect;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label lbl_cycleTimer;
        private System.Windows.Forms.Label lbl_elapsedTime;
        private System.Windows.Forms.Label lbl_Adetected;
        private System.Windows.Forms.Label lbl_nrDiscarded;
        private System.Windows.Forms.Label lbl_nrPicked;
        private System.Windows.Forms.Label lbl_discarded;
        private System.Windows.Forms.Label lbl_picked;
        private System.Windows.Forms.Label lbl_Bdetected;
    }
}
