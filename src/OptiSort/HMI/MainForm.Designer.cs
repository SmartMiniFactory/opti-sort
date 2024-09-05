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
            this.listBox = new System.Windows.Forms.ListBox();
            this.labelYaw = new System.Windows.Forms.Label();
            this.textBoxYaw = new System.Windows.Forms.TextBox();
            this.labelPitch = new System.Windows.Forms.Label();
            this.textBoxPitch = new System.Windows.Forms.TextBox();
            this.labelRoll = new System.Windows.Forms.Label();
            this.textBoxRoll = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.groupBoxTargetQueue = new System.Windows.Forms.GroupBox();
            this._targetQueue = new System.Windows.Forms.DataGridView();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBoxTargetQueue.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this._targetQueue)).BeginInit();
            this.SuspendLayout();
            // 
            // textBoxX
            // 
            this.textBoxX.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxX.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxX.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxX.Location = new System.Drawing.Point(41, 27);
            this.textBoxX.Name = "textBoxX";
            this.textBoxX.ReadOnly = true;
            this.textBoxX.Size = new System.Drawing.Size(100, 26);
            this.textBoxX.TabIndex = 0;
            // 
            // labelX
            // 
            this.labelX.AutoSize = true;
            this.labelX.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelX.Location = new System.Drawing.Point(5, 33);
            this.labelX.Name = "labelX";
            this.labelX.Size = new System.Drawing.Size(24, 20);
            this.labelX.TabIndex = 1;
            this.labelX.Text = "X:";
            // 
            // labelY
            // 
            this.labelY.AutoSize = true;
            this.labelY.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelY.Location = new System.Drawing.Point(6, 64);
            this.labelY.Name = "labelY";
            this.labelY.Size = new System.Drawing.Size(24, 20);
            this.labelY.TabIndex = 3;
            this.labelY.Text = "Y:";
            // 
            // textBoxY
            // 
            this.textBoxY.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxY.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxY.Location = new System.Drawing.Point(41, 59);
            this.textBoxY.Name = "textBoxY";
            this.textBoxY.ReadOnly = true;
            this.textBoxY.Size = new System.Drawing.Size(100, 26);
            this.textBoxY.TabIndex = 2;
            // 
            // labelZ
            // 
            this.labelZ.AutoSize = true;
            this.labelZ.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelZ.Location = new System.Drawing.Point(6, 96);
            this.labelZ.Name = "labelZ";
            this.labelZ.Size = new System.Drawing.Size(23, 20);
            this.labelZ.TabIndex = 5;
            this.labelZ.Text = "Z:";
            // 
            // textBoxZ
            // 
            this.textBoxZ.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxZ.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxZ.Location = new System.Drawing.Point(41, 90);
            this.textBoxZ.Name = "textBoxZ";
            this.textBoxZ.ReadOnly = true;
            this.textBoxZ.Size = new System.Drawing.Size(100, 26);
            this.textBoxZ.TabIndex = 4;
            // 
            // listBox
            // 
            this.listBox.Dock = System.Windows.Forms.DockStyle.Fill;
            this.listBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.listBox.FormattingEnabled = true;
            this.listBox.HorizontalScrollbar = true;
            this.listBox.ItemHeight = 15;
            this.listBox.Location = new System.Drawing.Point(2, 21);
            this.listBox.Name = "listBox";
            this.listBox.Size = new System.Drawing.Size(620, 287);
            this.listBox.TabIndex = 6;
            // 
            // labelYaw
            // 
            this.labelYaw.AutoSize = true;
            this.labelYaw.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelYaw.Location = new System.Drawing.Point(155, 93);
            this.labelYaw.Name = "labelYaw";
            this.labelYaw.Size = new System.Drawing.Size(44, 20);
            this.labelYaw.TabIndex = 12;
            this.labelYaw.Text = "Yaw:";
            // 
            // textBoxYaw
            // 
            this.textBoxYaw.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxYaw.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxYaw.Location = new System.Drawing.Point(207, 90);
            this.textBoxYaw.Name = "textBoxYaw";
            this.textBoxYaw.ReadOnly = true;
            this.textBoxYaw.Size = new System.Drawing.Size(100, 26);
            this.textBoxYaw.TabIndex = 11;
            // 
            // labelPitch
            // 
            this.labelPitch.AutoSize = true;
            this.labelPitch.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelPitch.Location = new System.Drawing.Point(155, 64);
            this.labelPitch.Name = "labelPitch";
            this.labelPitch.Size = new System.Drawing.Size(48, 20);
            this.labelPitch.TabIndex = 10;
            this.labelPitch.Text = "Pitch:";
            // 
            // textBoxPitch
            // 
            this.textBoxPitch.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxPitch.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxPitch.Location = new System.Drawing.Point(207, 58);
            this.textBoxPitch.Name = "textBoxPitch";
            this.textBoxPitch.ReadOnly = true;
            this.textBoxPitch.Size = new System.Drawing.Size(100, 26);
            this.textBoxPitch.TabIndex = 9;
            // 
            // labelRoll
            // 
            this.labelRoll.AutoSize = true;
            this.labelRoll.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelRoll.Location = new System.Drawing.Point(161, 33);
            this.labelRoll.Name = "labelRoll";
            this.labelRoll.Size = new System.Drawing.Size(40, 20);
            this.labelRoll.TabIndex = 8;
            this.labelRoll.Text = "Roll:";
            // 
            // textBoxRoll
            // 
            this.textBoxRoll.BackColor = System.Drawing.SystemColors.WindowFrame;
            this.textBoxRoll.ForeColor = System.Drawing.Color.LightGoldenrodYellow;
            this.textBoxRoll.Location = new System.Drawing.Point(207, 27);
            this.textBoxRoll.Name = "textBoxRoll";
            this.textBoxRoll.ReadOnly = true;
            this.textBoxRoll.Size = new System.Drawing.Size(100, 26);
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
            this.groupBox1.Location = new System.Drawing.Point(6, 6);
            this.groupBox1.Margin = new System.Windows.Forms.Padding(2);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Padding = new System.Windows.Forms.Padding(2);
            this.groupBox1.Size = new System.Drawing.Size(314, 123);
            this.groupBox1.TabIndex = 14;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Current Target";
            // 
            // groupBox2
            // 
            this.groupBox2.BackColor = System.Drawing.Color.Teal;
            this.groupBox2.Controls.Add(this.listBox);
            this.groupBox2.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBox2.Location = new System.Drawing.Point(5, 131);
            this.groupBox2.Margin = new System.Windows.Forms.Padding(2);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Padding = new System.Windows.Forms.Padding(2);
            this.groupBox2.Size = new System.Drawing.Size(624, 310);
            this.groupBox2.TabIndex = 15;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Console";
            // 
            // groupBoxTargetQueue
            // 
            this.groupBoxTargetQueue.BackColor = System.Drawing.Color.GreenYellow;
            this.groupBoxTargetQueue.Controls.Add(this._targetQueue);
            this.groupBoxTargetQueue.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBoxTargetQueue.ImeMode = System.Windows.Forms.ImeMode.Off;
            this.groupBoxTargetQueue.Location = new System.Drawing.Point(9, 445);
            this.groupBoxTargetQueue.Margin = new System.Windows.Forms.Padding(2);
            this.groupBoxTargetQueue.Name = "groupBoxTargetQueue";
            this.groupBoxTargetQueue.Padding = new System.Windows.Forms.Padding(2);
            this.groupBoxTargetQueue.Size = new System.Drawing.Size(620, 245);
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
            this._targetQueue.Location = new System.Drawing.Point(2, 21);
            this._targetQueue.Margin = new System.Windows.Forms.Padding(2);
            this._targetQueue.MultiSelect = false;
            this._targetQueue.Name = "_targetQueue";
            this._targetQueue.RowHeadersVisible = false;
            this._targetQueue.RowTemplate.Height = 24;
            this._targetQueue.Size = new System.Drawing.Size(616, 222);
            this._targetQueue.TabIndex = 0;
            this._targetQueue.UseWaitCursor = true;
            // 
            // ucScara
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Gainsboro;
            this.ClientSize = new System.Drawing.Size(1212, 700);
            this.Controls.Add(this.groupBoxTargetQueue);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Name = "ucScara";
            this.Text = "SCARA Remote Control";
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBoxTargetQueue.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this._targetQueue)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TextBox textBoxX;
        private System.Windows.Forms.Label labelX;
        private System.Windows.Forms.Label labelY;
        private System.Windows.Forms.TextBox textBoxY;
        private System.Windows.Forms.Label labelZ;
        private System.Windows.Forms.TextBox textBoxZ;
        private System.Windows.Forms.ListBox listBox;
        private System.Windows.Forms.Label labelYaw;
        private System.Windows.Forms.TextBox textBoxYaw;
        private System.Windows.Forms.Label labelPitch;
        private System.Windows.Forms.TextBox textBoxPitch;
        private System.Windows.Forms.Label labelRoll;
        private System.Windows.Forms.TextBox textBoxRoll;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.GroupBox groupBoxTargetQueue;
        private System.Windows.Forms.DataGridView _targetQueue;
    }
}

