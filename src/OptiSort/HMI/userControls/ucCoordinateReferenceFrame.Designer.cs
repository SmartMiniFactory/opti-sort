namespace OptiSort.userControls
{
    partial class ucCoordinateReferenceFrame
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
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.num_size = new System.Windows.Forms.NumericUpDown();
            this.num_rows = new System.Windows.Forms.NumericUpDown();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.btn_StartCoordinateRefCalibration = new System.Windows.Forms.Button();
            this.lbl_lastCalibrationDateTime = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.num_columns = new System.Windows.Forms.NumericUpDown();
            this.tableLayoutPanel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.num_size)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_rows)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_columns)).BeginInit();
            this.SuspendLayout();
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 3;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33333F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tableLayoutPanel1.Controls.Add(this.num_size, 2, 3);
            this.tableLayoutPanel1.Controls.Add(this.num_rows, 1, 3);
            this.tableLayoutPanel1.Controls.Add(this.label4, 2, 2);
            this.tableLayoutPanel1.Controls.Add(this.label3, 1, 2);
            this.tableLayoutPanel1.Controls.Add(this.btn_StartCoordinateRefCalibration, 1, 5);
            this.tableLayoutPanel1.Controls.Add(this.lbl_lastCalibrationDateTime, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.label2, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.label5, 0, 4);
            this.tableLayoutPanel1.Controls.Add(this.num_columns, 0, 3);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Margin = new System.Windows.Forms.Padding(2);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 7;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(741, 441);
            this.tableLayoutPanel1.TabIndex = 0;
            // 
            // num_size
            // 
            this.num_size.Dock = System.Windows.Forms.DockStyle.Fill;
            this.num_size.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.num_size.Location = new System.Drawing.Point(513, 134);
            this.num_size.Margin = new System.Windows.Forms.Padding(20, 2, 20, 2);
            this.num_size.Name = "num_size";
            this.num_size.Size = new System.Drawing.Size(208, 29);
            this.num_size.TabIndex = 17;
            this.num_size.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.num_size.Value = new decimal(new int[] {
            12,
            0,
            0,
            0});
            // 
            // num_rows
            // 
            this.num_rows.Dock = System.Windows.Forms.DockStyle.Fill;
            this.num_rows.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.num_rows.Location = new System.Drawing.Point(266, 134);
            this.num_rows.Margin = new System.Windows.Forms.Padding(20, 2, 20, 2);
            this.num_rows.Name = "num_rows";
            this.num_rows.Size = new System.Drawing.Size(207, 29);
            this.num_rows.TabIndex = 16;
            this.num_rows.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.num_rows.Value = new decimal(new int[] {
            4,
            0,
            0,
            0});
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label4.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.Location = new System.Drawing.Point(495, 88);
            this.label4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(244, 44);
            this.label4.TabIndex = 13;
            this.label4.Text = "Square Size (mm)";
            this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.Location = new System.Drawing.Point(248, 88);
            this.label3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(243, 44);
            this.label3.TabIndex = 12;
            this.label3.Text = "Grid Rows";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btn_StartCoordinateRefCalibration
            // 
            this.btn_StartCoordinateRefCalibration.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_StartCoordinateRefCalibration.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_StartCoordinateRefCalibration.Location = new System.Drawing.Point(259, 277);
            this.btn_StartCoordinateRefCalibration.Margin = new System.Windows.Forms.Padding(13);
            this.btn_StartCoordinateRefCalibration.Name = "btn_StartCoordinateRefCalibration";
            this.btn_StartCoordinateRefCalibration.Size = new System.Drawing.Size(221, 62);
            this.btn_StartCoordinateRefCalibration.TabIndex = 5;
            this.btn_StartCoordinateRefCalibration.Text = "Start Automatic Procedure";
            this.btn_StartCoordinateRefCalibration.UseVisualStyleBackColor = true;
            this.btn_StartCoordinateRefCalibration.Click += new System.EventHandler(this.btn_StartCoordinateRefCalibration_Click);
            // 
            // lbl_lastCalibrationDateTime
            // 
            this.lbl_lastCalibrationDateTime.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.lbl_lastCalibrationDateTime, 3);
            this.lbl_lastCalibrationDateTime.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_lastCalibrationDateTime.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_lastCalibrationDateTime.Location = new System.Drawing.Point(2, 0);
            this.lbl_lastCalibrationDateTime.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.lbl_lastCalibrationDateTime.Name = "lbl_lastCalibrationDateTime";
            this.lbl_lastCalibrationDateTime.Size = new System.Drawing.Size(737, 44);
            this.lbl_lastCalibrationDateTime.TabIndex = 6;
            this.lbl_lastCalibrationDateTime.Text = "Last Calibration - dd/mm/yyyy hh:mm";
            this.lbl_lastCalibrationDateTime.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(2, 88);
            this.label2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(242, 44);
            this.label2.TabIndex = 11;
            this.label2.Text = "Grid Columns";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.label5, 3);
            this.label5.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label5.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, ((System.Drawing.FontStyle)((System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Italic))), System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.Location = new System.Drawing.Point(2, 176);
            this.label5.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(737, 88);
            this.label5.TabIndex = 14;
            this.label5.Text = "Free flexibowl\'s surface from laid components!! SCARA will place the calibration " +
    "grid!";
            this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // num_columns
            // 
            this.num_columns.Dock = System.Windows.Forms.DockStyle.Fill;
            this.num_columns.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.num_columns.Location = new System.Drawing.Point(20, 134);
            this.num_columns.Margin = new System.Windows.Forms.Padding(20, 2, 20, 2);
            this.num_columns.Name = "num_columns";
            this.num_columns.Size = new System.Drawing.Size(206, 29);
            this.num_columns.TabIndex = 15;
            this.num_columns.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.num_columns.Value = new decimal(new int[] {
            5,
            0,
            0,
            0});
            // 
            // ucCoordinateReferenceFrame
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel1);
            this.Margin = new System.Windows.Forms.Padding(2);
            this.Name = "ucCoordinateReferenceFrame";
            this.Size = new System.Drawing.Size(741, 441);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.num_size)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_rows)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_columns)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Button btn_StartCoordinateRefCalibration;
        private System.Windows.Forms.Label lbl_lastCalibrationDateTime;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.NumericUpDown num_size;
        private System.Windows.Forms.NumericUpDown num_rows;
        private System.Windows.Forms.NumericUpDown num_columns;
    }
}
