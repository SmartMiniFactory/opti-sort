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
            this.lstLog = new System.Windows.Forms.ListBox();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.btnConfig = new System.Windows.Forms.Button();
            this.btnManual = new System.Windows.Forms.Button();
            this.btnProcess = new System.Windows.Forms.Button();
            this.pnlCurrentUc = new System.Windows.Forms.Panel();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // lstLog
            // 
            this.lstLog.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.lstLog.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lstLog.FormattingEnabled = true;
            this.lstLog.ItemHeight = 20;
            this.lstLog.Location = new System.Drawing.Point(3, 976);
            this.lstLog.Name = "lstLog";
            this.lstLog.Size = new System.Drawing.Size(1337, 98);
            this.lstLog.TabIndex = 0;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 4;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 73.91304F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 8.69565F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 8.69565F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 8.69565F));
            this.tableLayoutPanel1.Controls.Add(this.btnConfig, 3, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnManual, 2, 1);
            this.tableLayoutPanel1.Controls.Add(this.lstLog, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.btnProcess, 1, 1);
            this.tableLayoutPanel1.Controls.Add(this.pnlCurrentUc, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 2;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 90.38461F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 9.615385F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1818, 1077);
            this.tableLayoutPanel1.TabIndex = 1;
            // 
            // btnConfig
            // 
            this.btnConfig.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnConfig.Font = new System.Drawing.Font("Segoe UI", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnConfig.Location = new System.Drawing.Point(1662, 976);
            this.btnConfig.Name = "btnConfig";
            this.btnConfig.Size = new System.Drawing.Size(153, 98);
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
            this.btnManual.Location = new System.Drawing.Point(1504, 976);
            this.btnManual.Name = "btnManual";
            this.btnManual.Size = new System.Drawing.Size(152, 98);
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
            this.btnProcess.Location = new System.Drawing.Point(1346, 976);
            this.btnProcess.Name = "btnProcess";
            this.btnProcess.Size = new System.Drawing.Size(152, 98);
            this.btnProcess.TabIndex = 1;
            this.btnProcess.TabStop = false;
            this.btnProcess.Text = "AUTO";
            this.btnProcess.UseVisualStyleBackColor = true;
            this.btnProcess.Click += new System.EventHandler(this.btnProcess_Click);
            // 
            // pnlCurrentUc
            // 
            this.tableLayoutPanel1.SetColumnSpan(this.pnlCurrentUc, 4);
            this.pnlCurrentUc.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlCurrentUc.Location = new System.Drawing.Point(3, 3);
            this.pnlCurrentUc.Name = "pnlCurrentUc";
            this.pnlCurrentUc.Size = new System.Drawing.Size(1812, 967);
            this.pnlCurrentUc.TabIndex = 4;
            // 
            // frmMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.Color.Gainsboro;
            this.ClientSize = new System.Drawing.Size(1818, 1077);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "frmMain";
            this.Text = "OptiSort";
            this.tableLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ListBox lstLog;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Button btnProcess;
        private System.Windows.Forms.Button btnConfig;
        private System.Windows.Forms.Button btnManual;
        private System.Windows.Forms.Panel pnlCurrentUc;
    }
}

