namespace OptiSort
{
    partial class ucScara
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
            this.dgvTargetQueue = new System.Windows.Forms.DataGridView();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.btnConnect = new System.Windows.Forms.Button();
            this.btnDisconnect = new System.Windows.Forms.Button();
            this.chkEmulate = new System.Windows.Forms.CheckBox();
            this.btnPower = new System.Windows.Forms.Button();
            this.btnJog = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.dgvTargetQueue)).BeginInit();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // dgvTargetQueue
            // 
            this.dgvTargetQueue.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.dgvTargetQueue.Dock = System.Windows.Forms.DockStyle.Fill;
            this.dgvTargetQueue.Location = new System.Drawing.Point(3, 3);
            this.dgvTargetQueue.Name = "dgvTargetQueue";
            this.dgvTargetQueue.RowHeadersWidth = 62;
            this.tableLayoutPanel1.SetRowSpan(this.dgvTargetQueue, 5);
            this.dgvTargetQueue.RowTemplate.Height = 28;
            this.dgvTargetQueue.Size = new System.Drawing.Size(1324, 758);
            this.dgvTargetQueue.TabIndex = 0;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 92.61839F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 7.381614F));
            this.tableLayoutPanel1.Controls.Add(this.dgvTargetQueue, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnConnect, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.btnDisconnect, 1, 1);
            this.tableLayoutPanel1.Controls.Add(this.chkEmulate, 1, 2);
            this.tableLayoutPanel1.Controls.Add(this.btnPower, 1, 3);
            this.tableLayoutPanel1.Controls.Add(this.btnJog, 1, 4);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 5;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 20F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1436, 764);
            this.tableLayoutPanel1.TabIndex = 1;
            // 
            // btnConnect
            // 
            this.btnConnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnConnect.Location = new System.Drawing.Point(1333, 3);
            this.btnConnect.Name = "btnConnect";
            this.btnConnect.Size = new System.Drawing.Size(100, 146);
            this.btnConnect.TabIndex = 1;
            this.btnConnect.Text = "Connect";
            this.btnConnect.UseVisualStyleBackColor = true;
            this.btnConnect.Click += new System.EventHandler(this.btnConnect_Click);
            // 
            // btnDisconnect
            // 
            this.btnDisconnect.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnDisconnect.Location = new System.Drawing.Point(1333, 155);
            this.btnDisconnect.Name = "btnDisconnect";
            this.btnDisconnect.Size = new System.Drawing.Size(100, 146);
            this.btnDisconnect.TabIndex = 2;
            this.btnDisconnect.Text = "Disconnect";
            this.btnDisconnect.UseVisualStyleBackColor = true;
            this.btnDisconnect.Click += new System.EventHandler(this.btnDisconnect_Click);
            // 
            // chkEmulate
            // 
            this.chkEmulate.AutoSize = true;
            this.chkEmulate.Checked = true;
            this.chkEmulate.CheckState = System.Windows.Forms.CheckState.Checked;
            this.chkEmulate.Dock = System.Windows.Forms.DockStyle.Fill;
            this.chkEmulate.Location = new System.Drawing.Point(1333, 307);
            this.chkEmulate.Name = "chkEmulate";
            this.chkEmulate.Size = new System.Drawing.Size(100, 146);
            this.chkEmulate.TabIndex = 3;
            this.chkEmulate.Text = "Emulate";
            this.chkEmulate.UseVisualStyleBackColor = true;
            // 
            // btnPower
            // 
            this.btnPower.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnPower.Location = new System.Drawing.Point(1333, 459);
            this.btnPower.Name = "btnPower";
            this.btnPower.Size = new System.Drawing.Size(100, 146);
            this.btnPower.TabIndex = 4;
            this.btnPower.Text = "Power ON/OFF";
            this.btnPower.UseVisualStyleBackColor = true;
            // 
            // btnJog
            // 
            this.btnJog.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnJog.Location = new System.Drawing.Point(1333, 611);
            this.btnJog.Name = "btnJog";
            this.btnJog.Size = new System.Drawing.Size(100, 150);
            this.btnJog.TabIndex = 5;
            this.btnJog.Text = "JOG";
            this.btnJog.UseVisualStyleBackColor = true;
            // 
            // ucScara
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "ucScara";
            this.Size = new System.Drawing.Size(1436, 764);
            this.Load += new System.EventHandler(this.ucScara_Load);
            ((System.ComponentModel.ISupportInitialize)(this.dgvTargetQueue)).EndInit();
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.DataGridView dgvTargetQueue;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Button btnConnect;
        private System.Windows.Forms.Button btnDisconnect;
        private System.Windows.Forms.CheckBox chkEmulate;
        private System.Windows.Forms.Button btnPower;
        private System.Windows.Forms.Button btnJog;
    }
}
