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
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.pnlScara = new System.Windows.Forms.Panel();
            this.pnlFlexibowl = new System.Windows.Forms.Panel();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 1;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel1.Controls.Add(this.pnlScara, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.pnlFlexibowl, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 3;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 45F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 45F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1537, 1034);
            this.tableLayoutPanel1.TabIndex = 0;
            // 
            // pnlScara
            // 
            this.pnlScara.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlScara.Location = new System.Drawing.Point(3, 571);
            this.pnlScara.Name = "pnlScara";
            this.pnlScara.Size = new System.Drawing.Size(1531, 460);
            this.pnlScara.TabIndex = 20;
            // 
            // pnlFlexibowl
            // 
            this.pnlFlexibowl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlFlexibowl.Location = new System.Drawing.Point(3, 3);
            this.pnlFlexibowl.Name = "pnlFlexibowl";
            this.tableLayoutPanel1.SetRowSpan(this.pnlFlexibowl, 2);
            this.pnlFlexibowl.Size = new System.Drawing.Size(1531, 562);
            this.pnlFlexibowl.TabIndex = 24;
            // 
            // ucProcessView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "ucProcessView";
            this.Size = new System.Drawing.Size(1537, 1034);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Panel pnlScara;
        private System.Windows.Forms.Panel pnlFlexibowl;
    }
}
