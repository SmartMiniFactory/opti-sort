namespace OptiSort
{
    partial class ucRobotView
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
            this.pnl3D = new System.Windows.Forms.Panel();
            this.SuspendLayout();
            // 
            // pnl3D
            // 
            this.pnl3D.BackColor = System.Drawing.SystemColors.ButtonShadow;
            this.pnl3D.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnl3D.Location = new System.Drawing.Point(0, 0);
            this.pnl3D.Name = "pnl3D";
            this.pnl3D.Size = new System.Drawing.Size(648, 572);
            this.pnl3D.TabIndex = 0;
            // 
            // ucRobotView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.pnl3D);
            this.Name = "ucRobotView";
            this.Size = new System.Drawing.Size(648, 572);
            this.Load += new System.EventHandler(this.ucRobotView_Load);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel pnl3D;
    }
}
