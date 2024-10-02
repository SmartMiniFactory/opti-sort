namespace OptiSort
{
    partial class ucCameraStream
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
            this.pnlStream = new System.Windows.Forms.Panel();
            this.SuspendLayout();
            // 
            // pnlStream
            // 
            this.pnlStream.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlStream.Location = new System.Drawing.Point(0, 0);
            this.pnlStream.Name = "pnlStream";
            this.pnlStream.Size = new System.Drawing.Size(842, 752);
            this.pnlStream.TabIndex = 0;
            // 
            // CameraStream
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.pnlStream);
            this.Name = "CameraStream";
            this.Size = new System.Drawing.Size(842, 752);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel pnlStream;
    }
}
