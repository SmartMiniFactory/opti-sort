﻿namespace OptiSort.userControls
{
    partial class ucRobotSimulation
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
            this.pnlRobotView = new System.Windows.Forms.Panel();
            this.SuspendLayout();
            // 
            // pnlRobotView
            // 
            this.pnlRobotView.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlRobotView.Location = new System.Drawing.Point(0, 0);
            this.pnlRobotView.Name = "pnlRobotView";
            this.pnlRobotView.Size = new System.Drawing.Size(478, 474);
            this.pnlRobotView.TabIndex = 0;
            // 
            // ucRobotSimulation
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.pnlRobotView);
            this.Name = "ucRobotSimulation";
            this.Size = new System.Drawing.Size(478, 474);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel pnlRobotView;
    }
}
