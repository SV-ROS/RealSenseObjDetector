namespace raw_streams.cs
{
    partial class MainForm
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
            if (disposing)
                cleanPictureBitmap();
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.Start = new System.Windows.Forms.Button();
            this.Stop = new System.Windows.Forms.Button();
            this.DeviceMenu = new System.Windows.Forms.ToolStripMenuItem();
            this.MainMenu = new System.Windows.Forms.MenuStrip();
            this.DepthMenu = new System.Windows.Forms.ToolStripMenuItem();
            this.DepthNone = new System.Windows.Forms.ToolStripMenuItem();
            this.ModeMenu = new System.Windows.Forms.ToolStripMenuItem();
            this.ModeLive = new System.Windows.Forms.ToolStripMenuItem();
            this.ModePlayback = new System.Windows.Forms.ToolStripMenuItem();
            this.ModeRecord = new System.Windows.Forms.ToolStripMenuItem();
            this.Status2 = new System.Windows.Forms.StatusStrip();
            this.StatusLabel = new System.Windows.Forms.ToolStripStatusLabel();
            this.panel1 = new System.Windows.Forms.Panel();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanel2 = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanel3 = new System.Windows.Forms.TableLayoutPanel();
            this.buttonSaveToPcd = new System.Windows.Forms.Button();
            this.tableLayoutPanel7 = new System.Windows.Forms.TableLayoutPanel();
            this.textBoxPcdFilePath = new System.Windows.Forms.TextBox();
            this.buttonSelectPcdFilePath = new System.Windows.Forms.Button();
            this.tableLayoutPanel8 = new System.Windows.Forms.TableLayoutPanel();
            this.radioButtonBinary = new System.Windows.Forms.RadioButton();
            this.radioButtonAscii = new System.Windows.Forms.RadioButton();
            this.tableLayoutPanel6 = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanel4 = new System.Windows.Forms.TableLayoutPanel();
            this.comboBoxNormalEstimator1 = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.textBoxMaxDepthChangeFactor1 = new System.Windows.Forms.TextBox();
            this.textBoxNormalSmoothingSize1 = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.comboBoxQualityEstimator = new System.Windows.Forms.ComboBox();
            this.tableLayoutPanel5 = new System.Windows.Forms.TableLayoutPanel();
            this.trackBarMaxBad = new System.Windows.Forms.TrackBar();
            this.trackBarMinGood = new System.Windows.Forms.TrackBar();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.textBoxMaxDepthChangeFactor2 = new System.Windows.Forms.TextBox();
            this.textBoxNormalSmoothingSize2 = new System.Windows.Forms.TextBox();
            this.comboBoxNormalEstimator2 = new System.Windows.Forms.ComboBox();
            this.MainMenu.SuspendLayout();
            this.Status2.SuspendLayout();
            this.panel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.tableLayoutPanel1.SuspendLayout();
            this.tableLayoutPanel2.SuspendLayout();
            this.tableLayoutPanel3.SuspendLayout();
            this.tableLayoutPanel7.SuspendLayout();
            this.tableLayoutPanel8.SuspendLayout();
            this.tableLayoutPanel6.SuspendLayout();
            this.tableLayoutPanel4.SuspendLayout();
            this.tableLayoutPanel5.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarMaxBad)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarMinGood)).BeginInit();
            this.SuspendLayout();
            // 
            // Start
            // 
            this.Start.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.Start.Location = new System.Drawing.Point(3, 3);
            this.Start.Name = "Start";
            this.Start.Size = new System.Drawing.Size(82, 23);
            this.Start.TabIndex = 2;
            this.Start.Text = "Start";
            this.Start.UseVisualStyleBackColor = true;
            this.Start.Click += new System.EventHandler(this.Start_Click);
            // 
            // Stop
            // 
            this.Stop.Dock = System.Windows.Forms.DockStyle.Top;
            this.Stop.Enabled = false;
            this.Stop.Location = new System.Drawing.Point(3, 32);
            this.Stop.Name = "Stop";
            this.Stop.Size = new System.Drawing.Size(82, 23);
            this.Stop.TabIndex = 3;
            this.Stop.Text = "Stop";
            this.Stop.UseVisualStyleBackColor = true;
            this.Stop.Click += new System.EventHandler(this.Stop_Click);
            // 
            // DeviceMenu
            // 
            this.DeviceMenu.Name = "DeviceMenu";
            this.DeviceMenu.Size = new System.Drawing.Size(54, 20);
            this.DeviceMenu.Text = "Device";
            // 
            // MainMenu
            // 
            this.MainMenu.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.DeviceMenu,
            this.DepthMenu,
            this.ModeMenu});
            this.MainMenu.Location = new System.Drawing.Point(0, 0);
            this.MainMenu.Name = "MainMenu";
            this.MainMenu.RenderMode = System.Windows.Forms.ToolStripRenderMode.Professional;
            this.MainMenu.Size = new System.Drawing.Size(1052, 24);
            this.MainMenu.TabIndex = 0;
            this.MainMenu.Text = "MainMenu";
            // 
            // DepthMenu
            // 
            this.DepthMenu.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.DepthNone});
            this.DepthMenu.Name = "DepthMenu";
            this.DepthMenu.Size = new System.Drawing.Size(51, 20);
            this.DepthMenu.Text = "Depth";
            // 
            // DepthNone
            // 
            this.DepthNone.Name = "DepthNone";
            this.DepthNone.Size = new System.Drawing.Size(103, 22);
            this.DepthNone.Text = "None";
            // 
            // ModeMenu
            // 
            this.ModeMenu.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.ModeLive,
            this.ModePlayback,
            this.ModeRecord});
            this.ModeMenu.Name = "ModeMenu";
            this.ModeMenu.Size = new System.Drawing.Size(50, 20);
            this.ModeMenu.Text = "Mode";
            // 
            // ModeLive
            // 
            this.ModeLive.Checked = true;
            this.ModeLive.CheckState = System.Windows.Forms.CheckState.Checked;
            this.ModeLive.Name = "ModeLive";
            this.ModeLive.Size = new System.Drawing.Size(121, 22);
            this.ModeLive.Text = "Live";
            this.ModeLive.Click += new System.EventHandler(this.ModeLive_Click);
            // 
            // ModePlayback
            // 
            this.ModePlayback.Name = "ModePlayback";
            this.ModePlayback.Size = new System.Drawing.Size(121, 22);
            this.ModePlayback.Text = "Playback";
            this.ModePlayback.Click += new System.EventHandler(this.ModePlayback_Click);
            // 
            // ModeRecord
            // 
            this.ModeRecord.Name = "ModeRecord";
            this.ModeRecord.Size = new System.Drawing.Size(121, 22);
            this.ModeRecord.Text = "Record";
            this.ModeRecord.Click += new System.EventHandler(this.ModeRecord_Click);
            // 
            // Status2
            // 
            this.Status2.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.StatusLabel});
            this.Status2.LayoutStyle = System.Windows.Forms.ToolStripLayoutStyle.Flow;
            this.Status2.Location = new System.Drawing.Point(0, 514);
            this.Status2.Name = "Status2";
            this.Status2.Size = new System.Drawing.Size(1052, 20);
            this.Status2.TabIndex = 25;
            this.Status2.Text = "Status2";
            // 
            // StatusLabel
            // 
            this.StatusLabel.Name = "StatusLabel";
            this.StatusLabel.Size = new System.Drawing.Size(23, 15);
            this.StatusLabel.Text = "OK";
            // 
            // panel1
            // 
            this.panel1.AutoSize = true;
            this.panel1.Controls.Add(this.pictureBox1);
            this.panel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel1.Location = new System.Drawing.Point(3, 3);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(662, 484);
            this.panel1.TabIndex = 37;
            // 
            // pictureBox1
            // 
            this.pictureBox1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pictureBox1.Location = new System.Drawing.Point(0, 0);
            this.pictureBox1.MinimumSize = new System.Drawing.Size(642, 482);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(662, 484);
            this.pictureBox1.TabIndex = 0;
            this.pictureBox1.TabStop = false;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.AutoSize = true;
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel1.Controls.Add(this.tableLayoutPanel2, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.panel1, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 24);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 1;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1052, 490);
            this.tableLayoutPanel1.TabIndex = 38;
            // 
            // tableLayoutPanel2
            // 
            this.tableLayoutPanel2.AutoSize = true;
            this.tableLayoutPanel2.ColumnCount = 1;
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel2.Controls.Add(this.tableLayoutPanel3, 0, 0);
            this.tableLayoutPanel2.Controls.Add(this.tableLayoutPanel6, 0, 1);
            this.tableLayoutPanel2.Controls.Add(this.tableLayoutPanel4, 0, 2);
            this.tableLayoutPanel2.Controls.Add(this.tableLayoutPanel5, 0, 3);
            this.tableLayoutPanel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel2.Location = new System.Drawing.Point(671, 3);
            this.tableLayoutPanel2.Name = "tableLayoutPanel2";
            this.tableLayoutPanel2.RowCount = 4;
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel2.Size = new System.Drawing.Size(378, 484);
            this.tableLayoutPanel2.TabIndex = 0;
            // 
            // tableLayoutPanel3
            // 
            this.tableLayoutPanel3.ColumnCount = 2;
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel3.Controls.Add(this.buttonSaveToPcd, 0, 0);
            this.tableLayoutPanel3.Controls.Add(this.tableLayoutPanel7, 1, 0);
            this.tableLayoutPanel3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel3.Location = new System.Drawing.Point(3, 3);
            this.tableLayoutPanel3.Name = "tableLayoutPanel3";
            this.tableLayoutPanel3.RowCount = 1;
            this.tableLayoutPanel3.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel3.Size = new System.Drawing.Size(372, 88);
            this.tableLayoutPanel3.TabIndex = 0;
            // 
            // buttonSaveToPcd
            // 
            this.buttonSaveToPcd.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.buttonSaveToPcd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.buttonSaveToPcd.Location = new System.Drawing.Point(3, 3);
            this.buttonSaveToPcd.Name = "buttonSaveToPcd";
            this.buttonSaveToPcd.Size = new System.Drawing.Size(82, 82);
            this.buttonSaveToPcd.TabIndex = 28;
            this.buttonSaveToPcd.Text = "SaveToPcd";
            this.buttonSaveToPcd.UseVisualStyleBackColor = true;
            this.buttonSaveToPcd.Click += new System.EventHandler(this.buttonSaveToPcd_Click);
            // 
            // tableLayoutPanel7
            // 
            this.tableLayoutPanel7.ColumnCount = 2;
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel7.Controls.Add(this.textBoxPcdFilePath, 0, 0);
            this.tableLayoutPanel7.Controls.Add(this.buttonSelectPcdFilePath, 1, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel8, 0, 1);
            this.tableLayoutPanel7.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel7.Location = new System.Drawing.Point(91, 3);
            this.tableLayoutPanel7.Name = "tableLayoutPanel7";
            this.tableLayoutPanel7.RowCount = 2;
            this.tableLayoutPanel7.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel7.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel7.Size = new System.Drawing.Size(278, 82);
            this.tableLayoutPanel7.TabIndex = 29;
            // 
            // textBoxPcdFilePath
            // 
            this.textBoxPcdFilePath.Dock = System.Windows.Forms.DockStyle.Top;
            this.textBoxPcdFilePath.Location = new System.Drawing.Point(3, 3);
            this.textBoxPcdFilePath.Name = "textBoxPcdFilePath";
            this.textBoxPcdFilePath.ReadOnly = true;
            this.textBoxPcdFilePath.Size = new System.Drawing.Size(211, 20);
            this.textBoxPcdFilePath.TabIndex = 0;
            // 
            // buttonSelectPcdFilePath
            // 
            this.buttonSelectPcdFilePath.Dock = System.Windows.Forms.DockStyle.Top;
            this.buttonSelectPcdFilePath.Location = new System.Drawing.Point(220, 3);
            this.buttonSelectPcdFilePath.Name = "buttonSelectPcdFilePath";
            this.buttonSelectPcdFilePath.Size = new System.Drawing.Size(55, 23);
            this.buttonSelectPcdFilePath.TabIndex = 1;
            this.buttonSelectPcdFilePath.Text = "...";
            this.buttonSelectPcdFilePath.UseVisualStyleBackColor = true;
            this.buttonSelectPcdFilePath.Click += new System.EventHandler(this.buttonSelectPcdFilePath_Click);
            // 
            // tableLayoutPanel8
            // 
            this.tableLayoutPanel8.ColumnCount = 2;
            this.tableLayoutPanel8.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel8.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel8.Controls.Add(this.radioButtonBinary, 0, 0);
            this.tableLayoutPanel8.Controls.Add(this.radioButtonAscii, 1, 0);
            this.tableLayoutPanel8.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel8.Location = new System.Drawing.Point(3, 44);
            this.tableLayoutPanel8.Name = "tableLayoutPanel8";
            this.tableLayoutPanel8.RowCount = 1;
            this.tableLayoutPanel8.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel8.Size = new System.Drawing.Size(211, 35);
            this.tableLayoutPanel8.TabIndex = 2;
            // 
            // radioButtonBinary
            // 
            this.radioButtonBinary.AutoSize = true;
            this.radioButtonBinary.Dock = System.Windows.Forms.DockStyle.Top;
            this.radioButtonBinary.Location = new System.Drawing.Point(3, 3);
            this.radioButtonBinary.Name = "radioButtonBinary";
            this.radioButtonBinary.Size = new System.Drawing.Size(99, 17);
            this.radioButtonBinary.TabIndex = 0;
            this.radioButtonBinary.Text = "Binary";
            this.radioButtonBinary.UseVisualStyleBackColor = true;
            // 
            // radioButtonAscii
            // 
            this.radioButtonAscii.AutoSize = true;
            this.radioButtonAscii.Checked = true;
            this.radioButtonAscii.Dock = System.Windows.Forms.DockStyle.Top;
            this.radioButtonAscii.Location = new System.Drawing.Point(108, 3);
            this.radioButtonAscii.Name = "radioButtonAscii";
            this.radioButtonAscii.Size = new System.Drawing.Size(100, 17);
            this.radioButtonAscii.TabIndex = 1;
            this.radioButtonAscii.TabStop = true;
            this.radioButtonAscii.Text = "ASCII";
            this.radioButtonAscii.UseVisualStyleBackColor = true;
            // 
            // tableLayoutPanel6
            // 
            this.tableLayoutPanel6.ColumnCount = 1;
            this.tableLayoutPanel6.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel6.Controls.Add(this.Start, 0, 0);
            this.tableLayoutPanel6.Controls.Add(this.Stop, 0, 1);
            this.tableLayoutPanel6.Location = new System.Drawing.Point(3, 97);
            this.tableLayoutPanel6.Name = "tableLayoutPanel6";
            this.tableLayoutPanel6.RowCount = 2;
            this.tableLayoutPanel6.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel6.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel6.Size = new System.Drawing.Size(88, 59);
            this.tableLayoutPanel6.TabIndex = 3;
            // 
            // tableLayoutPanel4
            // 
            this.tableLayoutPanel4.AutoSize = true;
            this.tableLayoutPanel4.ColumnCount = 2;
            this.tableLayoutPanel4.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel4.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel4.Controls.Add(this.comboBoxNormalEstimator1, 1, 2);
            this.tableLayoutPanel4.Controls.Add(this.label1, 0, 2);
            this.tableLayoutPanel4.Controls.Add(this.label2, 0, 3);
            this.tableLayoutPanel4.Controls.Add(this.label3, 0, 4);
            this.tableLayoutPanel4.Controls.Add(this.textBoxMaxDepthChangeFactor1, 1, 3);
            this.tableLayoutPanel4.Controls.Add(this.textBoxNormalSmoothingSize1, 1, 4);
            this.tableLayoutPanel4.Controls.Add(this.label4, 0, 0);
            this.tableLayoutPanel4.Controls.Add(this.comboBoxQualityEstimator, 1, 0);
            this.tableLayoutPanel4.Controls.Add(this.label5, 0, 5);
            this.tableLayoutPanel4.Controls.Add(this.label6, 0, 6);
            this.tableLayoutPanel4.Controls.Add(this.label7, 0, 7);
            this.tableLayoutPanel4.Controls.Add(this.textBoxMaxDepthChangeFactor2, 1, 6);
            this.tableLayoutPanel4.Controls.Add(this.textBoxNormalSmoothingSize2, 1, 7);
            this.tableLayoutPanel4.Controls.Add(this.comboBoxNormalEstimator2, 1, 5);
            this.tableLayoutPanel4.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel4.GrowStyle = System.Windows.Forms.TableLayoutPanelGrowStyle.FixedSize;
            this.tableLayoutPanel4.Location = new System.Drawing.Point(3, 162);
            this.tableLayoutPanel4.Name = "tableLayoutPanel4";
            this.tableLayoutPanel4.RowCount = 8;
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 16.66667F));
            this.tableLayoutPanel4.Size = new System.Drawing.Size(372, 189);
            this.tableLayoutPanel4.TabIndex = 4;
            // 
            // comboBoxNormalEstimator1
            // 
            this.comboBoxNormalEstimator1.FormattingEnabled = true;
            this.comboBoxNormalEstimator1.Location = new System.Drawing.Point(189, 30);
            this.comboBoxNormalEstimator1.Name = "comboBoxNormalEstimator1";
            this.comboBoxNormalEstimator1.Size = new System.Drawing.Size(121, 21);
            this.comboBoxNormalEstimator1.TabIndex = 0;
            this.comboBoxNormalEstimator1.SelectedIndexChanged += new System.EventHandler(this.comboBoxNormalEstimator_SelectedIndexChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Dock = System.Windows.Forms.DockStyle.Top;
            this.label1.Location = new System.Drawing.Point(3, 27);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(180, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "1st normal estimation method";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Dock = System.Windows.Forms.DockStyle.Top;
            this.label2.Location = new System.Drawing.Point(3, 54);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(180, 13);
            this.label2.TabIndex = 2;
            this.label2.Text = "1st normal max depth change factor";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(3, 81);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(127, 13);
            this.label3.TabIndex = 3;
            this.label3.Text = "1st normal smoothing size";
            // 
            // textBoxMaxDepthChangeFactor1
            // 
            this.textBoxMaxDepthChangeFactor1.Location = new System.Drawing.Point(189, 57);
            this.textBoxMaxDepthChangeFactor1.Name = "textBoxMaxDepthChangeFactor1";
            this.textBoxMaxDepthChangeFactor1.Size = new System.Drawing.Size(100, 20);
            this.textBoxMaxDepthChangeFactor1.TabIndex = 4;
            this.textBoxMaxDepthChangeFactor1.TextChanged += new System.EventHandler(this.textBoxMaxDepthChangeFactor_TextChanged);
            // 
            // textBoxNormalSmoothingSize1
            // 
            this.textBoxNormalSmoothingSize1.Location = new System.Drawing.Point(189, 84);
            this.textBoxNormalSmoothingSize1.Name = "textBoxNormalSmoothingSize1";
            this.textBoxNormalSmoothingSize1.Size = new System.Drawing.Size(100, 20);
            this.textBoxNormalSmoothingSize1.TabIndex = 5;
            this.textBoxNormalSmoothingSize1.TextChanged += new System.EventHandler(this.textBoxNormalSmoothingSize_TextChanged);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(3, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(125, 13);
            this.label4.TabIndex = 6;
            this.label4.Text = "quality estimation method";
            // 
            // comboBoxQualityEstimator
            // 
            this.comboBoxQualityEstimator.FormattingEnabled = true;
            this.comboBoxQualityEstimator.Location = new System.Drawing.Point(189, 3);
            this.comboBoxQualityEstimator.Name = "comboBoxQualityEstimator";
            this.comboBoxQualityEstimator.Size = new System.Drawing.Size(121, 21);
            this.comboBoxQualityEstimator.TabIndex = 7;
            this.comboBoxQualityEstimator.SelectedIndexChanged += new System.EventHandler(this.comboBoxQualityEstimator_SelectedIndexChanged);
            // 
            // tableLayoutPanel5
            // 
            this.tableLayoutPanel5.ColumnCount = 1;
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel5.Controls.Add(this.trackBarMaxBad, 0, 0);
            this.tableLayoutPanel5.Controls.Add(this.trackBarMinGood, 0, 1);
            this.tableLayoutPanel5.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel5.Location = new System.Drawing.Point(3, 357);
            this.tableLayoutPanel5.Name = "tableLayoutPanel5";
            this.tableLayoutPanel5.RowCount = 2;
            this.tableLayoutPanel5.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel5.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel5.Size = new System.Drawing.Size(372, 145);
            this.tableLayoutPanel5.TabIndex = 5;
            // 
            // trackBarMaxBad
            // 
            this.trackBarMaxBad.Dock = System.Windows.Forms.DockStyle.Fill;
            this.trackBarMaxBad.Location = new System.Drawing.Point(3, 3);
            this.trackBarMaxBad.Maximum = 1000;
            this.trackBarMaxBad.Name = "trackBarMaxBad";
            this.trackBarMaxBad.Size = new System.Drawing.Size(366, 66);
            this.trackBarMaxBad.TabIndex = 0;
            this.trackBarMaxBad.Scroll += new System.EventHandler(this.trackBarMaxBad_Scroll);
            // 
            // trackBarMinGood
            // 
            this.trackBarMinGood.Dock = System.Windows.Forms.DockStyle.Fill;
            this.trackBarMinGood.Location = new System.Drawing.Point(3, 75);
            this.trackBarMinGood.Maximum = 1000;
            this.trackBarMinGood.Name = "trackBarMinGood";
            this.trackBarMinGood.Size = new System.Drawing.Size(366, 67);
            this.trackBarMinGood.TabIndex = 1;
            this.trackBarMinGood.Scroll += new System.EventHandler(this.trackBarMinGood_Scroll);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Dock = System.Windows.Forms.DockStyle.Top;
            this.label5.Location = new System.Drawing.Point(3, 108);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(180, 13);
            this.label5.TabIndex = 8;
            this.label5.Text = "2nd normal estimation method";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Dock = System.Windows.Forms.DockStyle.Top;
            this.label6.Location = new System.Drawing.Point(3, 135);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(180, 13);
            this.label6.TabIndex = 9;
            this.label6.Text = "2nd normal max depth change factor";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Dock = System.Windows.Forms.DockStyle.Top;
            this.label7.Location = new System.Drawing.Point(3, 162);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(180, 13);
            this.label7.TabIndex = 10;
            this.label7.Text = "2nd normal smoothing size";
            // 
            // textBoxMaxDepthChangeFactor2
            // 
            this.textBoxMaxDepthChangeFactor2.Location = new System.Drawing.Point(189, 138);
            this.textBoxMaxDepthChangeFactor2.Name = "textBoxMaxDepthChangeFactor2";
            this.textBoxMaxDepthChangeFactor2.Size = new System.Drawing.Size(100, 20);
            this.textBoxMaxDepthChangeFactor2.TabIndex = 11;
            this.textBoxMaxDepthChangeFactor2.TextChanged += new System.EventHandler(this.textBoxMaxDepthChangeFactor2_TextChanged);
            // 
            // textBoxNormalSmoothingSize2
            // 
            this.textBoxNormalSmoothingSize2.Location = new System.Drawing.Point(189, 165);
            this.textBoxNormalSmoothingSize2.Name = "textBoxNormalSmoothingSize2";
            this.textBoxNormalSmoothingSize2.Size = new System.Drawing.Size(100, 20);
            this.textBoxNormalSmoothingSize2.TabIndex = 12;
            this.textBoxNormalSmoothingSize2.TextChanged += new System.EventHandler(this.textBoxNormalSmoothingSize2_TextChanged);
            // 
            // comboBoxNormalEstimator2
            // 
            this.comboBoxNormalEstimator2.FormattingEnabled = true;
            this.comboBoxNormalEstimator2.Location = new System.Drawing.Point(189, 111);
            this.comboBoxNormalEstimator2.Name = "comboBoxNormalEstimator2";
            this.comboBoxNormalEstimator2.Size = new System.Drawing.Size(121, 21);
            this.comboBoxNormalEstimator2.TabIndex = 13;
            this.comboBoxNormalEstimator2.SelectedIndexChanged += new System.EventHandler(this.comboBoxNormalEstimator2_SelectedIndexChanged);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.ClientSize = new System.Drawing.Size(1052, 534);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Controls.Add(this.Status2);
            this.Controls.Add(this.MainMenu);
            this.DoubleBuffered = true;
            this.MainMenuStrip = this.MainMenu;
            this.MinimumSize = new System.Drawing.Size(577, 419);
            this.Name = "MainForm";
            this.Text = "Intel(R) RealSense(TM) SDK: Raw Streams.cs";
            this.MainMenu.ResumeLayout(false);
            this.MainMenu.PerformLayout();
            this.Status2.ResumeLayout(false);
            this.Status2.PerformLayout();
            this.panel1.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.tableLayoutPanel2.ResumeLayout(false);
            this.tableLayoutPanel2.PerformLayout();
            this.tableLayoutPanel3.ResumeLayout(false);
            this.tableLayoutPanel7.ResumeLayout(false);
            this.tableLayoutPanel7.PerformLayout();
            this.tableLayoutPanel8.ResumeLayout(false);
            this.tableLayoutPanel8.PerformLayout();
            this.tableLayoutPanel6.ResumeLayout(false);
            this.tableLayoutPanel4.ResumeLayout(false);
            this.tableLayoutPanel4.PerformLayout();
            this.tableLayoutPanel5.ResumeLayout(false);
            this.tableLayoutPanel5.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarMaxBad)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarMinGood)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button Start;
        private System.Windows.Forms.Button Stop;
        private System.Windows.Forms.ToolStripMenuItem DeviceMenu;
        private System.Windows.Forms.MenuStrip MainMenu;
        private System.Windows.Forms.StatusStrip Status2;
        private System.Windows.Forms.ToolStripStatusLabel StatusLabel;
        private System.Windows.Forms.ToolStripMenuItem ModeMenu;
        private System.Windows.Forms.ToolStripMenuItem ModeLive;
        private System.Windows.Forms.ToolStripMenuItem ModePlayback;
        private System.Windows.Forms.ToolStripMenuItem ModeRecord;
        private System.Windows.Forms.ToolStripMenuItem DepthMenu;
        private System.Windows.Forms.ToolStripMenuItem DepthNone;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel2;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel3;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel6;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.Button buttonSaveToPcd;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel4;
        private System.Windows.Forms.ComboBox comboBoxNormalEstimator1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel5;
        private System.Windows.Forms.TrackBar trackBarMaxBad;
        private System.Windows.Forms.TrackBar trackBarMinGood;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox textBoxMaxDepthChangeFactor1;
        private System.Windows.Forms.TextBox textBoxNormalSmoothingSize1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel7;
        private System.Windows.Forms.TextBox textBoxPcdFilePath;
        private System.Windows.Forms.Button buttonSelectPcdFilePath;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel8;
        private System.Windows.Forms.RadioButton radioButtonBinary;
        private System.Windows.Forms.RadioButton radioButtonAscii;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.ComboBox comboBoxQualityEstimator;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox textBoxMaxDepthChangeFactor2;
        private System.Windows.Forms.TextBox textBoxNormalSmoothingSize2;
        private System.Windows.Forms.ComboBox comboBoxNormalEstimator2;
    }
}