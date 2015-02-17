using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace raw_streams.cs
{
    public partial class MainForm : Form
    {
        private PXCMSession session;
        private string filename = null;

        private PictureBitmap pictureBitmap = new PictureBitmap();

        private int current_device_iuid = 0;
        private Dictionary<ToolStripMenuItem, PXCMCapture.DeviceInfo> devices=new Dictionary<ToolStripMenuItem,PXCMCapture.DeviceInfo>();
        private Dictionary<ToolStripMenuItem, int> devices_iuid=new Dictionary<ToolStripMenuItem,int>();
        private Dictionary<ToolStripMenuItem, PXCMCapture.Device.StreamProfile> profiles=new Dictionary<ToolStripMenuItem,PXCMCapture.Device.StreamProfile>();

        private GuiParams guiParams = GuiParams.Default;
        private volatile bool closing = false;
        private volatile bool stop = false;
        private string pcdFilePath = "c:/p/00-rs2pcd/";
        private int pcdFileIndex = 1;
        private bool saveToPcdRequested = false;

        public MainForm(PXCMSession session)
        {
            InitializeComponent();

            this.session = session;
            PopulateDeviceMenu();
            FormClosing += new FormClosingEventHandler(MainForm_FormClosing);
            pictureBox1.Paint += new PaintEventHandler(pictureBitmap_Paint);
            this.buttonSaveToPcd.Enabled = false;
            this.textBoxPcdFilePath.Text = this.pcdFilePath;

            this.comboBoxQualityEstimator.Items.AddRange(
                new object[] {"Curvature"
                        , "CurvatureStability"
                        , "DepthChange"
                });
            this.comboBoxQualityEstimator.SelectedIndex = (int)guiParams.processParams.qualityEstimationMethod;

            this.textBoxMaxDepthChangeFactor1.Text = guiParams.processParams.normalEstimationParams1.maxDepthChangeFactor.ToString();
            this.textBoxNormalSmoothingSize1.Text = guiParams.processParams.normalEstimationParams1.normalSmoothingSize.ToString();
            this.comboBoxNormalEstimator1.Items.AddRange(
                new object[] {"COVARIANCE_MATRIX"
                        , "AVERAGE_3D_GRADIENT"
                        , "AVERAGE_DEPTH_CHANGE"
                });
            this.comboBoxNormalEstimator1.SelectedIndex = (int)guiParams.processParams.normalEstimationParams1.normalEstimatorMethod;

            this.textBoxMaxDepthChangeFactor2.Text = guiParams.processParams.normalEstimationParams2.maxDepthChangeFactor.ToString();
            this.textBoxNormalSmoothingSize2.Text = guiParams.processParams.normalEstimationParams2.normalSmoothingSize.ToString();
            this.comboBoxNormalEstimator2.Items.AddRange(
                new object[] {"COVARIANCE_MATRIX"
                        , "AVERAGE_3D_GRADIENT"
                        , "AVERAGE_DEPTH_CHANGE"
                });
            this.comboBoxNormalEstimator2.SelectedIndex = (int)guiParams.processParams.normalEstimationParams2.normalEstimatorMethod;

            this.trackBarMaxBad.Value = (int)(1000f * guiParams.maxBadPixelQuality);
            this.trackBarMinGood.Value = (int)(1000f * guiParams.minGoodPixelQuality);

        }

        private delegate void UpdateFromOtherThreadDelegate();

        public bool IsSaveToPcdRequested()
        {
            return saveToPcdRequested;
        }
        public string getXyzPcdFileName()
        {
            return pcdFilePath + string.Format("/{0:D6}.xyz.pcd", pcdFileIndex);
        }
        public string getNormalsPcdFileName()
        {
            return pcdFilePath + string.Format("/{0:D6}.normals.pcd", pcdFileIndex);
        }
        public bool IsSaveToPcdBinary()
        {
            return this.radioButtonBinary.Checked;
        }
        public void OnSaveToPcdCompleted()
        {
            saveToPcdRequested = false;
            ++pcdFileIndex;
            this.Invoke(
                new UpdateFromOtherThreadDelegate(
                    delegate()
                    {
                        this.buttonSaveToPcd.Enabled = true;
                    }
                )
            );
        }

        internal GuiParams GetParams()
        {
            return guiParams;
        }

        private void PopulateDeviceMenu()
        {
            devices.Clear();
            devices_iuid.Clear();

            PXCMSession.ImplDesc desc = new PXCMSession.ImplDesc();
            desc.group = PXCMSession.ImplGroup.IMPL_GROUP_SENSOR;
            desc.subgroup = PXCMSession.ImplSubgroup.IMPL_SUBGROUP_VIDEO_CAPTURE;

            DeviceMenu.DropDownItems.Clear();

            for (int i = 0; ; i++)
            {
                PXCMSession.ImplDesc desc1;
                pxcmStatus rc = session.QueryImpl(desc, i, out desc1);
                if (rc < pxcmStatus.PXCM_STATUS_NO_ERROR)
                    break;
                PXCMCapture capture;
                rc = session.CreateImpl<PXCMCapture>(desc1, out capture);
                if (rc < pxcmStatus.PXCM_STATUS_NO_ERROR)
                    continue;
                using (capture)
                {
                    for (int j = 0; ; j++)
                    {
                        PXCMCapture.DeviceInfo dinfo;
                        rc = capture.QueryDeviceInfo(j, out dinfo);
                        if (rc < pxcmStatus.PXCM_STATUS_NO_ERROR)
                            break;

                        ToolStripMenuItem sm1 = new ToolStripMenuItem(dinfo.name, null, new EventHandler(Device_Item_Click));
                        devices[sm1] = dinfo;
                        devices_iuid[sm1] = desc1.iuid;
                        DeviceMenu.DropDownItems.Add(sm1);
                    }
                }
            }
            if (DeviceMenu.DropDownItems.Count > 0)
            {
                (DeviceMenu.DropDownItems[0] as ToolStripMenuItem).Checked = true;
                PopulateDepthMenu(DeviceMenu.DropDownItems[0] as ToolStripMenuItem);
            }
        }

        private bool PopulateDeviceFromFileMenu()
        {
            devices.Clear();
            devices_iuid.Clear();

            PXCMSession.ImplDesc desc = new PXCMSession.ImplDesc();
            desc.group = PXCMSession.ImplGroup.IMPL_GROUP_SENSOR;
            desc.subgroup = PXCMSession.ImplSubgroup.IMPL_SUBGROUP_VIDEO_CAPTURE;

            PXCMSession.ImplDesc desc1;
            PXCMCapture.DeviceInfo dinfo;
            PXCMSenseManager pp = PXCMSenseManager.CreateInstance();
            if (pp == null)
            {
                UpdateStatus("Init Failed");
                return false;
            }
            try
            {
                if (session.QueryImpl(desc, 0, out desc1) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                    throw null;
                if (pp.captureManager.SetFileName(filename, false) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                    throw null;
                if (pp.QueryCaptureManager().LocateStreams() < pxcmStatus.PXCM_STATUS_NO_ERROR)
                    throw null;
                if (pp.QueryCaptureManager().QueryDevice().QueryDeviceInfo(out dinfo) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                    throw null;
            }
            catch
            {
                pp.Dispose();
                UpdateStatus("Init Failed");
                return false;
            }
            DeviceMenu.DropDownItems.Clear();
            ToolStripMenuItem sm1 = new ToolStripMenuItem(dinfo.name, null, new EventHandler(Device_Item_Click));
            devices[sm1] = dinfo;
            devices_iuid[sm1] = desc1.iuid;
            DeviceMenu.DropDownItems.Add(sm1);

            sm1 = new ToolStripMenuItem("playback from the file : ", null);
            sm1.Enabled = false;
            DeviceMenu.DropDownItems.Add(sm1);
            sm1 = new ToolStripMenuItem(filename, null);
            sm1.Enabled = false;
            DeviceMenu.DropDownItems.Add(sm1);
            if (DeviceMenu.DropDownItems.Count > 0)
                (DeviceMenu.DropDownItems[0] as ToolStripMenuItem).Checked = true;

            // populate depth menu from the file
            profiles.Clear();
            DepthMenu.DropDownItems.Clear();
            PXCMCapture.Device device = pp.QueryCaptureManager().QueryDevice(); 
            
            PXCMCapture.Device.StreamProfileSet profile = new PXCMCapture.Device.StreamProfileSet(); 
            if (((int)dinfo.streams & (int)PXCMCapture.StreamType.STREAM_TYPE_DEPTH) != 0)
            {
                int num = device.QueryStreamProfileSetNum(PXCMCapture.StreamType.STREAM_TYPE_DEPTH);
                for (int p = 0; p < num; p++)
                {
                    if (device.QueryStreamProfileSet(PXCMCapture.StreamType.STREAM_TYPE_DEPTH, p, out profile) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                        break;
                    PXCMCapture.Device.StreamProfile sprofile = profile[PXCMCapture.StreamType.STREAM_TYPE_DEPTH];
                    sm1 = new ToolStripMenuItem(ProfileToString(sprofile), null, new EventHandler(Depth_Item_Click));
                    profiles[sm1] = sprofile;
                    DepthMenu.DropDownItems.Add(sm1);
                }
            }

            DepthNone = new ToolStripMenuItem("None", null, new EventHandler(Depth_Item_Click));
            profiles[DepthNone] = new PXCMCapture.Device.StreamProfile();
            DepthMenu.DropDownItems.Add(DepthNone);
            (DepthMenu.DropDownItems[0] as ToolStripMenuItem).Checked = true;

            CheckSelection();
            pp.Close();
            pp.Dispose();

            StatusLabel.Text = "Ok";
            return true;
        }

        private void PopulateDepthMenu(ToolStripMenuItem device_item)
        {
            PXCMSession.ImplDesc desc = new PXCMSession.ImplDesc();
            desc.group = PXCMSession.ImplGroup.IMPL_GROUP_SENSOR;
            desc.subgroup = PXCMSession.ImplSubgroup.IMPL_SUBGROUP_VIDEO_CAPTURE;
            desc.iuid = devices_iuid[device_item];
            current_device_iuid = desc.iuid;
            desc.cuids[0] = PXCMCapture.CUID;

            profiles.Clear();
            DepthMenu.DropDownItems.Clear();
            PXCMCapture capture;
            PXCMCapture.DeviceInfo dinfo2 = GetCheckedDevice(); 
            if (session.CreateImpl<PXCMCapture>(desc, out capture) >= pxcmStatus.PXCM_STATUS_NO_ERROR)
            {
                using (capture)
                {
                    using (PXCMCapture.Device device = capture.CreateDevice(dinfo2.didx))
                    {
                        if (device != null)
                        {
                            PXCMCapture.Device.StreamProfileSet profile = new PXCMCapture.Device.StreamProfileSet();
                            if (((int)dinfo2.streams & (int)PXCMCapture.StreamType.STREAM_TYPE_DEPTH) != 0)
                            {
                                int num = device.QueryStreamProfileSetNum(PXCMCapture.StreamType.STREAM_TYPE_DEPTH);
                                for (int p = 0; p < num; p++)
                                {
                                    if (device.QueryStreamProfileSet(PXCMCapture.StreamType.STREAM_TYPE_DEPTH, p, out profile) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                                        break;
                                    PXCMCapture.Device.StreamProfile sprofile = profile[PXCMCapture.StreamType.STREAM_TYPE_DEPTH];
                                    ToolStripMenuItem sm1 = new ToolStripMenuItem(ProfileToString(sprofile), null, new EventHandler(Depth_Item_Click));
                                    profiles[sm1] = sprofile;
                                    DepthMenu.DropDownItems.Add(sm1);
                                }
                            }
                        }
                    }
                }
            }
            DepthNone = new ToolStripMenuItem("None", null, new EventHandler(Depth_Item_Click));
            profiles[DepthNone] = new PXCMCapture.Device.StreamProfile();
            DepthMenu.DropDownItems.Add(DepthNone);
            (DepthMenu.DropDownItems[0] as ToolStripMenuItem).Checked = true;

            CheckSelection();
        }

        private static string ProfileToString(PXCMCapture.Device.StreamProfile pinfo)
        {
            string line = "Unknown ";
            if (Enum.IsDefined(typeof(PXCMImage.PixelFormat), pinfo.imageInfo.format))
                line = pinfo.imageInfo.format.ToString().Substring(13)+" "+pinfo.imageInfo.width+"x"+pinfo.imageInfo.height+"x";
            else
                line += pinfo.imageInfo.width + "x" + pinfo.imageInfo.height + "x";
            if (pinfo.frameRate.min != pinfo.frameRate.max) {
                line += (float)pinfo.frameRate.min + "-" +
                      (float)pinfo.frameRate.max;
            } else {
                float fps = (pinfo.frameRate.min!=0)?pinfo.frameRate.min:pinfo.frameRate.max;
                line += fps;
            }
            return line;
        }

        private void Device_Item_Click(object sender, EventArgs e)
        {
            foreach (ToolStripMenuItem e1 in DeviceMenu.DropDownItems)
                e1.Checked = (sender == e1);
            PopulateDepthMenu(sender as ToolStripMenuItem);
        }

        private void Depth_Item_Click(object sender, EventArgs e)
        {
            foreach (ToolStripMenuItem e1 in DepthMenu.DropDownItems)
                e1.Checked = (sender == e1);
            CheckSelection();
        }

        private void Start_Click(object sender, EventArgs e)
        {
            stop = false;
            System.Threading.Thread thread = new System.Threading.Thread(DoRendering);
            thread.Start();
            System.Threading.Thread.Sleep(5);
        }

        private void DoRendering()
        {
            try
            {
                this.Invoke(new UpdateFromOtherThreadDelegate(
                    delegate
                    {
                        MainMenu.Enabled = false;
                        Start.Enabled = false;
                        Stop.Enabled = true;
                        this.buttonSaveToPcd.Enabled = true;
                    }
                ));
                RenderStreams rs = new RenderStreams(this);
                rs.ShowStream();
            }
            catch (Exception e)
            {
                stop = true;
                UpdateStatus("Exception: " + e.ToString());
            }

            this.Invoke(new UpdateFromOtherThreadDelegate(
                delegate
                {
                    Start.Enabled = true;
                    Stop.Enabled = false;
                    buttonSaveToPcd.Enabled = false;
                    MainMenu.Enabled = true;
                    if (closing)
                        Close();
                }
            ));
        }

        public PXCMCapture.DeviceInfo GetCheckedDevice()
        {
            foreach (ToolStripMenuItem e in DeviceMenu.DropDownItems)
            {
                if (devices.ContainsKey(e))
                {
                    if (e.Checked)
                        return devices[e];
                }
            }
            return new PXCMCapture.DeviceInfo();
        }

        private PXCMCapture.Device.StreamProfile GetConfiguration(ToolStripMenuItem m)
        {
            foreach (ToolStripMenuItem e in m.DropDownItems)
                if (e.Checked)
                    return profiles[e];
            return new PXCMCapture.Device.StreamProfile();
        }

        private ToolStripMenuItem GetMenuItem(PXCMCapture.Device.StreamProfile profile)
        {
            foreach (ToolStripMenuItem key1 in profiles.Keys)
            {
                PXCMCapture.Device.StreamProfile profile1 = profiles[key1];
                if (ProfileToString(profile1) == ProfileToString(profile))
                    return key1;
            }
            return null;
        }

        public PXCMCapture.Device.StreamProfile GetDepthConfiguration()
        {
            return GetConfiguration(DepthMenu);
        }

        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            stop = true;
            e.Cancel = Stop.Enabled;
            closing = true;
        }

        private delegate void UpdateStatusDelegate(string status);
        public void UpdateStatus(string status)
        {
            Status2.Invoke(new UpdateStatusDelegate(delegate(string s) { StatusLabel.Text = s; }), new object[] { status });
        }

        private void Stop_Click(object sender, EventArgs e)
        {
            stop = true;
        }

        public void SetPicture(int width, int height, byte[] pixels)
        {
            if (pixels == null)
                return;
            lock (this)
            {
                pictureBitmap.SetPixels(width, height, pixels);
            }
        }

        private static Rectangle getScaledRect(Rectangle rc, int w, int h)
        {
            float xscale = (float)rc.Width / (float)w;
            float yscale = (float)rc.Height / (float)h;
            float xyscale = (xscale < yscale) ? xscale : yscale;
            int width = (int)(w * xyscale);
            int height = (int)(h * xyscale);
            rc.X = (rc.Width - width) / 2;
            rc.Y = (rc.Height - height) / 2;
            rc.Width = width;
            rc.Height = height;
            return rc;
        }

        private void pictureBitmap_Paint(object sender, PaintEventArgs e)
        {
            bool scaled = false;
            lock (this)
            {
                try
                {
                    Bitmap bitmap = pictureBitmap.GetBitmap();
                    if (bitmap == null)
                        return;
                    if (scaled)
                    {
                        /* Keep the aspect ratio */
                        Rectangle rc = (sender as Control).ClientRectangle;
                        rc = getScaledRect(rc, bitmap.Width, bitmap.Height);
                        e.Graphics.DrawImage(bitmap, rc);
                    }
                    else
                    {
                        e.Graphics.DrawImageUnscaled(bitmap, 0, 0);
                    }
                }
                catch (Exception excpt)
                {
                    stop = true;
                    UpdateStatus("Exception: " + excpt.GetType().ToString());
                }
            }
        }

        public void UpdatePicture()
        {
            pictureBox1.Invoke(
                new UpdateFromOtherThreadDelegate(
                    delegate()
                    {
                        pictureBox1.Invalidate();
                    }
                )
            );
        }

        private void CheckSelection()
        {
            PXCMCapture.Device.StreamProfile dprofile = GetDepthConfiguration();
            //?DepthNone.Enabled = (cprofile.imageInfo.format != 0 || irprofile.imageInfo.format != 0);


            PXCMSession.ImplDesc desc = new PXCMSession.ImplDesc();
            desc.group = PXCMSession.ImplGroup.IMPL_GROUP_SENSOR;
            desc.subgroup = PXCMSession.ImplSubgroup.IMPL_SUBGROUP_VIDEO_CAPTURE;
            desc.iuid = current_device_iuid;
            desc.cuids[0] = PXCMCapture.CUID;
            PXCMCapture capture;
            PXCMCapture.DeviceInfo dinfo2 = GetCheckedDevice();
            if (session.CreateImpl<PXCMCapture>(desc, out capture) >= pxcmStatus.PXCM_STATUS_NO_ERROR)
            {
                using (capture)
                {
                    using (PXCMCapture.Device device = capture.CreateDevice(dinfo2.didx))
                    {
                        if (device != null)
                        {
                            PXCMCapture.Device.StreamProfileSet profile = new PXCMCapture.Device.StreamProfileSet();
                            PXCMCapture.Device.StreamProfileSet test = new PXCMCapture.Device.StreamProfileSet();

                            /* Loop over all stream types and profiles and enable only compatible in menu */
                            for (int s = 0; s < PXCMCapture.STREAM_LIMIT; s++)
                            {
                                PXCMCapture.StreamType st = PXCMCapture.StreamTypeFromIndex(s);
                                if (((int)dinfo2.streams & (int)PXCMCapture.StreamType.STREAM_TYPE_COLOR) != 0)
                                {
                                    //?test[PXCMCapture.StreamType.STREAM_TYPE_COLOR] = cprofile;
                                    test[PXCMCapture.StreamType.STREAM_TYPE_DEPTH] = dprofile;
                                    //?test[PXCMCapture.StreamType.STREAM_TYPE_IR] = irprofile;
                                    int num = device.QueryStreamProfileSetNum(st);
                                    for (int p = 0; p < num; p++)
                                    {
                                        if (device.QueryStreamProfileSet(st, p, out profile) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                                            break;
                                        PXCMCapture.Device.StreamProfile sprofile = profile[st];
                                        ToolStripMenuItem sm1 = GetMenuItem(sprofile);
                                        if (sm1 != null)
                                        {
                                            test[st] = sprofile;
                                            sm1.Enabled = device.IsStreamProfileSetValid(test);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        public string GetFileName()
        {
            return filename;
        }

        public bool IsModeLive()
        {
            return ModeLive.Checked;
        }

        public bool IsModeReocrd()
        {
            return ModeRecord.Checked;
        }

        private void ModeLive_Click(object sender, EventArgs e)
        {
            ModeLive.Checked = true;
            ModePlayback.Checked = ModeRecord.Checked = false;
            PopulateDeviceMenu();
        }

        private void ModePlayback_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "RSSDK clip|*.rssdk|Old format clip|*.pcsdk|All files|*.*";
            ofd.CheckFileExists = true;
            ofd.CheckPathExists = true;
            filename=(ofd.ShowDialog() == DialogResult.OK)?ofd.FileName:null;
            if (filename == null)
            {
                ModeLive.Checked = true;
                ModePlayback.Checked = ModeRecord.Checked = false;
                PopulateDeviceMenu();
            } 
            else 
            {
                ModePlayback.Checked = true;
                ModeLive.Checked = ModeRecord.Checked = false;
                if (PopulateDeviceFromFileMenu() == false)
                {
                    ModeLive.Checked = true;
                    ModePlayback.Checked = ModeRecord.Checked = false;
                    MessageBox.Show("Incorrect file format, switching to live mode");
                }
            }
        }

        private void ModeRecord_Click(object sender, EventArgs e)
        {
            ModeRecord.Checked = true;
            ModeLive.Checked = ModePlayback.Checked = false;
            PopulateDeviceMenu();

            SaveFileDialog sfd = new SaveFileDialog();
            sfd.Filter = "RSSDK clip|*.rssdk|All files|*.*";
            sfd.CheckPathExists = true;
            sfd.OverwritePrompt = true;
            sfd.AddExtension    = true;
            filename = (sfd.ShowDialog() == DialogResult.OK) ? sfd.FileName : null;
        }

        public bool GetStopState()
        {
            return stop;
        }

        private void buttonSaveToPcd_Click(object sender, EventArgs e)
        {
            saveToPcdRequested = true;
            this.buttonSaveToPcd.Enabled = false;
        }

        private void trackBarMaxBad_Scroll(object sender, EventArgs e)
        {
            if (trackBarMinGood.Value < trackBarMaxBad.Value)
                trackBarMinGood.Value = trackBarMaxBad.Value;
            guiParams.maxBadPixelQuality = 0.001f * trackBarMaxBad.Value;
        }

        private void trackBarMinGood_Scroll(object sender, EventArgs e)
        {
            if (trackBarMinGood.Value < trackBarMaxBad.Value)
                trackBarMaxBad.Value = trackBarMinGood.Value;
            guiParams.minGoodPixelQuality = 0.001f * trackBarMinGood.Value;
        }

        private void comboBoxNormalEstimator_SelectedIndexChanged(object sender, EventArgs e)
        {
            guiParams.processParams.normalEstimationParams1.normalEstimatorMethod = (managed_pcl.NormalEstimationMethod)comboBoxNormalEstimator1.SelectedIndex;
        }

        private void textBoxMaxDepthChangeFactor_TextChanged(object sender, EventArgs e)
        {
            float.TryParse(textBoxMaxDepthChangeFactor1.Text, out guiParams.processParams.normalEstimationParams1.maxDepthChangeFactor);
        }

        private void textBoxNormalSmoothingSize_TextChanged(object sender, EventArgs e)
        {
            float.TryParse(textBoxNormalSmoothingSize1.Text, out guiParams.processParams.normalEstimationParams1.normalSmoothingSize);
        }

        private void cleanPictureBitmap()
        {
            if (pictureBitmap != null)
            {
                pictureBitmap.Dispose();
                pictureBitmap = null;
            }
        }

        private void buttonSelectPcdFilePath_Click(object sender, EventArgs e)
        {
            using (System.Windows.Forms.FolderBrowserDialog selectFolderDlg = new FolderBrowserDialog())
            {
                selectFolderDlg.SelectedPath = this.pcdFilePath;
                if (selectFolderDlg.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    this.textBoxPcdFilePath.Text = this.pcdFilePath = selectFolderDlg.SelectedPath;
                }
            }
        }

        private void comboBoxQualityEstimator_SelectedIndexChanged(object sender, EventArgs e)
        {
            guiParams.processParams.qualityEstimationMethod = (managed_pcl.QualityEstimationMethod)this.comboBoxQualityEstimator.SelectedIndex;
        }

        private void comboBoxNormalEstimator2_SelectedIndexChanged(object sender, EventArgs e)
        {
            guiParams.processParams.normalEstimationParams2.normalEstimatorMethod = (managed_pcl.NormalEstimationMethod)comboBoxNormalEstimator2.SelectedIndex;
        }

        private void textBoxMaxDepthChangeFactor2_TextChanged(object sender, EventArgs e)
        {
            float.TryParse(textBoxMaxDepthChangeFactor2.Text, out guiParams.processParams.normalEstimationParams2.maxDepthChangeFactor);
        }

        private void textBoxNormalSmoothingSize2_TextChanged(object sender, EventArgs e)
        {
            float.TryParse(textBoxNormalSmoothingSize2.Text, out guiParams.processParams.normalEstimationParams2.normalSmoothingSize);
        }

    }
}
