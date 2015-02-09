using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Linq;
using System.Text;

namespace raw_streams.cs
{
    class RenderStreams
    {
        private MainForm form;

        public RenderStreams(MainForm mf)
        {
            form = mf;
        }

        public static byte[] GetRGB32Pixels(PXCMImage image, out int cwidth, out int cheight)
        {
            PXCMImage.ImageData cdata;
            byte[] cpixels=null;
            cwidth = cheight = 0;
            if (image.AcquireAccess(PXCMImage.Access.ACCESS_READ, PXCMImage.PixelFormat.PIXEL_FORMAT_RGB32, out cdata)>=pxcmStatus.PXCM_STATUS_NO_ERROR) 
            {
                cwidth = (int)cdata.pitches[0] / sizeof(Int32);
                cheight = (int)image.info.height;
                cpixels = cdata.ToByteArray(0, (int)cdata.pitches[0] * cheight);
                image.ReleaseAccess(cdata);
            }
            return cpixels;
        }

        public void ShowStream()
        {
            bool sts = true;

            /* Create an instance of the PXCSenseManager interface */
            using (PXCMSenseManager pp = PXCMSenseManager.CreateInstance())
            {
                if (pp == null)
                {
                    form.UpdateStatus("Failed to create an SDK pipeline object");
                    return;
                }
                if (!form.IsModeLive())
                    pp.captureManager.SetFileName(form.GetFileName(), form.IsModeReocrd());

                /* Set Input Source */
                PXCMCapture.DeviceInfo dinfo2 = form.GetCheckedDevice();
                if (form.IsModeLive() || form.IsModeReocrd())
                    pp.captureManager.FilterByDeviceInfo(dinfo2);

                /* Set Depth Resolution */
                PXCMCapture.Device.StreamProfile dinfo = form.GetDepthConfiguration();
                if (dinfo.imageInfo.format != 0)
                {
                    Single dfps = dinfo.frameRate.max;
                    pp.EnableStream(PXCMCapture.StreamType.STREAM_TYPE_DEPTH, dinfo.imageInfo.width, dinfo.imageInfo.height, dfps);
                }

                /* Initialization */
                FPSTimer timer = new FPSTimer(form);
                form.UpdateStatus("Init Started");
                if (pp.Init() >= pxcmStatus.PXCM_STATUS_NO_ERROR)
                {
                    /* Configure the camera to return a non-mirrored image */
                    pp.captureManager.device.SetMirrorMode(PXCMCapture.Device.MirrorMode.MIRROR_MODE_DISABLED);

                    /* For UV Mapping & Projection only: Save certain properties */
                    using (Projection projection = new Projection(pp.session, pp.captureManager.device, dinfo.imageInfo))
                    {
                        form.UpdateStatus("Streaming");
                        int bitmap_width = dinfo.imageInfo.width;
                        int bitmap_height = dinfo.imageInfo.height;
                        while (!form.GetStopState())
                        {
                            if (pp.AcquireFrame(false) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                                break;

                            GuiParams guiParams = form.GetParams();
                            PXCMCapture.Sample sample = pp.QuerySample();
                            projection.DepthToScan(sample.depth);
                            pp.ReleaseFrame();
                            projection.ComputePixelQualityFromCurvature(guiParams.processParams);

                            byte[] bitmap_data = projection.GetPixelColors(guiParams.maxBadPixelQuality, guiParams.minGoodPixelQuality);
                            form.SetPicture(bitmap_width, bitmap_height, bitmap_data);
                            form.UpdatePicture();

                            if (form.IsSaveToPcdRequested())
                            {
                                projection.SaveToPcd();
                                form.OnSaveToPcdCompleted();
                            }
                            timer.Tick("Streaming.");
                        }
                    }
                }
                else
                {
                    form.UpdateStatus("Init Failed");
                    sts = false;
                }

                pp.Close();
            }
            if (sts)
                form.UpdateStatus("Stopped");
        }
    }
}
