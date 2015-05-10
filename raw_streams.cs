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
        private CameraSettings appliedCameraSettings;

        public RenderStreams(MainForm mf)
        {
            form = mf;
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

                /* Set Input Source */
                PXCMCapture.DeviceInfo dinfo2 = form.GetCheckedDevice();
                if (form.IsModeLive() || form.IsModeRecorded())
                    pp.captureManager.FilterByDeviceInfo(dinfo2);

                /* Set Depth Resolution */
                PXCMCapture.Device.StreamProfile dinfo = form.GetDepthConfiguration();
                if (dinfo.imageInfo.format != 0)
                {
                    Single dfps = dinfo.frameRate.max;
                    pp.EnableStream(PXCMCapture.StreamType.STREAM_TYPE_DEPTH, dinfo.imageInfo.width, dinfo.imageInfo.height, dfps);
                    //: enable also rgb and ir streams:
                    pp.EnableStream(PXCMCapture.StreamType.STREAM_TYPE_COLOR, dinfo.imageInfo.width, dinfo.imageInfo.height, dfps);
                    pp.EnableStream(PXCMCapture.StreamType.STREAM_TYPE_IR, dinfo.imageInfo.width, dinfo.imageInfo.height, dfps);
                }

                /* Initialization */
                FPSTimer timer = new FPSTimer(form);
                form.UpdateStatus("Init Started");
                if (pp.Init() >= pxcmStatus.PXCM_STATUS_NO_ERROR)
                {
                    /* Configure the camera to return a non-mirrored image */
                    pp.captureManager.device.SetMirrorMode(PXCMCapture.Device.MirrorMode.MIRROR_MODE_DISABLED);
                    appliedCameraSettings = CameraSettings.ReadFrom(pp.captureManager.device);
                    form.SetCameraSettings(appliedCameraSettings, CameraSettings.ReadPropInfo(pp.captureManager.device));

                    /* For UV Mapping & Projection only: Save certain properties */
                    using (Projection projection = new Projection(pp.session, pp.captureManager.device, dinfo.imageInfo))
                    {
                        form.UpdateStatus("Streaming");
                        int bitmap_width = dinfo.imageInfo.width;
                        int bitmap_height = dinfo.imageInfo.height;
                        while (!form.GetStopState())
                        {
                            CameraSettings newCameraSettings = form.CameraSettings;
                            if (newCameraSettings.changed)
                            {//: apply new camera settings:
                                newCameraSettings.WriteTo(pp.captureManager.device, this.appliedCameraSettings);
                                this.appliedCameraSettings = newCameraSettings;
                            }

                            //: acquire frame:
                            if (pp.AcquireFrame(true) < pxcmStatus.PXCM_STATUS_NO_ERROR)
                                break;

                            //: process frame:
                            GuiParams guiParams = form.GetParams();
                            PXCMCapture.Sample sample = pp.QuerySample();
                            projection.SearchForObject(sample, guiParams.processParams);
                            pp.ReleaseFrame();

                            byte[] bitmap_data = projection.GetPixelColors();
                            form.SetPicture(bitmap_width, bitmap_height, bitmap_data);
                            form.UpdatePicture();
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
