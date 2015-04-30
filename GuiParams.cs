using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace raw_streams.cs
{
    internal struct GuiParams
    {
        public managed_pcl.ProcessParams processParams;
        public float maxBadPixelQuality;
        public float minGoodPixelQuality;

        public static GuiParams Default
        {
            get
            {
                GuiParams result = new GuiParams();
                result.processParams.qualityEstimationMethod = managed_pcl.QualityEstimationMethod.DepthClusters;
                result.processParams.normalEstimationParams1.normalEstimatorMethod = managed_pcl.NormalEstimationMethod.COVARIANCE_MATRIX;
                result.processParams.normalEstimationParams1.maxDepthChangeFactor = 0.001f;
                result.processParams.normalEstimationParams1.normalSmoothingSize = 6.0f;
                result.processParams.normalEstimationParams2.normalEstimatorMethod = managed_pcl.NormalEstimationMethod.COVARIANCE_MATRIX;
                result.processParams.normalEstimationParams2.maxDepthChangeFactor = 0.001f;
                result.processParams.normalEstimationParams2.normalSmoothingSize = 60.0f;
                result.processParams.depthClustersParams.deltaDepthThreshold = 10;
                result.processParams.depthClustersParams.halfWindowSize = 5;
                result.processParams.depthClustersParams.ignoreInvalidDepth = false;
                result.processParams.depthClustersParams.maxDepth = 1100;
                result.processParams.depthClustersParams.minDepth = 100;
                result.processParams.depthClustersParams.minNumOfPixelsInBestCluster =200;
                result.processParams.colorClustersParams.colorDistanceThreshold = 20;
                result.processParams.colorClustersParams.halfWindowSize = 2;
                result.processParams.colorClustersParams.maxDepth = 1100;
                result.processParams.colorClustersParams.minDepth = 100;
                result.processParams.colorClustersParams.minNumOfPixelsInBestCluster = 100;
                result.maxBadPixelQuality = 0.001f;
                result.minGoodPixelQuality = 0.999f;
                return result;
            }
        }
    }

    internal struct CameraSettings
    {
        public ushort DepthConfidenceThreshold;
        public int IVCAMAccuracy;
        public int IVCAMFilterOption;
        public int IVCAMLaserPower;
        public int IVCAMMotionRangeTradeOff;
        public bool changed;

        public static CameraSettings ReadDefaultFrom(PXCMCapture.Device device)
        {
            CameraSettings result = new CameraSettings();
            result.DepthConfidenceThreshold = (ushort)device.QueryDepthConfidenceThresholdInfo().defaultValue;
            result.IVCAMAccuracy = (int)device.QueryIVCAMAccuracyDefaultValue();
            result.IVCAMFilterOption = (int)device.QueryIVCAMFilterOptionInfo().defaultValue;
            result.IVCAMLaserPower = (int)device.QueryIVCAMLaserPowerInfo().defaultValue;
            result.IVCAMMotionRangeTradeOff = (int)device.QueryIVCAMMotionRangeTradeOffInfo().defaultValue;
            result.changed = true;
            return result;
        }
        public static CameraSettings ReadFrom(PXCMCapture.Device device)
        {
            CameraSettings result = new CameraSettings();
            result.DepthConfidenceThreshold = device.QueryDepthConfidenceThreshold();
            result.IVCAMAccuracy = (int)device.QueryIVCAMAccuracy();
            result.IVCAMFilterOption = device.QueryIVCAMFilterOption();
            result.IVCAMLaserPower = device.QueryIVCAMLaserPower();
            result.IVCAMMotionRangeTradeOff = device.QueryIVCAMMotionRangeTradeOff();
            result.changed = false;
            return result;
        }
        public void WriteTo(PXCMCapture.Device device, CameraSettings oldSettings)
        {
            if (oldSettings.DepthConfidenceThreshold != DepthConfidenceThreshold)
                device.SetDepthConfidenceThreshold(DepthConfidenceThreshold);
            if (oldSettings.IVCAMAccuracy != IVCAMAccuracy)
                device.SetIVCAMAccuracy((PXCMCapture.Device.IVCAMAccuracy)IVCAMAccuracy);
            if (oldSettings.IVCAMFilterOption != IVCAMFilterOption)
                device.SetIVCAMFilterOption(IVCAMFilterOption);
            if (oldSettings.IVCAMLaserPower != IVCAMLaserPower)
                device.SetIVCAMLaserPower(IVCAMLaserPower);
            if (oldSettings.IVCAMMotionRangeTradeOff != IVCAMMotionRangeTradeOff)
                device.SetIVCAMMotionRangeTradeOff(IVCAMMotionRangeTradeOff);
            this.changed = false;
        }

        public static PXCMCapture.Device.PropertyInfo[] ReadPropInfo(PXCMCapture.Device device)
        {
            PXCMCapture.Device.PropertyInfo[] propInfo = new PXCMCapture.Device.PropertyInfo[5];
            propInfo[0] = device.QueryDepthConfidenceThresholdInfo();
            propInfo[1] = new PXCMCapture.Device.PropertyInfo() { range = { min = 0, max = 2 } }; //device.QueryIVCAMAccuracy();
            propInfo[2] = device.QueryIVCAMFilterOptionInfo();
            propInfo[3] = device.QueryIVCAMLaserPowerInfo();
            propInfo[4] = device.QueryIVCAMMotionRangeTradeOffInfo();
            return propInfo;
        }

        public void SetupTrackBars(System.Windows.Forms.TrackBar[] tb)
        {
            tb[0].Value = (int)this.DepthConfidenceThreshold;
            tb[1].Value = (int)this.IVCAMAccuracy;
            tb[2].Value = (int)this.IVCAMFilterOption;
            tb[3].Value = (int)this.IVCAMLaserPower;
            tb[4].Value = (int)this.IVCAMMotionRangeTradeOff;
        }
        public void SetupTrackBars(System.Windows.Forms.TrackBar[] tb, PXCMCapture.Device.PropertyInfo[] propInfo)
        {
            for (int i = 0; i < 5; ++i)
            {
                tb[i].Minimum = (int)propInfo[i].range.min;
                tb[i].Maximum = (int)propInfo[i].range.max;
            }
            SetupTrackBars(tb);
        }
    }
}
