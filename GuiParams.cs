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
                result.processParams.qualityEstimationMethod = managed_pcl.QualityEstimationMethod.Curvature;
                result.processParams.normalEstimationParams1.normalEstimatorMethod = managed_pcl.NormalEstimationMethod.COVARIANCE_MATRIX;
                result.processParams.normalEstimationParams1.maxDepthChangeFactor = 0.001f;
                result.processParams.normalEstimationParams1.normalSmoothingSize = 6.0f;
                result.processParams.normalEstimationParams2.normalEstimatorMethod = managed_pcl.NormalEstimationMethod.COVARIANCE_MATRIX;
                result.processParams.normalEstimationParams2.maxDepthChangeFactor = 0.001f;
                result.processParams.normalEstimationParams2.normalSmoothingSize = 60.0f;
                result.maxBadPixelQuality = 0.001f;
                result.minGoodPixelQuality = 0.999f;
                return result;
            }
        }
    }
}
