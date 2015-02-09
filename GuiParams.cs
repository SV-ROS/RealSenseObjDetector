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
                result.processParams.normalEstimatorMethod = managed_pcl.NormalEstimationMethod.COVARIANCE_MATRIX;
                result.processParams.maxDepthChangeFactor = 0.001f;
                result.processParams.normalSmoothingSize = 6.0f;
                result.maxBadPixelQuality = 0;
                result.minGoodPixelQuality = 1;
                return result;
            }
        }
    }
}
