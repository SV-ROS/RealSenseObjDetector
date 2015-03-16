// managed_pcl.h

#pragma once
#include <vcclr.h>

using namespace System;

namespace unmanaged_impl {
    struct ScanImpl;
}

namespace managed_pcl {

    public enum class QualityEstimationMethod
    {
      Curvature,
      CurvatureStability,
      DepthChange,
      DepthClusters
    };

    public enum class NormalEstimationMethod
    {
      COVARIANCE_MATRIX,
      AVERAGE_3D_GRADIENT,
      AVERAGE_DEPTH_CHANGE
    };

    public value struct NormalEstimationParams
    {
        NormalEstimationMethod normalEstimatorMethod;
        float maxDepthChangeFactor;
        float normalSmoothingSize;
    };

    public enum class PixelQualitySpecialValue
    {
      Boundary = -2,
      NoData = -3,
      SmallCluster = -4,
    };

    public value struct DepthClustersParams
    {
        int halfWindowSize;
        int deltaDepthThreshold;
        bool ignoreInvalidDepth;
        int minDepth;
        int maxDepth;
    };

    public value struct ProcessParams
    {
        QualityEstimationMethod qualityEstimationMethod;
        NormalEstimationParams normalEstimationParams1;
        NormalEstimationParams normalEstimationParams2;
        DepthClustersParams depthClustersParams;
    };

    public ref class Scan : public System::IDisposable
    {
    public:
        Scan(int w, int h)
            : impl_(0)
            , width_(w)
            , height_(h)
        {
            init();
        }
        !Scan() {
            clean();
        }
        ~Scan() {
            this->!Scan();
        }

        property int Width
        {
            int get() { return width_; }
        }
        property int Height
        {
            int get() { return height_; }
        }

        void setCoords(cli::array<PXCMPoint3DF32, 1>^ coords);
        void computePixelQualityFromNormals(cli::array<System::Single, 1>^ result, ProcessParams params);
        void computePixelQualityFromDepthClusters(cli::array<System::UInt16, 1>^ pixelDepths, System::UInt16 invalidDepthValue, cli::array<System::Single, 1>^ result, DepthClustersParams params);
        void saveToPcdFile(System::String^ xyzFileName, System::String^ normalsFileName, bool binary);

    private:
        void init();
        void clean();

    private:
        unmanaged_impl::ScanImpl* impl_;
        int width_;
        int height_;
    };

}
