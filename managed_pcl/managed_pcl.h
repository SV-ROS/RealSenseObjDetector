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
      DepthClusters,
      ColorClusters,
      Curvature,
      CurvatureStability,
      DepthChange
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
        int minNumOfPixelsInBestCluster;
    };

    public value struct ColorClustersParams
    {
        int halfWindowSize;
        int colorDistanceThreshold;
        int minDepth;
        int maxDepth;
        int minNumOfPixelsInBestCluster;
    };

    public value struct ProcessParams
    {
        QualityEstimationMethod qualityEstimationMethod;
        NormalEstimationParams normalEstimationParams1;
        NormalEstimationParams normalEstimationParams2;
        DepthClustersParams depthClustersParams;
        ColorClustersParams colorClustersParams;
    };

    public value struct XyzCoords
    {
        float x;
        float y;
        float z;
    };

    public value struct RcCoords
    {
        int row;
        int column;
    };

    typedef uint8_t uint8;
    typedef uint16_t RawDepth;

    public value struct RgbIrDXyzPoint
    {
        XyzCoords coords;
        uint8 r;
        uint8 g;
        uint8 b;
        uint8 ir;
        RawDepth depth;
    };

    public ref class Scan : public System::IDisposable
    {
    public:
        Scan(int w, int h)
            : impl_(0)
            , width_(w)
            , height_(h)
            , gotTarget_(false)
        {
            targetXyz_.x = targetXyz_.y = targetXyz_.z = 0;
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

        property bool GotTarget
        {
            bool get() { return gotTarget_; }
        }
        property XyzCoords TargetXyz
        {
            XyzCoords get() { return targetXyz_; }
        }

        void setCoords(cli::array<PXCMPoint3DF32, 1>^ coords);
        void computePixelQualityFromNormals(cli::array<System::Single, 1>^ result, ProcessParams params);
        void old_computePixelQualityFromDepthClusters(cli::array<System::UInt16, 1>^ pixelDepths, System::UInt16 invalidDepthValue, cli::array<System::Single, 1>^ result, DepthClustersParams params);
        void computePixelQualityFromDepthClusters(cli::array<System::UInt16, 1>^ pixelDepths, System::UInt16 invalidDepthValue, cli::array<System::Single, 1>^ result, DepthClustersParams params);
        void computePixelQualityFromClusters(cli::array<RgbIrDXyzPoint, 1>^ pixelPoints, System::UInt16 invalidDepthValue, cli::array<System::Single, 1>^ result, ProcessParams params);
        void saveToPcdFile(System::String^ xyzFileName, System::String^ normalsFileName, bool binary);

    private:
        void init();
        void clean();

    private:
        unmanaged_impl::ScanImpl* impl_;
        int width_;
        int height_;

        bool gotTarget_;
        XyzCoords targetXyz_;
    };

}
