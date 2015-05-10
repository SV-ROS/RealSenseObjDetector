// managed_obj_detector.h

#pragma once
#include <vcclr.h>

using namespace System;

namespace unmanaged_impl {
    struct ObjDetectorImpl;
}

namespace managed_obj_detector {

    public enum class QualityEstimationMethod
    {
      ColorClusters
    };

    public enum class PixelColoringMethodEnum
    {
        None,
        ByScore,
        ByClusterMeanColor,
        ByClusterMaxColor,
        ByClusterDepth
    };

    public enum class PixelQualitySpecialValue
    {
      Boundary = -2,
      NoData = -3,
      SmallCluster = -4,
    };

    public value struct ColorClustersParams
    {
        int halfWindowSize;
        int colorDistanceThreshold;
        int minDepth;
        int maxDepth;
        int minNumOfPixelsInBestCluster;
        PixelColoringMethodEnum pixelColoringMethod;
    };

    public value struct ProcessParams
    {
        QualityEstimationMethod qualityEstimationMethod;
        ColorClustersParams colorClustersParams;
        float maxBadPixelQuality;
        float minGoodPixelQuality;
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

    public ref class ObjDetector : public System::IDisposable
    {
    public:
        ObjDetector(int w, int h, RawDepth invalidDepthValue)
            : impl_(0)
            , width_(w)
            , height_(h)
            , numOfPixels_(w * h)
            , invalidDepthValue_(invalidDepthValue)
            , gotTarget_(false)
            , gotObjectCenterInPixels_(false)
        {
            targetXyz_.x = targetXyz_.y = targetXyz_.z = 0;
            pixelQuality_ = gcnew cli::array<System::Single, 1>(numOfPixels_);
            pixelColors_ = gcnew cli::array<uint8, 1>(4 * numOfPixels_); //: pixels in rgb32
            init();
        }
        !ObjDetector() {
            clean();
        }
        ~ObjDetector() {
            this->!ObjDetector();
        }

        property int Width {
            int get() { return width_; }
        }
        property int Height {
            int get() { return height_; }
        }
        property int NumOfPixels {
            int get() { return numOfPixels_; }
        }

        property cli::array<uint8, 1>^ PixelColors {
            cli::array<uint8, 1>^ get() { return pixelColors_; }
        }

        property bool GotTarget
        {
            bool get() { return gotTarget_; }
        }
        property XyzCoords TargetXyz
        {
            XyzCoords get() { return targetXyz_; }
        }

        property bool GotObjectCenterInPixels
        {
            bool get() { return gotObjectCenterInPixels_; }
        }
        property RcCoords ObjectCenterInPixels
        {
            RcCoords get() { return objectCenterInPixels_; }
        }

        void clusterize(cli::array<RgbIrDXyzPoint, 1>^ pixelPoints, ProcessParams params);

    private:
        void init();
        void clean();

        void colorPixels(ProcessParams params);
        void colorPixelsByScore(float maxBad, float minGood);
        void colorPixelsByClusterColor(PixelColoringMethodEnum aPixelColoringMethod);
        void colorPixelsByClusterDepth(float minDepth, float maxDepth);
        void setupTarget();

    private:
        unmanaged_impl::ObjDetectorImpl* impl_;
        int width_;
        int height_;
        int numOfPixels_;
        RawDepth invalidDepthValue_;

        cli::array<System::Single, 1>^ pixelQuality_;
        cli::array<uint8, 1>^ pixelColors_;

        bool gotTarget_;
        XyzCoords targetXyz_;

        bool gotObjectCenterInPixels_;
        RcCoords objectCenterInPixels_;
    };

}
