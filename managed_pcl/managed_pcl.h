// managed_pcl.h

#pragma once
#include <vcclr.h>

using namespace System;

namespace unmanaged_impl {
    struct ScanImpl;
}

namespace managed_pcl {

    public enum class NormalEstimationMethod
    {
      COVARIANCE_MATRIX,
      AVERAGE_3D_GRADIENT,
      AVERAGE_DEPTH_CHANGE
    };

    public value struct ProcessParams
    {
        NormalEstimationMethod normalEstimatorMethod;
        float maxDepthChangeFactor;
        float normalSmoothingSize;
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
        void computePixelQualityFromCurvature(cli::array<System::Single, 1>^ result, ProcessParams params);
        void saveToPcdFile(System::String^ fileName);

    private:
        void init();
        void clean();

    private:
        unmanaged_impl::ScanImpl* impl_;
        int width_;
        int height_;
    };

    public ref class Bridge abstract sealed
    {
    public:
        static void saveToPcdFile(System::String^ fileName, int w, int h, cli::array<PXCMPoint3DF32, 1>^ coords);
    };
}
