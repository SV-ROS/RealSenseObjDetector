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

    public value struct ProcessParams
    {
        QualityEstimationMethod qualityEstimationMethod;
        NormalEstimationParams normalEstimationParams1;
        NormalEstimationParams normalEstimationParams2;
        DepthClustersParams depthClustersParams;
    };

    public value struct XyzCoords
    {
        float x;
        float y;
        float z;
    };

    public value struct XyzBox
    {
		bool valid;
        XyzCoords xyzMin;
        XyzCoords xyzMax;

		void addPoint(float x, float y, float z)
		{
			if(valid)
			{
				xyzMin.x = System::Math::Min(xyzMin.x, x);
				xyzMax.x = System::Math::Max(xyzMax.x, x);
				xyzMin.y = System::Math::Min(xyzMin.y, y);
				xyzMax.y = System::Math::Max(xyzMax.y, y);
				xyzMin.z = System::Math::Min(xyzMin.z, z);
				xyzMax.z = System::Math::Max(xyzMax.z, z);
			}
			else
			{
				xyzMin.x = xyzMax.x = x;
				xyzMin.y = xyzMax.y = y;
				xyzMin.z = xyzMax.z = z;
			}
		}
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

		property XyzBox BBox
		{
			XyzBox get() { return bbox_; }
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
		XyzBox bbox_;
    };

}
