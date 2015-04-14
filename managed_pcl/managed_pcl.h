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

    public value struct XyzRcCoords
    {
        float x;
        float y;
        float z;
        int row;
        int column;
    };

    public value struct XyzBox
    {
        bool valid;

        XyzRcCoords xyzMinXPos;
        XyzRcCoords xyzMaxXPos;
        XyzRcCoords xyzMinYPos;
        XyzRcCoords xyzMaxYPos;
        XyzRcCoords xyzMinZPos;
        XyzRcCoords xyzMaxZPos;

        property XyzRcCoords TopPos
        {
            XyzRcCoords get() { return xyzMinZPos; }
        }
        property XyzCoords MinXyz
        {
            XyzCoords get() {
                XyzCoords res;
                res.x = xyzMinXPos.x;
                res.y = xyzMinYPos.y;
                res.z = xyzMinZPos.z;
                return res;
            }
        }
        property XyzCoords MaxXyz
        {
            XyzCoords get() {
                XyzCoords res;
                res.x = xyzMaxXPos.x;
                res.y = xyzMaxYPos.y;
                res.z = xyzMaxZPos.z;
                return res;
            }
        }

        void addPoint(float x, float y, float z, int row, int column)
        {
            if(valid)
            {
                if(x < xyzMinXPos.x) {
                    xyzMinXPos.row = row;
                    xyzMinXPos.column = column;
                    xyzMinXPos.x = x;
                    xyzMinXPos.y = y;
                    xyzMinXPos.z = z;
                }
                if(x > xyzMaxXPos.x) {
                    xyzMaxXPos.row = row;
                    xyzMaxXPos.column = column;
                    xyzMaxXPos.x = x;
                    xyzMaxXPos.y = y;
                    xyzMaxXPos.z = z;
                }
                if(y < xyzMinYPos.y) {
                    xyzMinYPos.row = row;
                    xyzMinYPos.column = column;
                    xyzMinYPos.x = x;
                    xyzMinYPos.y = y;
                    xyzMinYPos.z = z;
                }
                if(y > xyzMaxYPos.y) {
                    xyzMaxYPos.row = row;
                    xyzMaxYPos.column = column;
                    xyzMaxYPos.x = x;
                    xyzMaxYPos.y = y;
                    xyzMaxYPos.z = z;
                }
                if(z < xyzMinZPos.z) {
                    xyzMinZPos.row = row;
                    xyzMinZPos.column = column;
                    xyzMinZPos.x = x;
                    xyzMinZPos.y = y;
                    xyzMinZPos.z = z;
                }
                if(z > xyzMaxZPos.z) {
                    xyzMaxZPos.row = row;
                    xyzMaxZPos.column = column;
                    xyzMaxZPos.x = x;
                    xyzMaxZPos.y = y;
                    xyzMaxZPos.z = z;
                }
            }
            else
            {
                xyzMinXPos.row = xyzMaxXPos.row = xyzMinYPos.row = xyzMaxYPos.row = xyzMinZPos.row = xyzMaxZPos.row = row;
                xyzMinXPos.column = xyzMaxXPos.column = xyzMinYPos.column = xyzMaxYPos.column = xyzMinZPos.column = xyzMaxZPos.column = column;
                xyzMinXPos.x = xyzMaxXPos.x = xyzMinYPos.x = xyzMaxYPos.x = xyzMinZPos.x = xyzMaxZPos.x = x;
                xyzMinXPos.y = xyzMaxXPos.y = xyzMinYPos.y = xyzMaxYPos.y = xyzMinZPos.y = xyzMaxZPos.y = y;
                xyzMinXPos.z = xyzMaxXPos.z = xyzMinYPos.z = xyzMaxYPos.z = xyzMinZPos.z = xyzMaxZPos.z = z;
                valid = true;
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
        property XyzRcCoords TopPos
        {
            XyzRcCoords get() { return bbox_.TopPos; }
        }
        property int TopPosRow
        {
            int get() { return bbox_.TopPos.row; }
        }
        property int TopPosColumn
        {
            int get() { return bbox_.TopPos.column; }
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
