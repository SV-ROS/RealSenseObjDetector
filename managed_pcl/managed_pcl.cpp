//#pragma warning(disable : 4793) // 'xxx' : function compiled as native

// This is the main DLL file.

#include "stdafx.h"

#include "managed_pcl.h"

#include <cassert>

#pragma managed(push, off)
#include <memory>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

namespace umanaged_pcl {
    typedef pcl::PointXYZ PointXyz;
    typedef pcl::PointCloud<PointXyz> PointXyzCloud;

    template<typename t_Point>
    class ScanWrapper
    {
    private:
        typedef t_Point PclPoint;
        typedef pcl::PointCloud<PclPoint> PclCloud;
        typedef typename PclCloud::Ptr PclCloudPtr;

        PclCloudPtr cloud_;
        PclCloud* cloud_ptr_; //: for faster access

    public:
        typedef boost::shared_ptr<ScanWrapper> Ptr;

        ScanWrapper(int w, int h)
            : cloud_(new PclCloud(w, h))
            , cloud_ptr_(cloud_.get())
        {
        }

        PclCloudPtr const& getPclCloudPtr() const {
            return cloud_;
        }

        PclCloud const& getPclCloud() const {
            return *cloud_ptr_;
        }

        PclCloud& getMutablePclCloud() {
            return *cloud_ptr_;
        }

        PclPoint const& at(int column, int row) const {
            return cloud_ptr_->at(column, row);
        }
        PclPoint & at(int column, int row) {
            return cloud_ptr_->at(column, row);
        }

        void setPoint(int column, int row, float x, float y, float z) {
            PclPoint& p = cloud_ptr_->at(column, row);
            p.x = x;
            p.y = y;
            p.z = z;
        }

        void saveToPcdFile(const std::string& fileName) {
            pcl::io::savePCDFileBinary(fileName, *cloud_);
            //pcl::io::savePCDFileASCII(fileName, *cloud_);
        }
    };

    typedef ScanWrapper<pcl::PointXYZ> ScanXyz;
    typedef ScanWrapper<pcl::PointXYZRGB> ScanXyzRgb;
    typedef ScanWrapper<pcl::PointNormal> ScanXyzNormal;
    typedef ScanWrapper<pcl::Normal> ScanNormal;

    typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> NormalEstimator;
    typedef NormalEstimator::NormalEstimationMethod NormalEstimationMethod;

    void computeNormals(ScanNormal& result, ScanXyz const& scan, NormalEstimationMethod method, float maxDepthChangeFactor, float smoothingSize) {
        NormalEstimator ne;
        ne.setNormalEstimationMethod(method);
        ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
        ne.setNormalSmoothingSize(smoothingSize);
        ne.setInputCloud(scan.getPclCloudPtr());
        ne.compute(result.getMutablePclCloud());
    }
} // namespace umanaged_pcl

namespace unmanaged_impl {
    struct ScanImpl {
    public:
        ScanImpl(int w, int h)
            : scan(w, h)
            , normals(w, h)
        {
        }

        umanaged_pcl::ScanXyz scan;
        umanaged_pcl::ScanNormal normals;
    };
} // namespace unmanaged_impl


#pragma managed(pop)


namespace managed_pcl {

std::string toStdString(System::String ^ s) {
   using namespace System;
   using namespace System::Runtime::InteropServices;
   const char* chars = 
      (const char*)(Marshal::StringToHGlobalAnsi(s)).ToPointer();
   std::string res = chars;
   Marshal::FreeHGlobal(IntPtr((void*)chars));
   return res;
}


void Scan::setCoords(cli::array<PXCMPoint3DF32, 1>^ coords) {
    int length = coords->Length;
    assert(length == width_ * height_);
    for(int row = 0, i = 0; row < height_; ++row) {
        for(int column = 0; column < width_; ++column, ++i) {
            PXCMPoint3DF32 rsp = coords[i];
            impl_->scan.setPoint(column, row, rsp.x, rsp.y, rsp.z);
        }
    }
}

void Scan::computePixelQualityFromCurvature(cli::array<System::Single, 1>^ result, ProcessParams params) {
    int length = result->Length;
    assert(length == width_ * height_);
    umanaged_pcl::computeNormals(impl_->normals, impl_->scan
        , (umanaged_pcl::NormalEstimationMethod)params.normalEstimatorMethod
        , params.maxDepthChangeFactor
        , params.normalSmoothingSize);
    for(int row = 0, i = 0; row < height_; ++row) {
        for(int column = 0; column < width_; ++column, ++i) {
            float curvature = impl_->normals.getPclCloud().at(column, row).curvature;
            result[i] = (curvature == 0.0f) ? 0.0f : 1.0f - curvature; // exact zero means fail to compute normal
        }
    }
}

void Scan::saveToPcdFile(System::String^ fileName) {
    std::string fileNameInUtf8 = toStdString(fileName);
    impl_->scan.saveToPcdFile(fileNameInUtf8);
}

void Scan::init() {
    impl_ = new unmanaged_impl::ScanImpl(width_, height_);
}

void Scan::clean() {
    delete impl_;
}




void Bridge::saveToPcdFile(System::String^ fileName, int w, int h, cli::array<PXCMPoint3DF32, 1>^ coords) {
    std::unique_ptr<umanaged_pcl::ScanXyz> cloud(new umanaged_pcl::ScanXyz(w, h));
    int length = coords->Length;
    assert(length == w * h);
    for(int row = 0, i = 0; row < h; ++row)
        for(int column = 0; column < w; ++column, ++i) {
            PXCMPoint3DF32 rsp = coords[i];
            cloud->setPoint(column, row, rsp.x, rsp.y, rsp.z);
        }
    std::string fileNameInUtf8 = toStdString(fileName);
    cloud->saveToPcdFile(fileNameInUtf8);
}

}
