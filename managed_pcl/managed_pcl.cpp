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
        //PclPoint & at(int column, int row) {
        //    return cloud_ptr_->at(column, row);
        //}

        void setPoint(int column, int row, float x, float y, float z) {
            PclPoint& p = cloud_ptr_->at(column, row);
            if(z == 0.0f) { //: exact zero means invalid in Intel RealSense SDK; qnan means invalid in pcl
                p.x = std::numeric_limits<float>::quiet_NaN();
                p.y = std::numeric_limits<float>::quiet_NaN();
                p.z = std::numeric_limits<float>::quiet_NaN();
            } else { //: convert coords to meters to avoid confusion
                p.x = x / 100.0f;
                p.y = y / 100.0f;
                p.z = z / 100.0f;
            }
        }

        void saveToPcdFile(const std::string& fileName, bool binary) {
            if(binary)
                pcl::io::savePCDFileBinary(fileName, *cloud_);
            else
                pcl::io::savePCDFileASCII(fileName, *cloud_);
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
            , normals1(w, h)
            , normals2(w, h)
        {
        }

        umanaged_pcl::ScanXyz scan;
        umanaged_pcl::ScanNormal normals1;
        umanaged_pcl::ScanNormal normals2;
    };
} // namespace unmanaged_impl


#pragma managed(pop)


namespace managed_pcl {

std::string toStdString(System::String ^ s) {
    if(System::String::IsNullOrEmpty(s))
        return std::string();
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

void Scan::computePixelQualityFromNormals(cli::array<System::Single, 1>^ result, ProcessParams params) {
    int length = result->Length;
    assert(length == width_ * height_);
    if(params.qualityEstimationMethod == QualityEstimationMethod::Curvature) {
        umanaged_pcl::computeNormals(impl_->normals1, impl_->scan
            , umanaged_pcl::NormalEstimator::COVARIANCE_MATRIX //: COVARIANCE_MATRIX is the only method producing curvatures
            , params.normalEstimationParams1.maxDepthChangeFactor
            , params.normalEstimationParams1.normalSmoothingSize);
        for(int row = 0, i = 0; row < height_; ++row) {
            for(int column = 0; column < width_; ++column, ++i) {
                float curvature = impl_->normals1.getPclCloud().at(column, row).curvature;
                //result[i] = (curvature == 0.0f) ? curvature : 1.0f - std::fabs(curvature); // exact zero means fail to compute normal?
                result[i] = pcl_isfinite(curvature) ? 1.0f - std::fabs(curvature) : curvature;
            }
        }
    } else if(params.qualityEstimationMethod == QualityEstimationMethod::CurvatureStability) {
        umanaged_pcl::computeNormals(impl_->normals1, impl_->scan
            , (umanaged_pcl::NormalEstimationMethod)params.normalEstimationParams1.normalEstimatorMethod
            , params.normalEstimationParams1.maxDepthChangeFactor
            , params.normalEstimationParams1.normalSmoothingSize);
        umanaged_pcl::computeNormals(impl_->normals2, impl_->scan
            , (umanaged_pcl::NormalEstimationMethod)params.normalEstimationParams2.normalEstimatorMethod
            , params.normalEstimationParams2.maxDepthChangeFactor
            , params.normalEstimationParams2.normalSmoothingSize * 2);
        for(int row = 0, i = 0; row < height_; ++row) {
            for(int column = 0; column < width_; ++column, ++i) {
                if(pcl::isFinite(impl_->normals1.getPclCloud().at(column, row)) && pcl::isFinite(impl_->normals2.getPclCloud().at(column, row))) {
                    float dot = impl_->normals1.getPclCloud().at(column, row).getNormalVector3fMap().dot(impl_->normals2.getPclCloud().at(column, row).getNormalVector3fMap());
                    result[i] = std::fabs(dot);
                } else {
                    result[i] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
    }
}

void Scan::saveToPcdFile(System::String^ xyzFileName, System::String^ normalsFileName, bool binary) {
    std::string xyzFileNameInUtf8 = toStdString(xyzFileName);
    if(!xyzFileNameInUtf8.empty())
        impl_->scan.saveToPcdFile(xyzFileNameInUtf8, binary);
    std::string normalsFileNameInUtf8 = toStdString(normalsFileName);
    if(!normalsFileNameInUtf8.empty())
        impl_->normals1.saveToPcdFile(normalsFileNameInUtf8, binary);
}

void Scan::init() {
    impl_ = new unmanaged_impl::ScanImpl(width_, height_);
}

void Scan::clean() {
    delete impl_;
    impl_ = nullptr;
}


}
