//#pragma warning(disable : 4793) // 'xxx' : function compiled as native

// This is the main DLL file.

#include "stdafx.h"

#include "DisjointSet.h"
#include "managed_pcl.h"
#include "utils.h"

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
            static const float scale = 1000.0f;
            PclPoint& p = cloud_ptr_->at(column, row);
            if(z == 0.0f) { //: exact zero means invalid in Intel RealSense SDK; qnan means invalid in pcl
                p.x = std::numeric_limits<float>::quiet_NaN();
                p.y = std::numeric_limits<float>::quiet_NaN();
                p.z = std::numeric_limits<float>::quiet_NaN();
            } else { //: convert coords to meters to avoid confusion
                p.x = x / scale;
                p.y = y / scale;
                p.z = z / scale;
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
            //, zClusters(w, h)
        {
        }

        umanaged_pcl::ScanXyz scan;
        umanaged_pcl::ScanNormal normals1;
        umanaged_pcl::ScanNormal normals2;
        //ZClusters zClusters;
    };

} // namespace unmanaged_impl


#pragma managed(pop)


namespace managed_impl {

    struct Frame
    {
        int width;
        int height;
        int size;

        Frame(int w, int h)
            : width(w)
            , height(h)
            , size(w * h)
        {
        }

        int getIndex(int column, int row) const {
            return row * width + column;
        }
    };

    typedef uint16_t RawZ;
    typedef int ClusterIndex;

    enum {
        c_FakeRootIndex = -1,
        c_BoundaryIndex = -2,
        c_NoDataIndex = -3,
    };

    struct ZCluster {
        ClusterIndex parentClusterIndex;
        int numOfPixels;
        RawZ minZ;
        RawZ maxZ;

        bool isBoundaryCluster() const {
            return parentClusterIndex == c_BoundaryIndex;
        }
        bool isNoDataCluster() const {
            return parentClusterIndex == c_NoDataIndex;
        }
        bool isTopCluster() const {
            return parentClusterIndex == c_FakeRootIndex;
        }
        bool isRegularIntermediateCluster() const {
            return parentClusterIndex >= 0;
        }
        bool isRegularCluster() const {
            return isRegularIntermediateCluster() || isTopCluster();
        }
    };

    class ZClusters : public Frame
    {
    public:
        ZClusters(int w, int h)
            : Frame(w, h)
            , clusters_(w * h)
            , bestClusterIndex_(-1)
        {
        }

        void setData(RawZ const* zValues, RawZ invalidZValue, int halfWindowSize, RawZ deltaThreshold, bool ignoreInvalidZ, int minNumOfPixelsInBestCluster) {
            invalidZValue_ = invalidZValue;
            minNumOfPixelsInBestCluster_ = minNumOfPixelsInBestCluster;
            this->doInitCusters(zValues);
            this->mergeClusters(zValues, halfWindowSize, deltaThreshold, ignoreInvalidZ);
        }

        ZCluster const& getPixelCluster(int pixelIndex) const {
            pixelIndex = getTopClusterIndex(pixelIndex);
            return clusters_.at(pixelIndex);
        }

        ClusterIndex getTopClusterIndex(ClusterIndex aClusterIndex) const {
            assert(aClusterIndex >= 0);
            ClusterIndex result = aClusterIndex;
            while(clusters_[result].isRegularIntermediateCluster()) {
                result = clusters_[result].parentClusterIndex;
            }
            return result;
        }

        ZCluster const* getBestClusterPtr() const {
            return bestClusterIndex_ == -1 ? 0 : &clusters_.at(bestClusterIndex_);
        }

    private:
        void doInitCusters(RawZ const* zValues) {
            for(int i = 0; i < this->size; i++) {
                this->setPixel(clusters_.at(i), zValues[i]);
            }
            this->resetSpecialClusters();
        }

        void setPixel(ZCluster& inplaceResult, RawZ z) {
            inplaceResult.parentClusterIndex = (z != invalidZValue_) ? c_FakeRootIndex : c_NoDataIndex;
            inplaceResult.numOfPixels = 1;
            inplaceResult.minZ = z;
            inplaceResult.maxZ = z;
        }
        void resetSpecialClusters() {
            allClustersStatistics_.parentClusterIndex = c_FakeRootIndex;
            allClustersStatistics_.numOfPixels = 0;
            allClustersStatistics_.minZ = (RawZ)(-1);
            allClustersStatistics_.maxZ = (RawZ)(0);

            boundaryCluster_.parentClusterIndex = c_BoundaryIndex;
            boundaryCluster_.numOfPixels = 0;
            boundaryCluster_.minZ = (RawZ)(-1);
            boundaryCluster_.maxZ = (RawZ)(0);
        }

        void mergeClusters(RawZ const* zValues, int halfWindowSize, RawZ deltaThreshold, bool ignoreInvalidZ) {
            for(int row = 0, centerPixelIndex = 0; row < this->height; ++row) {
                for(int column = 0; column < this->width; ++column, ++centerPixelIndex) {
                    RawZ centerPixelDepth = zValues[centerPixelIndex];
                    if(centerPixelDepth == invalidZValue_)
                        //: skip pixels with unknown depth value:
                        continue;
                    if(this->isBoundaryPixel(zValues, column, row, centerPixelIndex, halfWindowSize, deltaThreshold, ignoreInvalidZ))
                        //: the pixel is on a bounday and is already processed in this->checkAndProcessBoundaryPixel(...)
                        this->setBoundaryPixel(clusters_[centerPixelIndex], centerPixelDepth);
                    else if(this->isPixelNeighborhoodRegular(column, row, centerPixelIndex, halfWindowSize))
                        //: process regular pixel:
                        this->processRegularPixel(zValues, column, row, centerPixelIndex, halfWindowSize);
                }
            }
            this->countTopClusters();
        }

        bool isBoundaryPixel(RawZ const* zValues, int column, int row, int centerPixelIndex, int halfWindowSize, RawZ deltaThreshold, bool ignoreInvalidZ) const {
            RawZ centerPixelDepth = zValues[centerPixelIndex];
            int rowStart = std::max(0, row - halfWindowSize);
            int rowEnd = std::min(row + halfWindowSize + 1, this->height);
            int colStart = std::max(0, column - halfWindowSize);
            int colEnd = std::min(column + halfWindowSize + 1, this->width);
            for(int iRow = rowStart; iRow < rowEnd; ++iRow) {
                for(int iColumn = colStart; iColumn < colEnd; ++iColumn) {
                    int index = this->getIndex(iColumn, iRow);
                    //if(index >= centerPixelIndex)
                    //    continue;
                    RawZ pixelDepth = zValues[index];
                    bool isInvalidZ = (pixelDepth == invalidZValue_);
                    bool isBoundary = !ignoreInvalidZ && isInvalidZ
                        || !isInvalidZ && (deltaThreshold <= getDeltaZ(pixelDepth, centerPixelDepth));
                    if(isBoundary) {
                        return true;
                    }
                }
            }
            //: check for concavity: to avoid clasters 'spit' to background let's treat vertically concave pixels as border pixels:
            ////int halfWindowSize1 = std::min(row - rowStart, rowEnd - row - 1);
            ////for(int iRow = row - halfWindowSize1; iRow < row; ++iRow) {
            ////    int index = this->getIndex(column, iRow);
            ////    RawZ pixel_depth = zValues[index];
            ////    int opposite_index = this->getIndex(column, 2 * row - iRow);
            ////    RawZ opposite_pixel_depth = zValues[opposite_index];
            ////    bool isInvalidZ = (pixel_depth == invalidZValue_ || opposite_pixel_depth == invalidZValue_);
            ////    double k = 1.1; //3./2; //fixme: add to params?
            ////    bool isBoundary = !isInvalidZ && (k * (row - iRow) <= centerPixelDepth - (pixel_depth + opposite_pixel_depth) / 2);
            ////    if(isBoundary) {
            ////        return true;
            ////    }
            ////}

            return false;
        }

        bool isPixelNeighborhoodRegular(int column, int row, int centerPixelIndex, int halfWindowSize) const {
            int rowStart = std::max(0, row - halfWindowSize);
            int rowEnd = std::min(row + halfWindowSize + 1, this->height);
            int colStart = std::max(0, column - halfWindowSize);
            int colEnd = std::min(column + halfWindowSize + 1, this->width);
            for(int iRow = rowStart; iRow < rowEnd; ++iRow) {
                for(int iColumn = colStart; iColumn < colEnd; ++iColumn) {
                    int index = this->getIndex(iColumn, iRow);
                    if(index >= centerPixelIndex)
                        continue;
                    ZCluster const& cluster = clusters_[index];
                    if(!cluster.isRegularCluster())
                        return false;
                }
            }
            return true;
        }

        void processRegularPixel(RawZ const* zValues, int column, int row, int centerPixelIndex, int halfWindowSize) {
            //: otherwise merge the neighbor clusters with smaller indices in the window:
            int rowStart = std::max(0, row - halfWindowSize);
            int rowEnd = std::min(row + halfWindowSize + 1, this->height);
            int colStart = std::max(0, column - halfWindowSize);
            int colEnd = std::min(column + halfWindowSize + 1, this->width);
            for(int iRow = rowStart; iRow < rowEnd; ++iRow) {
                for(int iColumn = colStart; iColumn < colEnd; ++iColumn) {
                    int index = this->getIndex(iColumn, iRow);
                    if(index >= centerPixelIndex)
                        continue;
                    this->mergeNeighborClusters(index, centerPixelIndex);
                }
            }
        }

        int compareClusterWithTopCluster(ClusterIndex aClusterIndex) const {
            ZCluster const& cluster = clusters_[aClusterIndex];
            bool ok = cluster.numOfPixels > minNumOfPixelsInBestCluster_;
            if(ok && bestClusterIndex_ != -1) {
                ZCluster const& best_cluster = clusters_[bestClusterIndex_];
                ok = best_cluster.minZ > cluster.minZ;
                ok = ok || (best_cluster.minZ >= 1 + cluster.minZ && best_cluster.numOfPixels < cluster.numOfPixels);
            }
            return ok ? aClusterIndex : bestClusterIndex_;
        }

        void countTopClusters() {
            numOfTopClusters_ = 0;
            bestClusterIndex_ = -1;
            for(ClusterIndex i = 0; i < this->size; ++i) {
                ZCluster& cluster = clusters_[i];
                if(cluster.isTopCluster()) {
                    //: count a top cluster:
                    ++numOfTopClusters_;
                    //: check if the cluster is the top custer:
                    bestClusterIndex_ = compareClusterWithTopCluster(i);
                    //?cluster.parentClusterIndex = i;
                } else if(cluster.isRegularIntermediateCluster()) {
                    //: streamline ref to top cluster:
                    cluster.parentClusterIndex = getTopClusterIndex(i);
                }
            }
        }

        static RawZ getDeltaZ(RawZ z1, RawZ z2) {
            return (z1 > z2) ? z1 - z2 : z2 - z1;
        }

        static void doMergeClustersData(ZCluster& parentCluster, ZCluster const& childCluster) {
            parentCluster.numOfPixels += childCluster.numOfPixels;
            parentCluster.minZ = std::min(parentCluster.minZ, childCluster.minZ);
            parentCluster.maxZ = std::max(parentCluster.maxZ, childCluster.maxZ);
        }

        void mergeTopClusters(ClusterIndex parentClusterIndex, ClusterIndex childClusterIndex) {
            assert(parentClusterIndex < childClusterIndex);
            ZCluster& parentCluster = clusters_[parentClusterIndex];
            ZCluster& childCluster = clusters_[childClusterIndex];
            childCluster.parentClusterIndex = parentClusterIndex;
            //: update parent cluster data
            doMergeClustersData(parentCluster, childCluster);
            //: update common statistics:
            //fixme: update for parent!
            doMergeClustersData(allClustersStatistics_, childCluster);
        }

        void mergeNeighborClusters(ClusterIndex aClusterIndex1, ClusterIndex aClusterIndex2) {
            assert(aClusterIndex1 >= 0 && aClusterIndex2 >= 0);
            ZCluster& cluster1 = clusters_[aClusterIndex1];
            ZCluster& cluster2 = clusters_[aClusterIndex2];
            if(!cluster1.isRegularCluster() || !cluster2.isRegularCluster())
                //: don't merge with boundary or no-value clasters:
                return;
            ClusterIndex topClusterIndex1 = getTopClusterIndex(aClusterIndex1);
            ClusterIndex topClusterIndex2 = getTopClusterIndex(aClusterIndex2);
            //: to awoid cycles always merge toward smaller index:
            ClusterIndex parentClusterIndex = std::min(topClusterIndex1, topClusterIndex2);
            ClusterIndex childClusterIndex = std::max(topClusterIndex1, topClusterIndex2);
            //: streamline the links to the top clusters:
            if(cluster1.isRegularIntermediateCluster() && parentClusterIndex != cluster1.parentClusterIndex)
                cluster1.parentClusterIndex = parentClusterIndex;
            if(cluster2.isRegularIntermediateCluster() && parentClusterIndex != cluster2.parentClusterIndex)
                cluster2.parentClusterIndex = parentClusterIndex;
            if(topClusterIndex1 == topClusterIndex2)
                //: nothing to do: the clusters are already merged;
                return;
            mergeTopClusters(parentClusterIndex, childClusterIndex);
        }

        void setBoundaryPixel(ZCluster& pixelCluster, RawZ pixelDepth) {
            pixelCluster.parentClusterIndex = c_BoundaryIndex;
            boundaryCluster_.numOfPixels += 1;
            boundaryCluster_.minZ = std::min(boundaryCluster_.minZ, pixelDepth);
            boundaryCluster_.maxZ = std::max(boundaryCluster_.maxZ, pixelDepth);
        }

    private:
        typedef std::vector<ZCluster> Clusters;

        RawZ invalidZValue_;
        int minNumOfPixelsInBestCluster_;

        Clusters clusters_;
        ZCluster allClustersStatistics_;
        ZCluster boundaryCluster_;
        std::size_t numOfTopClusters_;
        ClusterIndex bestClusterIndex_;
    };


} // namespace managed_impl

namespace managed_pcl {


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
                result[i] = pcl_isfinite(curvature) ? 1.0f - std::fabs(curvature) : (float)(PixelQualitySpecialValue::NoData);
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
                    result[i] = (float)(PixelQualitySpecialValue::NoData);
                }
            }
        }
    }
}

inline float getPixelQuality(managed_impl::RawZ maxDepth, managed_impl::RawZ depth, float delta) {
    float q = std::max(0.0f, (maxDepth - depth) / delta);
    return q * q * q;
}

void Scan::old_computePixelQualityFromDepthClusters(cli::array<System::UInt16, 1>^ pixelDepths, System::UInt16 invalidDepthValue, cli::array<System::Single, 1>^ result, DepthClustersParams params) {
    //////static managed_impl::ZClusters zClusters(width_, height_);
    //////bbox_.valid = false;
    //////int length = result->Length;
    //////assert(length == width_ * height_);
    //////assert(pixelDepths->Length == width_ * height_);
    //////cli::pin_ptr<System::UInt16> pixelDepthsPtr = &pixelDepths[0];   //: pin pointer to first element in arr
    //////managed_impl::RawZ const* nativePixelDepthsPtr = pixelDepthsPtr;
    //////zClusters.setData(nativePixelDepthsPtr, invalidDepthValue, params.halfWindowSize, params.deltaDepthThreshold, params.ignoreInvalidDepth, params.minNumOfPixelsInBestCluster);
    //////float delta = (float)(params.maxDepth - params.minDepth);
    //////managed_impl::ZCluster const* bestCluster = zClusters.getBestClusterPtr();
    ////////for(int i = 0; i < length; ++i) {
    //////for(int row = 0, i = 0; row < height_; ++row) {
    //////    for(int column = 0; column < width_; ++column, ++i) {
    //////        managed_impl::ZCluster const& pixelCluster = zClusters.getPixelCluster(i);
    //////        if(pixelCluster.isBoundaryCluster()) {
    //////            result[i] = (float)(PixelQualitySpecialValue::Boundary);
    //////        } else if(pixelCluster.isNoDataCluster()) {
    //////            result[i] = (float)(PixelQualitySpecialValue::NoData);
    //////        } else if(bestCluster == &pixelCluster) {
    //////            result[i] = 1;
    //////            bbox_.addPoint(impl_->scan.getPclCloud().at(column, row).x, impl_->scan.getPclCloud().at(column, row).y, impl_->scan.getPclCloud().at(column, row).z, row, column);
    //////        } else {
    //////            result[i] = getPixelQuality(params.maxDepth, pixelCluster.minZ, delta);
    //////        }
    //////        //} else if(pixelCluster.isTopCluster()) {
    //////        //    result[i] = getPixelQuality(params.maxDepth, pixelCluster.minZ, delta);
    //////        //} else if(pixelCluster.isRegularIntermediateCluster()) {
    //////        //    managed_impl::ZCluster const& parentCluster = zClusters.getTopClusterIndex(pixelCluster.parentClusterIndex);
    //////        //    result[i] = getPixelQuality(params.maxDepth, parentCluster.minZ, delta);
    //////        //} else
    //////        //    assert(false);
    //////    }
    //////}
}


void Scan::computePixelQualityFromDepthClusters(cli::array<System::UInt16, 1>^ pixelDepths, System::UInt16 invalidDepthValue, cli::array<System::Single, 1>^ result, DepthClustersParams params) {
    old_computePixelQualityFromDepthClusters(pixelDepths, invalidDepthValue, result, params);
    clustering::XyFrame frame(width_, height_);
    clustering::ZTraits traits(params.deltaDepthThreshold, invalidDepthValue);
    clustering::ZClusterComparer cluserComparer(params.minNumOfPixelsInBestCluster);
    clustering::DepthViewRules rules(frame, params.halfWindowSize, traits);
    clustering::ZRangeDisjointSet zClusters(rules);
    gotTarget_ = false;
    int length = result->Length;
    assert(length == width_ * height_);
    assert(pixelDepths->Length == width_ * height_);
    cli::pin_ptr<System::UInt16> pixelDepthsPtr = &pixelDepths[0];   //: pin pointer to first element in arr
    clustering::RawZ const* nativePixelDepthsPtr = pixelDepthsPtr;
    zClusters.setData(nativePixelDepthsPtr, width_ * height_);
    float delta = (float)(params.maxDepth - params.minDepth);
    clustering::ZRange const* bestCluster = zClusters.findBestTopCluster(cluserComparer);
    for(int row = 0, i = 0; row < height_; ++row) {
        for(int column = 0; column < width_; ++column, ++i) {
            float old = result[i];
            clustering::ClusterIndex pixelClusterIndex = zClusters.getTopClusterIndex(i);
            clustering::ZRange const& pixelCluster = zClusters.getTopCluster(i);
            if(zClusters.isBoundaryCluster(pixelClusterIndex)) {
                result[i] = (float)(PixelQualitySpecialValue::Boundary);
                result[i] = (result[i] == old) ? (float)(PixelQualitySpecialValue::Boundary) : -1;
            } else if(zClusters.isNoDataCluster(pixelClusterIndex)) {
                result[i] = (float)(PixelQualitySpecialValue::NoData);
            } else if(bestCluster == &pixelCluster) {
                result[i] = 1;
                //result[i] = (result[i] == old) ? (float)(PixelQualitySpecialValue::Boundary) : 1;
                //bbox_.addPoint(impl_->scan.getPclCloud().at(column, row).x, impl_->scan.getPclCloud().at(column, row).y, impl_->scan.getPclCloud().at(column, row).z, row, column);
            } else {
                result[i] = getPixelQuality(params.maxDepth, pixelCluster.min, delta);
                //result[i] = (result[i] == old) ? (float)(PixelQualitySpecialValue::Boundary) : 0.5 + result[i] - old;
            }
        }
    }
}

void Scan::computePixelQualityFromClusters(cli::array<RgbIrDXyzPoint, 1>^ pixelPoints, System::UInt16 invalidDepthValue, cli::array<System::Single, 1>^ result, ProcessParams params) {
    clustering::XyFrame frame(width_, height_);
    clustering::RgbTraits traits(params.colorClustersParams.colorDistanceThreshold);
    clustering::RgbIrDXyzClusterComparer cluserComparer(params.colorClustersParams.minNumOfPixelsInBestCluster);
    clustering::RgbIrDXyzPointRules rules(frame, params.colorClustersParams.halfWindowSize, traits);
    clustering::RgbIrDXyzDisjointSet clusters(rules);
    gotTarget_ = false;
    int length = result->Length;
    assert(length == width_ * height_);
    assert(pixelDepths->Length == width_ * height_);
    cli::pin_ptr<RgbIrDXyzPoint> pixelPointsPtr = &pixelPoints[0];   //: pin pointer to first element in arr
    RgbIrDXyzPoint const* nativePixelPointsPtr = pixelPointsPtr;
    clusters.setData(nativePixelPointsPtr, width_ * height_);
    //float delta = (float)(params.maxDepth - params.minDepth);
    clustering::RgbIrDXyzPointRange const* bestCluster = clusters.findBestTopCluster(cluserComparer);
    for(int row = 0, i = 0; row < height_; ++row) {
        for(int column = 0; column < width_; ++column, ++i) {
            clustering::ClusterIndex pixelClusterIndex = clusters.getTopClusterIndex(i);
            clustering::RgbIrDXyzPointRange const& pixelCluster = clusters.getTopCluster(i);
            if(clusters.isBoundaryCluster(pixelClusterIndex)) {
                result[i] = (float)(PixelQualitySpecialValue::Boundary);
            } else if(clusters.isNoDataCluster(pixelClusterIndex)) {
                result[i] = (float)(PixelQualitySpecialValue::NoData);
            } else if(bestCluster == &pixelCluster) {
                result[i] = -1;
                //bbox_.addPoint(impl_->scan.getPclCloud().at(column, row).x, impl_->scan.getPclCloud().at(column, row).y, impl_->scan.getPclCloud().at(column, row).z, row, column);
            } else {
                result[i] = getPixelQuality(3000, pixelCluster.distanceToWhite(), 3000);
            }
        }
    }

    gotObjectCenterInPixels_ = (bestCluster != 0);
    if(gotObjectCenterInPixels_) {
        clustering::PixelXy p = bestCluster->pixelXy.getTargetPoint();
        objectCenterInPixels_.column = p.column;
        objectCenterInPixels_.row = p.row;
    }

    gotTarget_ = (bestCluster != 0 && bestCluster->xyz.isValid());
    if(gotTarget_) {
        clustering::XyzCoords p = bestCluster->xyz.getTargetPoint();
        targetXyz_.x = p.x;
        targetXyz_.y = p.y;
        targetXyz_.z = p.z;
    }
}

void Scan::saveToPcdFile(System::String^ xyzFileName, System::String^ normalsFileName, bool binary) {
    std::string xyzFileNameInUtf8 = utils::toStdString(xyzFileName);
    if(!xyzFileNameInUtf8.empty())
        impl_->scan.saveToPcdFile(xyzFileNameInUtf8, binary);
    std::string normalsFileNameInUtf8 = utils::toStdString(normalsFileName);
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
