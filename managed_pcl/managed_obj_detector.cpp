#pragma warning(disable : 4793) // 'xxx' : function compiled as native

// This is the main DLL file.

#include "stdafx.h"

#include "DisjointSet.h"
#include "managed_obj_detector.h"
#include "utils.h"

#include <cassert>

namespace unmanaged_impl {

    struct ObjDetectorImpl {
    public:
        ObjDetectorImpl(clustering::RgbIrDXyzPointRules const& rules)
            : clusters_(rules)
            , bestCluster_(0)
        {
        }

        clustering::RgbIrDXyzDisjointSet clusters_;
        clustering::RgbIrDXyzPointRange const* bestCluster_;
    };

} // namespace unmanaged_impl


//namespace managed_impl {
//} // namespace managed_impl

namespace managed_obj_detector {


inline float getPixelQuality(float maxDepth, float depth, float delta) {
    float q = std::max(0.0f, (maxDepth - depth) / delta);
    return q * q * q;
}


void ObjDetector::clusterize(cli::array<RgbIrDXyzPoint, 1>^ pixelPoints, ProcessParams params) {
    impl_->clusters_.getMutableRules().setHalfWindowSize(params.colorClustersParams.halfWindowSize);
    impl_->clusters_.getMutableRules().getMutableValueTraits().distanceThreshold = (float)params.colorClustersParams.colorDistanceThreshold;

    gotTarget_ = false;
    assert(pixelPoints->Length == numOfPixels_);
    cli::pin_ptr<RgbIrDXyzPoint> pixelPointsPtr = &pixelPoints[0];   //: pin pointer to first element in arr
    RgbIrDXyzPoint const* nativePixelPointsPtr = pixelPointsPtr;
    impl_->clusters_.setData(nativePixelPointsPtr, numOfPixels_);

    clustering::RgbIrDXyzClusterComparer cluserComparer(params.colorClustersParams.minNumOfPixelsInBestCluster);
    impl_->bestCluster_ = impl_->clusters_.findBestTopCluster(cluserComparer);

    setupTarget();
    colorPixels(params);
}

void ObjDetector::init() {
    clustering::XyFrame frame(width_, height_);
    clustering::RgbTraits traits(0);
    clustering::RgbIrDXyzPointRules rules(frame, 0, traits);
    impl_ = new unmanaged_impl::ObjDetectorImpl(rules);
}

void ObjDetector::clean() {
    delete impl_;
    impl_ = nullptr;
}

void ObjDetector::colorPixels(ProcessParams params) {
    switch(params.colorClustersParams.pixelColoringMethod) {
    case PixelColoringMethodEnum::None: return;
    case PixelColoringMethodEnum::ByScore:
        colorPixelsByScore((float)params.colorClustersParams.minDepth, (float)params.colorClustersParams.maxDepth);
        break;
    case PixelColoringMethodEnum::ByClusterMeanColor:
    case PixelColoringMethodEnum::ByClusterMaxColor:
        colorPixelsByClusterColor(params.colorClustersParams.pixelColoringMethod);
        break;
    case PixelColoringMethodEnum::ByClusterDepth:
        colorPixelsByClusterDepth((float)params.colorClustersParams.minDepth, (float)params.colorClustersParams.maxDepth);
        break;
    }

    ////////: plot target point:
    //////if (GotTarget)
    //////{
    //////    this.targetXyzPos[0].x = obj_detector.TargetXyz.x;
    //////    this.targetXyzPos[0].y = obj_detector.TargetXyz.y;
    //////    this.targetXyzPos[0].z = obj_detector.TargetXyz.z;
    //////    if (this.projection.ProjectCameraToColor(this.targetXyzPos, this.targetRcPos) == pxcmStatus.PXCM_STATUS_NO_ERROR
    //////        && this.targetRcPos[0].x != -1 && this.targetRcPos[0].y != -1)
    //////    {
    //////        int halfPatchSize = 5;
    //////        int topColumn = (int)(this.targetRcPos[0].x + 0.5);
    //////        int topRow = (int)(this.targetRcPos[0].y + 0.5);
    //////        int rowStart = Math.Max(0, topRow - halfPatchSize);
    //////        int rowEnd = Math.Min(topRow + halfPatchSize + 1, this.height);
    //////        int colStart = Math.Max(0, topColumn - halfPatchSize);
    //////        int colEnd = Math.Min(topColumn + halfPatchSize + 1, this.width);
    //////        for (int iRow = rowStart; iRow < rowEnd; ++iRow)
    //////        {
    //////            for (int iColumn = colStart; iColumn < colEnd; ++iColumn)
    //////            {
    //////                int index = iRow * width + iColumn;
    //////                //: yellow:
    //////                pixelColors[4 * index + 0] = 0;
    //////                pixelColors[4 * index + 1] = 255;
    //////                pixelColors[4 * index + 2] = 255;
    //////            }
    //////        }
    //////    }
    //////}
}

void ObjDetector::colorPixelsByScore(float maxBad, float minGood) {
    if (maxBad > minGood)
        minGood = maxBad;
    double yellowZoneLength = (double)(minGood - maxBad);
    for(int row = 0, i = 0; row < height_; ++row) {
        for(int column = 0; column < width_; ++column, ++i) {
            float score = 0;
            clustering::ClusterIndex pixelClusterIndex = impl_->clusters_.getTopClusterIndex(i);
            clustering::RgbIrDXyzPointRange const& pixelCluster = impl_->clusters_.getTopCluster(i);
            if(impl_->clusters_.isBoundaryCluster(pixelClusterIndex)) {
                score = (float)(PixelQualitySpecialValue::Boundary);
            } else if(impl_->clusters_.isNoDataCluster(pixelClusterIndex)) {
                score = (float)(PixelQualitySpecialValue::NoData);
            } else if(impl_->bestCluster_ == &pixelCluster) {
                score = -1;
            } else {
                score = getPixelQuality(3000, (float)pixelCluster.distanceToWhite(), 3000);
            }

            if((float)PixelQualitySpecialValue::NoData == score) {
                //: black:
                pixelColors_[4 * i + 0] = 0;
                pixelColors_[4 * i + 1] = 0;
                pixelColors_[4 * i + 2] = 0;
            } else if((float)PixelQualitySpecialValue::Boundary == score) {
                //: gray:
                pixelColors_[4 * i + 0] = 128;
                pixelColors_[4 * i + 1] = 128;
                pixelColors_[4 * i + 2] = 128;
            } else if(score <= 0) {
                //: magenta:
                pixelColors_[4 * i + 0] = 255;
                pixelColors_[4 * i + 1] = 0;
                pixelColors_[4 * i + 2] = 255;
            } else if(score >= 1) {
                //: cyan:
                pixelColors_[4 * i + 0] = 255;
                pixelColors_[4 * i + 1] = 255;
                pixelColors_[4 * i + 2] = 0;
            } else if(score <= maxBad) {
                //: red:
                pixelColors_[4 * i + 0] = 0;
                pixelColors_[4 * i + 1] = 0;
                pixelColors_[4 * i + 2] = 255;
            } else if(score >= minGood) {
                //: green:
                pixelColors_[4 * i + 0] = 0;
                pixelColors_[4 * i + 1] = 255;
                pixelColors_[4 * i + 2] = 0;
            } else {
                //: gradient from red to green:
                double w = (score - maxBad) / yellowZoneLength;
                pixelColors_[4 * i + 0] = 0;
                pixelColors_[4 * i + 1] = (uint8)(255.0 * w);
                pixelColors_[4 * i + 2] = (uint8)(255.0 * (1.0 - w));
            }
        }
    }
}

void ObjDetector::colorPixelsByClusterColor(PixelColoringMethodEnum aPixelColoringMethod) {
    for(int row = 0, i = 0; row < height_; ++row) {
        for(int column = 0; column < width_; ++column, ++i) {
            clustering::ClusterIndex pixelClusterIndex = impl_->clusters_.getTopClusterIndex(i);
            clustering::RgbIrDXyzPointRange const& pixelCluster = impl_->clusters_.getTopCluster(i);
            if(impl_->clusters_.isBoundaryCluster(pixelClusterIndex)) {
                //: gray:
                pixelColors_[4 * i + 0] = 128;
                pixelColors_[4 * i + 1] = 128;
                pixelColors_[4 * i + 2] = 128;
            } else if(impl_->clusters_.isNoDataCluster(pixelClusterIndex)) {
                //: black:
                pixelColors_[4 * i + 0] = 0;
                pixelColors_[4 * i + 1] = 0;
                pixelColors_[4 * i + 2] = 0;
            } else if(impl_->bestCluster_ == &pixelCluster) {
                //: magenta:
                pixelColors_[4 * i + 0] = 255;
                pixelColors_[4 * i + 1] = 0;
                pixelColors_[4 * i + 2] = 255;
            } else {
                //if(aPixelColoringMethod == PixelColoringMethodEnum::ByClusterMeanColor) {
                //    pixelColors_[4 * i + 0] = (pixelCluster.rgb.min.b + pixelCluster.rgb.max.b) / 2;
                //    pixelColors_[4 * i + 1] = (pixelCluster.rgb.min.g + pixelCluster.rgb.max.g) / 2;
                //    pixelColors_[4 * i + 2] = (pixelCluster.rgb.min.r + pixelCluster.rgb.max.r) / 2;
                //} else if(aPixelColoringMethod == PixelColoringMethodEnum::ByClusterMaxColor) {
                //    pixelColors_[4 * i + 0] = (uint8_t)(pixelCluster.rgb_sample.b / impl_->clusters_.getTopClusterNumOfPoints(pixelClusterIndex));//rgb.max.b;
                //    pixelColors_[4 * i + 1] = (uint8_t)(pixelCluster.rgb_sample.g / impl_->clusters_.getTopClusterNumOfPoints(pixelClusterIndex));//rgb.max.g;
                //    pixelColors_[4 * i + 2] = (uint8_t)(pixelCluster.rgb_sample.r / impl_->clusters_.getTopClusterNumOfPoints(pixelClusterIndex));//rgb.max.r;
                //}
                pixelColors_[4 * i + 0] = pixelCluster.rgb_sample.b;
                pixelColors_[4 * i + 1] = pixelCluster.rgb_sample.g;
                pixelColors_[4 * i + 2] = pixelCluster.rgb_sample.r;
            }
        }
    }
}

void ObjDetector::colorPixelsByClusterDepth(float minDepth, float maxDepth) {
    if (minDepth > maxDepth)
        maxDepth = minDepth;
    float delta = (float)(maxDepth - minDepth);
    for(int row = 0, i = 0; row < height_; ++row) {
        for(int column = 0; column < width_; ++column, ++i) {
            clustering::ClusterIndex pixelClusterIndex = impl_->clusters_.getTopClusterIndex(i);
            clustering::RgbIrDXyzPointRange const& pixelCluster = impl_->clusters_.getTopCluster(i);
            if(impl_->clusters_.isBoundaryCluster(pixelClusterIndex)) {
                //: gray:
                pixelColors_[4 * i + 0] = 128;
                pixelColors_[4 * i + 1] = 128;
                pixelColors_[4 * i + 2] = 128;
            } else if(impl_->clusters_.isNoDataCluster(pixelClusterIndex)) {
                //: black:
                pixelColors_[4 * i + 0] = 0;
                pixelColors_[4 * i + 1] = 0;
                pixelColors_[4 * i + 2] = 0;
            } else if(impl_->bestCluster_ == &pixelCluster) {
                //: magenta:
                pixelColors_[4 * i + 0] = 255;
                pixelColors_[4 * i + 1] = 0;
                pixelColors_[4 * i + 2] = 255;
            } else {
                //: gradient from red to green:
                double w = getPixelQuality(maxDepth, pixelCluster.depth.min, delta);
                pixelColors_[4 * i + 0] = 0;
                pixelColors_[4 * i + 1] = (uint8)(255.0 * w);
                pixelColors_[4 * i + 2] = (uint8)(255.0 * (1.0 - w));
            }
        }
    }
}

void ObjDetector::setupTarget() {
    gotObjectCenterInPixels_ = (impl_->bestCluster_ != 0);
    if(gotObjectCenterInPixels_) {
        clustering::PixelXy p = impl_->bestCluster_->pixelXy.getTargetPoint();
        objectCenterInPixels_.column = p.column;
        objectCenterInPixels_.row = p.row;
    }

    gotTarget_ = (impl_->bestCluster_ != 0 && impl_->bestCluster_->xyz.isValid());
    if(gotTarget_) {
        clustering::XyzCoords p = impl_->bestCluster_->xyz.getTargetPoint();
        targetXyz_.x = p.x;
        targetXyz_.y = p.y;
        targetXyz_.z = p.z;
    }
}


//////uint8[] GetPixelColors()
//////{
//////    return pixelColors;
//////}
//////
//////private void setAllNan()
//////{
//////    int numOfPixels = pixelQuality.Length;
//////    for (int i = 0; i < numOfPixels; ++i)
//////    {
//////        pixelQuality[i] = (float)managed_obj_detector.PixelQualitySpecialValue.NoData;
//////    }
//////}



}
