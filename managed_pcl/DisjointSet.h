// DisjointSet.h
#ifndef DISJOINTSET_H
#define DISJOINTSET_H

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

namespace clustering {

    typedef int ClusterIndex;

    struct ClusterInfo {
        enum {
            c_NoDataIndex = -1,
            c_BoundaryIndex = -2,
        };

        ClusterIndex parentClusterIndex;
        int numOfPoints;

        bool isBoundaryCluster() const {
            return parentClusterIndex == c_BoundaryIndex;
        }
        bool isNoDataCluster() const {
            return parentClusterIndex == c_NoDataIndex;
        }
        bool isRegularCluster() const {
            return parentClusterIndex >= 0;
        }

        void setBoundary() {
            parentClusterIndex = c_BoundaryIndex;
        }
        void setNoData() {
            parentClusterIndex = c_NoDataIndex;
        }
    };


    template<typename t_Cluster, typename t_Rules>
    class DisjointSet
    {
    public:
        typedef t_Cluster Cluster;

        DisjointSet(t_Rules const& rules)
            : rules_(rules)
        {
        }

        template<typename t_PointsPtr>
        void setData(t_PointsPtr points, int numOfPoints) {
            initCusters(points, numOfPoints);
            mergeClusters(points, numOfPoints);
            countTopClusters();
        }

        bool isBoundaryCluster(ClusterIndex clusterIndex) const {
            return clusterInfos_[clusterIndex].isBoundaryCluster();
        }
        bool isNoDataCluster(ClusterIndex clusterIndex) const {
            return clusterInfos_[clusterIndex].isNoDataCluster();
        }
        bool isRegularCluster(ClusterIndex clusterIndex) const {
            return clusterInfos_[clusterIndex].isRegularCluster();
        }

        ClusterIndex getTopClusterIndex(ClusterIndex clusterIndex) const {
            //return findTopClusterIndex(clusterIndex);
            return clusterInfos_[clusterIndex].isRegularCluster() ? clusterInfos_[clusterIndex].parentClusterIndex : clusterIndex;
        }

        Cluster const& getTopCluster(ClusterIndex clusterIndex) const {
            ClusterIndex topClusterIndex = getTopClusterIndex(clusterIndex);
            return (topClusterIndex >= 0) ? clusters_[topClusterIndex] : clusters_[clusterIndex];
        }
        int getTopClusterNumOfPoints(ClusterIndex clusterIndex) const {
            clusterIndex = getTopClusterIndex(clusterIndex);
            return clusterInfos_[clusterIndex].numOfPoints;
        }

        std::vector<ClusterIndex> getTopClusterIndices() const { return topClusters_; }

        template<typename t_Comparer>
        ClusterIndex findBestTopClusterIndex(t_Comparer const& comparer) const {
            ClusterIndex result = -1;
            for(int i = 0; i < topClusters_.size(); ++i) {
                ClusterIndex topClusterIndex = topClusters_[i];
                if(comparer.isOk(topClusterIndex, *this) && (result == -1 || comparer.isBetter(topClusterIndex, result, *this)))
                    result = topClusterIndex;
            }
            return result;
        }
        template<typename t_Comparer>
        Cluster const* findBestTopCluster(t_Comparer const& comparer) const {
            ClusterIndex clusterIndex = findBestTopClusterIndex(comparer);
            return clusterIndex != -1 ? &clusters_[clusterIndex] : 0;
        }

    private:
        ClusterIndex findTopClusterIndex(ClusterIndex clusterIndex) const {
            assert(clusterIndex >= 0 && clusterIndex < clusterInfos_.size());
            ClusterIndex result = clusterIndex;
            while(clusterInfos_[result].isRegularCluster() && clusterInfos_[result].parentClusterIndex != result) {
                result = clusterInfos_[result].parentClusterIndex;
            }
            return result;
        }

        template<typename t_PointsPtr>
        void initCusters(t_PointsPtr points, int numOfPoints) {
            clusters_.resize(numOfPoints);
            clusterInfos_.resize(numOfPoints);
            for(int i = 0; i < numOfPoints; ++i) {
                rules_.initCluster(clusters_[i], i, points[i]);
                clusterInfos_[i].parentClusterIndex = i;
                clusterInfos_[i].numOfPoints = 1;
            }
        }
        template<typename t_PointsPtr>
        void mergeClusters(t_PointsPtr points, int numOfPoints) {
            for(ClusterIndex clusterIndex = 0; clusterIndex < numOfPoints; ++clusterIndex) {
                if(rules_.isInvalidData(clusters_[clusterIndex], clusterIndex, points[clusterIndex]))
                    clusterInfos_[clusterIndex].setNoData();
                else if(isAtClusterBoundary(clusterIndex, points))
                    clusterInfos_[clusterIndex].setBoundary();
                else
                    mergeRegularNeighborClusters(clusterIndex, points);
            }
        }

        template<typename t_PointsPtr>
        bool isAtClusterBoundary(ClusterIndex clusterIndex, t_PointsPtr points) const {
            ClusterIndex topClusterIndex = findTopClusterIndex(clusterIndex);
            typename t_Rules::NeighborIndicesEnumerator neighborIndicesEnumerator = rules_.getNeighborIndicesEnumerator(clusterIndex);
            while(!neighborIndicesEnumerator.empty()) {
                ClusterIndex neighborClusterIndex = *neighborIndicesEnumerator;
                ClusterIndex neighborTopClusterIndex = findTopClusterIndex(neighborClusterIndex);
                if(!rules_.isOfSameCluster(clusterIndex, neighborClusterIndex
                    , clusters_[topClusterIndex], clusters_[neighborTopClusterIndex]
                    , points[clusterIndex], points[neighborClusterIndex]))
                {
                    return true;
                }
                ++neighborIndicesEnumerator;
            }
            return false;
        }

        template<typename t_PointsPtr>
        void mergeRegularNeighborClusters(ClusterIndex clusterIndex, t_PointsPtr points) {
            typename t_Rules::NeighborIndicesEnumerator neighborIndicesEnumerator = rules_.getNeighborIndicesEnumerator(clusterIndex);
            while(!neighborIndicesEnumerator.empty()) {
                ClusterIndex neighborClusterIndex = *neighborIndicesEnumerator;
                ++neighborIndicesEnumerator;
                //: merge only with neighbor clusters with smaller indices:
                if(neighborClusterIndex >= clusterIndex)
                    continue;
                //: merge only regular clusters:
                if(!clusterInfos_[neighborClusterIndex].isRegularCluster())
                    continue;
                ClusterIndex topClusterIndex = findTopClusterIndex(clusterIndex);
                ClusterIndex neighborTopClusterIndex = findTopClusterIndex(neighborClusterIndex);
                if(topClusterIndex == neighborTopClusterIndex)
                    //: nothing to do: the clusters are already merged;
                    continue;
                //: to avoid cycles always merge toward smaller index:
                ClusterIndex parentClusterIndex = std::min(topClusterIndex, neighborTopClusterIndex);
                ClusterIndex childClusterIndex  = std::max(topClusterIndex, neighborTopClusterIndex);
                if(rules_.mergeNeighborClusters(parentClusterIndex, childClusterIndex
                    , clusters_[parentClusterIndex], clusters_[childClusterIndex]
                    , points[clusterIndex], points[neighborClusterIndex]))
                {
                    //: count points in top cluster:
                    clusterInfos_[parentClusterIndex].numOfPoints += clusterInfos_[childClusterIndex].numOfPoints;
                    //: streamline links to top clusters:
                    clusterInfos_[childClusterIndex].parentClusterIndex
                        = clusterInfos_[clusterIndex].parentClusterIndex
                        = clusterInfos_[neighborClusterIndex].parentClusterIndex
                        = parentClusterIndex;
                }
            }
        }

        void countTopClusters() {
            topClusters_.clear();
            for(ClusterIndex i = 0; i < clusterInfos_.size(); ++i) {
                ClusterIndex topClusterIndex = findTopClusterIndex(i);
                if(topClusterIndex == i) {
                    if(isRegularCluster(topClusterIndex))
                        topClusters_.push_back(i);
                } else
                    //: streamline the link to the top cluster:
                    clusterInfos_[i].parentClusterIndex = topClusterIndex;
            }
        }

    private:
        t_Rules rules_;
        std::vector<Cluster> clusters_;
        std::vector<ClusterInfo> clusterInfos_;
        std::vector<ClusterIndex> topClusters_;
    };

    struct Cluster_prototype
    {
        int dummy;
    };

    class Rules_prototype
    {
    public:
        struct NeighborIndicesEnumerator
        {
            bool empty() const {
                return true;
            }
            ClusterIndex operator*() const {
                return 0;
            }
            NeighborIndicesEnumerator& operator ++() {
                return *this;
            }
        };

        NeighborIndicesEnumerator getNeighborIndicesEnumerator(ClusterIndex clusterIndex) const {
            return NeighborIndicesEnumerator();
        }

        template<typename t_Cluster, typename t_Point>
        void initCluster(t_Cluster& cluster, int pointIndex, t_Point const& point) {
        }

        template<typename t_Cluster, typename t_Point>
        bool isInvalidData(t_Cluster const& cluster, ClusterIndex clusterIndex, t_Point const& point) const {
            return false;
        }
        template<typename t_Cluster, typename t_Point>
        bool isOfSameCluster(ClusterIndex clusterIndex, ClusterIndex neighborClusterIndex
                    , t_Cluster const& topCluster, t_Cluster const& neighborTopCluster
                    , t_Point const& point, t_Point const& neighborPoint) const {
            return false;
        }

        template<typename t_Cluster, typename t_Point>
        bool mergeNeighborClusters(ClusterIndex parentClusterIndex, ClusterIndex childClusterIndex
                    , t_Cluster& parentCluster, t_Cluster const& childCluster
                    , t_Point const& point, t_Point const& neighborPoint) {
            return false;
        }
    };

    typedef DisjointSet<Cluster_prototype, Rules_prototype> DisjointSet_prototype;

    struct XyFrame
    {
        int width;
        int height;
        int size;

        XyFrame(int w, int h)
            : width(w)
            , height(h)
            , size(w * h)
        {
        }

        int getIndex(int column, int row) const {
            return row * width + column;
        }
        int getRow(int index) const {
            return index / width;
        }
        int getColumn(int index) const {
            return index % width;
        }
    };

    struct XyNeighborhood
    {
        XyFrame frame;
        int halfWindowSize;
        int centerPixelIndex;
        int centerPixelRow;
        int centerPixelColumn;
        int startRow;
        int endRow;
        int startColumn;
        int endColumn;

        XyNeighborhood(XyFrame const& aFrame, int aHalfWindowSize, int aCenterPixelIndex)
            : frame(aFrame)
            , halfWindowSize(aHalfWindowSize)
            , centerPixelIndex(aCenterPixelIndex)
            , centerPixelRow(aFrame.getRow(aCenterPixelIndex))
            , centerPixelColumn(aFrame.getColumn(aCenterPixelIndex))
        {
            startRow = std::max(0, centerPixelRow - halfWindowSize);
            endRow = std::min(centerPixelRow + halfWindowSize + 1, frame.height);
            startColumn = std::max(0, centerPixelColumn - halfWindowSize);
            endColumn = std::min(centerPixelColumn + halfWindowSize + 1, frame.width);
        }
        XyNeighborhood(XyFrame const& aFrame, int aHalfWindowSize, int aCenterPixelColumn, int aCenterPixelRow)
            : frame(aFrame)
            , halfWindowSize(aHalfWindowSize)
            , centerPixelIndex(aFrame.getIndex(aCenterPixelColumn, aCenterPixelRow))
            , centerPixelRow(aCenterPixelRow)
            , centerPixelColumn(aCenterPixelColumn)
        {
            startRow = std::max(0, centerPixelRow - halfWindowSize);
            endRow = std::min(centerPixelRow + halfWindowSize + 1, frame.height);
            startColumn = std::max(0, centerPixelColumn - halfWindowSize);
            endColumn = std::min(centerPixelColumn + halfWindowSize + 1, frame.width);
        }

        int getStartPixelIndex() const {
            return frame.getIndex(startColumn, startRow);
        }
        int getEndPixelIndex() const {
            return 1 + frame.getIndex(endColumn - 1, endRow - 1);
        }
    };

    class XyNeighborIndicesEnumerator
    {
    private:
        XyNeighborhood neighborhood_;
        int currentPixelColumn_;
        int currentPixelRow_;

    public:
        XyNeighborIndicesEnumerator(XyNeighborhood const& neighborhood)
            : neighborhood_(neighborhood)
            , currentPixelColumn_(neighborhood.startColumn)
            , currentPixelRow_(neighborhood.startRow)
        {
        }
        bool empty() const {
            return currentPixelRow_ >= neighborhood_.endRow;
        }
        ClusterIndex operator*() const {
            return neighborhood_.frame.getIndex(currentPixelColumn_, currentPixelRow_);
        }
        XyNeighborIndicesEnumerator& operator ++() {
            if(++currentPixelColumn_ == neighborhood_.endColumn) {
                ++currentPixelRow_;
                currentPixelColumn_ = neighborhood_.startColumn;
            }
            return *this;
        }
    };

    template<typename t_Value>
    struct Range
    {
        typedef t_Value Value;
        Value min;
        Value max;

        void init(Value v, ...) {
            min = max = v;
        }
        void add(Range const& other) {
            if(min > other.min)
                min = other.min;
            if(max < other.max)
                max = other.max;
        }
    };

    template<typename t_Value>
    struct ValueTraits
    {
        typedef t_Value Value;
        Value distanceThreshold;
        Value invalidValue;

        ValueTraits(Value aDistanceThreshold, Value aInvalidValue)
            : distanceThreshold(aDistanceThreshold)
            , invalidValue(aInvalidValue)
        {
        }

        bool isValid(Value v) const {
            return v != invalidValue;
        }

        bool areClose(Value v1, Value v2) const {
            Value d = (v1 > v2) ? v1 - v2 : v2 - v1;
            return d < distanceThreshold;
        }
    };

    template<typename t_ValueTraits>
    class XyRules
    {
    private:
        XyFrame frame_;
        int halfWindowSize_;
        t_ValueTraits valueTraits_;

    public:
        typedef XyNeighborIndicesEnumerator NeighborIndicesEnumerator;

        XyRules(XyFrame const& aFrame, int aHalfWindowSize, t_ValueTraits const& aValueTraits)
            : frame_(aFrame)
            , halfWindowSize_(aHalfWindowSize)
            , valueTraits_(aValueTraits)
        {
        }

        NeighborIndicesEnumerator getNeighborIndicesEnumerator(ClusterIndex clusterIndex) const {
            return NeighborIndicesEnumerator(XyNeighborhood(frame_, halfWindowSize_, clusterIndex));
        }

        template<typename t_Cluster, typename t_Point>
        void initCluster(t_Cluster& cluster, int pointIndex, t_Point const& point) {
            cluster.init(point, frame_, pointIndex);
        }

        template<typename t_Cluster, typename t_Point>
        bool isInvalidData(t_Cluster const& cluster, ClusterIndex clusterIndex, t_Point const& point) const {
            return !valueTraits_.isValid(point);
        }
        template<typename t_Cluster, typename t_Point>
        bool isOfSameCluster(ClusterIndex clusterIndex, ClusterIndex neighborClusterIndex
                    , t_Cluster const& topCluster, t_Cluster const& neighborTopCluster
                    , t_Point const& point, t_Point const& neighborPoint) const {
            return valueTraits_.areClose(point, neighborPoint);
        }

        template<typename t_Cluster, typename t_Point>
        bool mergeNeighborClusters(ClusterIndex parentClusterIndex, ClusterIndex childClusterIndex
                    , t_Cluster& parentCluster, t_Cluster const& childCluster
                    , t_Point const& point, t_Point const& neighborPoint) {
            parentCluster.add(childCluster);
            return true;
        }
    };


    typedef uint16_t RawZ;
    typedef Range<RawZ> ZRange;
    typedef ValueTraits<RawZ> ZTraits;
    typedef XyRules<ZTraits> DepthViewRules;
    typedef DisjointSet<ZRange, DepthViewRules> ZRangeDisjointSet;

    struct ZClusterComparer
    {
        int minNumOfPoints;

        ZClusterComparer(int aMinNumOfPoints) : minNumOfPoints(aMinNumOfPoints) {}

        bool isOk(ClusterIndex clusterIndex, ZRangeDisjointSet const& clusters) const {
            return clusters.getTopClusterNumOfPoints(clusterIndex) >= minNumOfPoints;
        }
        bool isBetter(ClusterIndex clusterIndex1, ClusterIndex clusterIndex2, ZRangeDisjointSet const& clusters) const {
            return clusters.getTopCluster(clusterIndex1).min < clusters.getTopCluster(clusterIndex2).min;
        }
    };



    struct PixelXy
    {
        int column;
        int row;
    };

    struct PixelXyRange
    {
        PixelXy min, max;

        template<typename t_Point>
        void init(t_Point const& v, XyFrame const& frame, int pointIndex) {
            min.column = max.column = frame.getColumn(pointIndex);
            min.row = max.row = frame.getRow(pointIndex);
        }
        void add(PixelXyRange const& other) {
            if(min.column > other.min.column)
                min.column = other.min.column;
            if(max.column < other.max.column)
                max.column = other.max.column;

            if(min.row > other.min.row)
                min.row = other.min.row;
            if(max.row < other.max.row)
                max.row = other.max.row;
        }
    };

    struct XyzCoords
    {
        float x;
        float y;
        float z;

        bool isValid() const {
            return x != 0 || y != 0 || z != 0;
        }
    };

    struct XyzRange
    {
        XyzCoords min, max;

        bool isValid() const {
            return min.isValid() && max.isValid();
        }

        XyzCoords getTargetPoint() const {
            XyzCoords res = { (min.x + max.x) / 2, (min.y + max.y) / 2, (min.z + max.z) / 2 };
            return res;
        }

        template<typename t_Point>
        void init(t_Point const& v, XyFrame const& frame, int pointIndex) {
            min.x = max.x = v.coords.x;
            min.y = max.y = v.coords.y;
            min.z = max.z = v.coords.z;
        }
        void add(XyzRange const& other) {
            if(!other.isValid())
                //: do nothing, leave as it is:
                return;
            if(!this->isValid()) {
                *this = other;
                return;
            }

            if(min.x > other.min.x)
                min.x = other.min.x;
            if(max.x < other.max.x)
                max.x = other.max.x;

            if(min.y > other.min.y)
                min.y = other.min.y;
            if(max.y < other.max.y)
                max.y = other.max.y;

            if(min.z > other.min.z)
                min.z = other.min.z;
            if(max.z < other.max.z)
                max.z = other.max.z;

            //if(minDepth == 0 || (minDepth > other.minDepth && other.minDepth != 0)) //fixme: use invalid value?
            //    minDepth = other.minDepth;
            //if(maxDepth < other.maxDepth)
            //    maxDepth = other.maxDepth;
        }
    };

    struct RgbCoords
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;

        static RgbCoords const& getWhite() {
            static RgbCoords res = {255, 255, 255};
            return res;
        }
    };


    struct RgbTraits
    {
        float distanceThreshold;

        RgbTraits(float aDistanceThreshold)
            : distanceThreshold(aDistanceThreshold)
        {
        }

        template<typename t_Point>
        bool isValid(t_Point const& p) const {
            return true;
        }

        template<typename t_Point>
        bool areClose(t_Point const& p1, t_Point const& p2) const {
            float d = std::sqrt(sqRgbDistanceApprox(p1, p2));
            return d < distanceThreshold;
        }

        template<typename t_Point1, typename t_Point2>
        static float sqRgbDistanceApprox(t_Point1 const& p1, t_Point2 const& p2) {
            ///: grabbed from http://www.compuphase.com/cmetric.htm
          long rmean = ( (long)p1.r + (long)p2.r ) / 2;
          long r = (long)p1.r - (long)p2.r;
          long g = (long)p1.g - (long)p2.g;
          long b = (long)p1.b - (long)p2.b;
          return (((512+rmean)*r*r)>>8) + 4*g*g + (((767-rmean)*b*b)>>8);
        }

        template<typename t_Point1, typename t_Point2>
        static float sqRgbDistanceNaive(t_Point1 const& p1, t_Point2 const& p2) {
            float dr = p1.r;
            dr -= p2.r;
            float dg = p1.g;
            dg -= p2.g;
            float db = p1.b;
            db -= p2.b;
            return dr * dr + dg * dg + db * db;
        }

        template<typename t_Point1, typename t_Point2>
        static float sqSinRgbDistance(t_Point1 const& p1, t_Point2 const& p2) {
            float r1 = p1.r;
            float r2 = p2.r;
            float g1 = p1.g;
            float g2 = p2.g;
            float b1 = p1.b;
            float b2 = p2.b;
            float d1 = r1*r1 + g1*g1 + b1*b1;
            float d2 = r2*r2 + g2*g2 + b2*b2;
            float dot = r1*r2 + g1*g2 + b1*b2;
            static const float minSqNorm = 40*40;
            float sqcos = (d1 > minSqNorm && d2 > minSqNorm) ? dot * dot / (d1*d2) : 0;
            return 10000 * (1 - sqcos);
        }

        template<typename t_Point>
        static float sqIrDistance(t_Point const& p1, t_Point const& p2) {
            float d_ir = p1.ir;
            d_ir -= p2.ir;
            return d_ir * d_ir;
        }

        template<typename t_Point>
        static float sqDepthDistance(t_Point const& p1, t_Point const& p2) {
            float d_depth = p1.depth;
            d_depth -= p2.depth;
            return d_depth * d_depth;
        }

        template<typename t_Point>
        static float distanceToWhite(t_Point const& p) {
            return std::sqrt(sqRgbDistanceApprox(p, RgbCoords::getWhite()));
        }

    };

    struct RgbRange
    {
        RgbCoords min, max;

        template<typename t_Point>
        void init(t_Point const& v, XyFrame const& frame, int pointIndex) {
            min.r = max.r = v.r;
            min.g = max.g = v.g;
            min.b = max.b = v.b;
        }
        void add(RgbRange const& other) {
            if(min.r > other.min.r)
                min.r = other.min.r;
            if(max.r < other.max.r)
                max.r = other.max.r;

            if(min.g > other.min.g)
                min.g = other.min.g;
            if(max.g < other.max.g)
                max.g = other.max.g;

            if(min.b > other.min.b)
                min.b = other.min.b;
            if(max.b < other.max.b)
                max.b = other.max.b;
        }
    };

    typedef uint8_t Ir;
    typedef Range<Ir> IrRange;

    struct RgbIrDXyzPointRange
    {
        ZRange depth;
        IrRange ir;
        PixelXyRange pixelXy;
        RgbRange rgb;
        XyzRange xyz;

        template<typename t_Point>
        void init(t_Point const& v, XyFrame const& frame, int pointIndex) {
            depth.init(v.depth, frame, pointIndex);
            ir.init(v.ir, frame, pointIndex);
            pixelXy.init(v, frame, pointIndex);
            rgb.init(v, frame, pointIndex);
            xyz.init(v, frame, pointIndex);
        }
        void add(RgbIrDXyzPointRange const& other) {
            depth.add(other.depth);
            ir.add(other.ir);
            pixelXy.add(other.pixelXy);
            rgb.add(other.rgb);
            xyz.add(other.xyz);
        }

        float distanceToWhite() const {
            return RgbTraits::distanceToWhite(rgb.max);
        }

    };

    typedef XyRules<RgbTraits> RgbIrDXyzPointRules;
    typedef DisjointSet<RgbIrDXyzPointRange, RgbIrDXyzPointRules> RgbIrDXyzDisjointSet;

    struct RgbIrDXyzClusterComparer
    {
        int minNumOfPoints;

        RgbIrDXyzClusterComparer(int aMinNumOfPoints) : minNumOfPoints(aMinNumOfPoints) {}

        bool isOk(ClusterIndex clusterIndex, RgbIrDXyzDisjointSet const& clusters) const {
            return clusters.getTopClusterNumOfPoints(clusterIndex) >= minNumOfPoints;
        }
        bool isBetter(ClusterIndex clusterIndex1, ClusterIndex clusterIndex2, RgbIrDXyzDisjointSet const& clusters) const {
            float distanceToWhite1 = clusters.getTopCluster(clusterIndex1).distanceToWhite();
            float distanceToWhite2 = clusters.getTopCluster(clusterIndex2).distanceToWhite();
            return distanceToWhite1 < distanceToWhite2;
        }
    };


} // namespace clustering



#endif
