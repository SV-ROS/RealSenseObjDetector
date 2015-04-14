using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Linq;
using System.Text;



namespace raw_streams.cs
{
    class Projection: IDisposable
    {
        private PXCMProjection projection = null;
        private readonly UInt16 invalid_value; /* invalid depth values */
        private managed_pcl.Scan scan;
        private PXCMPoint3DF32[] coords;
        private float[] pixelQuality;
        private byte[] pixelColors;
        int height = 0;
        int width = 0;

        private managed_ros_ser.PosPublisher rosPublisher = new managed_ros_ser.PosPublisher();

        public Projection(PXCMSession session, PXCMCapture.Device device, PXCMImage.ImageInfo dinfo)
        {
            //: start ros serial node:
            rosPublisher.start("192.168.0.10");

            //: retrieve the invalid depth pixel values
            invalid_value = device.QueryDepthLowConfidenceValue();

            /* Create the projection instance */
            projection = device.CreateProjection();

            height = dinfo.height;
            width = dinfo.width;
            int numOfPixels = dinfo.width * dinfo.height;
            scan = new managed_pcl.Scan(dinfo.width, dinfo.height);
            coords = new PXCMPoint3DF32[numOfPixels];
            pixelQuality = new float[numOfPixels];
            pixelColors = new byte[4 * numOfPixels]; //: pixels in rgb32
        }

        public void Dispose()
        {
            if (rosPublisher != null)
            {
                rosPublisher.Dispose();
                rosPublisher = null;
            }
            if (projection != null)
            {
                projection.Dispose();
                projection = null;
            }
            if (scan != null)
            {
                scan.Dispose();
                scan = null;
            }
        }

        public void ComputePixelQuality(PXCMImage depth, managed_pcl.ProcessParams processParams)
        {
            if (processParams.qualityEstimationMethod == managed_pcl.QualityEstimationMethod.DepthChange)
            {
                this.computePixelQualityFromDepth(depth);
            }
            else if (processParams.qualityEstimationMethod == managed_pcl.QualityEstimationMethod.DepthClusters)
            {
                this.depthToScan(depth);
                this.computePixelQualityFromDepthClusters(depth, processParams);
            }
            else
            {
                this.depthToScan(depth);
                this.computePixelQualityFromNormals(processParams);
            }
        }

        public byte[] GetPixelColors(float maxBad, float minGood)
        {
            if (maxBad > minGood)
                minGood = maxBad;
            double yellowZoneLength = (double)(minGood - maxBad);
            int numOfPixels = pixelQuality.Length;
            for (int i = 0; i < numOfPixels; ++i)
            {
                float pq = pixelQuality[i];
                if ((float)managed_pcl.PixelQualitySpecialValue.NoData == pq)
                { //: black:
                    pixelColors[4 * i + 0] = 0;
                    pixelColors[4 * i + 1] = 0;
                    pixelColors[4 * i + 2] = 0;
                }
                else if ((float)managed_pcl.PixelQualitySpecialValue.Boundary == pq)
                { //: gray:
                    pixelColors[4 * i + 0] = 128;
                    pixelColors[4 * i + 1] = 128;
                    pixelColors[4 * i + 2] = 128;
                }
                else if (pq <= 0)
                { //: magenta:
                    pixelColors[4 * i + 0] = 255;
                    pixelColors[4 * i + 1] = 0;
                    pixelColors[4 * i + 2] = 255;
                }
                else if (pq >= 1)
                { //: cyan:
                    pixelColors[4 * i + 0] = 255;
                    pixelColors[4 * i + 1] = 255;
                    pixelColors[4 * i + 2] = 0;
                }
                else if (pq <= maxBad)
                { //: red:
                    pixelColors[4 * i + 0] = 0;
                    pixelColors[4 * i + 1] = 0;
                    pixelColors[4 * i + 2] = 255;
                }
                else if (pq >= minGood)
                { //: green:
                    pixelColors[4 * i + 0] = 0;
                    pixelColors[4 * i + 1] = 255;
                    pixelColors[4 * i + 2] = 0;
                }
                else
                { //: gradient from red to green:
                    double w = (pq - maxBad) / yellowZoneLength;
                    pixelColors[4 * i + 0] = 0;
                    pixelColors[4 * i + 1] = (byte)(255.0 * w);
                    pixelColors[4 * i + 2] = (byte)(255.0 * (1.0 - w));
                }
            }
            //: top point:
            if (scan.BBox.valid)
            {
                int halfPatchSize = 5;
                int topRow = scan.TopPosRow;
                int topColumn = scan.TopPosColumn;
                int rowStart = Math.Max(0, topRow - halfPatchSize);
                int rowEnd = Math.Min(topRow + halfPatchSize + 1, this.height);
                int colStart = Math.Max(0, topColumn - halfPatchSize);
                int colEnd = Math.Min(topColumn + halfPatchSize + 1, this.width);
                for(int iRow = rowStart; iRow < rowEnd; ++iRow) {
                    for(int iColumn = colStart; iColumn < colEnd; ++iColumn) {
                        int index = iRow * width + iColumn;
                        //: magenta:
                        pixelColors[4 * index + 0] = 255;
                        pixelColors[4 * index + 1] = 0;
                        pixelColors[4 * index + 2] = 255;
                    }
                }
            }
            return pixelColors;
        }

        public void SaveToPcd(string xyzFileName, string normalsFileName, bool binary)
        {
            scan.saveToPcdFile(xyzFileName, normalsFileName, binary);
        }

        private void depthToScan(PXCMImage depth)
        {
            pxcmStatus sts = projection.QueryVertices(depth, coords);
            scan.setCoords(coords);
        }

        private void computePixelQualityFromNormals(managed_pcl.ProcessParams processParams)
        {
            scan.computePixelQualityFromNormals(pixelQuality, processParams);
        }

        private void computePixelQualityFromDepth(PXCMImage depthImage)
        {
            PXCMImage.ImageData depthImageData;
            UInt16[] pixelDepths = null;
            const UInt16 minDepth = 100;
            UInt16 maxDepth = 1100;
            if (depthImage.AcquireAccess(PXCMImage.Access.ACCESS_READ, PXCMImage.PixelFormat.PIXEL_FORMAT_DEPTH, out depthImageData) >= pxcmStatus.PXCM_STATUS_NO_ERROR)
            {
                int width = (int)depthImageData.pitches[0] / sizeof(Int16);
                int height = (int)depthImage.info.height;
                pixelDepths = depthImageData.ToUShortArray(0, width * height);
                System.Diagnostics.Debug.Assert(pixelDepths.Length == pixelQuality.Length);
                depthImage.ReleaseAccess(depthImageData);
                //minDepth = pixelDepths.Min(p => (p == this.invalid_value ? ushort.MaxValue : p));
                //maxDepth = pixelDepths.Max();
            }
            if (pixelDepths != null) //minDepth < maxDepth)
            {
                int numOfPixels = pixelQuality.Length;
                float delta = (float)(maxDepth - minDepth);
                for (int i = 0; i < numOfPixels; ++i)
                {
                    if (pixelDepths[i] == this.invalid_value)
                        pixelQuality[i] = (float)managed_pcl.PixelQualitySpecialValue.NoData;
                    else
                    {
                        float p = Math.Max(0.0f, (maxDepth - pixelDepths[i]) / delta);
                        pixelQuality[i] = p * p * p;
                    }
                }
            }
            else
                setAllNan();
        }

        private void computePixelQualityFromDepthClusters(PXCMImage depthImage, managed_pcl.ProcessParams processParams)
        {
            PXCMImage.ImageData depthImageData;
            UInt16[] pixelDepths = null;
            if (depthImage.AcquireAccess(PXCMImage.Access.ACCESS_READ, PXCMImage.PixelFormat.PIXEL_FORMAT_DEPTH, out depthImageData) >= pxcmStatus.PXCM_STATUS_NO_ERROR)
            {
                int width = (int)depthImageData.pitches[0] / sizeof(Int16);
                int height = (int)depthImage.info.height;
                pixelDepths = depthImageData.ToUShortArray(0, width * height);
                System.Diagnostics.Debug.Assert(pixelDepths.Length == pixelQuality.Length);
                depthImage.ReleaseAccess(depthImageData);
                //minDepth = pixelDepths.Min(p => (p == this.invalid_value ? ushort.MaxValue : p));
                //maxDepth = pixelDepths.Max();
            }
            if (pixelDepths != null) //minDepth < maxDepth)
            {
                scan.computePixelQualityFromDepthClusters(pixelDepths, this.invalid_value, pixelQuality, processParams.depthClustersParams);
                managed_pcl.XyzBox bbox = scan.BBox;
                if (rosPublisher.isStarted() && bbox.valid)
                    rosPublisher.publishPose(bbox.TopPos.x, bbox.TopPos.y, bbox.TopPos.z);
            }
            else
                setAllNan();
        }

        private void setAllNan()
        {
            int numOfPixels = pixelQuality.Length;
            for (int i = 0; i < numOfPixels; ++i)
            {
                pixelQuality[i] = (float)managed_pcl.PixelQualitySpecialValue.NoData;
            }
        }

    }
}
