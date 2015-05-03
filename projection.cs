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
        private managed_pcl.RgbIrDXyzPoint[] rgb_ir_d_xyz_points = null;
        private float[] pixelQuality;
        private byte[] pixelColors;
        int height = 0;
        int width = 0;

        //: the following arays are made fields to avoid frequent allocation/dealocation of them:
        PXCMPoint3DF32[] targetXyzPos = new PXCMPoint3DF32[1];
        PXCMPoint3DF32[] targetIjzPos = new PXCMPoint3DF32[1];
        PXCMPointF32[] targetRcPos = new PXCMPointF32[1];
        bool gotTargetPos = false;
        bool gotTargetDir = false;
        bool gotTargetRcPos = false;

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
            rgb_ir_d_xyz_points = new managed_pcl.RgbIrDXyzPoint[numOfPixels];
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

        public void ComputePixelQuality(PXCMCapture.Sample sample, managed_pcl.ProcessParams processParams)
        {
            if (processParams.qualityEstimationMethod == managed_pcl.QualityEstimationMethod.DepthChange)
            {
                this.computePixelQualityFromDepth(sample.depth);
            }
            else if (processParams.qualityEstimationMethod == managed_pcl.QualityEstimationMethod.DepthClusters)
            {
                this.depthToScan(sample.depth);
                this.computePixelQualityFromDepthClusters(sample.depth, processParams);
            }
            else if (processParams.qualityEstimationMethod == managed_pcl.QualityEstimationMethod.ColorClusters)
            {
                this.computePixelQualityFromClusters(sample, processParams);
            }
            else
            {
                this.depthToScan(sample.depth);
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

            if (this.gotTargetRcPos)
            {
                //: plot target point:
                int halfPatchSize = 5;
                int topColumn = (int)(this.targetRcPos[0].x + 0.5);
                int topRow = (int)(this.targetRcPos[0].y + 0.5);
                int rowStart = Math.Max(0, topRow - halfPatchSize);
                int rowEnd = Math.Min(topRow + halfPatchSize + 1, this.height);
                int colStart = Math.Max(0, topColumn - halfPatchSize);
                int colEnd = Math.Min(topColumn + halfPatchSize + 1, this.width);
                for (int iRow = rowStart; iRow < rowEnd; ++iRow)
                {
                    for (int iColumn = colStart; iColumn < colEnd; ++iColumn)
                    {
                        int index = iRow * width + iColumn;
                        if (this.gotTargetPos)
                        {
                            //: yellow:
                            pixelColors[4 * index + 0] = 0;
                            pixelColors[4 * index + 1] = 255;
                            pixelColors[4 * index + 2] = 255;
                        }
                        else if (this.gotTargetDir)
                        {
                            //: orange:
                            pixelColors[4 * index + 0] = 0;
                            pixelColors[4 * index + 1] = 128;
                            pixelColors[4 * index + 2] = 255;
                        }
                        else
                        {
                            //: brown:
                            pixelColors[4 * index + 0] = 0;
                            pixelColors[4 * index + 1] = 128;
                            pixelColors[4 * index + 2] = 128;
                        }
                    }
                }
            }
            return pixelColors;
        }

        public void SaveToPcd(string xyzFileName, string normalsFileName, bool binary)
        {
            scan.saveToPcdFile(xyzFileName, normalsFileName, binary);
        }

        private void updateTargetPos()
        {
            this.gotTargetRcPos = false;
            this.gotTargetDir = this.gotTargetPos = scan.GotTarget;
            if(this.gotTargetPos)
            {
                this.targetXyzPos[0].x = scan.TargetXyz.x;
                this.targetXyzPos[0].y = scan.TargetXyz.y;
                this.targetXyzPos[0].z = scan.TargetXyz.z;
                this.gotTargetRcPos = (this.projection.ProjectCameraToColor(this.targetXyzPos, this.targetRcPos) == pxcmStatus.PXCM_STATUS_NO_ERROR
                    && this.targetRcPos[0].x != -1 && this.targetRcPos[0].y != -1);
            }
            else if (scan.GotObjectCenterInPixels)
            {
                this.gotTargetRcPos = true;
                this.targetRcPos[0].x = scan.ObjectCenterInPixels.column;
                this.targetRcPos[0].y = scan.ObjectCenterInPixels.row;
                //: instead of pos get the direction to target from object pixels:
                this.targetIjzPos[0].x = scan.ObjectCenterInPixels.column;
                this.targetIjzPos[0].y = scan.ObjectCenterInPixels.row;
                this.targetIjzPos[0].z = 1000;
                this.gotTargetDir = (this.projection.ProjectColorToCamera(this.targetIjzPos, this.targetXyzPos) == pxcmStatus.PXCM_STATUS_NO_ERROR);
            }
        }

        private void publishTargetPosOrDir()
        {
            if (rosPublisher.isStarted() && this.gotTargetDir)
                rosPublisher.publishPose(this.targetXyzPos[0].x / 1000.0, this.targetXyzPos[0].y / 1000.0, this.targetXyzPos[0].z / 1000.0);
        }

        private void depthToScan(PXCMImage depth)
        {
            pxcmStatus sts = projection.QueryVertices(depth, coords);
            scan.setCoords(coords);
        }

        public static byte[] getRGB32Pixels(PXCMImage image)
        {
            PXCMImage.ImageData cdata;
            byte[] cpixels = null;
            int cwidth = 0;
            int cheight = 0;
            if (image.AcquireAccess(PXCMImage.Access.ACCESS_READ, PXCMImage.PixelFormat.PIXEL_FORMAT_RGB32, out cdata) >= pxcmStatus.PXCM_STATUS_NO_ERROR)
            {
                cwidth = (int)cdata.pitches[0] / sizeof(Int32);
                cheight = (int)image.info.height;
                cpixels = cdata.ToByteArray(0, (int)cdata.pitches[0] * cheight);
                image.ReleaseAccess(cdata);
            }
            return cpixels;
        }

        private static UInt16[] getUInt16Depths(PXCMImage depthImage)
        {
            PXCMImage.ImageData depthImageData;
            UInt16[] pixelDepths = null;
            if (depthImage.AcquireAccess(PXCMImage.Access.ACCESS_READ, PXCMImage.PixelFormat.PIXEL_FORMAT_DEPTH, out depthImageData) >= pxcmStatus.PXCM_STATUS_NO_ERROR)
            {
                int width = (int)depthImageData.pitches[0] / sizeof(Int16);
                int height = (int)depthImage.info.height;
                pixelDepths = depthImageData.ToUShortArray(0, width * height);
                depthImage.ReleaseAccess(depthImageData);
            }
            return pixelDepths;
        }

        private void loadRgbIrDXyzPointCoords(PXCMImage depthImage)
        {
            UInt16[] pixelDepths = getUInt16Depths(depthImage);
            if (pixelDepths != null) //minDepth < maxDepth)
            {
                pxcmStatus sts = projection.QueryVertices(depthImage, coords);
                scan.setCoords(coords); //fixme: remove
                int numOfPixels = this.rgb_ir_d_xyz_points.Length;
                for (int i = 0; i < numOfPixels; ++i)
                {
                    rgb_ir_d_xyz_points[i].coords.x = coords[i].x;
                    rgb_ir_d_xyz_points[i].coords.y = coords[i].y;
                    rgb_ir_d_xyz_points[i].coords.z = coords[i].z;
                    rgb_ir_d_xyz_points[i].depth = pixelDepths[i];
                }
            }
            else
            {
                int numOfPixels = this.rgb_ir_d_xyz_points.Length;
                for (int i = 0; i < numOfPixels; ++i)
                {
                    rgb_ir_d_xyz_points[i].coords.x = 0;
                    rgb_ir_d_xyz_points[i].coords.y = 0;
                    rgb_ir_d_xyz_points[i].coords.z = 0;
                    rgb_ir_d_xyz_points[i].depth = 0;
                }
            }
        }

        private void loadRgbIrDXyzPointRgb(PXCMImage colorImage)
        {
            byte[] rgb32Pixels = getRGB32Pixels(colorImage);
            if (rgb32Pixels != null)
            {
                int numOfPixels = this.rgb_ir_d_xyz_points.Length;
                for (int i = 0; i < numOfPixels; ++i)
                {
                    rgb_ir_d_xyz_points[i].r = rgb32Pixels[4 * i + 2];
                    rgb_ir_d_xyz_points[i].g = rgb32Pixels[4 * i + 1];
                    rgb_ir_d_xyz_points[i].b = rgb32Pixels[4 * i + 0];
                }
            }
            else
            {
                int numOfPixels = this.rgb_ir_d_xyz_points.Length;
                for (int i = 0; i < numOfPixels; ++i)
                {
                    rgb_ir_d_xyz_points[i].r = 0;
                    rgb_ir_d_xyz_points[i].g = 0;
                    rgb_ir_d_xyz_points[i].b = 0;
                }
            }
        }

        private void loadRgbIrDXyzPointIr(PXCMImage irImage)
        {
            byte[] rgb32Pixels = getRGB32Pixels(irImage);
            if (rgb32Pixels != null)
            {
                int numOfPixels = this.rgb_ir_d_xyz_points.Length;
                for (int i = 0; i < numOfPixels; ++i)
                {
                    rgb_ir_d_xyz_points[i].ir = rgb32Pixels[4 * i + 0];
                }
            }
            else
            {
                int numOfPixels = this.rgb_ir_d_xyz_points.Length;
                for (int i = 0; i < numOfPixels; ++i)
                {
                    rgb_ir_d_xyz_points[i].ir = 0;
                }
            }
        }

        private void loadRgbIrDXyzPoints(PXCMCapture.Sample sample)
        {
            this.loadRgbIrDXyzPointCoords(sample.depth);
            this.loadRgbIrDXyzPointRgb(sample.color);
            this.loadRgbIrDXyzPointIr(sample.ir);
        }

        private void computePixelQualityFromNormals(managed_pcl.ProcessParams processParams)
        {
            scan.computePixelQualityFromNormals(pixelQuality, processParams);
        }

        private void computePixelQualityFromDepth(PXCMImage depthImage)
        {
            float maxDepth = 1200;
            float minDepth = 100;
            UInt16[] pixelDepths = getUInt16Depths(depthImage);
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
            UInt16[] pixelDepths = getUInt16Depths(depthImage);
            if (pixelDepths != null) //minDepth < maxDepth)
            {
                scan.computePixelQualityFromDepthClusters(pixelDepths, this.invalid_value, pixelQuality, processParams.depthClustersParams);
                this.updateTargetPos();
                this.publishTargetPosOrDir();
            }
            else
                setAllNan();
        }

        private void computePixelQualityFromClusters(PXCMCapture.Sample sample, managed_pcl.ProcessParams processParams)
        {
            this.loadRgbIrDXyzPoints(sample);
            scan.computePixelQualityFromClusters(this.rgb_ir_d_xyz_points, this.invalid_value, this.pixelQuality, processParams);
            this.updateTargetPos();
            this.publishTargetPosOrDir();
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
