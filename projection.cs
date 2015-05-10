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
        private managed_obj_detector.ObjDetector obj_detector;
        private managed_ros_ser.PosPublisher rosPublisher = new managed_ros_ser.PosPublisher();
        private readonly int height;
        private readonly int width;
        private readonly int numOfPixels;

        PXCMPoint3DF32[] coords;
        managed_obj_detector.RgbIrDXyzPoint[] rgb_ir_d_xyz_points = null;
        PXCMPoint3DF32[] targetXyzPos = new PXCMPoint3DF32[1];
        PXCMPointF32[] targetRcPos = new PXCMPointF32[1];

        public Projection(PXCMSession session, PXCMCapture.Device device, PXCMImage.ImageInfo dinfo)
        {
            //: start ros serial node:
//            rosPublisher.start("192.168.0.10");

            /* Create the projection instance */
            projection = device.CreateProjection();

            height = dinfo.height;
            width = dinfo.width;
            numOfPixels = dinfo.width * dinfo.height;
            UInt16 invalid_value = device.QueryDepthLowConfidenceValue();
            obj_detector = new managed_obj_detector.ObjDetector(dinfo.width, dinfo.height, invalid_value);
            coords = new PXCMPoint3DF32[numOfPixels];
            rgb_ir_d_xyz_points = new managed_obj_detector.RgbIrDXyzPoint[numOfPixels];
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
            if (obj_detector != null)
            {
                obj_detector.Dispose();
                obj_detector = null;
            }
        }

        public void SearchForObject(PXCMCapture.Sample sample, managed_obj_detector.ProcessParams processParams)
        {
            //////if (processParams.qualityEstimationMethod == managed_obj_detector.QualityEstimationMethod.DepthChange)
            //////{
            //////    this.computePixelQualityFromDepth(sample.depth);
            //////}
            //////else if (processParams.qualityEstimationMethod == managed_obj_detector.QualityEstimationMethod.DepthClusters)
            //////{
            //////    this.depthToScan(sample.depth);
            //////    this.computePixelQualityFromDepthClusters(sample.depth, processParams);
            //////}
            //////else if (processParams.qualityEstimationMethod == managed_obj_detector.QualityEstimationMethod.ColorClusters)
            //////{
            this.computePixelQualityFromClusters(sample, processParams);
            //////}
            //////else
            //////{
            //////    this.depthToScan(sample.depth);
            //////    this.computePixelQualityFromNormals(processParams);
            //////}
        }

        public byte[] GetPixelColors()
        {
            //: plot target point:
            if (obj_detector.GotTarget)
            {
                this.targetXyzPos[0].x = obj_detector.TargetXyz.x;
                this.targetXyzPos[0].y = obj_detector.TargetXyz.y;
                this.targetXyzPos[0].z = obj_detector.TargetXyz.z;
                if (this.projection.ProjectCameraToColor(this.targetXyzPos, this.targetRcPos) == pxcmStatus.PXCM_STATUS_NO_ERROR
                    && this.targetRcPos[0].x != -1 && this.targetRcPos[0].y != -1)
                {
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
                            //: yellow:
                            this.obj_detector.PixelColors[4 * index + 0] = 0;
                            this.obj_detector.PixelColors[4 * index + 1] = 255;
                            this.obj_detector.PixelColors[4 * index + 2] = 255;
                        }
                    }
                }
            }
            return this.obj_detector.PixelColors;
        }

        //private void depthToScan(PXCMImage depth)
        //{
        //    pxcmStatus sts = projection.QueryVertices(depth, coords);
        //    obj_detector.setCoords(coords);
        //}

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
            //////UInt16[] pixelDepths = getUInt16Depths(depthImage);
            //////if (pixelDepths != null) //minDepth < maxDepth)
            //////{
            //////    pxcmStatus sts = projection.QueryVertices(depthImage, coords);
            //////    obj_detector.setCoords(coords); //fixme: remove
            //////    int numOfPixels = this.rgb_ir_d_xyz_points.Length;
            //////    for (int i = 0; i < numOfPixels; ++i)
            //////    {
            //////        rgb_ir_d_xyz_points[i].coords.x = coords[i].x;
            //////        rgb_ir_d_xyz_points[i].coords.y = coords[i].y;
            //////        rgb_ir_d_xyz_points[i].coords.z = coords[i].z;
            //////        rgb_ir_d_xyz_points[i].depth = pixelDepths[i];
            //////    }
            //////}
            //////else
            //////{
            //////    int numOfPixels = this.rgb_ir_d_xyz_points.Length;
            //////    for (int i = 0; i < numOfPixels; ++i)
            //////    {
            //////        rgb_ir_d_xyz_points[i].coords.x = 0;
            //////        rgb_ir_d_xyz_points[i].coords.y = 0;
            //////        rgb_ir_d_xyz_points[i].coords.z = 0;
            //////        rgb_ir_d_xyz_points[i].depth = 0;
            //////    }
            //////}
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
                int numOfPixels
                    = this.rgb_ir_d_xyz_points.Length;
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

        private void computePixelQualityFromDepthClusters(PXCMImage depthImage, managed_obj_detector.ProcessParams processParams)
        {
            //////UInt16[] pixelDepths = getUInt16Depths(depthImage);
            //////if (pixelDepths != null) //minDepth < maxDepth)
            //////{
            //////    obj_detector.computePixelQualityFromDepthClusters(pixelDepths, this.invalid_value, pixelQuality, processParams.depthClustersParams);
            //////    if (rosPublisher.isStarted() && obj_detector.GotTarget)
            //////        rosPublisher.publishPose(obj_detector.TargetXyz.x / 1000.0, obj_detector.TargetXyz.y / 1000.0, obj_detector.TargetXyz.z / 1000.0);
            //////}
            //////else
            //////    setAllNan();
        }

        private void computePixelQualityFromClusters(PXCMCapture.Sample sample, managed_obj_detector.ProcessParams processParams)
        {
            this.loadRgbIrDXyzPoints(sample);
            obj_detector.clusterize(this.rgb_ir_d_xyz_points, processParams);
            if (rosPublisher.isStarted() && obj_detector.GotTarget)
                rosPublisher.publishPose(obj_detector.TargetXyz.x, obj_detector.TargetXyz.y, obj_detector.TargetXyz.z);
        }

        private void setAllNan()
        {
            //////int numOfPixels = pixelQuality.Length;
            //////for (int i = 0; i < numOfPixels; ++i)
            //////{
            //////    pixelQuality[i] = (float)managed_obj_detector.PixelQualitySpecialValue.NoData;
            //////}
        }

    }
}
