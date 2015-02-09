using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Linq;
using System.Text;



namespace raw_streams.cs
{
    class Projection: IDisposable
    {
        private PXCMProjection projection=null;
        private readonly UInt16 invalid_value; /* invalid depth values */
        private PXCMPointF32[] uvmap;
        private managed_pcl.Scan scan;
        private PXCMPoint3DF32[] coords;
        private float[] pixelQuality;
        private byte[] pixelColors;
        private int fileIndex = 0;

        public Projection(PXCMSession session, PXCMCapture.Device device, PXCMImage.ImageInfo dinfo)
        {
            /* retrieve the invalid depth pixel values */
            invalid_value = device.QueryDepthLowConfidenceValue();

            /* Create the projection instance */
            projection = device.CreateProjection();

            int numOfPixels = dinfo.width * dinfo.height;
            uvmap = new PXCMPointF32[numOfPixels];
            scan = new managed_pcl.Scan(dinfo.width, dinfo.height);
            coords = new PXCMPoint3DF32[numOfPixels];
            pixelQuality = new float[numOfPixels];
            pixelColors = new byte[4 * numOfPixels]; //: pixels in rgb32
        }

        public void Dispose()
        {
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

        public void DepthToScan(PXCMImage depth)
        {
            pxcmStatus sts = projection.QueryVertices(depth, coords);
            scan.setCoords(coords);
        }

        public void ComputePixelQualityFromCurvature(managed_pcl.ProcessParams processParams)
        {
            scan.computePixelQualityFromCurvature(pixelQuality, processParams);
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
                if (pq <= maxBad)
                {
                    pixelColors[4 * i + 0] = 255;
                    pixelColors[4 * i + 1] = 0;
                    pixelColors[4 * i + 2] = 0;
                } else if (pq >= minGood)
                {
                    pixelColors[4 * i + 0] = 0;
                    pixelColors[4 * i + 1] = 255;
                    pixelColors[4 * i + 2] = 0;
                }
                else
                {
                    double rw = (pq - maxBad) / yellowZoneLength;
                    pixelColors[4 * i + 0] = (byte)(255.0 * rw);
                    pixelColors[4 * i + 1] = (byte)(255.0 * (1.0 - rw));
                    pixelColors[4 * i + 2] = 255;
                }
            }
            return pixelColors;
        }

        public void SaveToPcd(string fileName)
        {
            scan.saveToPcdFile(fileName);
        }

        public void SaveToPcd()
        {
            string fileName = string.Format("c:/p/00-rs2pcd/{0:D6}.pcd", (++fileIndex));
            SaveToPcd(fileName);
        }
    }
}
