using System;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;

namespace raw_streams.cs
{
    public class PXCMBitmap : IDisposable
    {
        public void SetImage(PXCMImage newimage)
        {
            clean();
            image = newimage;
            if (image != null)
                image.AcquireAccess(PXCMImage.Access.ACCESS_READ, PXCMImage.PixelFormat.PIXEL_FORMAT_RGB32, out data);
        }
        public void SetPixels(int _width, int _height, byte[] _pixels)
        {
            clean();
            pixels = _pixels;
            width = _width;
            height = _height;
        }
        public Bitmap ToBitmap()
        {
            if (image != null && data != null)
            {
                return data.ToBitmap(0, image.info.width, image.info.height);
            }
            if (pixels != null)
            {
                Bitmap bitmap = new Bitmap(width, height, PixelFormat.Format32bppRgb);
                BitmapData bitmap_data = bitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format32bppRgb);
                Marshal.Copy(pixels, 0, bitmap_data.Scan0, width * height * 4);
                bitmap.UnlockBits(bitmap_data);
                return bitmap;
            }
            return null;
        }

        public void Dispose()
        {
            clean();
        }

        private void clean()
        {
            if (image != null && data != null)
                image.ReleaseAccess(data);
            image = null;
            data = null;
            pixels = null;
        }

        // PXCMImage
        private PXCMImage image;
        private PXCMImage.ImageData data;
        // byte array
        int width;
        int height;
        private byte[] pixels;
    }

}
