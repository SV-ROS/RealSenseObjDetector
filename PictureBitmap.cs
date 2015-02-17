using System;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;

namespace raw_streams.cs
{
    public class PictureBitmap : IDisposable
    {
        public void SetPixels(int width, int height, byte[] pixels)
        {
            if (width != width_ || height != height_ || bitmap_ == null)
            {
                width_ = width;
                height_ = height;
                bitmap_ = new Bitmap(width_, height_, PixelFormat.Format32bppRgb);
            }
            if (pixels != null && bitmap_ != null)
            {
                BitmapData bitmap_data = bitmap_.LockBits(new Rectangle(0, 0, width_, height_), ImageLockMode.WriteOnly, PixelFormat.Format32bppRgb);
                Marshal.Copy(pixels, 0, bitmap_data.Scan0, width_ * height_ * 4);
                bitmap_.UnlockBits(bitmap_data);
            }
        }
        public Bitmap GetBitmap()
        {
            return bitmap_;
        }

        public void Dispose()
        {
            if (bitmap_ != null)
            {
                bitmap_.Dispose();
                bitmap_ = null;
            }
        }

        private int width_ = 0;
        private int height_ = 0;
        private Bitmap bitmap_ = null;
    }

}
