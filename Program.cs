using System;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace raw_streams.cs
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            using (PXCMSession session = PXCMSession.CreateInstance())
            {
                if (session != null)
                    Application.Run(new MainForm(session));
            }
        }
    }
}
