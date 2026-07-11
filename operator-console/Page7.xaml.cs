using System;
using System.IO;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using OpenCvSharp;
using OpenCvSharp.Extensions;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// Page9.xaml 的交互逻辑
    /// </summary>
    public partial class Page7 : Page
    {
        private PageTest _paretWin;
        public PageTest ParentWindow
        {
            get { return _paretWin; }
            set { _paretWin = value; }
        }

        private Page3 _parentPage;

        public Page3 ParentPage
        {
            get { return _parentPage; }
            set { _parentPage = value; }
        }

        public Page7()
        {
            InitializeComponent();
            RecordSwitch.IsEnabled = false;
            Capture.IsEnabled = false;
        }

        private Thread thread_cam;
        private bool cam_open = false;
        private bool record_start = false;
        private bool capture_save = false;
        private bool init_success = false;
        private string image_name;
        private string video_name;
        private VideoWriter outputVideo;
        private OpenCvSharp.Size imgSize;
        int init_flag = 0;
        private object record_lock = new object();

        private void CamSwitch_Click(object sender, RoutedEventArgs e)
        {
            if (!cam_open)
            {
                thread_cam = new Thread(run_cap);
                thread_cam.Start();

                init_flag = 0;
                
                while (init_flag == 0)
                { }

                if (!init_success)
                {
                    thread_cam.Abort();
                    return;
                }
                RecordSwitch.IsEnabled = true;
                Capture.IsEnabled = true;
                CamSwitch.Content = "相机关闭(Close)";
            }
            else
            {
                if (record_start)
                    RecordSwitch_End();
                CamSwitch.Content = "相机开启(Open)";
                RecordSwitch.IsEnabled = false;
                Capture.IsEnabled = false;
                thread_cam.Abort();
                init_success = false;
            }
            cam_open = !cam_open;
        }

        private void RecordSwitch_Click(object sender, RoutedEventArgs e)
        {
            if (!record_start)
                RecordSwitch_Start();
            else
                RecordSwitch_End();
        }

        private void Capture_Click(object sender, RoutedEventArgs e)
        {
            capture_save = true;
        }

        private void Switch_Click(object sender, RoutedEventArgs e)
        {
            if (cam_open)
            {
                if (record_start)
                    RecordSwitch_End();
                CamSwitch.Content = "相机开启(Open)";
                RecordSwitch.IsEnabled = false;
                Capture.IsEnabled = false;
                if (thread_cam.IsAlive)
                    thread_cam.Abort();
                init_success = false;
            }

            this.NavigationService.Navigate(this.ParentPage.page4);
        }

        private void Return_Click(object sender, RoutedEventArgs e)
        {
            if (cam_open)
            {
                if (record_start)
                    RecordSwitch_End();
                CamSwitch.Content = "相机开启(Open)";
                RecordSwitch.IsEnabled = false;
                Capture.IsEnabled = false;
                if (thread_cam.IsAlive)
                    thread_cam.Abort();
                init_success = false;
            }

            this.NavigationService.Navigate(this.ParentPage);
        }

        private void RecordSwitch_Start()
        {
            lock (record_lock)
            {
                DateTime t = DateTime.Now;
                video_name = t.ToString("yyyy-MM-dd-HH-mm-ss");
                video_name += ".avi";

                outputVideo = new VideoWriter(video_name, VideoWriter.FourCC(@"MJPG"), 20, imgSize, true);

                RecordSwitch.Content = "录制结束(End)";
                record_start = true;
            }
        }

        private void RecordSwitch_End()
        {
            lock (record_lock)
            {
                record_start = false;
                outputVideo.Release();
                RecordSwitch.Content = "录制开始(Start)";
            }
        }

        void run_cap()
        {
            Mat src = new Mat();
            FrameSource frame;
            try
            {
                int cam_id = 0;

                for (int i = 0; i < 9; i++) 
                {
                    frame = Cv2.CreateFrameSource_Camera(i);
                    frame.NextFrame(src);
                    imgSize = src.Size();
                    if (imgSize.Height != 0) 
                    {
                        cam_id = i;
                        break;
                    }
                }

                frame = Cv2.CreateFrameSource_Camera(cam_id);
            }
            catch
            {
                init_flag = 1;
                return;
            }

            init_flag = 1;
            init_success = true;

            while (true)
            {
                frame.NextFrame(src);

                imgSize = src.Size();

                if (capture_save)
                {
                    DateTime t = DateTime.Now;
                    image_name = t.ToString("yyyy-MM-dd-HH-mm-ss");
                    image_name += ".jpg";

                    src.SaveImage(image_name);
                    capture_save = false;
                }

                lock (record_lock)
                {
                    if (record_start)
                    {
                        outputVideo.Write(src);
                    }
                }

                System.Drawing.Bitmap bitmap = BitmapConverter.ToBitmap(src);


                Action action1 = () =>
                {
                    iCamera.Source = BitmapToBitmapImage(bitmap);
                };
                this.Dispatcher.BeginInvoke(action1);
            }
        }

        public System.Windows.Media.Imaging.BitmapImage BitmapToBitmapImage(System.Drawing.Bitmap bitmap)
        {
            MemoryStream ms = new MemoryStream();
            bitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
            System.Windows.Media.Imaging.BitmapImage bit3 = new System.Windows.Media.Imaging.BitmapImage();
            bit3.BeginInit();
            bit3.StreamSource = ms;
            bit3.EndInit();
            return bit3;
        }
    }
}

