using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using System.Threading;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// Page9.xaml 的交互逻辑
    /// </summary>
    public partial class Page9 : Page
    {
        //无影灯模式
        private bool light_type_on = false;
        bool OP_focus_state = false;

        Mat blackImage = new Mat(540, 960, MatType.CV_8UC3, Scalar.Black);

        int flag = 0;
        //鼠标是否按下
        bool _isMouseDown = false;
        //鼠标按下的位置
        System.Windows.Point _mouseDownPosition;
        //鼠标按下控件的位置
        System.Windows.Point _mouseDownControlPosition;

        System.Windows.Point box_position;

        char thisState = 'S', lastState = 'S';

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
        public Page9()
        {
            InitializeComponent();
            this.touch.Visibility = Visibility.Hidden;
            this.box_view.Visibility = Visibility.Hidden;

            FocusFar.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.FocusFar_MouseLeftButtonDown), true);
            FocusFar.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.FocusFar_MouseLeftButtonUp), true);
            FocusNear.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.FocusNear_MouseLeftButtonDown), true);
            FocusNear.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.FocusNear_MouseLeftButtonUp), true);

            IrisUp.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.IrisUp_MouseLeftButtonDown), true);
            IrisUp.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.IrisUp_MouseLeftButtonUp), true);
            IrisDown.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.IrisDown_MouseLeftButtonDown), true);
            IrisDown.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.IrisDown_MouseLeftButtonUp), true);
        }

        private void return_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage);
            flag = 1;
        }

        private void read_video_Click(object sender, RoutedEventArgs e)
        {
            if (!this.ParentPage.read_video_state)
            {
                this.read_video.Content = "关闭视频读取";

                this.ParentPage.capture = new VideoCapture(this.ParentPage.CameraRealID);

                this.ParentPage.read_video_state = !this.ParentPage.read_video_state;
            }
            else
            {
                this.ParentPage.read_video_state = !this.ParentPage.read_video_state;
                this.ParentPage.capture.Release();
                Thread.Sleep(100);
                CameraPlayer.Image = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(blackImage);
                this.read_video.Content = "视频读取";
            }
        }

        private void touch_Down(object sender, MouseButtonEventArgs e)
        {
            var c = sender as Control;
            _isMouseDown = true;
            _mouseDownPosition = e.GetPosition(this);
            var transform = c.RenderTransform as TranslateTransform;
            if (transform == null)
            {
                transform = new TranslateTransform();
                c.RenderTransform = transform;
            }
            _mouseDownControlPosition = new System.Windows.Point(transform.X, transform.Y);

            var box_transform = box_view.RenderTransform as TranslateTransform;
            if (box_transform == null)
            {
                box_transform = new TranslateTransform();
                box_view.RenderTransform = box_transform;
            }
            box_position = new System.Windows.Point(box_transform.X, box_transform.Y);
            c.CaptureMouse();
        }

        private void touch_Move(object sender, MouseEventArgs e)
        {
            if (_isMouseDown)
            {
                var c = sender as Control;
                var pos = e.GetPosition(this);
                var dp = pos - _mouseDownPosition;
                var transform = c.RenderTransform as TranslateTransform;
                string dx, dy;

                //var box_position= box_view.TranslatePoint(new System.Windows.Point(0, 0), (UIElement)MainGrid);
                var box_transform = box_view.RenderTransform as TranslateTransform;
                if (box_transform == null)
                {
                    box_transform = new TranslateTransform();
                    box_view.RenderTransform = box_transform;
                }

                if (dp.X <= 400 && dp.Y <= 200 && dp.X >= -400 && dp.Y >= -200)
                {
                    transform.X = _mouseDownControlPosition.X + dp.X;
                    transform.Y = _mouseDownControlPosition.Y + dp.Y;

                    //box_transform.X = box_position.X + dp.X;
                    //box_transform.Y = box_position.Y + dp.Y;

                    dx = dp.X.ToString();
                    dy = dp.Y.ToString();
                    //Thread.Sleep(10);
                    //this.ParentPage.SendWord("e" + dx + "e" + dy);
                    if (dp.X < 0 && dp.X>-400 && System.Math.Abs(dp.X)> System.Math.Abs(-dp.Y))
                    {
                        thisState = 'L';
                    }
                    else if (dp.X < 400 && dp.X > 0 && System.Math.Abs(dp.X) > System.Math.Abs(-dp.Y))
                    {
                        thisState = 'R';
                    }
                    else if (-dp.Y < 200 && -dp.Y > 0 && System.Math.Abs(-dp.Y) > System.Math.Abs(dp.X))
                    {
                        thisState = 'U';
                    }
                    else if (-dp.Y < 0 && -dp.Y > -200 && System.Math.Abs(-dp.Y) > System.Math.Abs(dp.X))
                    {
                        thisState = 'D';
                    }
                    else
                    {
                        thisState = 'S';
                    }

                    if (thisState != lastState)
                    {
                        char joy1State = thisState;
                        //根据手柄1状态控制机械臂做出相应动作
                        switch (joy1State)
                        {
                            case 'U':
                                this.ParentPage.SendWord("eU");
                                break;
                            case 'D':
                                this.ParentPage.SendWord("eD");
                                break;
                            case 'L':
                                this.ParentPage.SendWord("eL");
                                break;
                            case 'R':
                                this.ParentPage.SendWord("eR");
                                break;
                            //case 'S':
                            //    this.ParentPage.SendWord("eS");
                            //    break;
                        }
                    }
                    lastState = thisState;
                }
                //transform.X = _mouseDownControlPosition.X + dp.X;
                //transform.Y = _mouseDownControlPosition.Y + dp.Y;
            }
        }

        private void view_Click(object sender, RoutedEventArgs e)
        {
            this.touch.Visibility = Visibility.Visible;
            this.box_view.Visibility = Visibility.Visible;
        }

        private void inview_Click(object sender, RoutedEventArgs e)
        {
            this.touch.Visibility = Visibility.Hidden;
            this.box_view.Visibility = Visibility.Hidden;
        }

        private void auto_position_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cd");//具体指令待定
        }

        private void FocusNear_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("cf2");
        }

        private void FocusFar_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("cf1");
        }

        private void FocusNear_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("cf0");
        }

        private void FocusFar_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("cf0");
        }

        private void IrisDown_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("ci2");
        }

        private void IrisUp_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("ci1");
        }

        private void IrisDown_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("ci0");
        }

        private void IrisUp_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("ci0");
        }

        private void auto_focus_Click(object sender, RoutedEventArgs e)
        {
            if (!OP_focus_state)
            {
                this.auto_position.IsEnabled = false;
                this.view_button.IsEnabled = false;
                this.inview_button.IsEnabled = false;
                this.visualservo.IsEnabled = false;
                this.auto_fouus.Content = "解除避障\r\n锁定拍摄";

                OP_focus_state = !OP_focus_state;

                this.ParentPage.SendWord("f3");
            }
            else
            {
                this.auto_position.IsEnabled = true;
                this.view_button.IsEnabled = true;
                this.inview_button.IsEnabled = true;
                this.visualservo.IsEnabled = true;
                this.auto_fouus.Content = "开启避障\r\n锁定拍摄";

                OP_focus_state = !OP_focus_state;

                this.ParentPage.SendWord("f4");
            }
        }

        private void visualservo_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("f5");
        }

        private void figure_save_Click(object sender, RoutedEventArgs e)
        {
            //*******************************************************************************************//
            //var capture = new VideoCapture(8);

            //Mat image = new Mat();
            //capture.Read(image);
            //if (!image.Empty())
            //{
            //    // 定义保存图像的文件路径
            //    string savePath = "C:/Users/Butel/Desktop/SSBOT2/shuye/shuye.jpg";
            //    // 保存图像到指定路径
            //    image.SaveImage(savePath);
            //}
            //capture.Release();
            //************************************************************************************************//
            System.Drawing.Bitmap bitmap = (System.Drawing.Bitmap)this.CameraPlayer.Image;

            bitmap.Save("C:/Users/Butel/Desktop/SSBOT2/shuye/output.jpg", System.Drawing.Imaging.ImageFormat.Jpeg);
        }

        private void touch_Up(object sender, MouseButtonEventArgs e)
        {
            var c = sender as Control;
            var transform = c.RenderTransform as TranslateTransform;
            _isMouseDown = false;
            c.ReleaseMouseCapture();
            transform.X = _mouseDownControlPosition.X;
            transform.Y = _mouseDownControlPosition.Y;

            var box_transform = box_view.RenderTransform as TranslateTransform;
            if (box_transform == null)
            {
                box_transform = new TranslateTransform();
                box_view.RenderTransform = box_transform;
            }
            box_transform.X = box_position.X;
            box_transform.Y = box_position.Y;
            //this.ParentPage.SendWord("ee");
            this.ParentPage.SendWord("eS");
        }
    }
}
