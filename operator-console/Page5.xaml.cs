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
using System.Runtime.InteropServices;
using System.IO;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// Page5.xaml 的交互逻辑
    /// </summary>
    public partial class Page5 : Page
    {
        private bool light_type_on = false;
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

        public Page5()
        {
            InitializeComponent();

            List<MotionParameter> FocusMode_list = new List<MotionParameter>();
            FocusMode_list.Add(new MotionParameter { Name = "自动", Speed = "1" });
            FocusMode_list.Add(new MotionParameter { Name = "手动", Speed = "2" });

            List<MotionParameter> ExposureMode_list = new List<MotionParameter>();
            ExposureMode_list.Add(new MotionParameter { Name = "自动", Speed = "1" });
            ExposureMode_list.Add(new MotionParameter { Name = "手动", Speed = "2" });

            List<MotionParameter> WhiteBalanceMode_list = new List<MotionParameter>();
            WhiteBalanceMode_list.Add(new MotionParameter { Name = "自动", Speed = "1" });
            WhiteBalanceMode_list.Add(new MotionParameter { Name = "手动", Speed = "2" });

            yaw_Left.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.yaw_Left_MouseLeftButtonDown), true);
            yaw_Left.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.yaw_Left_MouseLeftButtonUp), true);
            yaw_Right.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.yaw_Right_MouseLeftButtonDown), true);
            yaw_Right.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.yaw_Right_MouseLeftButtonUp), true);

            pitch_Left.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.pitch_Left_MouseLeftButtonDown), true);
            pitch_Left.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.pitch_Left_MouseLeftButtonUp), true);
            pitch_Right.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.pitch_Right_MouseLeftButtonDown), true);
            pitch_Right.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.pitch_Right_MouseLeftButtonUp), true);

            roll_Left.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.roll_Left_MouseLeftButtonDown), true);
            roll_Left.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.roll_Left_MouseLeftButtonUp), true);
            roll_Right.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.roll_Right_MouseLeftButtonDown), true);
            roll_Right.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.roll_Right_MouseLeftButtonUp), true);


            ZoomIn.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.ZoomIn_MouseLeftButtonDown), true);
            ZoomIn.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.ZoomIn_MouseLeftButtonUp), true);
            ZoomOut.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.ZoomOut_MouseLeftButtonDown), true);
            ZoomOut.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.ZoomOut_MouseLeftButtonUp), true);
            FocusFar.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.FocusFar_MouseLeftButtonDown), true);
            FocusFar.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.FocusFar_MouseLeftButtonUp), true);
            FocusNear.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.FocusNear_MouseLeftButtonDown), true);
            FocusNear.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.FocusNear_MouseLeftButtonUp), true);

            IrisUp.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.IrisUp_MouseLeftButtonDown), true);
            IrisDown.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.IrisDown_MouseLeftButtonDown), true);
            IrisDown.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.IrisDown_MouseLeftButtonDown), true);
        }

        private void ZoomIn_MouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cz1");
        }

        private void ZoomIn_MouseLeftButtonUp(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cz0");
        }

        private void ZoomOut_MouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cz2");
        }

        private void ZoomOut_MouseLeftButtonUp(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cz0");
        }

        private void FocusFar_MouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cf1");
        }

        private void FocusFar_MouseLeftButtonUp(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cf0");
        }

        private void FocusNear_MouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cf2");
        }

        private void FocusNear_MouseLeftButtonUp(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("cf0");
        }

        private void IrisUp_MouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("ci1");
        }

        private void IrisDown_MouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("ci2");
        }

        private void light_Click(object sender, RoutedEventArgs e)
        {
            if(!this.light_type_on)
            {
                this.ParentPage.SendWord("cl1");
                light.Content = "关闭无影灯模式";
            }
            else
            {
                this.ParentPage.SendWord("cl2");
                light.Content = "开启无影灯模式";
            }

            this.light_type_on = !this.light_type_on;
        }

        private void return_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage);
        }

        private void pitch_Right_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t1");
        }

        private void pitch_Right_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t7");
        }

        private void yaw_Left_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t4");
        }

        private void yaw_Left_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t7");
        }

        private void yaw_Right_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t3");
        }

        private void yaw_Right_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t7");
        }

        private void pitch_Left_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t2");
        }

        private void pitch_Left_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t7");
        }

        private void roll_Left_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t6");
        }

        private void roll_Left_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t7");
        }

        private void roll_Right_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t5");
        }

        private void roll_Right_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("t7");
        }
    }
}
