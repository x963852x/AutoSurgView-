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
using System.Windows.Shapes;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// PageTest.xaml 的交互逻辑
    /// </summary>
    public partial class PageTest : Window
    {
        public int AutoMoveFlag = 0;
        public int MovePatternFlag = 1;
        public Page3 page3;
        public static PageTest topmost_page;
        public PageTest()
        {
            topmost_page = this;
            this.WindowStartupLocation = WindowStartupLocation.CenterScreen;
            InitializeComponent();
            ChangeGrid(1);
            this.ResizeMode = System.Windows.ResizeMode.NoResize;
            this.Closing += Window_Closing;
            // 设置全屏
            this.WindowState = System.Windows.WindowState.Normal;
            this.WindowStyle = System.Windows.WindowStyle.None;
            this.ResizeMode = System.Windows.ResizeMode.NoResize;
            //this.ResizeMode = System.Windows.ResizeMode.CanMinimize;
            this.Topmost = false;
            
            this.Left = 0.0;
            this.Top = 0.0;
            this.Width = System.Windows.SystemParameters.PrimaryScreenWidth;
            this.Height = System.Windows.SystemParameters.PrimaryScreenHeight;
        }

        public void ChangeGrid(int a)
        {
            if(a == 1)
            {
                this.MainGrid.RowDefinitions[0].Height = new System.Windows.GridLength(0);
            }
            else
            {
                this.MainGrid.RowDefinitions[0].Height = new System.Windows.GridLength(70);
            }
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            ChangeGrid(1);
        }

        private void FirstPage_Initialized(object sender, EventArgs e)
        {
            Page3 a = new Page3(AutoMoveFlag, MovePatternFlag);
            a.ParentWindow = this;
            this.FirstPage.Content = a;
            this.page3 = a;
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            page3.CloseUDPReceive();
            page3.page6.CloseUDPReceive();
        }
    }
}
