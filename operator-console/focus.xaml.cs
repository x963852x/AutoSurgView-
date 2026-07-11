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
using System.Configuration;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Windows.Interop;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// focus.xaml 的交互逻辑
    /// </summary>
    public partial class focus : Window
    {
        [DllImport("user32.dll")]
        static extern IntPtr GetSystemMenu(IntPtr hWnd, bool bRevert);
        [DllImport("user32.dll")]
        static extern bool EnableMenuItem(IntPtr hMenu, uint uIDEnableItem, uint uEnable);



        const uint MF_BYCOMMAND = 0x00000000;
        const uint MF_GRAYED = 0x00000001;
        const uint MF_ENABLED = 0x00000000;

        const uint SC_CLOSE = 0xF060;

        const int WM_SHOWWINDOW = 0x00000018;
        const int WM_CLOSE = 0x0000F060;


        bool focus_state = false;

        private Page3 _parentPage;
        public Page3 ParentPage
        {
            get { return _parentPage; }
            set { _parentPage = value; }
        }

        public focus()
        {
            InitializeComponent();
            this.Closing += Window_Closing;
            this.HeightBox.Text = ConfigurationManager.AppSettings["Height_focus"];
            this.set_focus_state.IsEnabled = false;
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.ParentPage.System_enflod.IsEnabled = true;
            this.ParentPage.System_unflod.IsEnabled = true;
            this.ParentPage.Debug.IsEnabled = true;
            this.ParentPage.Camera.IsEnabled = true;
            this.ParentPage.PTZ_Camera.IsEnabled = true;
            this.ParentPage.Focus.IsEnabled = true;
        }

        private void set_focus_state_Click(object sender, RoutedEventArgs e)
        {
            var hwnd = new WindowInteropHelper(this).Handle;  //获取window的句柄
            IntPtr hMenu = GetSystemMenu(hwnd, false);

            if (!focus_state)
            {
                EnableMenuItem(hMenu, SC_CLOSE, MF_GRAYED);

                this.set_focus_point.IsEnabled = false;
                this.return_button.IsEnabled = false;
                this.HeightBox.IsEnabled = false;
                this.set_focus_state.Content = "解除目标锁定";

                focus_state = !focus_state;

                this.ParentPage.SendWord("f1");
            }
            else
            {
                EnableMenuItem(hMenu, SC_CLOSE, MF_ENABLED);
                this.set_focus_point.IsEnabled = true;
                this.return_button.IsEnabled = true;
                this.HeightBox.IsEnabled = true;
                this.set_focus_state.Content = "开始目标锁定";

                focus_state = !focus_state;

                this.ParentPage.SendWord("f0");
            }
        }
        private void set_focus_point_Click(object sender, RoutedEventArgs e)
        {
            this.set_focus_state.IsEnabled = true;
            this.ParentPage.SendWord("f2" + this.HeightBox.Text);
        }
        private void return_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }

        private void HeightBox_MouseEnter(object sender, MouseEventArgs e)
        {

            //虚拟键盘尝试
            Process proc = new Process();
            proc.StartInfo.FileName = @"C:\Windows\System32\osk.exe";
            proc.StartInfo.UseShellExecute = true;
            proc.Start();
        }

    }
}
