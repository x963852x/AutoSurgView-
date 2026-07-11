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
    /// Notice.xaml 的交互逻辑
    /// </summary>
    public partial class Notice : Window
    {
        //父页面定义
        private Page3 _parentPage;
        public Page3 ParentPage
        {
            get { return _parentPage; }
            set { _parentPage = value; }
        }

        public Notice()
        {
            InitializeComponent();
            this.Closing += Window_Closing;
        }

        private void OK_Click(object sender, RoutedEventArgs e)
        {
            Auto_up_and_down wd1 = new Auto_up_and_down();
            wd1.WindowStartupLocation = WindowStartupLocation.CenterScreen;
            wd1.ParentPage = this.ParentPage;
            wd1.Show();
            wd1.Activate();
            wd1.Owner = this.ParentPage.ParentWindow;
            wd1.Topmost = true;
            this.Close();
            //主窗体按钮无效化
            this.ParentPage.System_enflod.IsEnabled = false;
            this.ParentPage.System_unflod.IsEnabled = false;
            this.ParentPage.Debug.IsEnabled = false;
            this.ParentPage.Camera.IsEnabled = false;
            this.ParentPage.PTZ_Camera.IsEnabled = false;
            this.ParentPage.Focus.IsEnabled = false;
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

        private void cancel_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }
    }
}
