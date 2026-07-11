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
    /// Notice_2.xaml 的交互逻辑
    /// </summary>
    public partial class Notice_2 : Window
    {
        //父页面定义
        private Page3 _parentPage;
        public Page3 ParentPage
        {
            get { return _parentPage; }
            set { _parentPage = value; }
        }

        public Notice_2()
        {
            InitializeComponent();
            this.Closing += Window_Closing;
        }

        private void OK_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("2");
            this.Close();
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
