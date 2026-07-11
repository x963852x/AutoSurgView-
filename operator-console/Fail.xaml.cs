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
    /// Fail.xaml 的交互逻辑
    /// </summary>
    public partial class Fail : Window
    {
                //父页面定义
        private Page3 _parentPage;
        public Page3 ParentPage
        {
            get { return _parentPage; }
            set { _parentPage = value; }
        }


        public Fail()
        {
            InitializeComponent();
            this.Closing += Window_Closing;
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.ParentPage.Debug.IsEnabled = true;
            this.ParentPage.System_enflod.IsEnabled = true;
            this.ParentPage.System_unflod.IsEnabled = true;
        }

        private void next_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.NavigationService.Navigate(this.ParentPage);
            this.Close();
        }
    }
}
