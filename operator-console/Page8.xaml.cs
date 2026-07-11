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
using System.Threading;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.Diagnostics;
namespace 手术机器人上位机程序
{
    /// <summary>
    /// Page8.xaml 的交互逻辑
    /// </summary>
    public partial class Page8 : Page
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

        public Page8()
        {
            InitializeComponent();
        }

        private void Return_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage);
        }

        private void open_soft_Click(object sender, RoutedEventArgs e)
        {
            //{
            //    Thread newThread = new Thread(() =>
            //    {
            //        IntPtr parent = IntPtr.Zero;
            //        this.Dispatcher.Invoke(() =>
            //        {
            //            parent = RealPlayWnd.Handle;

            //        });
            //        SetWindow.Find_external_Window_And_Embed(SetWindow.Record_Tool_form_name, parent, 4800);
            //    });
            //    newThread.Start();
            //}
            {
                Thread newThread = new Thread(() =>
                {
                    Record_Tool.kill_process("Record Tool", 500);
                    this.Dispatcher.Invoke(() =>
                    {
                        PageTest.topmost_page.Topmost = false;
                    });
                    Record_Tool.run_exe(@"C:\software\Record Tool\Record Tool\Record Tool.exe");
                    Thread.Sleep(2000);
                    bool continueloop = true;
                    while (true)
                    {
                        Process[] processes = Process.GetProcessesByName("Record Tool");
                        if (processes.Length == 0)  //没有进程
                        {
                            this.Dispatcher.Invoke(() =>
                            {
                                PageTest.topmost_page.Topmost = true;
                                continueloop = false;
                            });
                            if(continueloop == false)
                            {
                                break;
                            }
                            Thread.Sleep(50);
                        }
                    }
                });
                newThread.Start();
            }

        }
    }
}
