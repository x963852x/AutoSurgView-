using System;
using System.Collections.Generic;
using System.Text;
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

namespace 手术机器人上位机程序
{
    /// <summary>
    /// Auto_up_and_down.xaml 的交互逻辑
    /// </summary>
    public partial class Auto_up_and_down : Window
    {
        //父页面定义
        private Page3 _parentPage;
        public Page3 ParentPage
        {
            get { return _parentPage; }
            set { _parentPage = value; }
        }

        //命令相关
        private RoutedCommand autoCmd = new RoutedCommand("AutoMove", typeof(Auto_up_and_down));

        private void InitializeCommand()
        {
            //把命令赋值给命令源并指定快捷键
            this.AutoMove.Command = this.autoCmd;
            this.autoCmd.InputGestures.Add(new KeyGesture(Key.C, ModifierKeys.Alt));

            //指定命令目标

            this.AutoMove.CommandTarget = this.MainGrid;

            CommandBinding cb = new CommandBinding();
            cb.Command = this.autoCmd;
            cb.CanExecute += new CanExecuteRoutedEventHandler(cb_CanExecute);
            cb.Executed += new ExecutedRoutedEventHandler(cb_Executed);

            //把命令关联到外围控件上
            this.MainGrid.CommandBindings.Add(cb);

        }

        public int Validate()//输入校验方法
        {
            double d = 0;
            if (double.TryParse(this.HeightBox.Text.ToString(), out d))
            {
                if (d >= 120 && d <= 190)
                {
                    return 1;
                }
            }

            return 0;
        }

        //当探测命令是否可执行时，此方法被调用
        void cb_CanExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            int v = Validate();
            if (v == 1)
            {
                e.CanExecute = true;
            }
            else
            {
                e.CanExecute = false;
            }

            e.Handled = true;
        }

        //当命令送达目标后，此方法被调用
        void cb_Executed(object sender, ExecutedRoutedEventArgs e)
        {
            //MessageBox.Show(this.HeightBox.Text);
            this.ParentPage.SendWord("z" + this.HeightBox.Text);
            Configuration cfa = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
            cfa.AppSettings.Settings["Height"].Value = this.HeightBox.Text;
            cfa.Save();
            ConfigurationManager.RefreshSection("appSettings");
            this.Close();
        }

        public Auto_up_and_down()
        {
            InitializeComponent();
            InitializeCommand();
            this.Closing += Window_Closing;
            this.HeightBox.Text = ConfigurationManager.AppSettings["Height"];
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

        private void HeightBox_MouseEnter(object sender, MouseEventArgs e)
        {

            //虚拟键盘尝试
            Process proc = new Process();
            proc.StartInfo.FileName = @"C:\Windows\System32\osk.exe";
            proc.StartInfo.UseShellExecute = true;
            proc.Start();
        }

        private void cancel_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }
    }
}
