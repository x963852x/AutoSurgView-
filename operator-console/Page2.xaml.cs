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
using System.Threading;
using System.Windows.Shapes;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// Page2.xaml 的交互逻辑
    /// </summary>
    public partial class Page2 : Page
    {
        public void ChangeButton(Button sender, int t)
        {
            if (t == 0)
            {
                sender.IsEnabled = false;
                sender.Opacity = 0.5;
            }
            else
            {
                sender.IsEnabled = true;
                sender.Opacity = 1;
            }
        }

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

        public Page2()
        {
            InitializeComponent();
            List<MotionParameter> motionParameters1 = new List<MotionParameter>();

            Motor1_Left.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Motor1_Left_MouseLeftButtonDown), true);
            Motor1_Left.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Motor1_Left_MouseLeftButtonUp), true);
            Motor1_Right.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Motor1_Right_MouseLeftButtonDown), true);
            Motor1_Right.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Motor1_Right_MouseLeftButtonUp), true);

            Motor2_Left.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Motor2_Left_MouseLeftButtonDown), true);
            Motor2_Left.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Motor2_Left_MouseLeftButtonUp), true);
            Motor2_Right.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Motor2_Right_MouseLeftButtonDown), true);
            Motor2_Right.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Motor2_Right_MouseLeftButtonUp), true);

            Motor3_Left.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Motor3_Left_MouseLeftButtonDown), true);
            Motor3_Left.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Motor3_Left_MouseLeftButtonUp), true);
            Motor3_Right.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Motor3_Right_MouseLeftButtonDown), true);
            Motor3_Right.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Motor3_Right_MouseLeftButtonUp), true);

            //combox绑定设置
            motionParameters1.Add(new MotionParameter { Name = "低速(Low)", Speed = "1" });
            motionParameters1.Add(new MotionParameter { Name = "高速(High)", Speed = "100" });

            Motor1_SpeedCombo.ItemsSource = motionParameters1;
            Motor1_SpeedCombo.DisplayMemberPath = "Name";
            Motor1_SpeedCombo.SelectedValuePath = "Speed";
            Motor1_SpeedCombo.SelectedIndex = 0;

            ChangeButton(this.Motor1_Left, 0);
            ChangeButton(this.Motor1_Right, 0);
            ChangeButton(this.Motor2_Left, 0);
            ChangeButton(this.Motor2_Right, 0);
            ChangeButton(this.Motor3_Left, 0);
            ChangeButton(this.Motor3_Right, 0);

        }


        private void return_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage);
        }

        private void Motor1_SpeedCombo_DropDownClosed(object sender, EventArgs e)
        {
            if (this.Motor1_SpeedCombo.SelectedIndex == 0)
            {
                this.ParentPage.SendWord("a4");
            }
            else
                this.ParentPage.SendWord("a3");
        }

        private void Motor1_Left_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("a6");
        }

        private void Motor1_Left_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("a7");
        }

        private void Motor1_Right_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("a5");
        }

        private void Motor1_Right_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("a7");
        }

        private void Motor2_Left_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("s6");
        }

        private void Motor2_Left_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("s7");
        }

        private void Motor2_Right_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("s5");
        }

        private void Motor2_Right_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("s7");
        }

        private void Motor3_Left_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("d6");
        }

        private void Motor3_Right_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("d7");
        }

        private void Motor3_Left_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("d7");
        }

        private void Motor3_Right_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            this.ParentPage.SendWord("d5");
        }

        private void Motor1_Enable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("a1");
            this.Motor1_Enable.Background = Brushes.Purple;
            this.Motor1_Disnable.Background = Brushes.Gray;
            ChangeButton(this.Motor1_Left, 1);
            ChangeButton(this.Motor1_Right, 1);
        }

        private void Motor1_Disnable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("a2");
            this.Motor1_Enable.Background = Brushes.Gray;
            this.Motor1_Disnable.Background = Brushes.Purple;
            ChangeButton(this.Motor1_Left, 0);
            ChangeButton(this.Motor1_Right, 0);
        }

        private void Motor2_Enable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("s1");
            this.Motor2_Enable.Background = Brushes.Purple;
            this.Motor2_Disnable.Background = Brushes.Gray;
            ChangeButton(this.Motor2_Left, 1);
            ChangeButton(this.Motor2_Right, 1);
        }

        private void Motor2_Disnable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("s2");
            this.Motor2_Enable.Background = Brushes.Gray;
            this.Motor2_Disnable.Background = Brushes.Purple;
            ChangeButton(this.Motor2_Left, 0);
            ChangeButton(this.Motor2_Right, 0);
        }

        private void Motor3_Enable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("d1");
            this.Motor3_Enable.Background = Brushes.Purple;
            this.Motor3_Disnable.Background = Brushes.Gray;
            ChangeButton(this.Motor3_Left, 1);
            ChangeButton(this.Motor3_Right, 1);
        }

        private void Motor3_Disnable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("d2");
            this.Motor3_Enable.Background = Brushes.Gray;
            this.Motor3_Disnable.Background = Brushes.Purple;
            ChangeButton(this.Motor3_Left, 0);
            ChangeButton(this.Motor3_Right, 0);
        }

        private void Motor1_SpeedCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void advanced_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage.page6);
        }
    }
}
