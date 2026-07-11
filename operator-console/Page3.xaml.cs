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
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Threading;
using System.Net.Sockets;
using System.Collections;
using System.Net;
using System.IO.Ports;
using System.Windows.Threading;
using System.IO;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using System.Text;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// Page3.xaml 的交互逻辑
    /// </summary>
    public class MotionParameter
    {
        public string Name { get; set; }
        public string Speed { get; set; }
    }
    public partial class Page3 : Page
    {
        public Dictionary<string, string> my_parameters = new Dictionary<string, string>();

        public Page2 page2;
        public Page4 page4;
        public Page5 page5;
        public Page6 page6;
        public Page7 page7;
        public Page8 page8;
        public Page9 page9;

        public System.Windows.Media.MediaPlayer player;

        private int RL_picture_save_count=0;

        private int sendflag = 0;
        private int recognizeflag = 0;
        public int CameraRealID = 0;
        public string cameraName;

        private UdpClient udpcSend;
        public UdpClient udpcRecv;
        Thread thrRecv;

        public Thread thrReadVideo;
        public VideoCapture capture;
        public bool read_video_state = false;

        public SerialPort myPort = new SerialPort();

        public void SendMessage(object obj)
        {
            string message = (string) obj;
            byte[] sendbytes = Encoding.ASCII.GetBytes(message);
            IPEndPoint remoteIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 6060);
            while(true)
            {
                if (sendflag == 1)
                {
                    udpcSend.Send(sendbytes, sendbytes.Length, remoteIpep);
                    Thread.Sleep(100);
                }
                else break;
            }
            udpcSend.Close();
        }

        public void SendSingleWord(object obj)
        {
            string message = (string)obj;
            byte[] sendbytes = Encoding.ASCII.GetBytes(message);
            IPEndPoint remoteIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 6060);
            udpcSend.Send(sendbytes, sendbytes.Length, remoteIpep);
            udpcSend.Close();
        }

        public void Send(string cmd)
        {
            IPEndPoint localIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 1234); // 本机IP，指定的端口号  
            udpcSend = new UdpClient(localIpep);
            Thread thrSend = new Thread(new ParameterizedThreadStart(SendMessage));
            thrSend.Start(cmd);
        }

        public void SendWord(string cmd)
        {
            IPEndPoint localIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 1234); // 本机IP，指定的端口号  
            udpcSend = new UdpClient(localIpep);
            Thread thrSend = new Thread(new ParameterizedThreadStart(SendSingleWord));
            thrSend.Start(cmd);
        }

        public string debug_text_string="";
        public void ReceiveMessage(object obj)
        {
            IPEndPoint remoteIpep = new IPEndPoint(IPAddress.Any, 0);
            while (true)
            {
                try
                {
                    byte[] bytRecv = udpcRecv.Receive(ref remoteIpep);
                    if(bytRecv.Length==4)
                    {
                        if(bytRecv[0]==0x0A && bytRecv[1] == 0x0B && bytRecv[3] == (bytRecv[0]+ bytRecv[1]+ bytRecv[2]))
                        {
                            switch(bytRecv[2])
                            {
                                case 0x00:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "急停按钮未旋起！";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;

                                                if (this.System_enflod.IsEnabled)
                                                {
                                                    ChangeButton(this.System_enflod, 0);
                                                    ChangeButton(this.System_unflod, 0);
                                                    ChangeButton(this.Debug, 0);
                                                    ChangeButton(this.PTZ_Camera, 0);
                                                    ChangeButton(this.Camera, 0);
                                                    ChangeButton(this.Focus, 0);
                                                    ChangeButton(this.Record_tool, 0);
                                                    ChangeButton(this.touch_control, 0);
                                                }
                                            }
                                            )
                                        );
                                    break;
                                case 0x06:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "机械臂初始化成功";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                this.Image_Source.Source= bmp;
                                                this.DebugText.Foreground = Brushes.Blue;
                                            }
                                            )
                                        );
                                    break;
                                case 0x01:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "机械臂处于非法位置";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/danger.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;

                                                if (this.System_enflod.IsEnabled)
                                                {
                                                    ChangeButton(this.System_enflod, 0);
                                                    ChangeButton(this.System_unflod, 0);
                                                    ChangeButton(this.Debug, 0);
                                                    ChangeButton(this.PTZ_Camera, 0);
                                                    ChangeButton(this.Camera, 0);
                                                    ChangeButton(this.Focus, 0);
                                                    ChangeButton(this.Record_tool, 0);
                                                    ChangeButton(this.touch_control, 0);
                                                }
                                            }
                                            )
                                        );
                                    break;
                                case 0x0B:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "机械臂处于非法位置，请手动调整";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;
                                            }
                                            )
                                        );
                                    break;
                                case 0x0C:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "机械臂已调整至合法位置";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Blue;
                                            }
                                            )
                                        );
                                    break;
                                case 0x08:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "3D相机初始化成功";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Blue;

                                                if(!this.Focus.IsEnabled)
                                                {
                                                    ChangeButton(this.System_enflod, 1);
                                                    ChangeButton(this.System_unflod, 1);
                                                    ChangeButton(this.Debug, 1);
                                                    ChangeButton(this.PTZ_Camera, 1);
                                                    ChangeButton(this.Camera, 1);
                                                    ChangeButton(this.Focus,1);
                                                    ChangeButton(this.Record_tool, 1);
                                                    ChangeButton(this.touch_control, 1);
                                                }
                                            }
                                            )
                                        );
                                    break;
                                case 0x0D:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "3D相机功能未开启";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Blue;

                                                if (!this.System_enflod.IsEnabled)
                                                {
                                                    ChangeButton(this.System_enflod, 1);
                                                    ChangeButton(this.System_unflod, 1);
                                                    ChangeButton(this.Debug, 1);
                                                    ChangeButton(this.PTZ_Camera, 1);
                                                    ChangeButton(this.Camera, 1);
                                                    ChangeButton(this.Record_tool, 1);
                                                    ChangeButton(this.touch_control, 1);
                                                }
                                            }
                                            )
                                        );
                                    break;
                                case 0x04:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "3D相机未识别到靶标位置，请调整靶标位姿或重试！";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;

                                                this.player.Open(new Uri(@"C:/software/music/靶标识别失败.mp3"));

                                                this.player.Play();
                                            }
                                            )
                                        );
                                    break;
                                case 0x05:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "靶标指向不可达观测位姿，请调整靶标位姿或重试！";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;

                                                this.player.Open(new Uri(@"C:/software/music/靶标指向位置不可达.mp3"));
                                                this.player.Play();
                                            }
                                            )
                                        );
                                    break;
                                case 0x09:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "3D相机识别靶标位置成功";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Blue;

                                                this.recognizeflag = 1;
                                            }
                                            )
                                        );
                                    break;
                                case 0x0A:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "靶标指向可达观测位姿，机械臂即将开始运动";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Blue;
                                            }
                                            )
                                        );
                                    break;
                                case 0x0E:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "路径规划超时，请确认机械臂周围环境是否过于拥挤";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;
                                            }
                                            )
                                        );
                                    break;
                                case 0x0F:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "路径规划失败，请确认机械臂当前状态或目标状态是否存在碰撞可能";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/warn.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;
                                            }
                                            )
                                        );
                                    break;
                                case 0x11:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "机械臂距离障碍物过近，已紧急停止";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/danger.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Red;

                                                this.player.Open(new Uri(@"C:/software/music/机械臂距离障碍物太近.mp3"));
                                                this.player.Play();
                                            }
                                            )
                                        );
                                    break;
                                case 0x10:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "路径规划成功，机械臂将开始运动";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Blue;

                                                if(this.recognizeflag==1)
                                                {
                                                    this.player.Open(new Uri(@"C:/software/music/靶标识别成功.mp3"));
                                                    this.player.Play();
                                                    Thread.Sleep(1600);
                                                    this.recognizeflag = 0;
                                                }

                                                this.player.Open(new Uri(@"C:/software/music/规划成功开始运动.mp3"));
                                                this.player.Play();
                                            }
                                            )
                                        );
                                    break;
                                case 0x12:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                if(!this.System_enflod.IsEnabled)
                                                {
                                                    this.DebugText.Text = "初始化成功";
                                                    BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                    this.Image_Source.Source = bmp;
                                                    this.DebugText.Foreground = Brushes.Blue;

                                                    ChangeButton(this.System_enflod, 1);
                                                    ChangeButton(this.System_unflod, 1);
                                                    ChangeButton(this.Debug, 1);
                                                    ChangeButton(this.PTZ_Camera, 1);
                                                    ChangeButton(this.Camera, 1);
                                                    ChangeButton(this.Focus, 1);
                                                    ChangeButton(this.Record_tool, 1);
                                                    ChangeButton(this.touch_control, 1);
                                                }
                                            }
                                            )
                                        );
                                    break;
                                case 0x13:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                if (!this.System_enflod.IsEnabled)
                                                {
                                                    this.DebugText.Text = "初始化成功";
                                                    BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                    this.Image_Source.Source = bmp;
                                                    this.DebugText.Foreground = Brushes.Blue;

                                                    ChangeButton(this.System_enflod, 1);
                                                    ChangeButton(this.System_unflod, 1);
                                                    ChangeButton(this.Debug, 1);
                                                    ChangeButton(this.PTZ_Camera, 1);
                                                    ChangeButton(this.Camera, 1);
                                                    ChangeButton(this.Record_tool, 1);
                                                    ChangeButton(this.touch_control, 1);
                                                }
                                            }
                                            )
                                        );
                                    break;
                                case 0x15:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                this.DebugText.Text = "图像采集完成";
                                                BitmapImage bmp = new BitmapImage(new Uri(@"C:/software/tubiao/normal.png"));
                                                this.Image_Source.Source = bmp;
                                                this.DebugText.Foreground = Brushes.Blue;

                                                this.player.Open(new Uri(@"C:/software/music/图像采集完成.mp3"));
                                                this.player.Play();
                                            }
                                            )
                                        );
                                    break;
                                case 0x16:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                var capture = new VideoCapture(8);

                                                Mat image = new Mat();
                                                capture.Read(image); 
                                                int col = image.Width;
                                                int rows = image.Height;
                                                Mat dst = new Mat();
                                                Cv2.Resize(image, dst, new OpenCvSharp.Size(2.2 * col, 1.6 * rows), 0, 0, InterpolationFlags.Cubic);
                                                if (!image.Empty())
                                                {
                                                    // 定义保存图像的文件路径
                                                    string savePath = "C:/Users/Butel/Desktop/SSBOT2/shuye/OA_start" + RL_picture_save_count.ToString() + ".jpg";
                                                    // 保存图像到指定路径
                                                    dst.SaveImage(savePath);
                                                }
                                                capture.Release();
                                            }
                                            )
                                        );
                                    break;
                                case 0x17:
                                    this.DebugText.Dispatcher.Invoke(
                                        new Action(
                                            delegate
                                            {
                                                var capture = new VideoCapture(8);

                                                Mat image = new Mat();
                                                capture.Read(image);
                                                int col = image.Width;
                                                int rows = image.Height;
                                                Mat dst = new Mat();
                                                Cv2.Resize(image, dst, new OpenCvSharp.Size(2.2 * col, 1.6 * rows), 0, 0, InterpolationFlags.Cubic);
                                                if (!image.Empty())
                                                {
                                                    // 定义保存图像的文件路径
                                                    string savePath = "C:/Users/Butel/Desktop/SSBOT2/shuye/OA_end" + RL_picture_save_count.ToString() + ".jpg";
                                                    // 保存图像到指定路径
                                                    dst.SaveImage(savePath);
                                                }
                                                capture.Release();

                                                RL_picture_save_count++;
                                            }
                                            )
                                        );
                                    break;
                            }
                        }
                    }
                }
                catch(Exception ex)
                {
                    break;
                }
            }
        }

        void readvideo()
        {
            //此处参考网上的读取方法
            int sleepTime=33;
            // 声明实例 Mat类
            Mat image = new Mat();
            Mat dst = new Mat();
            int col = 0, rows = 0;
            // 进入读取视频每镇的循环
            while (true)
            {
                Thread.Sleep(10);
                if (capture == null)
                    continue;
                if (!capture.IsOpened())
                    continue;
                capture.Read(image);
                //判断是否还有没有视频图像 
                if (image.Empty())
                    continue;

                //sleepTime = (int)Math.Round(1000 / capture.Fps);
                col = image.Width;
                rows = image.Height;

                //缩放视频大小
                Cv2.Resize(image, dst, new OpenCvSharp.Size(2.2 * col, 1.6 * rows), 0, 0, InterpolationFlags.Cubic);

                // 在picturebox中播放视频， 需要先转换成bitmap格式
                page9.CameraPlayer.Image = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(dst);

                Cv2.WaitKey(sleepTime-10);
            }
        }
        void InitializReadVideo()
        {
            thrReadVideo = new Thread(readvideo);
            thrReadVideo.IsBackground = true;
            thrReadVideo.Start();
        }
        public void CloseUDPReceive()
        {
            udpcRecv.Close();
        }

        void InitializeUDPReceivce()
        {
            IPEndPoint localIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 3001);
            udpcRecv = new UdpClient(localIpep);
            thrRecv = new Thread(ReceiveMessage);
            thrRecv.Start();
        }

        private PageTest _parentWin;
        public PageTest ParentWindow
        {
            get { return _parentWin; }
            set { _parentWin = value; }
        }

        private RoutedCommand autoCnd = new RoutedCommand("AutoMove", typeof(Page2));

        public void ChangeButton(Button sender, int t)
        {
            if(t == 0)
            {
                sender.IsEnabled = false;
                sender.Opacity = 0.2;
            }
            else
            {
                sender.IsEnabled = true;
                sender.Opacity = 1;
            }
        }

        public void ChangeMovePattern()
        {

        }

        private void MyPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string str = "";
            int n = myPort.BytesToRead;
            byte[] buf = new byte[n];
            try
            {
                if(n != 0)
                {
                    myPort.Read(buf, 0, n);
                    str = Encoding.ASCII.GetString(buf);
                    //if (buf[0] == 0x31 && buf[1] == 0x63 && n == 16)
                    //{
                    //    IPEndPoint localIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 1234); // 本机IP，指定的端口号  
                    //    udpcSend = new UdpClient(localIpep);
                    //    IPEndPoint remoteIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 6060);
                    //    udpcSend.Send(buf, buf.Length, remoteIpep);
                    //    udpcSend.Close();
                    //    myPort.DiscardInBuffer();
                    //}
                    if(str == "1c")
                    {
                        SendWord("1c");
                        myPort.DiscardInBuffer();
                    }
                    if (str == "4c")
                    {
                        SendWord("3");
                        myPort.DiscardInBuffer();
                    }
                    if (str == "5c")
                    {
                        SendWord("4");
                        myPort.DiscardInBuffer();
                    }
                    if (str == "2c")
                    {
                        SendWord("5");
                        myPort.DiscardInBuffer();
                    }
                    if (str == "3c")
                    {
                        SendWord("6");
                        myPort.DiscardInBuffer();
                    }
                    if (str == "6c")
                    {
                        SendWord("7");
                        myPort.DiscardInBuffer();
                    }
                }
            }
            catch
            {

            }
        }

        private void InitializeCommand()
        {
            ChangeButton(this.System_enflod, 0);
            ChangeButton(this.System_unflod, 0);
            ChangeButton(this.Debug, 0);
            ChangeButton(this.PTZ_Camera, 0);
            ChangeButton(this.Camera, 0);
            ChangeButton(this.Focus, 0);
            ChangeButton(this.Record_tool, 0);
            ChangeButton(this.touch_control, 0);
        }

        public Page3(int AutoMoveFlag, int MovePatternFlag)
        {
            InitializeComponent();
            InitializeCommand();
            InitializeUDPReceivce();
            page2 = new Page2();
            page2.ParentWindow = this.ParentWindow;
            page2.ParentPage = this;

            page4 = new Page4();
            page4.ParentWindow = this.ParentWindow;
            page4.ParentPage = this;

            page5 = new Page5();
            page5.ParentWindow = this.ParentWindow;
            page5.ParentPage = this;

            page6 = new Page6();
            page6.ParentWindow = this.ParentWindow;
            page6.ParentPage = this;

            page7 = new Page7();
            page7.ParentWindow = this.ParentWindow;
            page7.ParentPage = this;

            page8 = new Page8();
            page8.ParentWindow = this.ParentWindow;
            page8.ParentPage = this;

            page9 = new Page9();
            page9.ParentWindow = this.ParentWindow;
            page9.ParentPage = this;

            //readvideoID();
            InitializReadVideo();

            player = new System.Windows.Media.MediaPlayer();

            FileStream fs = new FileStream("C:\\software\\ssbot_lib\\params_vis.txt", FileMode.Open);
            byte[] array = new byte[fs.Length];//初始化字节数组
            fs.Read(array, 0, array.Length);  //读取流中数据到字节数组中
            fs.Close();//关闭流

            string str = Encoding.Default.GetString(array);//将字节数组转化为
            string []strs = str.Split();

            for (int i = 0; i < strs.Length; i++) 
            {
                if(i+1 == strs.Length)
                {
                    break;
                }

                my_parameters[strs[i]] = strs[i + 1];
                i += 2;
            }

            myPort.BaudRate = 9600;
            myPort.DataBits = 8;
            myPort.PortName = my_parameters["serial"];
            myPort.Open();
            myPort.DataReceived += new SerialDataReceivedEventHandler(MyPort_DataReceived);

            CameraRealID= int.Parse(my_parameters["cameraID"]);

            List<MotionParameter> motionParameters = new List<MotionParameter>();

            motionParameters.Add(new MotionParameter { Name = "低速", Speed = "1" });
            motionParameters.Add(new MotionParameter { Name = "中速", Speed = "10" });
            motionParameters.Add(new MotionParameter { Name = "高速", Speed = "100" });
        }

        private void readvideoID()
        {
            int cameraID = 0;
            Mat testimage = new Mat();
            while (true)
            {
                VideoCapture cap = new VideoCapture(cameraID);

                cap.Read(testimage);
                if (!testimage.Empty())
                {
                    // 如果无法打开该 ID 的摄像头，退出循环
                    if(cap.GetBackendName() == "MSMF")
                    {
                        CameraRealID = cameraID;
                        cap.Release();
                        break;
                    }
                }
                cameraID++;
                cap.Release();
            }
        }
        private void Debug_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(page2);
        }

        private void System_enflod_Click(object sender, RoutedEventArgs e)
        {
            //弹出提示窗
            Notice_2 wd1 = new Notice_2();
            wd1.WindowStartupLocation = WindowStartupLocation.CenterScreen;
            wd1.ParentPage = this;
            wd1.Show();
            wd1.Activate();
            wd1.Owner = this.ParentWindow;
            wd1.Topmost = true;
            //主窗体按钮无效化
            this.System_enflod.IsEnabled = false;
            this.System_unflod.IsEnabled = false;
            this.Debug.IsEnabled = false;
            this.Camera.IsEnabled = false;
            this.PTZ_Camera.IsEnabled = false;
            this.Focus.IsEnabled = false;
        }

        private void System_unflod_Click(object sender, RoutedEventArgs e)
        {
            Notice wdl = new Notice();
            wdl.WindowStartupLocation = WindowStartupLocation.CenterScreen;
            wdl.ParentPage = this;
            wdl.Show();
            wdl.Activate();
            wdl.Owner = this.ParentWindow;
            wdl.Topmost = true;
            this.System_enflod.IsEnabled = false;
            this.System_unflod.IsEnabled = false;
            this.Debug.IsEnabled = false;
            this.Camera.IsEnabled = false;
            this.PTZ_Camera.IsEnabled = false;
            this.Focus.IsEnabled = false;
        }

        private void PTZ_Camera_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(page4);
        }

        private void Camera_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(page5);
        }

        private void Focus_Click(object sender, RoutedEventArgs e)
        {
            focus win_focus = new focus();
            win_focus.WindowStartupLocation = WindowStartupLocation.CenterScreen;
            win_focus.ParentPage = this;
            win_focus.Show();
            win_focus.Activate();
            win_focus.Owner = this.ParentWindow;
            win_focus.Topmost = true;
            this.System_enflod.IsEnabled = false;
            this.System_unflod.IsEnabled = false;
            this.Debug.IsEnabled = false;
            this.Camera.IsEnabled = false;
            this.PTZ_Camera.IsEnabled = false;
            this.Focus.IsEnabled = false;

        }

        private void Record_Tool_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(page8);
        }

        private void Touch_Control_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(page9);
        }
    }
}
