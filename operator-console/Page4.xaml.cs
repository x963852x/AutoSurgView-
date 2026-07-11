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
    /// Page4.xaml 的交互逻辑
    /// </summary>
    public partial class Page4 : Page
    {

        private uint iLastErr = 0;
        private Int32 m_lUserID = -1;
        private bool m_bInitSDK = false;
        private bool m_bRecord = false;
        private bool m_bTalk = false;
        private Int32 m_lRealHandle = -1;
        private int lVoiceComHandle = -1;
        private string str;

        public int m_lChannel = 1;
        private bool bAuto = false;
        public int PreSetNo = 0;
        public CHCNetSDK.NET_DVR_PTZPOS m_struPtzCfg;
        public CHCNetSDK.NET_DVR_PTZSCOPE m_struPtzCfg1;
        public CHCNetSDK.NET_DVR_PRESET_NAME[] m_struPreSetCfg = new CHCNetSDK.NET_DVR_PRESET_NAME[300];


        CHCNetSDK.REALDATACALLBACK RealData = null;
        CHCNetSDK.LOGINRESULTCALLBACK LoginCallBack = null;
        public CHCNetSDK.NET_DVR_USER_LOGIN_INFO struLogInfo;
        public CHCNetSDK.NET_DVR_DEVICEINFO_V40 DeviceInfo;

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
        public Page4()
        {
            InitializeComponent();

            List<MotionParameter> PTZ_speed_list = new List<MotionParameter>();
            PTZ_speed_list.Add(new MotionParameter { Name = "1", Speed = "1" });
            PTZ_speed_list.Add(new MotionParameter { Name = "2", Speed = "2" });
            PTZ_speed_list.Add(new MotionParameter { Name = "3", Speed = "3" });
            PTZ_speed_list.Add(new MotionParameter { Name = "4", Speed = "4" });
            PTZ_speed_list.Add(new MotionParameter { Name = "5", Speed = "5" });
            PTZ_speed_list.Add(new MotionParameter { Name = "6", Speed = "6" });
            PTZ_speed_list.Add(new MotionParameter { Name = "7", Speed = "7" });

            PTZ_SpeedCombo.ItemsSource = PTZ_speed_list;
            PTZ_SpeedCombo.DisplayMemberPath = "Name";
            PTZ_SpeedCombo.SelectedValuePath = "Speed";
            PTZ_SpeedCombo.SelectedIndex = 2;

            List<MotionParameter> Operate_type_list = new List<MotionParameter>();
            Operate_type_list.Add(new MotionParameter { Name = "1-定位PTZ参数", Speed = "1" });
            Operate_type_list.Add(new MotionParameter { Name = "2-定位P参数", Speed = "2" });
            Operate_type_list.Add(new MotionParameter { Name = "3-定位T参数", Speed = "3" });
            Operate_type_list.Add(new MotionParameter { Name = "4-定位Z参数", Speed = "4" });
            Operate_type_list.Add(new MotionParameter { Name = "5-定位PT参数", Speed = "5" });

            Operate_TypeCombo.ItemsSource = Operate_type_list;
            Operate_TypeCombo.DisplayMemberPath = "Name";
            Operate_TypeCombo.SelectedValuePath = "Speed";
            Operate_TypeCombo.SelectedIndex = 0;

            Up.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Up_MouseLeftButtonDown), true);
            Up.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Up_MouseLeftButtonUp), true);
            Down.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Down_MouseLeftButtonDown), true);
            Down.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Down_MouseLeftButtonUp), true);
            Right.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Right_MouseLeftButtonDown), true);
            Right.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Right_MouseLeftButtonUp), true);
            Left.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.Left_MouseLeftButtonDown), true);
            Left.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.Left_MouseLeftButtonUp), true);
            ZoomIn.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.ZoomIn_MouseLeftButtonDown), true);
            ZoomIn.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.ZoomIn_MouseLeftButtonUp), true);
            ZoomOut.AddHandler(Button.MouseLeftButtonDownEvent, new MouseButtonEventHandler(this.ZoomOut_MouseLeftButtonDown), true);
            ZoomOut.AddHandler(Button.MouseLeftButtonUpEvent, new MouseButtonEventHandler(this.ZoomOut_MouseLeftButtonUp), true);

            m_bInitSDK = CHCNetSDK.NET_DVR_Init();
            if (m_bInitSDK == false)
            {
                MessageBox.Show("NET_DVR_Init error!");
                return;
            }
            else
            {
                //保存SDK日志 To save the SDK log
                CHCNetSDK.NET_DVR_SetLogToFile(3, "C:\\SdkLog\\", true);
            }
        }

        private void return_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage);
        }

        private void switch_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage.page7);
        }


        private void Up_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, CHCNetSDK.TILT_UP, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.TILT_UP, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void Up_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, CHCNetSDK.TILT_UP, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.TILT_UP, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void Down_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, CHCNetSDK.TILT_DOWN, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.TILT_DOWN, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void Down_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, CHCNetSDK.TILT_DOWN, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.TILT_DOWN, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void Right_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.PAN_RIGHT, 0, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.PAN_RIGHT, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void Right_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.PAN_RIGHT, 1, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.PAN_RIGHT, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void Left_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.PAN_LEFT, 0, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.PAN_LEFT, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void Left_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.PAN_LEFT, 1, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.PAN_LEFT, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void ZoomIn_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.ZOOM_IN, 0, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.ZOOM_IN, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void ZoomIn_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.ZOOM_IN, 1, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.ZOOM_IN, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void ZoomOut_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.ZOOM_OUT, 0, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.ZOOM_OUT, 0, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }

        private void ZoomOut_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (m_lRealHandle > -1)
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed(m_lRealHandle, (uint)CHCNetSDK.ZOOM_OUT, 1, (uint)(PTZ_SpeedCombo.SelectedIndex + 1));
            }
            else
            {
                CHCNetSDK.NET_DVR_PTZControlWithSpeed_Other(m_lUserID, m_lChannel, CHCNetSDK.ZOOM_OUT, 1, (uint)PTZ_SpeedCombo.SelectedIndex + 1);
            }
        }



        private void PTZ_SpeedCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void PTZ_SpeedCombo_DropDownClosed(object sender, EventArgs e)
        {

        }

        private void T_Text_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void P_Text_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void PtzSet_Click(object sender, RoutedEventArgs e)
        {
            int flag = 1;
            float wPanPos, wTiltPos, wZoomPos;
            String str1, str2, str3;
            if (Operate_TypeCombo.Text == "")
            {
                MessageBox.Show("Please input the operation type  ");
            }
            /* wPanPos = ushort.Parse(textBoxPanPos.Text);
             wTiltPos = ushort.Parse(textBoxTiltPos.Text);
             wZoomPos = ushort.Parse(textBoxZoomPos.Text);*/
            switch (Operate_TypeCombo.SelectedIndex)//下拉框中的数据
            {
                case 0:

                    if (P_Text.Text == "" || T_Text.Text == "" ||
                        Z_Text.Text == "")
                    {

                        MessageBox.Show("Please input prarameters of P,T,Z: ");
                        return;
                    }
                    else
                    {
                        flag = 0;
                        m_struPtzCfg.wAction = 1;
                        str1 = Convert.ToString(float.Parse(P_Text.Text) * 10);
                        m_struPtzCfg.wPanPos = (ushort)(Convert.ToUInt16(str1, 16));
                        str2 = Convert.ToString(float.Parse(T_Text.Text) * 10);
                        m_struPtzCfg.wTiltPos = (ushort)(Convert.ToUInt16(str2, 16));
                        str3 = Convert.ToString(float.Parse(Z_Text.Text) * 10);
                        m_struPtzCfg.wZoomPos = (ushort)(Convert.ToUInt16(str3, 16));
                    }
                    break;
                case 1:
                    if (P_Text.Text == "")
                    {
                        MessageBox.Show("Please input prarameters of P: ");
                        return;
                    }
                    else
                    {
                        flag = 0;
                        m_struPtzCfg.wAction = 2;

                        //wPanPos = float.Parse(textBoxPanPos.Text);
                        str1 = Convert.ToString(float.Parse(P_Text.Text) * 10);
                        m_struPtzCfg.wPanPos = (ushort)(Convert.ToUInt16(str1, 16));

                        //m_struPtzCfg.wPanPos = ushort.Parse(textBoxPanPos.Text);

                    }
                    break;
                case 2:
                    if (T_Text.Text == "")
                    {
                        MessageBox.Show("Please input prarameters of T: ");
                        return;
                    }
                    else
                    {
                        flag = 0;
                        m_struPtzCfg.wAction = 3;
                        m_struPtzCfg.wTiltPos = ushort.Parse(T_Text.Text);

                        str2 = Convert.ToString(float.Parse(T_Text.Text) * 10);
                        m_struPtzCfg.wTiltPos = (ushort)(Convert.ToUInt16(str2, 16));

                    }
                    break;
                case 3:
                    if (Z_Text.Text == "")
                    {
                        MessageBox.Show("Please input prarameters of Z: ");
                        return;
                    }
                    else
                    {
                        flag = 0;
                        m_struPtzCfg.wAction = 4;

                        str3 = Convert.ToString(float.Parse(Z_Text.Text) * 10);
                        m_struPtzCfg.wZoomPos = (ushort)(Convert.ToUInt16(str3, 16));
                    }
                    break;
                case 4:
                    if (T_Text.Text == "" || P_Text.Text == "")
                    {
                        MessageBox.Show("Please input prarameters of P,T: ");
                        return;
                    }
                    else
                    {
                        flag = 0;
                        m_struPtzCfg.wAction = 5;
                        m_struPtzCfg.wPanPos = ushort.Parse(P_Text.Text);
                        m_struPtzCfg.wTiltPos = ushort.Parse(T_Text.Text);

                        str1 = Convert.ToString(float.Parse(P_Text.Text) * 10);
                        m_struPtzCfg.wPanPos = (ushort)(Convert.ToUInt16(str1, 16));
                        str2 = Convert.ToString(float.Parse(T_Text.Text) * 10);
                        m_struPtzCfg.wTiltPos = (ushort)(Convert.ToUInt16(str2, 16));

                    }
                    break;


            }


            while (flag == 0)
            {

                Int32 nSize = Marshal.SizeOf(m_struPtzCfg);
                IntPtr ptrPtzCfg = Marshal.AllocHGlobal(nSize);
                Marshal.StructureToPtr(m_struPtzCfg, ptrPtzCfg, false);

                if (!CHCNetSDK.NET_DVR_SetDVRConfig(m_lUserID, CHCNetSDK.NET_DVR_SET_PTZPOS, 1, ptrPtzCfg, (UInt32)nSize))
                {
                    iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                    str = "NET_DVR_SetDVRConfig failed, error code= " + iLastErr;
                    //设置POS参数失败
                    MessageBox.Show(str);
                    return;
                }
                else
                {
                    MessageBox.Show("设置成功!");
                    break;
                }

                Marshal.FreeHGlobal(ptrPtzCfg);

            }
            return;
        }

        private void PtzGet_Click(object sender, RoutedEventArgs e)
        {
            UInt32 dwReturn = 0;
            Int32 nSize = Marshal.SizeOf(m_struPtzCfg);
            IntPtr ptrPtzCfg = Marshal.AllocHGlobal(nSize);
            Marshal.StructureToPtr(m_struPtzCfg, ptrPtzCfg, false);
            //获取参数失败
            if (!CHCNetSDK.NET_DVR_GetDVRConfig(m_lUserID, CHCNetSDK.NET_DVR_GET_PTZPOS, -1, ptrPtzCfg, (UInt32)nSize, ref dwReturn))
            {
                iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                str = "NET_DVR_GetDVRConfig failed, error code= " + iLastErr;
                MessageBox.Show(str);
                return;
            }
            else
            {
                m_struPtzCfg = (CHCNetSDK.NET_DVR_PTZPOS)Marshal.PtrToStructure(ptrPtzCfg, typeof(CHCNetSDK.NET_DVR_PTZPOS));
                //成功获取显示ptz参数
                ushort wPanPos = Convert.ToUInt16(Convert.ToString(m_struPtzCfg.wPanPos, 16));
                float WPanPos = wPanPos * 0.1f;
                P_Text.Text = Convert.ToString(WPanPos);
                ushort wTiltPos = Convert.ToUInt16(Convert.ToString(m_struPtzCfg.wTiltPos, 16));
                float WTiltPos = wTiltPos * 0.1f;
                T_Text.Text = Convert.ToString(WTiltPos);
                ushort wZoomPos = Convert.ToUInt16(Convert.ToString(m_struPtzCfg.wZoomPos, 16));
                float WZoomPos = wZoomPos * 0.1f;
                Z_Text.Text = Convert.ToString(WZoomPos);
            }
            return;
        }

        public void cbLoginCallBack(int lUserID, int dwResult, IntPtr lpDeviceInfo, IntPtr pUser)
        {
            string strLoginCallBack = "登录设备，lUserID：" + lUserID + "，dwResult：" + dwResult;

            if (dwResult == 0)
            {
                uint iErrCode = CHCNetSDK.NET_DVR_GetLastError();
                strLoginCallBack = strLoginCallBack + "，错误号:" + iErrCode;
            }
        }

        private void Login_Click(object sender, RoutedEventArgs e)
        {
            if (m_lUserID < 0)
            {

                struLogInfo = new CHCNetSDK.NET_DVR_USER_LOGIN_INFO();

                //设备IP地址或者域名
                byte[] byIP = System.Text.Encoding.Default.GetBytes(this.ParentPage.my_parameters["hikvision_ip"]);
                struLogInfo.sDeviceAddress = new byte[129];
                byIP.CopyTo(struLogInfo.sDeviceAddress, 0);

                //设备用户名
                byte[] byUserName = System.Text.Encoding.Default.GetBytes(this.ParentPage.my_parameters["hikvision_name"]);
                struLogInfo.sUserName = new byte[64];
                byUserName.CopyTo(struLogInfo.sUserName, 0);

                //设备密码
                byte[] byPassword = System.Text.Encoding.Default.GetBytes(this.ParentPage.my_parameters["hikvision_password"]);
                struLogInfo.sPassword = new byte[64];
                byPassword.CopyTo(struLogInfo.sPassword, 0);

                struLogInfo.wPort = ushort.Parse("8000");//设备服务端口号

                if (LoginCallBack == null)
                {
                    LoginCallBack = new CHCNetSDK.LOGINRESULTCALLBACK(cbLoginCallBack);//注册回调函数                    
                }
                struLogInfo.cbLoginResult = LoginCallBack;
                struLogInfo.bUseAsynLogin = false; //是否异步登录：0- 否，1- 是 

                DeviceInfo = new CHCNetSDK.NET_DVR_DEVICEINFO_V40();

                //登录设备 Login the device
                m_lUserID = CHCNetSDK.NET_DVR_Login_V40(ref struLogInfo, ref DeviceInfo);
                if (m_lUserID < 0)
                {
                    iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                    str = "NET_DVR_Login_V40 failed, error code= " + iLastErr; //登录失败，输出错误号
                    MessageBox.Show(str);
                    return;
                }
                else
                {
                    //登录成功
                    MessageBox.Show("Login Success!");
                    Login.Content = "登出(Logout)";
                }

            }
            else
            {
                //注销登录 Logout the device
                if (m_lRealHandle >= 0)
                {
                    MessageBox.Show("Please stop live view firstly");
                    return;
                }

                if (!CHCNetSDK.NET_DVR_Logout(m_lUserID))
                {
                    iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                    str = "NET_DVR_Logout failed, error code= " + iLastErr;
                    MessageBox.Show(str);
                    return;
                }
                m_lUserID = -1;
                Login.Content = "登录(Login)";
            }
            return;
        }

        private void Preview_Click(object sender, RoutedEventArgs e)
        {
            if (m_lUserID < 0)
            {
                MessageBox.Show("Please login the device firstly");
                return;
            }

            if (m_lRealHandle < 0)
            {
                CHCNetSDK.NET_DVR_PREVIEWINFO lpPreviewInfo = new CHCNetSDK.NET_DVR_PREVIEWINFO();
                lpPreviewInfo.hPlayWnd = RealPlayWnd.Handle;//预览窗口
                lpPreviewInfo.lChannel = Int16.Parse("1");//预te览的设备通道
                lpPreviewInfo.dwStreamType = 0;//码流类型：0-主码流，1-子码流，2-码流3，3-码流4，以此类推
                lpPreviewInfo.dwLinkMode = 0;//连接方式：0- TCP方式，1- UDP方式，2- 多播方式，3- RTP方式，4-RTP/RTSP，5-RSTP/HTTP 
                lpPreviewInfo.bBlocked = true; //0- 非阻塞取流，1- 阻塞取流
                lpPreviewInfo.dwDisplayBufNum = 1; //播放库播放缓冲区最大缓冲帧数
                lpPreviewInfo.byProtoType = 0;
                lpPreviewInfo.byPreviewMode = 0;


                if (RealData == null)
                {
                    RealData = new CHCNetSDK.REALDATACALLBACK(RealDataCallBack);//预览实时流回调函数
                }

                IntPtr pUser = new IntPtr();//用户数据

                //打开预览 Start live view 
                m_lRealHandle = CHCNetSDK.NET_DVR_RealPlay_V40(m_lUserID, ref lpPreviewInfo, null/*RealData*/, pUser);
                if (m_lRealHandle < 0)
                {
                    iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                    str = "NET_DVR_RealPlay_V40 failed, error code= " + iLastErr; //预览失败，输出错误号
                    MessageBox.Show(str);
                    return;
                }
                else
                {
                    //预览成功
                    Preview.Content = "停止预览(Stop)";
                }
            }
            else
            {
                //停止预览 Stop live view 
                if (!CHCNetSDK.NET_DVR_StopRealPlay(m_lRealHandle))
                {
                    iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                    str = "NET_DVR_StopRealPlay failed, error code= " + iLastErr;
                    MessageBox.Show(str);
                    return;
                }
                m_lRealHandle = -1;
                Preview.Content = "预览(Preview)";

            }
            return;
        }

        public void RealDataCallBack(Int32 lRealHandle, UInt32 dwDataType, IntPtr pBuffer, UInt32 dwBufSize, IntPtr pUser)
        {
            if (dwBufSize > 0)
            {
                byte[] sData = new byte[dwBufSize];
                Marshal.Copy(pBuffer, sData, 0, (Int32)dwBufSize);

                string str = "实时流数据.ps";
                FileStream fs = new FileStream(str, FileMode.Create);
                int iLen = (int)dwBufSize;
                fs.Write(sData, 0, iLen);
                fs.Close();
            }
        }

        private void Jpeg_Click(object sender, RoutedEventArgs e)
        {
            string sJpegPicFileName;
            //图片保存路径和文件名 the path and file name to save
            //sJpegPicFileName = "JPEG_test.jpg";

            DateTime t = DateTime.Now;
            sJpegPicFileName = t.ToString("yyyy-MM-dd-HH-mm-ss");
            sJpegPicFileName += ".jpg";

            int lChannel = Int16.Parse("1"); //通道号 Channel number

            CHCNetSDK.NET_DVR_JPEGPARA lpJpegPara = new CHCNetSDK.NET_DVR_JPEGPARA();
            lpJpegPara.wPicQuality = 0; //图像质量 Image quality
            lpJpegPara.wPicSize = 0xff; //抓图分辨率 Picture size: 2- 4CIF，0xff- Auto(使用当前码流分辨率)，抓图分辨率需要设备支持，更多取值请参考SDK文档

            //JPEG抓图 Capture a JPEG picture
            if (!CHCNetSDK.NET_DVR_CaptureJPEGPicture(m_lUserID, lChannel, ref lpJpegPara, sJpegPicFileName))
            {
                iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                str = "NET_DVR_CaptureJPEGPicture failed, error code= " + iLastErr;
                MessageBox.Show(str);
                return;
            }
            else
            {
                str = "Successful to capture the JPEG file and the saved file is " + sJpegPicFileName;
                MessageBox.Show(str);
            }
            return;
        }

        private void Record_Click(object sender, RoutedEventArgs e)
        {
            //录像保存路径和文件名 the path and file name to save
            string sVideoFileName;
            //sVideoFileName = "Record_test.mp4";

            DateTime t = DateTime.Now;
            sVideoFileName = t.ToString("yyyy-MM-dd-HH-mm-ss");
            sVideoFileName += ".mp4";

            if (m_bRecord == false)
            {
                //强制I帧 Make a I frame
                int lChannel = Int16.Parse("1"); //通道号 Channel number
                CHCNetSDK.NET_DVR_MakeKeyFrame(m_lUserID, lChannel);

                //开始录像 Start recording
                if (!CHCNetSDK.NET_DVR_SaveRealData(m_lRealHandle, sVideoFileName))
                {
                    iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                    str = "NET_DVR_SaveRealData failed, error code= " + iLastErr;
                    MessageBox.Show(str);
                    return;
                }
                else
                {
                    Record.Content = "停止录像";
                    m_bRecord = true;
                }
            }
            else
            {
                //停止录像 Stop recording
                if (!CHCNetSDK.NET_DVR_StopSaveRealData(m_lRealHandle))
                {
                    iLastErr = CHCNetSDK.NET_DVR_GetLastError();
                    str = "NET_DVR_StopSaveRealData failed, error code= " + iLastErr;
                    MessageBox.Show(str);
                    return;
                }
                else
                {
                    str = "Successful to stop recording and the saved file is " + sVideoFileName;
                    MessageBox.Show(str);
                    Record.Content = "录像(Record)";
                    m_bRecord = false;
                }
            }

            return;
        }

        private void Operate_TypeComboCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void Operate_TypeCombo_DropDownClosed(object sender, EventArgs e)
        {

        }

        private void release_Click(object sender, RoutedEventArgs e)
        {
            if (m_lRealHandle >= 0)
            {
                CHCNetSDK.NET_DVR_StopRealPlay(m_lRealHandle);
            }
            if (m_lUserID >= 0)
            {
                CHCNetSDK.NET_DVR_Logout(m_lUserID);
            }
            if (m_bInitSDK == true)
            {
                CHCNetSDK.NET_DVR_Cleanup();
            }
        }
    }
}
