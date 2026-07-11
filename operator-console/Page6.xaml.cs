using SharpGL;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.Windows.Media;
using System.Text;
using System.Windows.Threading;

namespace 手术机器人上位机程序
{
    /// <summary>
    /// Page6.xaml 的交互逻辑
    /// </summary>
    public partial class Page6 : Page
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

        #region 机器人
        public double joint_lizhu = 0;
        public double joint_1 = 0;
        public double joint_2 = 0;
        public double joint_3 = 0;
        public double joint_4 = 0;
        public double joint_5 = 0;

        private List<List<Triangle>> listTrianglesRobot = new List<List<Triangle>>();
        private List<int> jointIndexes = new List<int>();
        private double robotScale;
        private double robotScaleRatio;
        private double robotScaleRange = 5;
        private uint robotList;                            // 用于显示列表起始编号
        DenseMatrix m = new DenseMatrix(4, 4);
        #endregion

        #region 操作机器人
        private double move_robot_x = 0;                // Z和C键控制
        private double move_robot_y = 0;                // S和X键控制
        private double move_robot_z = 0;                // A和D键控制
        #endregion

        #region 光源材质
        // 光源
        private float[] lightPos0 = { 30, 30, 30, 0f };             // 光源0（w为1表示位置光源，为0表示方向性光源）
        private float[] lightPos1 = { 20, 20, 20, 1f };             // 光源1
        private float[] lightAmbient = { 0f, 0f, 0f, 1f };          // 环境光
        private float[] lightDiffuse = { 1f, 1f, 1f, 1f };          // 漫射光
        private float[] lightSpecular = { 1f, 1f, 1f, 1f };         // 镜面光
        /*
         * 对于材质设置，一般材质的环境光反射和漫射光反射属性设置相同，可以达到真实的效果
         */
        // 机器人材质
        private float[] robot_no_mat = { 0.0f, 0.0f, 0.0f, 1.0f };                // 无材质颜色
        private float[] robot_mat_ambient_diffuse = { 0.5f, 0.5f, 0.5f, 1.0f };   // 灰色
        //private float[] robot_mat_ambient_diffuse = { 0.6f, 0.4f, 0.2f, 1.0f };   // 橙棕色
        //private float[] robot_mat_ambient_diffuse = { 0.2f, 0.5f, 0.8f, 1.0f };   // 蓝色
        //private float[] robot_mat_ambient_diffuse = { 0.2f, 0.1f, 0.2f, 1.0f };   // 酒红色
        //private float[] robot_mat_ambient_diffuse = { 0.1f, 0.3f, 0.1f, 1.0f };   // 青绿色
        private float[] robot_mat_specular = { 1.0f, 1.0f, 1.0f, 1.0f };          // 镜面反射
        private float[] robot_no_shininess = { 0.0f };                            // 镜面反射指数为0
        private float[] robot_low_shininess = { 5.0f };                           // 镜面反射指数为5.0
        private float[] robot_media_shininess = { 50.0f };                        // 镜面反射指数为50.0
        private float[] robot_high_shininess = { 100.0f };                        // 镜面反射指数为100.0
        private float[] robot_mat_emission = { 0.3f, 0.2f, 0.3f, 0.0f };          // 发射光颜色
        #endregion


        double t_z = 0;
        private void OpenGLControl_OpenGLDraw(object sender, SharpGL.WPF.OpenGLRoutedEventArgs args)
        {
            OpenGL gl = args.OpenGL;
            gl.ClearColor(0.15f, 0.2f, 0.3f, 0.5f);     // 设置界面颜色
            gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
            gl.LoadIdentity();

            gl.Translate(2, -4, -12);

            RefreshText();

            DrawRobot(gl);
        }

        public void RefreshText()
        {
            joint_1_text.Text = joint_1.ToString() + "°";
            joint_2_text.Text = joint_2.ToString() + "°";
            joint_3_text.Text = joint_3.ToString() + "°";
            joint_4_text.Text = joint_4.ToString() + "°";
            joint_5_text.Text = joint_5.ToString() + "°";
            joint_lizhu_text.Text = (joint_lizhu/10).ToString() + "cm";
        }

        private void OpenGLControl_OpenGLInitialized(object sender, SharpGL.WPF.OpenGLRoutedEventArgs args)
        {
            OpenGL gl = args.OpenGL;

            // 刷新颜色缓冲区时所用的颜色.
            gl.ClearColor(0, 0, 0, 0);

            AddJoint(@"STL\用于可视化-底座台车-二代.STL", 0);
            AddJoint(@"STL\用于可视化-升降立柱.STL", 1);
            AddJoint(@"STL\关节臂1.STL", 2);
            AddJoint(@"STL\关节臂2.STL", 3);
            AddJoint(@"STL\关节臂3.STL", 4);
            AddJoint(@"STL\云台关节1.STL", 5);
            AddJoint(@"STL\云台关节2.STL", 6);

            SetDisList(gl);
            SetRenderContext(gl);
        }


        private void OpenGLControl_Loaded(object sender, RoutedEventArgs e)
        {
        }

        #region 机器人数据导入

        /// <summary>
        /// 添加机器人关节数据
        /// </summary>
        /// <param name="fileName">关节名</param>
        /// <param name="jointIndex">关节索引</param>
        private void AddJoint(string fileName, int jointIndex)
        {
            // 获取全路径（StartupPath在bin\debug路径下）
            string fn = AppDomain.CurrentDomain.BaseDirectory + fileName;
            // 二进制格式数据
            listTrianglesRobot.Add(STLFilter.LoadFromBINFile(fn));
            //// ASCII格式数据
            //listTrianglesRobot.Add(STLFilter.LoadFromASCIIFile(fileName));

            // 导入的关节数据位置匹配关节索引，后面根据索引顺序绘制
            jointIndexes.Add(jointIndex);

            // 求整个导入模型的特征
            var listTriangleRobot = new List<Triangle>();
            foreach (var list in listTrianglesRobot)
            {
                listTriangleRobot.AddRange(list);
            }
            // 中心
            // 最大尺度
            robotScale = STLFilter.GetModelFeature(listTriangleRobot).modelScale;
            // 缩放到指定robotScaleRange尺度内的缩放率
            robotScaleRatio = robotScaleRange / robotScale;
        }
        #endregion

        #region 图形和模型绘制部分
        /// <summary>
        /// 绘制导入的机器人（实现关节间的旋转）
        /// </summary>
        private void DrawRobot(OpenGL gl)
        {
            gl.PushAttrib(OpenGL.GL_CURRENT_BIT);
            gl.PushMatrix();

            gl.Translate(move_robot_x, move_robot_y, move_robot_z);
            // 缩放至合适大小
            gl.Scale(robotScaleRatio, robotScaleRatio, robotScaleRatio);

            /*
             * 调用显示列表绘制模型
             * 控制模型的各个关节进行串联运动
             */
            // 绘制机器人基座
            gl.Rotate(135, 0.0f, 1.0f, 0.0f);

            gl.CallList(robotList);

            m[0, 0] = 1;
            m[0, 1] = 0;
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = joint_lizhu;

            m[2, 0] = 0;
            m[2, 1] = 0;
            m[2, 2] = 1;
            m[2, 3] = -232;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());
            // 绘制机器人关节1  
            gl.CallList(robotList + 1);


            m[0, 0] = 1;
            m[0, 1] = 0;
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 1020.5;

            m[2, 0] = 0;
            m[2, 1] = 0;
            m[2, 2] = 1;
            m[2, 3] = 0;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());

            m[0, 0] = Math.Cos(joint_1/180*Math.PI);
            m[0, 1] = 0;
            m[0, 2] = Math.Sin(joint_1 / 180 * Math.PI);
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 0;

            m[2, 0] = -Math.Sin(joint_1 / 180 * Math.PI);
            m[2, 1] = 0;
            m[2, 2] = Math.Cos(joint_1 / 180 * Math.PI);
            m[2, 3] = 0;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());
            // 绘制机器人关节2
            gl.CallList(robotList + 2);

            m[0, 0] = 1;
            m[0, 1] = 0;
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 87.5;

            m[2, 0] = 0;
            m[2, 1] = 0;
            m[2, 2] = 1;
            m[2, 3] = -700;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());

            m[0, 0] = Math.Cos(joint_2 / 180 * Math.PI);
            m[0, 1] = 0;
            m[0, 2] = Math.Sin(joint_2 / 180 * Math.PI);
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 0;

            m[2, 0] = -Math.Sin(joint_2 / 180 * Math.PI);
            m[2, 1] = 0;
            m[2, 2] = Math.Cos(joint_2 / 180 * Math.PI);
            m[2, 3] = 0;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());
            // 绘制机器人关节3
            gl.CallList(robotList + 3);


            m[0, 0] = 1;
            m[0, 1] = 0;
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 1;

            m[2, 0] = 0;
            m[2, 1] = 0;
            m[2, 2] = 1;
            m[2, 3] = -500;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());

            m[0, 0] = Math.Cos(joint_3 / 180 * Math.PI);
            m[0, 1] = 0;
            m[0, 2] = Math.Sin(joint_3 / 180 * Math.PI);
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 0;

            m[2, 0] = -Math.Sin(joint_3 / 180 * Math.PI);
            m[2, 1] = 0;
            m[2, 2] = Math.Cos(joint_3 / 180 * Math.PI);
            m[2, 3] = 0;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());
            // 绘制机器人关节4
            gl.CallList(robotList + 4);


            m[0, 0] = 1;
            m[0, 1] = 0;
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 16.5;

            m[2, 0] = 0;
            m[2, 1] = 0;
            m[2, 2] = 1;
            m[2, 3] = -428.9;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());

            m[0, 0] = Math.Cos(-joint_4 / 180 * Math.PI);
            m[0, 1] = -Math.Sin(-joint_4 / 180 * Math.PI);
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = Math.Sin(-joint_4 / 180 * Math.PI);
            m[1, 1] = Math.Cos(-joint_4 / 180 * Math.PI);
            m[1, 2] = 0;
            m[1, 3] = 0;

            m[2, 0] = 0;
            m[2, 1] = 0;
            m[2, 2] = 1;
            m[2, 3] = 0;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());
            // 绘制机器人关节5
            gl.CallList(robotList + 5);


            m[0, 0] = 1;
            m[0, 1] = 0;
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = 1;
            m[1, 2] = 0;
            m[1, 3] = 0;

            m[2, 0] = 0;
            m[2, 1] = 0;
            m[2, 2] = 1;
            m[2, 3] = -122;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());

            m[0, 0] = 1;
            m[0, 1] = 0;
            m[0, 2] = 0;
            m[0, 3] = 0;

            m[1, 0] = 0;
            m[1, 1] = Math.Cos(joint_5 / 180 * Math.PI);
            m[1, 2] = -Math.Sin(joint_5 / 180 * Math.PI);
            m[1, 3] = 0;

            m[2, 0] = 0;
            m[2, 1] = Math.Sin(joint_5 / 180 * Math.PI);
            m[2, 2] = Math.Cos(joint_5 / 180 * Math.PI);
            m[2, 3] = 0;

            m[3, 0] = 0;
            m[3, 1] = 0;
            m[3, 2] = 0;
            m[3, 3] = 1;

            gl.MultMatrix(m.ToColumnMajorArray());
            // 绘制机器人关节6
            gl.CallList(robotList + 6);

            gl.PopMatrix();
            gl.PopAttrib();
        }

        /// <summary>
        /// 载入到显示列表的机器人绘制部分
        /// </summary>
        /// <param name="gl"></param>
        /// <param name="n">第n个关节</param>
        private void ListOfRobot(OpenGL gl, int n)
        {
            // 开启光照
            gl.Enable(OpenGL.GL_LIGHTING);
            gl.Enable(OpenGL.GL_LIGHT0);
            gl.Enable(OpenGL.GL_LIGHT1);

            // 设置模型材质（可以从源STL文件中获取）
            SetRobotMaterial(gl);

            // 绘制模型
            gl.Begin(OpenGL.GL_TRIANGLES);
            for (int i = 0; i < listTrianglesRobot[n].Count; i++)
            {
                gl.Normal(listTrianglesRobot[n][i].Normal.Nx, listTrianglesRobot[n][i].Normal.Ny, listTrianglesRobot[n][i].Normal.Nz);  //设置三角面的法线
                gl.Vertex(listTrianglesRobot[n][i].P1.X, listTrianglesRobot[n][i].P1.Y, listTrianglesRobot[n][i].P1.Z);
                gl.Vertex(listTrianglesRobot[n][i].P2.X, listTrianglesRobot[n][i].P2.Y, listTrianglesRobot[n][i].P2.Z);
                gl.Vertex(listTrianglesRobot[n][i].P3.X, listTrianglesRobot[n][i].P3.Y, listTrianglesRobot[n][i].P3.Z);
            }
            gl.End();
        }

        #endregion

        #region 显示列表

        private void SetDisList(OpenGL gl)
        {
            /*
             * 
             * 电脑长时间运行可能导致工件加载不出来的现象
             * 可能与创建的显示列表有关，也可能是工件数据过大（60M），OpenGL内部采用保护机制间歇性地终止其显示
             * 电脑重启后可以暂时解决问题
             * 
             */

            // 机器人显示列表
            robotList = gl.GenLists(listTrianglesRobot.Count + 2);
            if (robotList != 0)
            {
                for (int i = 0; i < listTrianglesRobot.Count; i++)
                {
                    gl.NewList(robotList + (uint)i, OpenGL.GL_COMPILE);
                    ListOfRobot(gl, i);
                    gl.EndList();
                }
            }
        }

        #endregion


        #region 环境设置

        private void SetRenderContext(OpenGL gl)
        {
            // 设置纹理
            SetTexture(gl);
            // 设置光源
            SetLight(gl);
            // 法线向量规范化（增强显示效果,模型缩放时会影响法向量）,还可以采用GL_RESCALE_NORMAL在变换后恢复单位长度
            gl.Enable(OpenGL.GL_NORMALIZE);
            // 背面隐藏设置时需启用深度测试
            gl.Enable(OpenGL.GL_DEPTH_TEST);
            // 启用阴影平滑
            gl.ShadeModel(OpenGL.GL_SMOOTH);
            // 深度测试的类型
            gl.DepthFunc(OpenGL.GL_LEQUAL);
            // 进行透视修正
            gl.Hint(OpenGL.GL_PERSPECTIVE_CORRECTION_HINT, OpenGL.GL_NICEST);
        }

        /// <summary>
        /// 设置纹理
        /// </summary>
        /// <param name="gl"></param>
        private void SetTexture(OpenGL gl)
        {
            // 常用加载纹理方式
            SharpGL.SceneGraph.Assets.Texture texture;
            texture = new SharpGL.SceneGraph.Assets.Texture();
            texture.Create(gl, AppDomain.CurrentDomain.BaseDirectory + @"\STL\moon.bmp");  //创建星球纹理
        }

        /// <summary>
        /// 设置光照参数
        /// </summary>
        private void SetLight(OpenGL gl)
        {

            // 光源LIGHT0
            gl.Light(OpenGL.GL_LIGHT0, OpenGL.GL_AMBIENT, lightAmbient);   // 环境光
            gl.Light(OpenGL.GL_LIGHT0, OpenGL.GL_DIFFUSE, lightDiffuse);   // 漫射光
            gl.Light(OpenGL.GL_LIGHT0, OpenGL.GL_SPECULAR, lightSpecular); // 镜面反射光
            gl.Light(OpenGL.GL_LIGHT0, OpenGL.GL_POSITION, lightPos0);     // 光源位置

            // 光源LIGHT1
            gl.Light(OpenGL.GL_LIGHT1, OpenGL.GL_AMBIENT, lightAmbient);   // 环境光
            gl.Light(OpenGL.GL_LIGHT1, OpenGL.GL_DIFFUSE, lightDiffuse);   // 漫射光
            gl.Light(OpenGL.GL_LIGHT1, OpenGL.GL_SPECULAR, lightSpecular); // 镜面反射光
            gl.Light(OpenGL.GL_LIGHT1, OpenGL.GL_POSITION, lightPos1);     // 光源位置
        }

        /// <summary>
        /// 设置机器人材质
        /// </summary>
        private void SetRobotMaterial(OpenGL gl)
        {

            gl.Material(OpenGL.GL_FRONT, OpenGL.GL_AMBIENT, robot_mat_ambient_diffuse);
            gl.Material(OpenGL.GL_FRONT, OpenGL.GL_DIFFUSE, robot_mat_ambient_diffuse);
            gl.Material(OpenGL.GL_FRONT, OpenGL.GL_SPECULAR, robot_mat_specular);
            gl.Material(OpenGL.GL_FRONT, OpenGL.GL_SHININESS, robot_high_shininess);
            gl.Material(OpenGL.GL_FRONT, OpenGL.GL_EMISSION, robot_no_mat);
        }

        #endregion





        public Page6()
        {
            InitializeComponent();
            InitializeUDPReceivce();

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
        }

        #region udp监听
        public UdpClient udpcRecv;
        Thread thrRecv;
        void InitializeUDPReceivce()
        {
            IPEndPoint localIpep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 3000);
            udpcRecv = new UdpClient(localIpep);
            thrRecv = new Thread(ReceiveMessage);
            thrRecv.Start();
        }

        public void ReceiveMessage(object obj)
        {
            IPEndPoint remoteIpep = new IPEndPoint(IPAddress.Any, 0);
            while (true)
            {
                try
                {
                    byte[] bytRecv = udpcRecv.Receive(ref remoteIpep);
                    //string message = Encoding.ASCII.GetString(bytRecv, 0, bytRecv.Length);
                    //Console.WriteLine(message);

                    joint_lizhu = (Int16)((bytRecv[1] & 0x00ff) << 8 | bytRecv[0] & 0x00ff);
                    joint_1 = (double)((Int16)((bytRecv[3] & 0x00ff) << 8 | bytRecv[2] & 0x00ff))/100; 
                    joint_2 = (double)((Int16)((bytRecv[5] & 0x00ff) << 8 | bytRecv[4] & 0x00ff)) / 100; 
                    joint_3 = (double)((Int16)((bytRecv[7] & 0x00ff) << 8 | bytRecv[6] & 0x00ff) )/ 100; 
                    joint_4 = (double)((Int16)((bytRecv[9] & 0x00ff) << 8 | bytRecv[8] & 0x00ff) )/ 100;
                    joint_5 = (double)((Int16)((bytRecv[11] & 0x00ff) << 8 | bytRecv[10] & 0x00ff)) / 100;

                }
                catch (Exception ex)
                {
                    break;
                }
            }
        }

        public void CloseUDPReceive()
        {
            udpcRecv.Close();
        }
        #endregion

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
        }

        private void Motor1_Disnable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("a2");
            this.Motor1_Enable.Background = Brushes.Gray;
            this.Motor1_Disnable.Background = Brushes.Purple;
        }

        private void Motor2_Enable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("s1");
            this.Motor2_Enable.Background = Brushes.Purple;
            this.Motor2_Disnable.Background = Brushes.Gray;
        }

        private void Motor2_Disnable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("s2");
            this.Motor2_Enable.Background = Brushes.Gray;
            this.Motor2_Disnable.Background = Brushes.Purple;
        }

        private void Motor3_Enable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("d1");
            this.Motor3_Enable.Background = Brushes.Purple;
            this.Motor3_Disnable.Background = Brushes.Gray;
        }

        private void Motor3_Disnable_Click(object sender, RoutedEventArgs e)
        {
            this.ParentPage.SendWord("d2");
            this.Motor3_Enable.Background = Brushes.Gray;
            this.Motor3_Disnable.Background = Brushes.Purple;
        }

        private void Motor1_SpeedCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void return_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage);
        }

        private void basic_Click(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(this.ParentPage.page2);
        }
    }
}
