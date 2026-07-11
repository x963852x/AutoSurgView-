using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Diagnostics;

namespace 手术机器人上位机程序
{

    public static class SetWindow
    {
        public static string Record_Tool_form_name = "录制管理工具";
        //public static string Record_Tool_form_name = "Record Tool";
        public static string every_thing_form_name = "Everything";
        /* 快捷键数组，分别为F12,F11,ctrl+s,Alt+F4,shift+F4 */
        public static string[] key_string_arr = new string[] { "{F12}", "{F11}", "^s", "%{F4}" , "+{F4}" };
        private const int VK_F12 = 0x7B;
        private const int VK_ALT = 0x12;
        private const int VK_P = 0x50;
        private const uint MOUSEEVENTF_LEFTDOWN = 0x0002;
        private const uint MOUSEEVENTF_LEFTUP = 0x0004;
        #region API 需要using System.Runtime.InteropServices;
        [DllImport("user32.dll")]
        private static extern void mouse_event(uint dwFlags, int dx, int dy, uint dwData, UIntPtr dwExtraInfo);

        [DllImport("user32.dll", SetLastError = true)]
        static extern IntPtr GetParent(IntPtr hWnd);
        [DllImport("user32.dll", SetLastError = true)]
        private static extern bool SetForegroundWindow(IntPtr hWnd);
        [DllImport("user32.dll", SetLastError = true)]
        static extern int SendMessage(IntPtr hWnd, int Msg, IntPtr wParam, IntPtr lParam);
        [DllImport("user32.dll", SetLastError = true)]
        static extern bool AttachThreadInput(uint idAttach, uint idAttachTo, bool fAttach);
        [DllImport("user32.dll", SetLastError = true)]
        static extern IntPtr GetForegroundWindow();
        [DllImport("user32.dll", SetLastError = true)]
        static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);
        [DllImport("user32.dll")] 
        static extern IntPtr GetDesktopWindow();
        [DllImport("user32.dll", SetLastError = true)]
        private static extern bool PostMessage(IntPtr hWnd, uint Msg, int wParam, int lParam);
        [DllImport("user32.dll ", EntryPoint = "SetParent")]
        private static extern IntPtr SetParent(IntPtr hWndChild, IntPtr hWndNewParent);   //将外部窗体嵌入程序

        [DllImport("user32.dll")]
        public static extern IntPtr FindWindow(string lpszClass, string lpszWindow);      //按照窗体类名或窗体标题查找窗体

        [DllImport("user32.dll", EntryPoint = "ShowWindow", CharSet = CharSet.Auto)]
        private static extern int ShowWindow(IntPtr hwnd, int nCmdShow);                  //设置窗体属性
  
        // 导入Win32 API函数
        [DllImport("user32.dll")]
        static extern bool GetWindowRect(IntPtr hwnd, out RECT lpRect);
        [DllImport("user32.dll", EntryPoint = "SetWindowLong")]
        private static extern int SetWindowLong(IntPtr hWnd, int nIndex, int dwNewLong);

        [DllImport("user32.dll", EntryPoint = "GetWindowLong")]
        private static extern int GetWindowLong(IntPtr hWnd, int nIndex);


        [DllImport("gdi32.dll")]
        static extern IntPtr CreateRectRgn(int nLeftRect, int nTopRect, int nRightRect, int nBottomRect);

        [DllImport("user32.dll")]
        static extern int SetWindowRgn(IntPtr hWnd, IntPtr hRgn, bool bRedraw);

        [DllImport("gdi32.dll")]
        static extern int CombineRgn(IntPtr hrgnDest, IntPtr hrgnSrc1, IntPtr hrgnSrc2, int fnCombineMode);
        [DllImport("user32.dll")]
        static extern int GetWindowRgn(IntPtr hWnd, IntPtr hRgn);
        //[DllImport("user32.dll", EntryPoint = "SetWindowLongPtr")]
        //private static extern IntPtr SetWindowLongPtr(IntPtr hWnd, int nIndex, IntPtr dwNewLong);
        [DllImport("user32.dll")]
        static extern IntPtr GetDC(IntPtr hWnd);
        [DllImport("user32.dll", SetLastError = true)]
        static extern IntPtr DefWindowProc(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam);
        [DllImport("user32.dll")]
        public static extern IntPtr GetWindowDC(IntPtr hWnd);

        [DllImport("gdi32.dll")]
        public static extern IntPtr CreateCompatibleDC(IntPtr hdc);

        [DllImport("gdi32.dll")]
        public static extern IntPtr CreateCompatibleBitmap(IntPtr hdc, int nWidth, int nHeight);
        [DllImport("gdi32.dll")]
        static extern IntPtr SelectObject(IntPtr hdc, IntPtr hgdiobj);
        [DllImport("gdi32.dll")]
        static extern bool BitBlt(IntPtr hdc, int nXDest, int nYDest, int nWidth, int nHeight, IntPtr hdcSrc, int nXSrc, int nYSrc, uint dwRop);
        #endregion

        public static IntPtr now_form2_external_form_intPtr=IntPtr.Zero;         //当前form2第三方应用窗口的句柄
        public static IntPtr now_form3_external_form_intPtr = IntPtr.Zero;         //当前form9-参数表里的第三方应用窗口的句柄
        
        public const int WM_KEYDOWN = 0x0100;
        public const int WM_KEYUP = 0x0101;
        public const int WM_CHAR = 0x0102;
        public const int WM_MOUSEMOVE = 0x0200;
        public const int WM_LBUTTONDOWN = 0x0201;
        public const int WM_LBUTTONUP = 0x0202;
        public const int WM_RBUTTONDOWN = 0x0204;
        public const int WM_RBUTTONUP = 0x0205;
        public const int WM_MBUTTONDOWN = 0x0207;
        public const int WM_MBUTTONUP = 0x0208;
        private const int GWLP_WNDPROC = -4;
        public static IntPtr targetHdc;
        public static IntPtr targetWnd_Proc;
        public delegate IntPtr WindowProcDelegate(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam);
        public const uint SRCCOPY = 0x00CC0020;

        // 定义RECT结构体
        [StructLayout(LayoutKind.Sequential)]
        public struct RECT
        {
            public int Left;
            public int Top;
            public int Right;
            public int Bottom;
        }
        /// <summary>
        /// 调整第三方应用窗体大小
        /// </summary>
        public static void ResizeWindow(IntPtr external_window)
        {
            ShowWindow(external_window, 0);  //先将窗口隐藏
            ShowWindow(external_window, 3);  //再将窗口最大化，可以让第三方窗口自适应容器的大小
        }

        /// <summary>
        /// 循环查找第三方窗体
        /// </summary>
        /// <returns></returns>
        /// public const int WM_CHAR = 256;
        public static IntPtr TargetWndProc(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam)
        {
            // 处理消息
            return DefWindowProc(hWnd, msg, wParam, lParam);
        }
        public static IntPtr Find_external_Window(string formName, IntPtr hWndNewParent, int wait_time_s)
        {
            int circle_number = wait_time_s * 10;
            IntPtr vHandle = IntPtr.Zero;

            for (int i = 0; i < circle_number; i++)   //每100ms查找一次，直到找到，最多查找60s
            {
                vHandle = FindWindow(null, formName);
                if (vHandle == IntPtr.Zero)
                {
                    Thread.Sleep(100);  //每100ms查找一次，直到找到，最多查找10s
                    continue;
                }
                else
                {
                    Thread.Sleep(200);
                    break;
                }
            }

            return vHandle;
        }
        /* 需要外部创建线程 */
        public static IntPtr Find_external_Window_And_Embed(string formName, IntPtr hWndNewParent,int wait_time_s)
        {
            int circle_number = wait_time_s * 10;
            IntPtr vHandle = IntPtr.Zero;

            for (int i = 0; i < circle_number; i++)   //每100ms查找一次，直到找到，最多查找60s
            {
                vHandle = FindWindow(null, formName);
                if (vHandle == IntPtr.Zero)
                {
                    Thread.Sleep(100);  //每100ms查找一次，直到找到，最多查找10s
                    continue;
                }
                else
                {
                    Thread.Sleep(200);
                    Embed(vHandle, hWndNewParent);
                    break;
                }
            }

            return vHandle;
        }

        //public static void direct_Embed_And_Send_key(string formName, int x, IntPtr hWndNewParent, string s, int key_arr_index, int wait_time_s)
        //{
        //    ref IntPtr now_ptr = ref now_form2_external_form_intPtr;
        //    if (x == 2)
        //    {
        //        now_ptr = ref now_form2_external_form_intPtr;
        //    }
        //    else if (x == 3)
        //    {
        //        now_ptr = ref now_form3_external_form_intPtr;
        //    }
        //    if (s == "before")
        //    {
        //        /* 嵌入弹出循环1次，以确保嵌入可靠性 */
        //        for (int i = 0; i < 1; i++)
        //        {
        //            if (now_ptr != IntPtr.Zero)
        //            {
        //                Eject(now_ptr);
        //                now_ptr = IntPtr.Zero;
        //            }
        //            if ((now_ptr = SetWindow.Find_external_Window(formName, hWndNewParent, wait_time_s)) != IntPtr.Zero)
        //            {
        //                Thread.Sleep(200);
        //                Embed(now_ptr, hWndNewParent);
        //            }
        //        }
        //        /* 弹出 */
        //        if (now_ptr != IntPtr.Zero)
        //        {
        //            Eject(now_ptr);
        //            now_ptr = IntPtr.Zero;
        //        }
        //        /* 继续找到窗口，发送快捷键 */
        //        if ((now_ptr = SetWindow.Find_external_Window(formName, hWndNewParent, wait_time_s)) != IntPtr.Zero)
        //        {
        //            Thread.Sleep(200);
        //            if (key_arr_index >= 0)
        //            {
        //                /* 在外面发送快捷键稍微等久一点再发 */
        //                Thread.Sleep(600);
        //                Send_key(now_ptr, key_string_arr[key_arr_index]);
        //            }
        //            else
        //            {
        //                /* 嵌入之前等以下 */
        //                Thread.Sleep(100);
        //            }
        //        }
        //        /* 嵌入 */
        //        if ((now_ptr = SetWindow.Find_external_Window(formName, hWndNewParent, wait_time_s)) != IntPtr.Zero)
        //        {
        //            Thread.Sleep(100);
        //            Embed(now_ptr, hWndNewParent);
        //        }
        //    }
        //    else if (s == "after")
        //    {
        //        /* 嵌入弹出循环1次，以确保嵌入可靠性 */
        //        for (int i = 0; i < 1; i++)
        //        {
        //            if (now_ptr != IntPtr.Zero)
        //            {
        //                Eject(now_ptr);
        //                now_ptr = IntPtr.Zero;
        //            }
        //            if ((now_ptr = SetWindow.Find_external_Window(formName, hWndNewParent, wait_time_s)) != IntPtr.Zero)
        //            {
        //                Thread.Sleep(200);
        //                Embed(now_ptr, hWndNewParent);
        //            }
        //        }
        //        /* 弹出 */
        //        if (now_ptr != IntPtr.Zero)
        //        {
        //            Eject(now_ptr);
        //            now_ptr = IntPtr.Zero;
        //        }
        //        /* 嵌入 */
        //        if ((now_ptr = SetWindow.Find_external_Window(formName, hWndNewParent, wait_time_s)) != IntPtr.Zero)
        //        {
        //            Thread.Sleep(200);
        //            Embed(now_ptr, hWndNewParent);
        //        }
        //        /* 发送快捷键 */
        //        Thread.Sleep(200);
        //        if (key_arr_index >= 0)
        //        {
        //            Send_key(now_ptr, key_string_arr[key_arr_index]);
        //        }
        //    }
        //}

        public static void Embed(IntPtr external_window,IntPtr hWndNewParent)
        {
            ShowWindow(external_window, 0);                 //先将窗体隐藏，防止出现闪烁
            SetParent(external_window, hWndNewParent);      //将第三方窗体嵌入父容器
            Thread.Sleep(100);                      //略加延时
            ShowWindow(external_window, 3);                 //让第三方窗体在容器中最大化显示
            //RemoveWindowTitle(external_window);             // 去除窗体标题
        }
        public static void Eject(IntPtr external_window)
        {
            SetParent(external_window, IntPtr.Zero);
            Thread.Sleep(200);
            ShowWindowTitle(external_window);
        }

        private const int GWL_STYLE = -16;
        private const int WS_SYSMENU = 0x00080000;
        private const int WS_CAPTION = 0x00C00000;
        public static void RemoveWindowTitle(IntPtr vHandle)
        {
            int style = GetWindowLong(vHandle, GWL_STYLE);
            //style &= ~(WS_SYSMENU | WS_CAPTION);
            style &= ~(WS_CAPTION);
            SetWindowLong(vHandle, GWL_STYLE, style);
        }
        public static void ShowWindowTitle(IntPtr vHandle)
        {
            int style = GetWindowLong(vHandle, GWL_STYLE);
            style |= (WS_CAPTION);
            SetWindowLong(vHandle, GWL_STYLE, style);
        }
        
        //public static void Send_key(IntPtr external_window,string s)
        //{
        //    // Get the process ID of the UG window
        //    uint processId;
        //    GetWindowThreadProcessId(external_window, out processId);

        //    // Get the ID of the foreground window
        //    uint foregroundWindowThreadId;
        //    Process[] processes = Process.GetProcessesByName(overall_situation.my_form_process_name);

        //    foregroundWindowThreadId = (uint)processes[0].Id;

        //    if (SetForegroundWindow(external_window))
        //    {
        //        bool j = true;
        //    }
        //    SendKeys.SendWait(s);

        //}
    }
}
