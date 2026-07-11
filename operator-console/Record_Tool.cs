using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml;
using System.Collections;
using System.IO;
using System.Threading;
using System.Diagnostics;
using System.Net.Sockets;
namespace 手术机器人上位机程序
{
    static class Record_Tool
    {
        public static void kill_process(string process_name, int sleep_time_ms)
        {
            Process[] processes = Process.GetProcessesByName(process_name);
            if (process_name.Length != 0)  //有进程
            {
                foreach (Process process_1 in processes)
                {
                    process_1.Kill(); // 关闭进程
                    Thread.Sleep(100);
                }
                Thread.Sleep(sleep_time_ms);
            }
        }
        public static void run_exe(string path) //自己在外面创建线程run
        {
            string fileName = Path.GetFileName(path);
            string directoryName = Path.GetDirectoryName(path);

            ProcessStartInfo my_startInfo = new ProcessStartInfo();
            my_startInfo.FileName = fileName;
            my_startInfo.WorkingDirectory = directoryName;

            Process my_process = new Process();
            my_process.StartInfo = my_startInfo;
            my_process.Start();
            my_process.WaitForExit();
            Thread.Sleep(1000);
        }
    }
}
