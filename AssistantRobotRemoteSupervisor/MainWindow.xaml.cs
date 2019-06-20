using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
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
using System.Windows.Threading;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Net.NetworkInformation;
using System.Security.Cryptography;
using System.Reflection;
using System.Configuration;

using MahApps.Metro.Controls;
using MahApps.Metro.Controls.Dialogs;
using LogPrinter;
using Emgu.CV;
using Emgu.CV.Structure;

namespace AssistantRobotRemoteSupervisor
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow
    {
        /// <summary>
        /// 协议关键字
        /// </summary>
        public enum VideoTransferProtocolKey : byte
        {
            Header1 = 34,
            Header2 = 84,
            RSAKey = 104,
            BeginTransferVideo = 114,
            VideoTransfer = 204,
            PingSignal = 244,
            EndTransferVideo = 254
        }

        #region 字段
        readonly bool ifAtSamePC = true;
        readonly bool ifAtSameLAN = true;
        readonly string netAdapterName = "unknown";

        const string clientIPAtSamePC = "127.0.0.1";
        const int clientPortTCPAtSamePC = 40007;
        const int clientPortUDPAtSamePC = 40008;
        const string serverIPAtSamePC = "127.0.0.1";

        private string clientIPAtSameLAN;
        const int clientPortTCPAtSameLAN = 40005;
        const int clientPortUDPAtSameLAN = 40006;
        readonly string serverIPAtSameLAN = "192.168.1.13";

        private string clientIPAtWAN;
        const int clientPortTCPAtWAN = 40005;
        const int clientPortUDPAtWAN = 40006;
        readonly string serverIPAtWAN = "202.120.48.24"; // 路由器的公网IP

        const int serverPortTCPAtAll = 40005; // 端口转发应该设置同一端口
        const int serverPortUDPAtAll = 40006; //端口转发应该设置同一端口

        readonly byte clientDeviceIndex = 1;
        private EndPoint serverEndPoint = new IPEndPoint(0, 0);

        private Socket tcpTransferSocket;
        private bool ifTcpConnectionEstablished = false;
        readonly int tcpTransferSocketSendTimeOut = 500;
        readonly int tcpTransferSocketInterval = 1000;
        private System.Timers.Timer tcpSendClocker;
        private Task tcpTransferSendTask;
        private CancellationTokenSource tcpTransferCancel;
        private Queue<VideoTransferProtocolKey> tcpSendQueue = new Queue<VideoTransferProtocolKey>(100);
        private static readonly object queueLocker = new object();
        readonly int sleepMsForQueueSend = 10;

        private Socket udpTransferSocket;
        private CancellationTokenSource udpTransferCancel;
        private Task udpTransferRecieveTask;
        private string publicKey;
        private string privateKey;
        readonly int udpTransferSocketSendMaxTimeOut = 5000;
        readonly int udpTransferSocketSendTimeOut = 3000;
        const int keyLength = 1024;
        const int maxVideoByteLength = 60000;
        private byte lastReceivePackIndex = 0;
        private byte bufferReceivePackIndex = 0;
        private List<byte[]> bufferReceivePackContent = new List<byte[]>(byte.MaxValue);
        private List<byte> bufferReceivePackNum = new List<byte>(byte.MaxValue);

        private bool ifAppConfRight = true;
        private bool ifStartVideoShow = false;

        private readonly double titleSize = 18;
        private readonly double messageSize = 22;
        private bool canUseSwitchBtnNow = true;
        private bool existError = false;
        #endregion

        public MainWindow()
        {
            InitializeComponent();

            // 检查环境
            if (!Functions.CheckEnvironment()) return;
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor starts with successful checked.");

            // 加载App配置
            ifAppConfRight = true;
            bool parseResult = true;

            bool ifAtSamePCTemp;
            parseResult = bool.TryParse(ConfigurationManager.AppSettings["ifAtSamePC"], out ifAtSamePCTemp);
            if (parseResult) ifAtSamePC = ifAtSamePCTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "ifAtSamePC" + ") is wrong");
                return;
            }

            bool ifAtSameLANTemp;
            parseResult = bool.TryParse(ConfigurationManager.AppSettings["ifAtSameLAN"], out ifAtSameLANTemp);
            if (parseResult) ifAtSameLAN = ifAtSameLANTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "ifAtSameLAN" + ") is wrong");
                return;
            }

            netAdapterName = ConfigurationManager.AppSettings["netAdapterName"];
            if (!ifAtSamePC)
            {
                parseResult = false;
                NetworkInterface[] adapters = NetworkInterface.GetAllNetworkInterfaces();
                foreach (NetworkInterface adapter in adapters)
                {
                    if (adapter.Name == netAdapterName)
                    {
                        UnicastIPAddressInformationCollection unicastIPAddressInformation = adapter.GetIPProperties().UnicastAddresses;
                        foreach (var item in unicastIPAddressInformation)
                        {
                            if (item.Address.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                            {
                                if (!ifAtSamePC && ifAtSameLAN)
                                {
                                    clientIPAtSameLAN = item.Address.ToString();
                                }
                                else
                                {
                                    clientIPAtWAN = item.Address.ToString();
                                }
                                parseResult = true;
                                break;
                            }
                        }
                    }
                }
            }
            if (!parseResult)
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "netAdapterName" + ") is wrong");
                return;
            }

            string serverIPAtSameLANTemp = ConfigurationManager.AppSettings["serverIPAtSameLAN"];
            if (new string(serverIPAtSameLANTemp.Take(10).ToArray()) == "192.168.1.") serverIPAtSameLAN = serverIPAtSameLANTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "serverIPAtSameLAN" + ") is wrong");
                return;
            }

            string serverIPAtWANTemp = ConfigurationManager.AppSettings["serverIPAtWAN"];
            if (serverIPAtWANTemp.Trim() == serverIPAtWANTemp) serverIPAtWAN = serverIPAtWANTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "serverIPAtWAN" + ") is wrong");
                return;
            }

            byte clientDeviceIndexTemp;
            parseResult = byte.TryParse(ConfigurationManager.AppSettings["clientDeviceIndex"], out clientDeviceIndexTemp);
            if (parseResult) clientDeviceIndex = clientDeviceIndexTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "clientDeviceIndex" + ") is wrong");
                return;
            }

            int tcpTransferSocketSendTimeOutTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["tcpTransferSocketSendTimeOut"], out tcpTransferSocketSendTimeOutTemp);
            if (parseResult) tcpTransferSocketSendTimeOut = tcpTransferSocketSendTimeOutTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "tcpTransferSocketSendTimeOut" + ") is wrong");
                return;
            }

            int tcpTransferSocketIntervalTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["tcpTransferSocketInterval"], out tcpTransferSocketIntervalTemp);
            if (parseResult) tcpTransferSocketInterval = tcpTransferSocketIntervalTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "tcpTransferSocketInterval" + ") is wrong");
                return;
            }

            int udpTransferSocketSendMaxTimeOutTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["udpTransferSocketSendMaxTimeOut"], out udpTransferSocketSendMaxTimeOutTemp);
            if (parseResult) udpTransferSocketSendMaxTimeOut = udpTransferSocketSendMaxTimeOutTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "udpTransferSocketSendMaxTimeOut" + ") is wrong");
                return;
            }

            int udpTransferSocketSendTimeOutTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["udpTransferSocketSendTimeOut"], out udpTransferSocketSendTimeOutTemp);
            if (parseResult) udpTransferSocketSendTimeOut = udpTransferSocketSendTimeOutTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "udpTransferSocketSendTimeOut" + ") is wrong");
                return;
            }

            int sleepMsForQueueSendTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["sleepMsForQueueSend"], out sleepMsForQueueSendTemp);
            if (parseResult) sleepMsForQueueSend = sleepMsForQueueSendTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "sleepMsForQueueSend" + ") is wrong");
                return;
            }

            double titleSizeTemp;
            parseResult = double.TryParse(ConfigurationManager.AppSettings["titleSize"], out titleSizeTemp);
            if (parseResult) titleSize = titleSizeTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "titleSize" + ") is wrong");
                return;
            }

            double messageSizeTemp;
            parseResult = double.TryParse(ConfigurationManager.AppSettings["messageSize"], out messageSizeTemp);
            if (parseResult) messageSize = messageSizeTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "messageSize" + ") is wrong");
                return;
            }

            // 装上TCP定时器
            tcpSendClocker = new System.Timers.Timer(tcpTransferSocketInterval);
            tcpSendClocker.AutoReset = true;
            tcpSendClocker.Elapsed += tcpSendClocker_Elapsed;
        }

        private void MetroWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (!ifAppConfRight)
            {
                ShowCloseMsg("程序配置参数出错，确定关闭程序！", "错误");
                return;
            }

            SwitchVideo();
        }

        private async void CloseWindowsLater()
        {
            await Task.Delay(200);
            this.Close();
        }

        /// <summary>
        /// 主窗口弹窗
        /// </summary>
        /// <param name="message">消息</param>
        /// <param name="title">抬头</param>
        private async void ShowCloseMsg(string message, string title)
        {
            await ShowDialog(message, title);
            CloseWindowsLater();
        }

        /// <summary>
        /// 主窗口弹窗
        /// </summary>
        /// <param name="message">消息</param>
        /// <param name="title">抬头</param>
        private async Task ShowDialog(string message, string title)
        {
            var mySettings = new MetroDialogSettings()
            {
                AffirmativeButtonText = "确认",
                DialogTitleFontSize = titleSize,
                DialogMessageFontSize = messageSize,
                ColorScheme = MetroDialogColorScheme.Theme
            };

            if (this.CheckAccess())
            {
                await this.ShowMessageAsync(title, message, MessageDialogStyle.Affirmative, mySettings);
            }
        }

        /// <summary>
        /// TCP发送队列数据任务
        /// </summary>
        /// <param name="cancelFlag">停止标志</param>
        private void TcpTransferSendTaskWork(CancellationToken cancelFlag)
        {
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer begins to send datas.");

            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                Thread.Sleep(sleepMsForQueueSend);

                VideoTransferProtocolKey waitSentKey = VideoTransferProtocolKey.VideoTransfer;
                lock (queueLocker)
                {
                    if (tcpSendQueue.Count > 0)
                    {
                        waitSentKey = tcpSendQueue.Dequeue();
                    }
                }

                if (waitSentKey == VideoTransferProtocolKey.VideoTransfer) continue;

                List<byte> sendBytes = new List<byte>(4);
                sendBytes.Add((byte)VideoTransferProtocolKey.Header1);
                sendBytes.Add((byte)VideoTransferProtocolKey.Header2);
                sendBytes.Add(clientDeviceIndex);
                sendBytes.Add((byte)waitSentKey);
                if (waitSentKey == VideoTransferProtocolKey.RSAKey)
                {

                    byte[] publicKeyBytes = Encoding.UTF8.GetBytes(publicKey);
                    sendBytes.AddRange(BitConverter.GetBytes(IPAddress.HostToNetworkOrder(publicKeyBytes.Length)));
                    sendBytes.AddRange(publicKeyBytes);
                }
                try
                {
                    tcpTransferSocket.Send(sendBytes.ToArray());
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode == SocketError.ConnectionReset || ex.SocketErrorCode == SocketError.ConnectionAborted || ex.SocketErrorCode == SocketError.TimedOut)
                    {
                        EndAllLoop();
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer send datas failed.", ex);
                        break;
                    }
                    else
                    {
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                        EndAllLoop(false);
                        break;
                    }
                }
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer send cmd '" + waitSentKey.ToString() + "'.");
            }

            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer stops to send datas.");

            udpTransferRecieveTask.Wait();
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer stops to send datas, and udp stops too.");

            FinishAllConnection();

            ifTcpConnectionEstablished = false;
        }

        /// <summary>
        /// 发送指令
        /// </summary>
        private void SendCmd(VideoTransferProtocolKey protocolKey)
        {
            lock (queueLocker)
            {
                tcpSendQueue.Enqueue(protocolKey);
            }
        }

        /// <summary>
        /// UDP接收数据任务
        /// </summary>
        /// <param name="cancelFlag">停止标志</param>
        private void UDPTransferRecieveTaskWork(CancellationToken cancelFlag)
        {
            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                // 接收收到的数据并处理
                byte[] recieveBuffer = new byte[maxVideoByteLength + 11];
                try
                {
                    udpTransferSocket.ReceiveFrom(recieveBuffer, ref serverEndPoint);
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode == SocketError.Interrupted || ex.SocketErrorCode == SocketError.TimedOut)
                    {
                        EndAllLoop();
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor udp transfer recieve datas failed.", ex);
                        break;
                    }
                    else
                    {
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                        EndAllLoop(false);
                        break;
                    }
                }
                if (!((IPEndPoint)serverEndPoint).Address.Equals(IPAddress.Parse(ifAtSamePC ? serverIPAtSamePC : (ifAtSameLAN ? serverIPAtSameLAN : serverIPAtWAN)))) continue;
                UDPRecieveDatasDeal(recieveBuffer);
            }
        }

        /// <summary>
        /// 处理UDP传输socket接收到的数据
        /// </summary>
        /// <param name="datas">所收数据</param>
        private void UDPRecieveDatasDeal(byte[] datas)
        {
            if (datas[0] != (byte)VideoTransferProtocolKey.Header1 || datas[1] != (byte)VideoTransferProtocolKey.Header2)
            {
                return;
            }

            if (datas[2] != clientDeviceIndex || (VideoTransferProtocolKey)datas[3] != VideoTransferProtocolKey.VideoTransfer)
            {
                return;
            }

            if (udpTransferSocket.ReceiveTimeout > udpTransferSocketSendTimeOut) udpTransferSocket.ReceiveTimeout = udpTransferSocketSendTimeOut;

            int packDataLength = Convert.ToInt32(
                             IPAddress.NetworkToHostOrder(
                             BitConverter.ToInt32(datas, 4)));
            byte packIndex = datas[8];
            byte packCount = datas[9];
            byte packNum = datas[10];

            if (bufferReceivePackIndex == 0) // New data
            {
                if (packCount > 1) // Multiply packs
                {
                    bufferReceivePackIndex = packIndex;
                    bufferReceivePackNum.Add(packNum);
                    IEnumerable<byte> byteDatas = datas.Skip(11).Take(packDataLength - 3);
                    bufferReceivePackContent.Add(byteDatas.ToArray());
                }
                else // Single Pack
                {
                    lastReceivePackIndex = packIndex;
                    IEnumerable<byte> byteDatas = datas.Skip(11).Take(packDataLength - 3);
                    DecryptAndShowImg(byteDatas.ToArray());
                }
            }
            else // Old Data
            {
                if (packIndex == bufferReceivePackIndex) // Same Index
                {
                    bufferReceivePackNum.Add(packNum);
                    IEnumerable<byte> byteDatas = datas.Skip(11).Take(packDataLength - 3);
                    bufferReceivePackContent.Add(byteDatas.ToArray());

                    if (packNum == packCount) // Last pack
                    {
                        byte[] sortedIndex = new byte[packCount];
                        for (byte i = 0; i < packCount; ++i)
                        {
                            sortedIndex[bufferReceivePackNum[i] - 1] = i;
                        }
                        List<byte> byteTotalDatas = new List<byte>(131070);
                        for (byte k = 0; k < packCount; ++k)
                        {
                            byteTotalDatas.AddRange(bufferReceivePackContent[sortedIndex[k]]);
                        }
                        DecryptAndShowImg(byteTotalDatas.ToArray());

                        bufferReceivePackIndex = 0;
                        bufferReceivePackNum.Clear();
                        bufferReceivePackContent.Clear();
                    }
                }
                else // Different Index
                {
                    bufferReceivePackIndex = 0;
                    bufferReceivePackNum.Clear();
                    bufferReceivePackContent.Clear();

                    if (packCount > 1) // Multiply packs
                    {
                        bufferReceivePackIndex = packIndex;
                        bufferReceivePackNum.Add(packNum);
                        IEnumerable<byte> byteDatas = datas.Skip(11).Take(packDataLength - 3);
                        bufferReceivePackContent.Add(byteDatas.ToArray());
                    }
                    else // Single Pack
                    {
                        lastReceivePackIndex = packIndex;
                        IEnumerable<byte> byteDatas = datas.Skip(11).Take(packDataLength - 3);
                        DecryptAndShowImg(byteDatas.ToArray());
                    }
                }
            }

            if (!canUseSwitchBtnNow)
            {
                canUseSwitchBtnNow = true;

                this.Dispatcher.BeginInvoke(
                    new Action(() =>
                    {
                        btnSwitchVideo.IsEnabled = true;
                    }),
                    DispatcherPriority.Normal);
            }
        }

        /// <summary>
        /// 解密并显示图像
        /// </summary>
        /// <param name="encryptedBytes">加密数据</param>
        private void DecryptAndShowImg(byte[] encryptedBytes)
        {
            int byteLength = encryptedBytes.Length;
            int unitLength = keyLength / 8;
            if (byteLength % unitLength != 0) return;
            int segmentNum = byteLength / unitLength;
            List<byte> decryptedBytesList = new List<byte>(byteLength);
            using (RSACryptoServiceProvider rsa = new RSACryptoServiceProvider())
            {
                rsa.FromXmlString(privateKey);
                for (int i = 0; i < segmentNum; ++i)
                {
                    IEnumerable<byte> buffer = encryptedBytes.Skip(i * unitLength).Take(unitLength);
                    decryptedBytesList.AddRange(rsa.Decrypt(buffer.ToArray(), false));
                }
            }

            using (MemoryStream ms = new MemoryStream(decryptedBytesList.ToArray()))
            {
                BitmapImage img = new BitmapImage();
                img.BeginInit();
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.StreamSource = ms;
                img.EndInit();
                img.Freeze();

                this.Dispatcher.BeginInvoke(
                    new Action<BitmapImage>(
                        (imgSource) =>
                        {
                            BitmapFrame imgframe = BitmapFrame.Create(imgSource);
                            IBShow.Source = imgframe; // Bitmap->Image
                        }),
                    DispatcherPriority.Normal,
                    new object[] { img });
            }
        }

        private BitmapImage BitmapToBitmapImage(System.Drawing.Bitmap bitmap)
        {
            BitmapImage bitmapImage = new BitmapImage();

            using (System.IO.MemoryStream ms = new System.IO.MemoryStream())
            {
                bitmap.Save(ms, bitmap.RawFormat);
                bitmapImage.BeginInit();
                bitmapImage.StreamSource = ms;
                bitmapImage.CacheOption = BitmapCacheOption.OnLoad;
                bitmapImage.EndInit();
                bitmapImage.Freeze();
            }

            return bitmapImage;
        }

        /// <summary>
        /// TCP传输心跳定时器
        /// </summary>
        private void tcpSendClocker_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            SendCmd(VideoTransferProtocolKey.PingSignal);
        }

        /// <summary>
        /// 结束所有循环等待
        /// </summary>
        /// <param name="noError">没有错误</param>
        private void EndAllLoop(bool noError = true)
        {
            tcpTransferCancel.Cancel();
            tcpSendClocker.Stop();
            udpTransferCancel.Cancel();

            if (!noError) existError = true;
        }

        /// <summary>
        /// 结束所有连接
        /// </summary>
        private void FinishAllConnection()
        {
            tcpTransferSocket.Shutdown(SocketShutdown.Both);
            tcpTransferSocket.Close();

            udpTransferSocket.Shutdown(SocketShutdown.Both);
            udpTransferSocket.Close();

            ifStartVideoShow = false;
            this.Dispatcher.BeginInvoke(
                new Action(() =>
                {
                    btnIcon.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PlayCircleRegular;
                    btnText.Text = "播放监控画面";
                    btnSwitchVideo.IsEnabled = true;
                }),
                DispatcherPriority.Normal);

            if (existError) ShowCloseMsg("网络连接出现未知错误，确定关闭程序！", "错误");
        }

        /// <summary>
        /// 播放/停止视频播放
        /// </summary>
        private void btnSwitchVideo_Click(object sender, RoutedEventArgs e)
        {
            SwitchVideo();
            e.Handled = true;
        }

        private void SwitchVideo()
        {
            if (!ifStartVideoShow)
            {
                // TCP连接已经建立就退出
                if (ifTcpConnectionEstablished) return;

                // 获取RSA密钥
                RSACryptoServiceProvider rsa = new RSACryptoServiceProvider(1024);
                publicKey = rsa.ToXmlString(false);
                privateKey = rsa.ToXmlString(true);

                // 重新建立新的TCP连接
                tcpTransferSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                tcpTransferSocket.Bind(new IPEndPoint(IPAddress.Parse(ifAtSamePC ? clientIPAtSamePC : (ifAtSameLAN ? clientIPAtSameLAN : clientIPAtWAN)), ifAtSamePC ? clientPortTCPAtSamePC : (ifAtSameLAN ? clientPortTCPAtSameLAN : clientPortTCPAtWAN)));
                tcpTransferSocket.SendTimeout = tcpTransferSocketSendTimeOut;
                tcpTransferSocket.ReceiveTimeout = tcpTransferSocketSendTimeOut;
                try
                {
                    tcpTransferSocket.Connect(new IPEndPoint(IPAddress.Parse(ifAtSamePC ? serverIPAtSamePC : (ifAtSameLAN ? serverIPAtSameLAN : serverIPAtWAN)), serverPortTCPAtAll));
                }
                catch (SocketException ex)
                {
                    tcpTransferSocket.Close();
                    if (ex.SocketErrorCode == SocketError.ConnectionRefused || ex.SocketErrorCode == SocketError.TimedOut)
                    {
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp connection can not established.", ex);
                        ShowDialog("网络连接失败！\r\n问题：" + ex.Message, "问题");
                        return;
                    }
                    else
                    {
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                        ShowCloseMsg("网络连接出现未知错误，确定关闭程序！", "错误");
                        return;
                    }
                }

                ifTcpConnectionEstablished = true;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp connection has been established.");

                // 开始允许TCP传输socket发送队列内的数据
                tcpTransferCancel = new CancellationTokenSource();
                tcpTransferSendTask = new Task(() => TcpTransferSendTaskWork(tcpTransferCancel.Token));
                tcpTransferSendTask.Start();
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp tranfer can send datas.");

                // 发送公钥
                SendCmd(VideoTransferProtocolKey.RSAKey);

                // 开始接收视频
                SendCmd(VideoTransferProtocolKey.BeginTransferVideo);

                // 开始允许UDP传输socket接收数据
                udpTransferSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                udpTransferSocket.Bind(new IPEndPoint(IPAddress.Parse(ifAtSamePC ? clientIPAtSamePC : (ifAtSameLAN ? clientIPAtSameLAN : clientIPAtWAN)), ifAtSamePC ? clientPortUDPAtSamePC : (ifAtSameLAN ? clientPortUDPAtSameLAN : clientPortUDPAtWAN)));
                udpTransferSocket.ReceiveTimeout = udpTransferSocketSendMaxTimeOut;
                udpTransferCancel = new CancellationTokenSource();
                udpTransferRecieveTask = new Task(() => UDPTransferRecieveTaskWork(udpTransferCancel.Token));
                udpTransferRecieveTask.Start();
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor udp tranfer can recieve datas.");

                // 心跳发送定时器打开
                tcpSendClocker.Start();
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp begin to beat.");

                btnIcon.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PauseCircleRegular;
                btnText.Text = "停止监控画面";

                canUseSwitchBtnNow = false;
            }
            else
            {
                // TCP连接未建立就退出
                if (!ifTcpConnectionEstablished) return;

                // 发送停止接收视频
                SendCmd(VideoTransferProtocolKey.EndTransferVideo);

                btnIcon.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PlayCircleRegular;
                btnText.Text = "播放监控画面";
            }

            ifStartVideoShow = !ifStartVideoShow;
            btnSwitchVideo.IsEnabled = false;
        }

        private void MetroWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (ifTcpConnectionEstablished)
            {
                // 发送停止接收视频
                SendCmd(VideoTransferProtocolKey.EndTransferVideo);
            }

            while (ifTcpConnectionEstablished)
            {
                Thread.Sleep(100);
            }
        }
    }

}
