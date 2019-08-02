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
using System.ComponentModel;

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
    public partial class MainWindow : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private BitmapFrame leftImage;
        /// <summary>
        /// 左显示区
        /// </summary>
        public BitmapFrame LeftImage
        {
            get { return leftImage; }
            set
            {
                leftImage = value;
                if (this.PropertyChanged != null)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("LeftImage"));
                }
            }
        }

        private BitmapFrame rightImage;
        /// <summary>
        /// 右显示区
        /// </summary>
        public BitmapFrame RightImage
        {
            get { return rightImage; }
            set
            {
                rightImage = value;
                if (this.PropertyChanged != null)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("RightImage"));
                }
            }
        }

        /// <summary>
        /// 协议关键字
        /// </summary>
        public enum VideoTransferProtocolKey : byte
        {
            Header1 = 34,
            Header2 = 84,
            RSAKey = 104,
            AESKey = 108,
            BeginTransferVideo = 114,
            VideoTransfer = 204,
            PingSignal = 244,
            EndTransferVideo = 254
        }

        /// <summary>
        /// 密钥数据报格式
        /// </summary>
        public enum SecurityKeyLength : int
        {
            AESIVLength = 16,
            AESKeyLength = 32,
            RSAKeyLength = 1024
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

        private byte lastDealedPackIndex = 0;
        private byte currentDealedPackIndex = 0;
        private byte predictNextDealedPackIndex = 1;
        private Dictionary<byte, byte[]> lastRecDatas = new Dictionary<byte, byte[]>();
        private Dictionary<byte, byte[]> currentRecDatas = new Dictionary<byte, byte[]>();

        private const int recBufferMaxSize = 100;
        private Queue<byte[]> recieveBufferQueue = new Queue<byte[]>(recBufferMaxSize);
        private readonly static object recBufferLocker = new object();
        private Task udpBufferWorkTask;
        private CancellationTokenSource udpBufferCancel;
        private const int sleepMsForBuffer = 1;

        private byte[] commonKey = null;
        private byte[] commonIV = null;

        private bool ifAppConfRight = true;
        private bool ifStartVideoShow = false;

        private readonly double titleSize = 18;
        private readonly double messageSize = 22;
        private bool canUseSwitchBtnNow = true;
        #endregion

        #region 字段Ultra
        const int clientPortTCPUltraAtSamePC = 40011;
        const int clientPortUDPUltraAtSamePC = 40012;

        const int clientPortTCPUltraAtSameLAN = 40009;
        const int clientPortUDPUltraAtSameLAN = 40010;

        const int clientPortTCPUltraAtWAN = 40009;
        const int clientPortUDPUltraAtWAN = 40010;

        const int serverPortTCPUltraAtAll = 40009; // 端口转发应该设置同一端口
        const int serverPortUDPUltraAtAll = 40010; //端口转发应该设置同一端口

        private EndPoint serverUltraEndPoint = new IPEndPoint(0, 0);

        private Socket tcpTransferUltraSocket;
        private bool ifTcpConnectionUltraEstablished = false;
        private System.Timers.Timer tcpSendUltraClocker;
        private Task tcpTransferUltraSendTask;
        private CancellationTokenSource tcpTransferUltraCancel;
        private Queue<VideoTransferProtocolKey> tcpUltraSendQueue = new Queue<VideoTransferProtocolKey>(100);
        private static readonly object queueUltraLocker = new object();

        private Socket udpTransferUltraSocket;
        private CancellationTokenSource udpTransferUltraCancel;
        private Task udpTransferUltraRecieveTask;
        private string publicKeyUltra;
        private string privateKeyUltra;

        private byte lastDealedPackIndexUltra = 0;
        private byte currentDealedPackIndexUltra = 0;
        private byte predictNextDealedPackIndexUltra = 1;
        private Dictionary<byte, byte[]> lastRecDatasUltra = new Dictionary<byte, byte[]>();
        private Dictionary<byte, byte[]> currentRecDatasUltra = new Dictionary<byte, byte[]>();

        private const int recBufferMaxSizeUltra = 100;
        private Queue<byte[]> recieveBufferQueueUltra = new Queue<byte[]>(recBufferMaxSize);
        private readonly static object recBufferUltraLocker = new object();
        private Task udpBufferWorkUltraTask;
        private CancellationTokenSource udpBufferUltraCancel;
        private const int sleepMsForUltraBuffer = 1;

        private byte[] commonKeyUltra = null;
        private byte[] commonIVUltra = null;

        private bool ifStartVideoShowUltra = false;

        private bool canUseUltraSwitchBtnNow = true;
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
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "ifAtSamePC" + ") is wrong.");
                return;
            }

            bool ifAtSameLANTemp;
            parseResult = bool.TryParse(ConfigurationManager.AppSettings["ifAtSameLAN"], out ifAtSameLANTemp);
            if (parseResult) ifAtSameLAN = ifAtSameLANTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "ifAtSameLAN" + ") is wrong.");
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
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "netAdapterName" + ") is wrong.");
                return;
            }

            string serverIPAtSameLANTemp = ConfigurationManager.AppSettings["serverIPAtSameLAN"];
            if (new string(serverIPAtSameLANTemp.Take(10).ToArray()) == "192.168.1.") serverIPAtSameLAN = serverIPAtSameLANTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "serverIPAtSameLAN" + ") is wrong.");
                return;
            }

            string serverIPAtWANTemp = ConfigurationManager.AppSettings["serverIPAtWAN"];
            if (serverIPAtWANTemp.Trim() == serverIPAtWANTemp) serverIPAtWAN = serverIPAtWANTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "serverIPAtWAN" + ") is wrong.");
                return;
            }

            byte clientDeviceIndexTemp;
            parseResult = byte.TryParse(ConfigurationManager.AppSettings["clientDeviceIndex"], out clientDeviceIndexTemp);
            if (parseResult) clientDeviceIndex = clientDeviceIndexTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "clientDeviceIndex" + ") is wrong.");
                return;
            }

            int tcpTransferSocketSendTimeOutTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["tcpTransferSocketSendTimeOut"], out tcpTransferSocketSendTimeOutTemp);
            if (parseResult) tcpTransferSocketSendTimeOut = tcpTransferSocketSendTimeOutTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "tcpTransferSocketSendTimeOut" + ") is wrong.");
                return;
            }

            int tcpTransferSocketIntervalTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["tcpTransferSocketInterval"], out tcpTransferSocketIntervalTemp);
            if (parseResult) tcpTransferSocketInterval = tcpTransferSocketIntervalTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "tcpTransferSocketInterval" + ") is wrong.");
                return;
            }

            int udpTransferSocketSendMaxTimeOutTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["udpTransferSocketSendMaxTimeOut"], out udpTransferSocketSendMaxTimeOutTemp);
            if (parseResult) udpTransferSocketSendMaxTimeOut = udpTransferSocketSendMaxTimeOutTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "udpTransferSocketSendMaxTimeOut" + ") is wrong.");
                return;
            }

            int udpTransferSocketSendTimeOutTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["udpTransferSocketSendTimeOut"], out udpTransferSocketSendTimeOutTemp);
            if (parseResult) udpTransferSocketSendTimeOut = udpTransferSocketSendTimeOutTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "udpTransferSocketSendTimeOut" + ") is wrong.");
                return;
            }

            int sleepMsForQueueSendTemp;
            parseResult = int.TryParse(ConfigurationManager.AppSettings["sleepMsForQueueSend"], out sleepMsForQueueSendTemp);
            if (parseResult) sleepMsForQueueSend = sleepMsForQueueSendTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "sleepMsForQueueSend" + ") is wrong.");
                return;
            }

            double titleSizeTemp;
            parseResult = double.TryParse(ConfigurationManager.AppSettings["titleSize"], out titleSizeTemp);
            if (parseResult) titleSize = titleSizeTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "titleSize" + ") is wrong.");
                return;
            }

            double messageSizeTemp;
            parseResult = double.TryParse(ConfigurationManager.AppSettings["messageSize"], out messageSizeTemp);
            if (parseResult) messageSize = messageSizeTemp;
            else
            {
                ifAppConfRight = false;
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "App configuration parameter(" + "messageSize" + ") is wrong.");
                return;
            }

            // Binding
            Binding binding1 = new Binding();
            binding1.Source = this;
            binding1.Path = new PropertyPath("LeftImage");
            binding1.Mode = BindingMode.OneWay;
            binding1.UpdateSourceTrigger = UpdateSourceTrigger.PropertyChanged;
            BindingOperations.SetBinding(ShowBoxLeft, Image.SourceProperty, binding1);

            Binding binding2 = new Binding();
            binding2.Source = this;
            binding2.Path = new PropertyPath("RightImage");
            binding2.Mode = BindingMode.OneWay;
            binding2.UpdateSourceTrigger = UpdateSourceTrigger.PropertyChanged;
            BindingOperations.SetBinding(ShowBoxRight, Image.SourceProperty, binding2);

            // 装上TCP定时器
            tcpSendClocker = new System.Timers.Timer(tcpTransferSocketInterval);
            tcpSendClocker.AutoReset = true;
            tcpSendClocker.Elapsed += tcpSendClocker_Elapsed;

            tcpSendUltraClocker = new System.Timers.Timer(tcpTransferSocketInterval);
            tcpSendUltraClocker.AutoReset = true;
            tcpSendUltraClocker.Elapsed += tcpSendUltraClocker_Elapsed;

            // 装载图像显示
            udpBufferCancel = new CancellationTokenSource();
            udpBufferWorkTask = new Task(() => UDPBufferTaskWork(udpBufferCancel.Token));
            udpBufferWorkTask.Start();

            udpBufferUltraCancel = new CancellationTokenSource();
            udpBufferWorkUltraTask = new Task(() => UDPBufferUltraTaskWork(udpBufferUltraCancel.Token));
            udpBufferWorkUltraTask.Start();
        }

        private void MetroWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (!ifAppConfRight)
            {
                ShowCloseMsg("程序配置参数出错，确定关闭程序！", "错误");
                return;
            }

            Task.Run(new Action(AutoConnect));
        }

        private async void AutoConnect()
        {
            autoConnected = true;
            this.Dispatcher.BeginInvoke(new Action(AutoConectShow));

            int result = SwitchVideo();
            if (result == 0)
            {
                result = SwitchVideo(false);
                if (result == -2)
                {
                    await this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        ShowDialog("网络连接失败！", "问题");
                    }));
                }
                else if (result == -3)
                {
                    await this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        ShowCloseMsg("网络连接出现未知错误，确定关闭程序！", "错误");
                    }));
                }
                else if (result == -4)
                {
                    await this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        ShowCloseMsg("网络连接出现未知错误，确定关闭程序！", "错误");
                    }));
                }
            }
            else if (result == -2)
            {
                await this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    ShowDialog("网络连接失败！", "问题");
                }));
            }
            else if (result == -3)
            {
                await this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    ShowCloseMsg("网络连接出现未知错误，确定关闭程序！", "错误");
                }));
            }
            else if (result == -4)
            {
                await this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    ShowCloseMsg("网络连接出现未知错误，确定关闭程序！", "错误");
                }));
            }

            autoConnected = false;
        }
        private bool autoConnected = false;
        private async void AutoConectShow()
        {
            var controller = await this.ShowProgressAsync("请稍后", "正在尝试自动连接。。。", settings: new MetroDialogSettings()
            {
                AnimateShow = true,
                AnimateHide = false,
                DialogTitleFontSize = titleSize,
                DialogMessageFontSize = messageSize,
                ColorScheme = MetroDialogColorScheme.Theme
            });

            controller.SetIndeterminate();
            while (autoConnected) await Task.Delay(100);
            await controller.CloseAsync();
        }

        private async void CloseWindowsLater()
        {
            await Task.Delay(200);
            btnPowerOff_Click(null, null);
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

            if (!Object.Equals(udpTransferUltraRecieveTask, null))
                udpTransferRecieveTask.Wait();
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer stops to send datas, and udp stops too.");

            FinishAllConnection();

            ifTcpConnectionEstablished = false;
        }

        /// <summary>
        /// TCP发送队列数据任务
        /// </summary>
        /// <param name="cancelFlag">停止标志</param>
        private void TcpTransferUltraSendTaskWork(CancellationToken cancelFlag)
        {
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer begins to send datas.");

            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                Thread.Sleep(sleepMsForQueueSend);

                VideoTransferProtocolKey waitSentKey = VideoTransferProtocolKey.VideoTransfer;
                lock (queueUltraLocker)
                {
                    if (tcpUltraSendQueue.Count > 0)
                    {
                        waitSentKey = tcpUltraSendQueue.Dequeue();
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

                    byte[] publicKeyBytes = Encoding.UTF8.GetBytes(publicKeyUltra);
                    sendBytes.AddRange(BitConverter.GetBytes(IPAddress.HostToNetworkOrder(publicKeyBytes.Length)));
                    sendBytes.AddRange(publicKeyBytes);
                }
                try
                {
                    tcpTransferUltraSocket.Send(sendBytes.ToArray());
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode == SocketError.ConnectionReset || ex.SocketErrorCode == SocketError.ConnectionAborted || ex.SocketErrorCode == SocketError.TimedOut)
                    {
                        EndAllLoopUltra();
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer send datas failed.", ex);
                        break;
                    }
                    else
                    {
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                        EndAllLoopUltra(false);
                        break;
                    }
                }
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer send cmd '" + waitSentKey.ToString() + "'.");
            }

            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer stops to send datas.");

            if (!Object.Equals(udpTransferUltraRecieveTask, null))
                udpTransferUltraRecieveTask.Wait();
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp transfer stops to send datas, and udp stops too.");

            FinishAllConnectionUltra();

            ifTcpConnectionUltraEstablished = false;
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
        /// 发送指令
        /// </summary>
        private void SendCmdUltra(VideoTransferProtocolKey protocolKey)
        {
            lock (queueUltraLocker)
            {
                tcpUltraSendQueue.Enqueue(protocolKey);
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

                int actualLength = 0;
                // 接收收到的数据并处理
                byte[] recieveBuffer = new byte[maxVideoByteLength + 11];
                try
                {
                    actualLength = udpTransferSocket.ReceiveFrom(recieveBuffer, ref serverEndPoint);
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

                // 入队
                Task.Run(new Action(() =>
                {
                    lock (recBufferLocker)
                    {
                        if (recieveBufferQueue.Count >= recBufferMaxSize)
                            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Udp buffer is full, consider to fasten pic show.");
                        else
                            recieveBufferQueue.Enqueue(recieveBuffer.Take(actualLength).ToArray());
                    }
                }));
            }
        }

        /// <summary>
        /// UDP接收数据任务
        /// </summary>
        /// <param name="cancelFlag">停止标志</param>
        private void UDPTransferUltraRecieveTaskWork(CancellationToken cancelFlag)
        {
            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                int actualLength = 0;
                // 接收收到的数据并处理
                byte[] recieveBuffer = new byte[maxVideoByteLength + 11];
                try
                {
                    actualLength = udpTransferUltraSocket.ReceiveFrom(recieveBuffer, ref serverUltraEndPoint);
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode == SocketError.Interrupted || ex.SocketErrorCode == SocketError.TimedOut)
                    {
                        EndAllLoopUltra();
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor udp transfer recieve datas failed.", ex);
                        break;
                    }
                    else
                    {
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                        EndAllLoopUltra(false);
                        break;
                    }
                }
                if (!((IPEndPoint)serverUltraEndPoint).Address.Equals(IPAddress.Parse(ifAtSamePC ? serverIPAtSamePC : (ifAtSameLAN ? serverIPAtSameLAN : serverIPAtWAN)))) continue;

                // 入队
                Task.Run(new Action(() =>
                {
                    lock (recBufferUltraLocker)
                    {
                        if (recieveBufferQueueUltra.Count >= recBufferMaxSizeUltra)
                            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Udp buffer is full, consider to fasten pic show.");
                        else
                            recieveBufferQueueUltra.Enqueue(recieveBuffer.Take(actualLength).ToArray());
                    }
                }));
            }
        }

        /// <summary>
        /// UDP缓存区数据处理
        /// </summary>
        /// <param name="cancelFlag">停止标志</param>
        private void UDPBufferTaskWork(CancellationToken cancelFlag)
        {
            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                byte[] getBytes = null;
                lock (recBufferLocker)
                {
                    if (recieveBufferQueue.Count > 0)
                    {
                        getBytes = recieveBufferQueue.Dequeue();
                    }
                }
                if (Object.Equals(getBytes, null))
                {
                    Thread.Sleep(sleepMsForBuffer);
                    continue;
                }

                UDPRecieveDatasDeal(getBytes);
                //Thread.Sleep(sleepMsForBuffer);
            }
        }

        /// <summary>
        /// UDP缓存区数据处理
        /// </summary>
        /// <param name="cancelFlag">停止标志</param>
        private void UDPBufferUltraTaskWork(CancellationToken cancelFlag)
        {
            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                byte[] getBytes = null;
                lock (recBufferUltraLocker)
                {
                    if (recieveBufferQueueUltra.Count > 0)
                    {
                        getBytes = recieveBufferQueueUltra.Dequeue();
                    }
                }
                if (Object.Equals(getBytes, null))
                {
                    Thread.Sleep(sleepMsForUltraBuffer);
                    continue;
                }

                UDPRecieveDatasDealUltra(getBytes);
                //Thread.Sleep(sleepMsForBuffer);
            }
        }

        /// <summary>
        /// 处理UDP传输socket接收到的数据
        /// </summary>
        /// <param name="datas">所收数据</param>
        private void UDPRecieveDatasDeal(byte[] datas)
        {
            if (datas[0] != (byte)VideoTransferProtocolKey.Header1 || datas[1] != (byte)VideoTransferProtocolKey.Header2)
                return;

            if (datas[2] != clientDeviceIndex || (VideoTransferProtocolKey)datas[3] != VideoTransferProtocolKey.VideoTransfer)
                return;

            if (udpTransferSocket.ReceiveTimeout > udpTransferSocketSendTimeOut) udpTransferSocket.ReceiveTimeout = udpTransferSocketSendTimeOut;

            int packDataLength = Convert.ToInt32(
                             IPAddress.NetworkToHostOrder(
                             BitConverter.ToInt32(datas, 4)));
            byte packIndex = datas[8];
            byte packCount = datas[9];
            byte packNum = datas[10];

            if (packIndex == currentDealedPackIndex)
            {
                if (!currentRecDatas.ContainsKey(packNum) && currentRecDatas.Count < packCount)
                {
                    byte[] thisNumDatas = null;
                    if (packNum == packCount)
                        thisNumDatas = datas.Skip(11).Take(packDataLength - 3 - maxVideoByteLength * (packCount - 1)).ToArray();
                    else
                        thisNumDatas = datas.Skip(11).Take(maxVideoByteLength).ToArray();

                    currentRecDatas.Add(packNum, thisNumDatas);

                    if (currentRecDatas.Count == packCount)
                    {
                        List<byte> thisIndexDatas = new List<byte>();
                        for (byte i = 1; i <= packCount; ++i)
                        {
                            byte[] oneNumDatas;
                            bool getSuccess = currentRecDatas.TryGetValue(i, out oneNumDatas);
                            if (!getSuccess)
                            {
                                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "This index data can not be repacked.");
                                break;
                            }

                            thisIndexDatas.AddRange(oneNumDatas);
                        }

                        DecryptAndShowImg(thisIndexDatas.ToArray());
                    }
                }
            }
            else if (packIndex == lastDealedPackIndex)
            {
                if (!lastRecDatas.ContainsKey(packNum) && lastRecDatas.Count < packCount)
                {
                    byte[] thisNumDatas = null;
                    if (packNum == packCount)
                        thisNumDatas = datas.Skip(11).Take(packDataLength - 3 - maxVideoByteLength * (packCount - 1)).ToArray();
                    else
                        thisNumDatas = datas.Skip(11).Take(maxVideoByteLength).ToArray();

                    lastRecDatas.Add(packNum, thisNumDatas);

                    if (lastRecDatas.Count == packCount)
                    {
                        List<byte> thisIndexDatas = new List<byte>();
                        for (byte i = 1; i <= packCount; ++i)
                        {
                            byte[] oneNumDatas;
                            bool getSuccess = lastRecDatas.TryGetValue(i, out oneNumDatas);
                            if (!getSuccess)
                            {
                                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "This index data can not be repacked.");
                                break;
                            }

                            thisIndexDatas.AddRange(oneNumDatas);
                        }

                        DecryptAndShowImg(thisIndexDatas.ToArray());
                    }
                }
            }
            else if (packIndex == predictNextDealedPackIndex)
            {
                lastDealedPackIndex = currentDealedPackIndex;
                lastRecDatas.Clear();
                foreach (var item in currentRecDatas)
                    lastRecDatas.Add(item.Key, item.Value);

                currentDealedPackIndex = predictNextDealedPackIndex;
                currentRecDatas.Clear();
                byte[] thisNumDatas = null;
                if (packNum == packCount)
                    thisNumDatas = datas.Skip(11).Take(packDataLength - 3 - maxVideoByteLength * (packCount - 1)).ToArray();
                else
                    thisNumDatas = datas.Skip(11).Take(maxVideoByteLength).ToArray();
                currentRecDatas.Add(packNum, thisNumDatas);

                predictNextDealedPackIndex = Convert.ToByte(predictNextDealedPackIndex % byte.MaxValue + 1);

                if (packCount == 1 && packNum == 1)
                {
                    DecryptAndShowImg(thisNumDatas.ToArray());
                }
            }
            else
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Skipped index got.");
            }

            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Get package [" + packIndex.ToString() + "] of " + packDataLength.ToString() + " bytes with " + packCount + " segments " + packNum + ".");

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
        /// 处理UDP传输socket接收到的数据
        /// </summary>
        /// <param name="datas">所收数据</param>
        private void UDPRecieveDatasDealUltra(byte[] datas)
        {
            if (datas[0] != (byte)VideoTransferProtocolKey.Header1 || datas[1] != (byte)VideoTransferProtocolKey.Header2)
                return;

            if (datas[2] != clientDeviceIndex || (VideoTransferProtocolKey)datas[3] != VideoTransferProtocolKey.VideoTransfer)
                return;

            if (udpTransferUltraSocket.ReceiveTimeout > udpTransferSocketSendTimeOut) udpTransferUltraSocket.ReceiveTimeout = udpTransferSocketSendTimeOut;

            int packDataLength = Convert.ToInt32(
                             IPAddress.NetworkToHostOrder(
                             BitConverter.ToInt32(datas, 4)));
            byte packIndex = datas[8];
            byte packCount = datas[9];
            byte packNum = datas[10];

            if (packIndex == currentDealedPackIndexUltra)
            {
                if (!currentRecDatasUltra.ContainsKey(packNum) && currentRecDatasUltra.Count < packCount)
                {
                    byte[] thisNumDatas = null;
                    if (packNum == packCount)
                        thisNumDatas = datas.Skip(11).Take(packDataLength - 3 - maxVideoByteLength * (packCount - 1)).ToArray();
                    else
                        thisNumDatas = datas.Skip(11).Take(maxVideoByteLength).ToArray();

                    currentRecDatasUltra.Add(packNum, thisNumDatas);

                    if (currentRecDatasUltra.Count == packCount)
                    {
                        List<byte> thisIndexDatas = new List<byte>();
                        for (byte i = 1; i <= packCount; ++i)
                        {
                            byte[] oneNumDatas;
                            bool getSuccess = currentRecDatasUltra.TryGetValue(i, out oneNumDatas);
                            if (!getSuccess)
                            {
                                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "This index data can not be repacked.");
                                break;
                            }

                            thisIndexDatas.AddRange(oneNumDatas);
                        }

                        DecryptAndShowImgUltra(thisIndexDatas.ToArray());
                    }
                }
            }
            else if (packIndex == lastDealedPackIndexUltra)
            {
                if (!lastRecDatasUltra.ContainsKey(packNum) && lastRecDatasUltra.Count < packCount)
                {
                    byte[] thisNumDatas = null;
                    if (packNum == packCount)
                        thisNumDatas = datas.Skip(11).Take(packDataLength - 3 - maxVideoByteLength * (packCount - 1)).ToArray();
                    else
                        thisNumDatas = datas.Skip(11).Take(maxVideoByteLength).ToArray();

                    lastRecDatasUltra.Add(packNum, thisNumDatas);

                    if (lastRecDatasUltra.Count == packCount)
                    {
                        List<byte> thisIndexDatas = new List<byte>();
                        for (byte i = 1; i <= packCount; ++i)
                        {
                            byte[] oneNumDatas;
                            bool getSuccess = lastRecDatasUltra.TryGetValue(i, out oneNumDatas);
                            if (!getSuccess)
                            {
                                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "This index data can not be repacked.");
                                break;
                            }

                            thisIndexDatas.AddRange(oneNumDatas);
                        }

                        DecryptAndShowImgUltra(thisIndexDatas.ToArray());
                    }
                }
            }
            else if (packIndex == predictNextDealedPackIndexUltra)
            {
                lastDealedPackIndexUltra = currentDealedPackIndexUltra;
                lastRecDatasUltra.Clear();
                foreach (var item in currentRecDatasUltra)
                    lastRecDatasUltra.Add(item.Key, item.Value);

                currentDealedPackIndexUltra = predictNextDealedPackIndexUltra;
                currentRecDatasUltra.Clear();
                byte[] thisNumDatas = null;
                if (packNum == packCount)
                    thisNumDatas = datas.Skip(11).Take(packDataLength - 3 - maxVideoByteLength * (packCount - 1)).ToArray();
                else
                    thisNumDatas = datas.Skip(11).Take(maxVideoByteLength).ToArray();
                currentRecDatasUltra.Add(packNum, thisNumDatas);

                predictNextDealedPackIndexUltra = Convert.ToByte(predictNextDealedPackIndexUltra % byte.MaxValue + 1);

                if (packCount == 1 && packNum == 1)
                {
                    DecryptAndShowImgUltra(thisNumDatas.ToArray());
                }
            }
            else
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Skipped index got.");
            }

            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Get package [" + packIndex.ToString() + "] of " + packDataLength.ToString() + " bytes with " + packCount + " segments " + packNum + ".");

            if (!canUseUltraSwitchBtnNow)
            {
                canUseUltraSwitchBtnNow = true;

                this.Dispatcher.BeginInvoke(
                    new Action(() =>
                    {
                        btnSwitchUltra.IsEnabled = true;
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
            byte[] decryptedBytes = DecryptByAES(encryptedBytes);

            using (MemoryStream ms = new MemoryStream(decryptedBytes))
            {
                BitmapImage img = new BitmapImage();
                img.BeginInit();
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.StreamSource = ms;
                img.EndInit();
                img.Freeze();
                this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    LeftImage = BitmapFrame.Create(img);
                }));
            }
        }

        /// <summary>
        /// 解密并显示图像
        /// </summary>
        /// <param name="encryptedBytes">加密数据</param>
        private void DecryptAndShowImgUltra(byte[] encryptedBytes)
        {
            byte[] decryptedBytes = DecryptByAESUltra(encryptedBytes);

            using (MemoryStream ms = new MemoryStream(decryptedBytes))
            {
                BitmapImage img = new BitmapImage();
                img.BeginInit();
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.StreamSource = ms;
                img.EndInit();
                img.Freeze();
                this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    RightImage = BitmapFrame.Create(img);
                }));
            }
        }

        /// <summary>
        /// TCP传输心跳定时器
        /// </summary>
        private void tcpSendClocker_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            SendCmd(VideoTransferProtocolKey.PingSignal);
        }

        /// <summary>
        /// TCP传输心跳定时器
        /// </summary>
        private void tcpSendUltraClocker_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            SendCmdUltra(VideoTransferProtocolKey.PingSignal);
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
        }

        /// <summary>
        /// 结束所有循环等待
        /// </summary>
        /// <param name="noError">没有错误</param>
        private void EndAllLoopUltra(bool noError = true)
        {
            tcpTransferUltraCancel.Cancel();
            tcpSendUltraClocker.Stop();
            udpTransferUltraCancel.Cancel();
        }

        /// <summary>
        /// 结束所有连接
        /// </summary>
        private void FinishAllConnection()
        {
            tcpTransferSocket.Shutdown(SocketShutdown.Both);
            tcpTransferSocket.Close();

            if (!Object.Equals(udpTransferSocket, null))
            {
                udpTransferSocket.Shutdown(SocketShutdown.Both);
                udpTransferSocket.Close();
            }

            ifStartVideoShow = false;
            this.Dispatcher.BeginInvoke(
                new Action(() =>
                {
                    btnIcon.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PlayCircleRegular;
                    btnText.Text = "播放监控画面";
                    btnSwitchVideo.IsEnabled = true;
                }),
                DispatcherPriority.Normal);
        }

        /// <summary>
        /// 结束所有连接
        /// </summary>
        private void FinishAllConnectionUltra()
        {
            tcpTransferUltraSocket.Shutdown(SocketShutdown.Both);
            tcpTransferUltraSocket.Close();

            if (!Object.Equals(udpTransferUltraSocket, null))
            {
                udpTransferUltraSocket.Shutdown(SocketShutdown.Both);
                udpTransferUltraSocket.Close();
            }

            ifStartVideoShowUltra = false;
            this.Dispatcher.BeginInvoke(
                new Action(() =>
                {
                    btnIconUltra.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PlayCircleRegular;
                    btnTextUltra.Text = "打开超声图像";
                    btnSwitchUltra.IsEnabled = true;
                }),
                DispatcherPriority.Normal);
        }

        /// <summary>
        /// 播放/停止视频播放
        /// </summary>
        private void btnSwitchVideo_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(new Action(() =>
            {
                ManualConnect();
            }));
            e.Handled = true;
        }

        /// <summary>
        /// 播放/停止图像播放
        /// </summary>
        private void btnSwitchUltra_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(new Action(() =>
            {
                ManualConnect(false);
            }));
            e.Handled = true;
        }

        private async void ManualConnect(bool flag = true)
        {
            int result = SwitchVideo(flag);
            if (result == -2)
                await this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    ShowDialog("网络连接失败！", "问题");
                }));
            else if (result == -3)
                await this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    ShowCloseMsg("网络连接出现未知错误，确定关闭程序！", "错误");
                }));
            else if (result == -4) { ;}
        }

        private int SwitchVideo(bool ifVideo = true)
        {
            if (ifVideo)
            {
                if (!ifStartVideoShow)
                {
                    // TCP连接已经建立就退出
                    if (ifTcpConnectionEstablished) return -1;

                    this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        btnSwitchVideo.IsEnabled = false;
                    }));

                    // 获取RSA密钥
                    RSACryptoServiceProvider rsa = new RSACryptoServiceProvider(1024);
                    publicKey = rsa.ToXmlString(false);
                    privateKey = rsa.ToXmlString(true);

                    // 重置缓存
                    tcpSendQueue.Clear();

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
                            this.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                btnSwitchVideo.IsEnabled = true;
                            }));
                            return -2;
                        }
                        else
                        {
                            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                            this.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                btnSwitchVideo.IsEnabled = true;
                            }));
                            return -3;
                        }
                    }

                    ifTcpConnectionEstablished = true;
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp connection has been established.");

                    // 开始允许TCP传输socket发送队列内的数据
                    tcpTransferCancel = new CancellationTokenSource();
                    tcpTransferSendTask = new Task(() => TcpTransferSendTaskWork(tcpTransferCancel.Token));
                    tcpTransferSendTask.Start();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp tranfer can send datas.");

                    // 发送RSA公钥
                    SendCmd(VideoTransferProtocolKey.RSAKey);

                    // 接收AES密钥
                    int aesResult = RecieveAESKey();
                    if (aesResult < 0)
                    {
                        this.Dispatcher.BeginInvoke(new Action(() =>
                        {
                            btnSwitchVideo.IsEnabled = true;
                        }));
                        if (aesResult == -3) return -4;
                        else return aesResult;
                    }

                    // 重置标志
                    lastDealedPackIndex = 0;
                    currentDealedPackIndex = 0;
                    predictNextDealedPackIndex = 1;
                    lastRecDatas.Clear();
                    currentRecDatas.Clear();
                    recieveBufferQueue.Clear();

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

                    this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        btnIcon.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PauseCircleRegular;
                        btnText.Text = "停止监控画面";

                        canUseSwitchBtnNow = false;

                        btnSwitchVideo.IsEnabled = false;
                    }));
                }
                else
                {
                    // TCP连接未建立就退出
                    if (!ifTcpConnectionEstablished) return -1;

                    // 发送停止接收视频
                    SendCmd(VideoTransferProtocolKey.EndTransferVideo);

                    this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        btnIcon.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PlayCircleRegular;
                        btnText.Text = "播放监控画面";

                        btnSwitchVideo.IsEnabled = false;
                    }));
                }

                ifStartVideoShow = !ifStartVideoShow;

                return 0;
            }
            else
            {
                if (!ifStartVideoShowUltra)
                {
                    // TCP连接已经建立就退出
                    if (ifTcpConnectionUltraEstablished) return -1;

                    this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        btnSwitchUltra.IsEnabled = false;
                    }));

                    // 获取RSA密钥
                    RSACryptoServiceProvider rsa = new RSACryptoServiceProvider(1024);
                    publicKeyUltra = rsa.ToXmlString(false);
                    privateKeyUltra = rsa.ToXmlString(true);

                    // 重置缓存
                    tcpUltraSendQueue.Clear();

                    // 重新建立新的TCP连接
                    tcpTransferUltraSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                    tcpTransferUltraSocket.Bind(new IPEndPoint(IPAddress.Parse(ifAtSamePC ? clientIPAtSamePC : (ifAtSameLAN ? clientIPAtSameLAN : clientIPAtWAN)), ifAtSamePC ? clientPortTCPUltraAtSamePC : (ifAtSameLAN ? clientPortTCPUltraAtSameLAN : clientPortTCPUltraAtWAN)));
                    tcpTransferUltraSocket.SendTimeout = tcpTransferSocketSendTimeOut;
                    tcpTransferUltraSocket.ReceiveTimeout = tcpTransferSocketSendTimeOut;
                    try
                    {
                        tcpTransferUltraSocket.Connect(new IPEndPoint(IPAddress.Parse(ifAtSamePC ? serverIPAtSamePC : (ifAtSameLAN ? serverIPAtSameLAN : serverIPAtWAN)), serverPortTCPUltraAtAll));
                    }
                    catch (SocketException ex)
                    {
                        tcpTransferUltraSocket.Close();
                        if (ex.SocketErrorCode == SocketError.ConnectionRefused || ex.SocketErrorCode == SocketError.TimedOut)
                        {
                            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp connection can not established.", ex);
                            this.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                btnSwitchUltra.IsEnabled = true;
                            }));
                            return -2;
                        }
                        else
                        {
                            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                            this.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                btnSwitchUltra.IsEnabled = true;
                            }));
                            return -3;
                        }
                    }

                    ifTcpConnectionUltraEstablished = true;
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp connection has been established.");

                    // 开始允许TCP传输socket发送队列内的数据
                    tcpTransferUltraCancel = new CancellationTokenSource();
                    tcpTransferUltraSendTask = new Task(() => TcpTransferUltraSendTaskWork(tcpTransferUltraCancel.Token));
                    tcpTransferUltraSendTask.Start();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp tranfer can send datas.");

                    // 发送RSA公钥
                    SendCmdUltra(VideoTransferProtocolKey.RSAKey);

                    // 接收AES密钥
                    int aesResult = RecieveAESKeyUltra();
                    if (aesResult < 0)
                    {
                        this.Dispatcher.BeginInvoke(new Action(() =>
                        {
                            btnSwitchUltra.IsEnabled = true;
                        }));
                        if (aesResult == -3) return -4;
                        else return aesResult;
                    }

                    // 重置标志
                    lastDealedPackIndexUltra = 0;
                    currentDealedPackIndexUltra = 0;
                    predictNextDealedPackIndexUltra = 1;
                    lastRecDatasUltra.Clear();
                    currentRecDatasUltra.Clear();
                    recieveBufferQueueUltra.Clear();

                    // 开始接收视频
                    SendCmdUltra(VideoTransferProtocolKey.BeginTransferVideo);

                    // 开始允许UDP传输socket接收数据
                    udpTransferUltraSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                    udpTransferUltraSocket.Bind(new IPEndPoint(IPAddress.Parse(ifAtSamePC ? clientIPAtSamePC : (ifAtSameLAN ? clientIPAtSameLAN : clientIPAtWAN)), ifAtSamePC ? clientPortUDPUltraAtSamePC : (ifAtSameLAN ? clientPortUDPUltraAtSameLAN : clientPortUDPUltraAtWAN)));
                    udpTransferUltraSocket.ReceiveTimeout = udpTransferSocketSendMaxTimeOut;
                    udpTransferUltraCancel = new CancellationTokenSource();
                    udpTransferUltraRecieveTask = new Task(() => UDPTransferUltraRecieveTaskWork(udpTransferUltraCancel.Token));
                    udpTransferUltraRecieveTask.Start();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor udp tranfer can recieve datas.");

                    // 心跳发送定时器打开
                    tcpSendUltraClocker.Start();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor tcp begin to beat.");

                    this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        btnIconUltra.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PauseCircleRegular;
                        btnTextUltra.Text = "关闭超声图像";

                        canUseUltraSwitchBtnNow = false;

                        btnSwitchUltra.IsEnabled = false;
                    }));
                }
                else
                {
                    // TCP连接未建立就退出
                    if (!ifTcpConnectionUltraEstablished) return -1;

                    // 发送停止接收视频
                    SendCmdUltra(VideoTransferProtocolKey.EndTransferVideo);

                    this.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        btnIconUltra.Kind = MahApps.Metro.IconPacks.PackIconFontAwesomeKind.PlayCircleRegular;
                        btnTextUltra.Text = "打开超声图像";

                        btnSwitchUltra.IsEnabled = false;
                    }));
                }

                ifStartVideoShowUltra = !ifStartVideoShowUltra;

                return 0;
            }
        }

        /// <summary>
        /// 接收AES密钥
        /// </summary>
        /// <returns>接收结果</returns>
        private int RecieveAESKey()
        {
            try
            {
                byte[] reciveDatas = new byte[1024 + 8];
                int actualLength = tcpTransferSocket.Receive(reciveDatas);
                byte[] datas = reciveDatas.Take(actualLength).ToArray();

                if (datas.Length < 4) return -2; // 长度不可能出现
                if (datas[0] != (byte)VideoTransferProtocolKey.Header1 ||
                    datas[1] != (byte)VideoTransferProtocolKey.Header2) return -2; // 协议头不匹配

                byte deviceIndex = datas[2];
                if (deviceIndex != clientDeviceIndex) return -2;
                VideoTransferProtocolKey workCmd = (VideoTransferProtocolKey)datas[3];
                if (workCmd != VideoTransferProtocolKey.AESKey) return -2;

                int keyLength = Convert.ToInt32(
                                             IPAddress.NetworkToHostOrder(
                                             BitConverter.ToInt32(datas, 4)));
                if (keyLength != datas.Length - 8) return -2; // 长度不匹配

                byte[] aesKeys = DecryptByRSA(datas.Skip(8).ToArray());
                if (Object.Equals(aesKeys, null)) return -2;
                if (aesKeys.Length != (int)SecurityKeyLength.AESIVLength + (int)SecurityKeyLength.AESKeyLength) return -2;

                commonIV = aesKeys.Take((byte)SecurityKeyLength.AESIVLength).ToArray();
                commonKey = aesKeys.Skip((byte)SecurityKeyLength.AESIVLength).ToArray();

                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AESKey saved.");
                return 0;
            }
            catch (SocketException ex)
            {
                if (ex.SocketErrorCode == SocketError.ConnectionReset || ex.SocketErrorCode == SocketError.TimedOut)
                {
                    tcpTransferCancel.Cancel();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor recieve no AES key.");
                    return -2;
                }
                else
                {
                    tcpTransferCancel.Cancel();
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                    return -3;
                }
            }
        }

        /// <summary>
        /// 接收AES密钥
        /// </summary>
        /// <returns>接收结果</returns>
        private int RecieveAESKeyUltra()
        {
            try
            {
                byte[] reciveDatas = new byte[1024 + 8];
                int actualLength = tcpTransferUltraSocket.Receive(reciveDatas);
                byte[] datas = reciveDatas.Take(actualLength).ToArray();

                if (datas.Length < 4) return -2; // 长度不可能出现
                if (datas[0] != (byte)VideoTransferProtocolKey.Header1 ||
                    datas[1] != (byte)VideoTransferProtocolKey.Header2) return -2; // 协议头不匹配

                byte deviceIndex = datas[2];
                if (deviceIndex != clientDeviceIndex) return -2;
                VideoTransferProtocolKey workCmd = (VideoTransferProtocolKey)datas[3];
                if (workCmd != VideoTransferProtocolKey.AESKey) return -2;

                int keyLength = Convert.ToInt32(
                                             IPAddress.NetworkToHostOrder(
                                             BitConverter.ToInt32(datas, 4)));
                if (keyLength != datas.Length - 8) return -2; // 长度不匹配

                byte[] aesKeys = DecryptByRSAUltra(datas.Skip(8).ToArray());
                if (Object.Equals(aesKeys, null)) return -2;
                if (aesKeys.Length != (int)SecurityKeyLength.AESIVLength + (int)SecurityKeyLength.AESKeyLength) return -2;

                commonIVUltra = aesKeys.Take((byte)SecurityKeyLength.AESIVLength).ToArray();
                commonKeyUltra = aesKeys.Skip((byte)SecurityKeyLength.AESIVLength).ToArray();

                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AESKey saved.");
                return 0;
            }
            catch (SocketException ex)
            {
                if (ex.SocketErrorCode == SocketError.ConnectionReset || ex.SocketErrorCode == SocketError.TimedOut)
                {
                    tcpTransferUltraCancel.Cancel();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AssistantRobot supervisor recieve no AES key.");
                    return -2;
                }
                else
                {
                    tcpTransferUltraCancel.Cancel();
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Not deal exception.", ex);
                    return -3;
                }
            }
        }

        #region 加解密
        /// <summary>
        /// RSA密钥解密数据
        /// </summary>
        /// <param name="nonDecryptedBytes">待解密字节流</param>
        /// <returns>解密后的字节流</returns>
        private byte[] DecryptByRSA(byte[] nonDecryptedBytes)
        {
            if (Object.Equals(nonDecryptedBytes, null) || nonDecryptedBytes.Length < 1)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Datas for decrypting by RSA is abnormal.");
                return null; // 待解密数据异常
            }
            if (Object.Equals(publicKey, null) || Object.Equals(privateKey, null))
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "RSA keys have not been known yet.");
                return null; // RSA公密钥未知
            }

            byte[] decryptedBytes = null;
            using (RSACryptoServiceProvider rsa = new RSACryptoServiceProvider())
            {
                rsa.FromXmlString(privateKey);
                if (nonDecryptedBytes.Length > ((int)SecurityKeyLength.RSAKeyLength) / 8) return null; // 待解密数据过长

                decryptedBytes = rsa.Decrypt(nonDecryptedBytes, false);
            }
            return decryptedBytes;
        }

        /// <summary>
        /// RSA密钥解密数据
        /// </summary>
        /// <param name="nonDecryptedBytes">待解密字节流</param>
        /// <returns>解密后的字节流</returns>
        private byte[] DecryptByRSAUltra(byte[] nonDecryptedBytes)
        {
            if (Object.Equals(nonDecryptedBytes, null) || nonDecryptedBytes.Length < 1)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Datas for decrypting by RSA is abnormal.");
                return null; // 待解密数据异常
            }
            if (Object.Equals(publicKeyUltra, null) || Object.Equals(privateKeyUltra, null))
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "RSA keys have not been known yet.");
                return null; // RSA公密钥未知
            }

            byte[] decryptedBytes = null;
            using (RSACryptoServiceProvider rsa = new RSACryptoServiceProvider())
            {
                rsa.FromXmlString(privateKeyUltra);
                if (nonDecryptedBytes.Length > ((int)SecurityKeyLength.RSAKeyLength) / 8) return null; // 待解密数据过长

                decryptedBytes = rsa.Decrypt(nonDecryptedBytes, false);
            }
            return decryptedBytes;
        }

        /// <summary>
        /// AES加密数据
        /// </summary>
        /// <param name="nonEncryptedBytes">待加密字节流</param>
        /// <returns>加密后的字节流</returns>
        private byte[] EncryptByAES(byte[] nonEncryptedBytes)
        {
            if (Object.Equals(nonEncryptedBytes, null) || nonEncryptedBytes.Length < 1)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Datas for encrypting by AES is abnormal.");
                return null; // 待加密数据异常
            }
            if (Object.Equals(commonIV, null) ||
                Object.Equals(commonKey, null))
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AES key has not been known yet.");
                return null; // AES密钥和初始向量未知
            }

            string nonEncryptedString = Convert.ToBase64String(nonEncryptedBytes);

            byte[] encryptedBytes = null;
            using (AesCryptoServiceProvider aes = new AesCryptoServiceProvider())
            {
                aes.Key = commonKey; aes.IV = commonIV;
                ICryptoTransform encryptorByAES = aes.CreateEncryptor();

                using (MemoryStream msEncrypt = new MemoryStream())
                {
                    using (CryptoStream csEncrypt = new CryptoStream(msEncrypt, encryptorByAES, CryptoStreamMode.Write))
                    {
                        using (StreamWriter swEncrypt = new StreamWriter(csEncrypt))
                        {
                            swEncrypt.Write(nonEncryptedString);
                        }
                        encryptedBytes = msEncrypt.ToArray();
                    }
                }
            }

            return encryptedBytes;
        }

        /// <summary>
        /// AES加密数据
        /// </summary>
        /// <param name="nonEncryptedBytes">待加密字节流</param>
        /// <returns>加密后的字节流</returns>
        private byte[] EncryptByAESUltra(byte[] nonEncryptedBytes)
        {
            if (Object.Equals(nonEncryptedBytes, null) || nonEncryptedBytes.Length < 1)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Datas for encrypting by AES is abnormal.");
                return null; // 待加密数据异常
            }
            if (Object.Equals(commonIVUltra, null) ||
                Object.Equals(commonKeyUltra, null))
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AES key has not been known yet.");
                return null; // AES密钥和初始向量未知
            }

            string nonEncryptedString = Convert.ToBase64String(nonEncryptedBytes);

            byte[] encryptedBytes = null;
            using (AesCryptoServiceProvider aes = new AesCryptoServiceProvider())
            {
                aes.Key = commonKeyUltra; aes.IV = commonIVUltra;
                ICryptoTransform encryptorByAES = aes.CreateEncryptor();

                using (MemoryStream msEncrypt = new MemoryStream())
                {
                    using (CryptoStream csEncrypt = new CryptoStream(msEncrypt, encryptorByAES, CryptoStreamMode.Write))
                    {
                        using (StreamWriter swEncrypt = new StreamWriter(csEncrypt))
                        {
                            swEncrypt.Write(nonEncryptedString);
                        }
                        encryptedBytes = msEncrypt.ToArray();
                    }
                }
            }

            return encryptedBytes;
        }

        /// <summary>
        /// AES解密数据
        /// </summary>
        /// <param name="encryptedBytes">待解密字节流</param>
        /// <returns>解密后的字节流</returns>
        private byte[] DecryptByAES(byte[] encryptedBytes)
        {
            if (Object.Equals(encryptedBytes, null) || encryptedBytes.Length < 1)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Datas for decrypting by AES is abnormal.");
                return null; // 待解密数据异常
            }
            if (Object.Equals(commonIV, null) ||
                Object.Equals(commonKey, null))
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AES key has not been known yet.");
                return null; // AES密钥和初始向量未知
            }

            byte[] decryptedBytes = null;
            using (AesCryptoServiceProvider aes = new AesCryptoServiceProvider())
            {
                aes.Key = commonKey; aes.IV = commonIV;
                ICryptoTransform decryptorByAES = aes.CreateDecryptor();

                using (MemoryStream msDecrypt = new MemoryStream(encryptedBytes))
                {
                    using (CryptoStream csDecrypt = new CryptoStream(msDecrypt, decryptorByAES, CryptoStreamMode.Read))
                    {
                        using (StreamReader swDecrypt = new StreamReader(csDecrypt))
                        {
                            string decryptedString = swDecrypt.ReadToEnd();
                            decryptedBytes = Convert.FromBase64String(decryptedString);
                        }
                    }
                }
            }
            return decryptedBytes;
        }

        /// <summary>
        /// AES解密数据
        /// </summary>
        /// <param name="encryptedBytes">待解密字节流</param>
        /// <returns>解密后的字节流</returns>
        private byte[] DecryptByAESUltra(byte[] encryptedBytes)
        {
            if (Object.Equals(encryptedBytes, null) || encryptedBytes.Length < 1)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Datas for decrypting by AES is abnormal.");
                return null; // 待解密数据异常
            }
            if (Object.Equals(commonIVUltra, null) ||
                Object.Equals(commonKeyUltra, null))
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "AES key has not been known yet.");
                return null; // AES密钥和初始向量未知
            }

            byte[] decryptedBytes = null;
            using (AesCryptoServiceProvider aes = new AesCryptoServiceProvider())
            {
                aes.Key = commonKeyUltra; aes.IV = commonIVUltra;
                ICryptoTransform decryptorByAES = aes.CreateDecryptor();

                using (MemoryStream msDecrypt = new MemoryStream(encryptedBytes))
                {
                    using (CryptoStream csDecrypt = new CryptoStream(msDecrypt, decryptorByAES, CryptoStreamMode.Read))
                    {
                        using (StreamReader swDecrypt = new StreamReader(csDecrypt))
                        {
                            string decryptedString = swDecrypt.ReadToEnd();
                            decryptedBytes = Convert.FromBase64String(decryptedString);
                        }
                    }
                }
            }
            return decryptedBytes;
        }
        #endregion

        private async void btnPowerOff_Click(object sender, RoutedEventArgs e)
        {
            var controller = await this.ShowProgressAsync("请稍后", "正在关闭程序。。。", settings: new MetroDialogSettings()
            {
                AnimateShow = false,
                AnimateHide = false,
                DialogTitleFontSize = titleSize,
                DialogMessageFontSize = messageSize,
                ColorScheme = MetroDialogColorScheme.Theme
            });

            controller.SetIndeterminate();

            if (ifTcpConnectionEstablished)
            {
                // 发送停止接收视频
                SendCmd(VideoTransferProtocolKey.EndTransferVideo);
            }

            if (ifTcpConnectionUltraEstablished)
            {
                // 发送停止接收视频
                SendCmdUltra(VideoTransferProtocolKey.EndTransferVideo);
            }

            int numCount = 0;
            while (ifTcpConnectionEstablished || ifTcpConnectionUltraEstablished)
            {
                numCount++;
                await Task.Delay(100);
                if (numCount > 80) break;
            }

            if (ifTcpConnectionEstablished) EndAllLoop();
            while (ifTcpConnectionEstablished) await Task.Delay(100);

            if (ifTcpConnectionUltraEstablished) EndAllLoopUltra();
            while (ifTcpConnectionUltraEstablished) await Task.Delay(100);

            udpBufferCancel.Cancel();
            udpBufferWorkTask.Wait();

            udpBufferUltraCancel.Cancel();
            udpBufferWorkUltraTask.Wait();

            await controller.CloseAsync();
            this.Close();
        }

        private void btnEnlarge_Click(object sender, RoutedEventArgs e)
        {
            if (enlargeIcon.Kind == MahApps.Metro.IconPacks.PackIconOcticonsKind.ScreenFull)
            {
                WindowState = WindowState.Maximized;
                enlargeIcon.Kind = MahApps.Metro.IconPacks.PackIconOcticonsKind.ScreenNormal;
            }
            else
            {
                WindowState = WindowState.Normal;
                enlargeIcon.Kind = MahApps.Metro.IconPacks.PackIconOcticonsKind.ScreenFull;
            }
        }
    }

}
