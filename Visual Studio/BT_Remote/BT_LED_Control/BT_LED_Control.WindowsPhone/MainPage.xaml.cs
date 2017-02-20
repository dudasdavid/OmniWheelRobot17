using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

using Windows.Networking.Sockets;
using Windows.Networking;
using Windows.Storage.Streams;
using Windows.Devices.Bluetooth.Rfcomm;
using Windows.Devices.Enumeration;
using Windows.UI.Popups;
using System.Diagnostics;
using Windows.UI.Xaml.Media.Imaging;
using Windows.Graphics.Imaging;

//using System.Threading;
using Windows.System.Threading;
// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=234238

namespace BT_LED_Control
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {

        StreamSocket socket = null;
        HostName hostName = null;
        DataWriter writer = null;
        DataReader reader = null;
        RfcommDeviceService BTService = null;
        DeviceInformation BTDevice = null;
        DeviceInformationCollection DeviceInfoCollection = null;

        bool IsConnected = false;


        RPhi polarCoordinates = new RPhi(0, 0);
        XY XYCoordinates = new XY(0,0);
        DC speedsLR = new DC(0, 0);

        byte leftSliderValue = 100;
        byte rightSliderValue = 100;

        byte length = 0;
        byte angle = 0;

        int scaleFactor = 50;
        //Int16 ctr = 0;
        int leftRefreshCtr = 0;
        int rightRefreshCtr = 0;
        int refreshInterval = 4;
        int debounceCounter = 3;
        bool highSpeed = false;

        

        public struct RPhi
        {
            public RPhi(double r, double phi)
            {
                this.R = r;
                this.Phi = phi;
            }
            public double R;
            public double Phi;
        }

        public struct XY
        {
            public XY(double x, double y)
            {
                this.X = x;
                this.Y = y;
            }
            public double X;
            public double Y;
        }

        public struct DC
        {
            public DC(Byte l, Byte r)
            {
                this.L = l;
                this.R = r;
            }
            public Byte L;
            public Byte R;
        }

        public MainPage()
        {
            this.InitializeComponent();
            hostName = new HostName("BT115200");

            int period = 50;

            ThreadPoolTimer PeriodicTimer =
                ThreadPoolTimer.CreatePeriodicTimer(TimerElapsedHandler,
                                                    TimeSpan.FromMilliseconds(period));
            Pivot.IsLocked = true;
            this.NavigationCacheMode = NavigationCacheMode.Required;
        }


        /// <summary>
        /// Invoked when this page is about to be displayed in a Frame.
        /// </summary>
        /// <param name="e">Event data that describes how this page was reached.
        /// This parameter is typically used to configure the page.</param>
        protected override void OnNavigatedTo(NavigationEventArgs e)
        {
            // TODO: Prepare page for display here.

            // TODO: If your application contains multiple pages, ensure that you are
            // handling the hardware Back button by registering for the
            // Windows.Phone.UI.Input.HardwareButtons.BackPressed event.
            // If you are using the NavigationHelper provided by some templates,
            // this event is handled for you.
        }

        private RPhi XY2RPhi(double X, double Y)
        {
            RPhi tempPolarCoordinates = new RPhi(0, 0);
            double tempPhi;

            tempPolarCoordinates.R = Math.Sqrt(X * X + Y * Y);
            tempPhi = Math.Atan2(-Y, X);
            if (tempPhi >= 0) tempPolarCoordinates.Phi = tempPhi;
            else tempPolarCoordinates.Phi = tempPhi + 2 * Math.PI;
            return tempPolarCoordinates;
        }

        private XY RPhi2XY(double R, double Phi)
        {
            XY tempXY = new XY(0, 0);
            tempXY.X = R * Math.Cos(Phi);
            tempXY.Y = R * Math.Sin(Phi + Math.PI);

            return tempXY;
        }

        private DC rphi2dc(double r, double phi)
        {
            DC speedLeftRight = new DC(0, 0);

            if ((phi >= 0) && (phi < Math.PI / 2.0)) //1st quadrant
            {
                speedLeftRight.L = Convert.ToByte(r * scaleFactor + 100);
                speedLeftRight.R = Convert.ToByte(-r * scaleFactor * Math.Cos(2 * phi) + 100);
            }
            else if ((phi >= Math.PI / 2.0) && (phi < Math.PI)) //2nd quadrant
            {
                speedLeftRight.L = Convert.ToByte(-r * scaleFactor * Math.Cos(2 * phi) + 100);
                speedLeftRight.R = Convert.ToByte(r * scaleFactor + 100);
            }
            else if ((phi >= Math.PI) && (phi < 3 * Math.PI / 2.0)) //3rd quadrant
            {
                speedLeftRight.L = Convert.ToByte(-r * scaleFactor + 100);
                speedLeftRight.R = Convert.ToByte(r * scaleFactor * Math.Cos(2 * phi) + 100);
            }
            else if ((phi >= 3 * Math.PI / 2.0) && (phi <= 2 * Math.PI)) //4th quadrant
            {
                speedLeftRight.L = Convert.ToByte(r * scaleFactor * Math.Cos(2 * phi) + 100);
                speedLeftRight.R = Convert.ToByte(-r * scaleFactor + 100);
            }
            return speedLeftRight;
        }

        private async void Connect(object sender, RoutedEventArgs e)
        {
            ConnectButton.IsEnabled = false;
            if (IsConnected == false)
            {
                DeviceInfoCollection = await DeviceInformation.FindAllAsync(RfcommDeviceService.GetDeviceSelector(RfcommServiceId.SerialPort));
                var numDevices = DeviceInfoCollection.Count();
                if (numDevices == 0)
                {
                    MessageDialog md = new MessageDialog("No paired devices found!", "Warning");
                    await md.ShowAsync();
                    ConnectButton.IsEnabled = true;
                    return;
                }
                else
                {
                    foreach (var deviceInfo in DeviceInfoCollection)
                    {
                        if (deviceInfo.Name == "BT115200") BTDevice = deviceInfo;
                    }
                }

                if (BTDevice == null)
                {
                    MessageDialog md = new MessageDialog("Robot car is not paired to your device!", "Warning");
                    await md.ShowAsync();
                    ConnectButton.IsEnabled = true;
                    return;
                }
                BTService = await RfcommDeviceService.FromIdAsync(BTDevice.Id);
                if (BTService == null)
                {
                    MessageDialog md = new MessageDialog("Access to the device is denied because the application was not granted access!", "Warning");
                    await md.ShowAsync();
                    ConnectButton.IsEnabled = true;
                    return;
                }

                lock (this) socket = new StreamSocket();

                try
                {
                    await socket.ConnectAsync(BTService.ConnectionHostName, BTService.ConnectionServiceName);
                    reader = new DataReader(socket.InputStream);
                    writer = new DataWriter(socket.OutputStream);

                    //writer.WriteString("OK\r");
                    //await writer.StoreAsync();

                    //uint count = await reader.LoadAsync(3);
                    //uint byteCount = reader.UnconsumedBufferLength;
                    //Debug.WriteLine(byteCount);

                    //string receivedData = reader.ReadString(byteCount);
                    //Debug.WriteLine(receivedData);

                    //if (receivedData == "OK\r") Debug.WriteLine("Minden franko");
                    IsConnected = true;
                    ConnectButton.IsEnabled = true;
                    ConnectButton.Icon = new SymbolIcon(Symbol.Cancel);
                    ConnectButton.Label = "Disconnect";
                }
                catch
                {
                    Disconnect();
                    MessageDialog md = new MessageDialog("Uhh oh something went wrong!", "Warning");
                    md.ShowAsync();
                    ConnectButton.IsEnabled = true;
                    AppBar.ClosedDisplayMode = AppBarClosedDisplayMode.Minimal;
                    return;
                }
            }
            else
            {
                Disconnect();
                IsConnected = false;
                ConnectButton.Icon = new SymbolIcon(Symbol.Accept);
                ConnectButton.Label = "Connect";
                ConnectButton.IsEnabled = true;
                AppBar.ClosedDisplayMode = AppBarClosedDisplayMode.Compact;
            }
        }

        private void Disconnect()
        {
            if (reader != null)
            {
                reader.DetachStream();
                reader.Dispose();
                reader = null;
            }
            if (writer != null)
            {
                writer.DetachStream();
                writer.Dispose();
                writer = null;
            }
            if (socket != null)
            {
                lock (this)
                {
                    socket.Dispose();
                }
                socket = null;
            }
            BTService = null;
            BTDevice = null;
            DeviceInfoCollection = null;
        }

        private void SwitchHighSpeed(object sender, RoutedEventArgs e)
        {
            if (highSpeed)
            {
                highSpeed = false;
                PowerButton.Label = "HiPower ON";
                scaleFactor = 50;
                hiPowerTextBlock.Visibility = Visibility.Collapsed;
            }
            else
            {
                highSpeed = true;
                PowerButton.Label = "HiPower OFF";
                scaleFactor = 100;
                hiPowerTextBlock.Visibility = Visibility.Visible;
            }
        }

        private void IntensityChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (writer != null)
            {
                int intensity = (int)IntensitySlider.Value;
                //if (intensity == 13) { intensity = 14; }
                //writer.WriteByte(0x49);
                //writer.WriteByte((byte)intensity);
                //writer.WriteByte(0x0D);
                //await writer.StoreAsync();
            }
        }

        private async void TimerElapsedHandler(ThreadPoolTimer timer)
        {
            if (writer != null)
            {

                writer.WriteByte(0xFF);
                writer.WriteByte(length);
                writer.WriteByte(angle);

                await writer.StoreAsync();

                //Debug.WriteLine("{0} ; {1}", length, angle);

                debounceCounter--;
                if (debounceCounter <= 0)
                {
                    debounceCounter = 0;
                    leftSliderValue = 0;
                    rightSliderValue = 0;
                    length = 0;
                    angle = 0;
                }
            }
        }

        private void ColorCanvasPointerStarted(object sender, PointerRoutedEventArgs e)
        {
            //xPos = e.GetCurrentPoint(ColorMap).Position.X;
            //yPos = e.GetCurrentPoint(ColorMap).Position.Y;
            //Canvas.SetLeft(ColorPicker, xPos - ColorPicker.Width / 2);
            //Canvas.SetTop(ColorPicker, yPos - ColorPicker.Width / 2);
            ColorPicker.Opacity = 0.5;
            Pivot.IsLocked = true;
            e.Handled = true;
        }

        private void ColorCanvasPointerMoved(object sender, PointerRoutedEventArgs e)
        {
            

            XYCoordinates.X = e.GetCurrentPoint(ColorMap).Position.X - ColorMap.Width/2;
            XYCoordinates.Y = e.GetCurrentPoint(ColorMap).Position.Y - ColorMap.Height/2;

            polarCoordinates = XY2RPhi(XYCoordinates.X, XYCoordinates.Y);
            //textBox1.Text = Convert.ToString(XYCoordinates.X);
            //textBox2.Text = Convert.ToString(XYCoordinates.Y);
            //textBox3.Text = Convert.ToString(polarCoordinates.R);
            //textBox4.Text = Convert.ToString(polarCoordinates.Phi);
            //Debug.WriteLine("{0} ; {1}", XYCoordinates.X, XYCoordinates.Y);
            //Debug.WriteLine("{0} ; {1}", polarCoordinates.R, polarCoordinates.Phi);


            if (polarCoordinates.R < 20)
            {
                polarCoordinates.R = 0;
                XYCoordinates.X = 0;
                XYCoordinates.Y = 0;
            }
            if (polarCoordinates.R > ColorMap.Width / 2 - ColorPicker.Width / 2)
            {
                //e.Handled = true;
                //return;
                polarCoordinates.R = ColorMap.Width / 2 - ColorPicker.Width / 2;
                XYCoordinates = RPhi2XY(polarCoordinates.R, polarCoordinates.Phi);
            }

            Canvas.SetLeft(ColorPicker, XYCoordinates.X + ColorMap.Width / 2 - ColorPicker.Width / 2);
            Canvas.SetTop(ColorPicker, XYCoordinates.Y + ColorMap.Height / 2 - ColorPicker.Height / 2);
            Canvas.SetLeft(HorizontalLine, XYCoordinates.X + ColorMap.Width / 2);
            Canvas.SetTop(VerticalLine, XYCoordinates.Y + ColorMap.Height / 2);

            //ctr++;
            //Debug.WriteLine(ctr);
            speedsLR = rphi2dc(polarCoordinates.R/(ColorMap.Width / 2 - ColorPicker.Width / 2), polarCoordinates.Phi);
            //Debug.WriteLine("{0} ; {1}", speedsLR.L, speedsLR.R);
            //Debug.WriteLine("{0} ; {1}", polarCoordinates.R / (ColorMap.Width / 2 - ColorPicker.Width / 2) * 100, polarCoordinates.Phi / 2 / Math.PI * 360);
            leftSliderValue = speedsLR.L;
            rightSliderValue = speedsLR.R;

            length = Convert.ToByte(polarCoordinates.R / (ColorMap.Width / 2 - ColorPicker.Width / 2) * 100);
            angle = Convert.ToByte(polarCoordinates.Phi / 2 / Math.PI * 180);

            debounceCounter = 3;

            e.Handled = true;
        }

        private async void CanvasPointerReleased(object sender, PointerRoutedEventArgs e)
        {

            leftSliderValue = 100;
            rightSliderValue = 100;

            double xDistance = ((RemoteCanvas.Width / 2) - Canvas.GetLeft(HorizontalLine)) / 20;
            double yDistance = ((RemoteCanvas.Height / 2) - Canvas.GetTop(VerticalLine)) / 20;

            for (int i = 0; i < 10; i++)
            {
                Canvas.SetLeft(ColorPicker, Canvas.GetLeft(ColorPicker) + xDistance);
                Canvas.SetTop(ColorPicker, Canvas.GetTop(ColorPicker) + yDistance);
                Canvas.SetLeft(HorizontalLine, Canvas.GetLeft(HorizontalLine) + xDistance);
                Canvas.SetTop(VerticalLine, Canvas.GetTop(VerticalLine) + yDistance);
                ColorPicker.Opacity -= 0.01;
                await System.Threading.Tasks.Task.Delay(10);
            }
            Canvas.SetLeft(ColorPicker, RemoteCanvas.Width / 2 - ColorPicker.Width / 2);
            Canvas.SetTop(ColorPicker, RemoteCanvas.Height / 2 - ColorPicker.Height / 2);
            Canvas.SetLeft(HorizontalLine, RemoteCanvas.Width / 2);
            Canvas.SetTop(VerticalLine, RemoteCanvas.Height / 2);
            ColorPicker.Opacity = 0.3;

            //Pivot.IsLocked = false;
            e.Handled = true;
        }

        private void LeftCanvasPointerStarted(object sender, PointerRoutedEventArgs e)
        {
            LeftCircle.Opacity = 0.5;
            Pivot.IsLocked = true;
            e.Handled = true;
        }

        private void LeftCanvasPointerMoved(object sender, PointerRoutedEventArgs e)
        {
            double y = e.GetCurrentPoint(LeftPic).Position.Y;
            double tempValue = 0;
            //Debug.WriteLine("{0}", y);
            leftRefreshCtr++;

            if (y > LeftPic.Height - LeftCircle.Height / 2)
            {
                y = LeftPic.Height - LeftCircle.Height / 2;
            }
            if (y < LeftCircle.Height / 2)
            {
                y = LeftCircle.Height / 2;
            }
            if ((y > LeftPic.Height / 2 - 15) && (y < LeftPic.Height / 2  + 15))
            {
                y = LeftPic.Height / 2;
            }

            tempValue = (-1*(y - LeftPic.Height) - LeftCircle.Height / 2) / (LeftPic.Height - LeftCircle.Height) * scaleFactor*2 + (200 - scaleFactor*2)/2;
            //Debug.WriteLine("{0}", tempValue);
            leftSliderValue = Convert.ToByte(tempValue);
            debounceCounter = 3;

            if (leftRefreshCtr % refreshInterval == 0)
            {
                Canvas.SetTop(LeftCircle, y - LeftCircle.Height / 2);
            }


            e.Handled = true;
        }

        private async void LeftCanvasPointerReleased(object sender, PointerRoutedEventArgs e)
        {
            leftSliderValue = 100;

            double yDistance = ((LeftCanvas.Height / 2 - LeftCircle.Width / 2) - Canvas.GetTop(LeftCircle)) / 20;

            for (int i = 0; i < 10; i++)
            {
                Canvas.SetTop(LeftCircle, Canvas.GetTop(LeftCircle) + yDistance);
                LeftCircle.Opacity -= 0.01;
                await System.Threading.Tasks.Task.Delay(10);
            }

            Canvas.SetTop(LeftCircle, LeftCanvas.Height / 2 - LeftCircle.Width / 2);
            LeftCircle.Opacity = 0.3;

            //Pivot.IsLocked = false;
            e.Handled = true;
        }

        private void RightCanvasPointerStarted(object sender, PointerRoutedEventArgs e)
        {
            RightCircle.Opacity = 0.5;
            Pivot.IsLocked = true;
            e.Handled = true;
        }

        private void RightCanvasPointerMoved(object sender, PointerRoutedEventArgs e)
        {
            double y = e.GetCurrentPoint(RightPic).Position.Y;
            double tempValue = 0;
            //Debug.WriteLine("{0}", y);
            rightRefreshCtr++;

            if (y > RightPic.Height - RightCircle.Height / 2)
            {
                y = RightPic.Height - RightCircle.Height / 2;
            }
            if (y < RightCircle.Height / 2)
            {
                y = RightCircle.Height / 2;
            }
            if ((y > RightPic.Height / 2 - 15) && (y < RightPic.Height / 2 + 15))
            {
                y = RightPic.Height / 2;
            }

            tempValue = (-1 * (y - RightPic.Height) - RightCircle.Height / 2) / (RightPic.Height - RightCircle.Height) * scaleFactor*2 + (200 - scaleFactor*2) / 2;
            //Debug.WriteLine("{0}", tempValue);
            rightSliderValue = Convert.ToByte(tempValue);
            debounceCounter = 3;

            if (rightRefreshCtr % refreshInterval == 0)
            {
                Canvas.SetTop(RightCircle, y - RightCircle.Height / 2);
            }

            e.Handled = true;
        }

        private async void RightCanvasPointerReleased(object sender, PointerRoutedEventArgs e)
        {
            
            rightSliderValue = 100;

            double yDistance = ((RightCanvas.Height / 2 - RightCircle.Width / 2) - Canvas.GetTop(RightCircle)) / 20;

            for (int i = 0; i < 10; i++)
            {
                Canvas.SetTop(RightCircle, Canvas.GetTop(RightCircle) + yDistance);
                RightCircle.Opacity -= 0.01;
                await System.Threading.Tasks.Task.Delay(10);
            }

            Canvas.SetTop(RightCircle, RightCanvas.Height / 2 - RightCircle.Width / 2);
            RightCircle.Opacity = 0.3;


            //Pivot.IsLocked = false;
            e.Handled = true;
        }

        private void DirectionPointerStarted(object sender, PointerRoutedEventArgs e)
        {

        }

        private void DirectionPointerMoved(object sender, PointerRoutedEventArgs e)
        {

        }

        private void DirectionPointerReleased(object sender, PointerRoutedEventArgs e)
        {

        }
    }

}
