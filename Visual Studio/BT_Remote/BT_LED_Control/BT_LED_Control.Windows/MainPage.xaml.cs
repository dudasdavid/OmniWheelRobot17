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
        WriteableBitmap writeableBmp = null;
        Uri imageUri1 = null;
        Brush brush = null;
        Windows.UI.Color color;

        RPhi polarCoordinates = new RPhi(0, 0);
        XY XYCoordinates = new XY(0, 0);

        int radialLimit = 150;

        double R;
        double Rmin = 3.42;
        double Rmax = 1.52;

        double B;
        double Bmin = 0;
        double Bmax = 3.42;

        double G;
        double Gmin = 1.52;
        double Gmax = 2 * Math.PI;

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

        public MainPage()
        {
            this.InitializeComponent();
            hostName = new HostName("HC-06");
            //LoadPicture();
        }


        private async void ConnectClick(object sender, RoutedEventArgs e)
        {

            DeviceInfoCollection = await DeviceInformation.FindAllAsync(RfcommDeviceService.GetDeviceSelector(RfcommServiceId.SerialPort));
            var numDevices = DeviceInfoCollection.Count();
            //Debug.WriteLine(numDevices);
            if (numDevices == 0)
            {
                MessageDialog md = new MessageDialog("No paired devices found!", "Title");
                await md.ShowAsync();
                return;
            }
            else
            {
                // Found paired devices.
                foreach (var deviceInfo in DeviceInfoCollection)
                {
                    //_pairedDevices.Add(new PairedDeviceInfo(deviceInfo));
                    //Debug.WriteLine(deviceInfo.Name);
                    if (deviceInfo.Name == "HC-06")
                    {
                        BTDevice = deviceInfo;
                    }
                }
            }

            if (BTDevice == null)
            {
                MessageDialog md = new MessageDialog("LED lamp is not paired to your device!", "Title");
                await md.ShowAsync();
                return;
            }
            BTService = await RfcommDeviceService.FromIdAsync(BTDevice.Id);
            if (BTService == null)
            {
                MessageDialog md = new MessageDialog("Access to the device is denied because the application was not granted access!", "Title");
                await md.ShowAsync();
                return;
            }

            lock (this)
            {
                socket = new StreamSocket();
            }
            //socket.Control.KeepAlive = true;
            //socket.Control.NoDelay = true;
            try
            {
                await socket.ConnectAsync(BTService.ConnectionHostName, BTService.ConnectionServiceName);
                reader = new DataReader(socket.InputStream);
                writer = new DataWriter(socket.OutputStream);

                writer.WriteString("OK\r");
                await writer.StoreAsync();

                uint count = await reader.LoadAsync(3);
                uint byteCount = reader.UnconsumedBufferLength;
                Debug.WriteLine(byteCount);
                //BufferText.Text = byteCount.ToString();
                //BufferBar.Value = byteCount;
                string receivedData = reader.ReadString(byteCount);
                Debug.WriteLine(receivedData);

                if (receivedData == "OK\r") Debug.WriteLine("Minden franko");

            }
            catch
            {
                MessageDialog md = new MessageDialog("Uhh oh something went wrong!", "Title");
                md.ShowAsync();
                return;
            }
        }

        private void DisconnectClick(object sender, RoutedEventArgs e)
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

        private async void SendClick(object sender, RoutedEventArgs e)
        {
            writer.WriteString(SendText.Text + "\r");
            await writer.StoreAsync();
        }

        private async void IntensityChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            int intensity = (int)IntensitySlider.Value;
            if (intensity == 13) { intensity = 14; }
            writer.WriteByte(0x49);
            writer.WriteByte((byte)intensity);
            writer.WriteByte(0x0D);
            await writer.StoreAsync();
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            Application.Current.Exit();
        }

        private void ColorCanvasPointerStarted(object sender, PointerRoutedEventArgs e)
        {
            //xPos = e.GetCurrentPoint(ColorMap).Position.X;
            //yPos = e.GetCurrentPoint(ColorMap).Position.Y;
            //Canvas.SetLeft(ColorPicker, xPos - ColorPicker.Width / 2);
            //Canvas.SetTop(ColorPicker, yPos - ColorPicker.Width / 2);

            e.Handled = true;
        }

        private async void ColorCanvasPointerMoved(object sender, PointerRoutedEventArgs e)
        {
            XYCoordinates.X = e.GetCurrentPoint(ColorMap).Position.X - ColorMap.Width / 2;
            XYCoordinates.Y = e.GetCurrentPoint(ColorMap).Position.Y - ColorMap.Height / 2;

            polarCoordinates = XY2RPhi(XYCoordinates.X, XYCoordinates.Y);
            //Debug.WriteLine("{0} ; {1}", polarCoordinates.R, polarCoordinates.Phi);

            if (polarCoordinates.R > ColorMap.Width / 2 - ColorPicker.Width / 2)
            {
                e.Handled = true;
                return;
            }

            Canvas.SetLeft(ColorPicker, XYCoordinates.X + ColorMap.Width / 2 - ColorPicker.Width / 2);
            Canvas.SetTop(ColorPicker, XYCoordinates.Y + ColorMap.Height / 2 - ColorPicker.Height / 2);

            if ((polarCoordinates.Phi < Rmax) || (polarCoordinates.Phi > Rmin))
            {
                if ((polarCoordinates.Phi >= 4.15) || (polarCoordinates.Phi <= 0.5)) R = 255;
                else if (polarCoordinates.Phi <= Rmax) R = 255 * (Rmax - polarCoordinates.Phi) / (Rmax - 0.5);
                else R = 255 * (polarCoordinates.Phi - Rmin) / (4.15 - Rmin);
            }
            else R = 0;

            if ((polarCoordinates.Phi <= Bmax) && (polarCoordinates.Phi >= Bmin))
            {
                if ((polarCoordinates.Phi >= 0.5) && (polarCoordinates.Phi <= 2.45)) B = 255;
                else if (polarCoordinates.Phi < 0.5) B = 255 * (Bmin - polarCoordinates.Phi) / (Bmin - 0.5);
                else B = 255 * (Bmax - polarCoordinates.Phi) / (Bmax - 2.45);
            }
            else B = 0;

            if ((polarCoordinates.Phi <= Gmax) && (polarCoordinates.Phi >= Gmin))
            {
                if ((polarCoordinates.Phi >= 2.45) && (polarCoordinates.Phi <= 4.15)) G = 255;
                else if (polarCoordinates.Phi < 2.45) G = 255 * (Gmin - polarCoordinates.Phi) / (Gmin - 2.45);
                else G = 255 * (Gmax - polarCoordinates.Phi) / (Gmax - 4.15);
            }
            else G = 0;

            if (polarCoordinates.R < radialLimit)
            {
                R += (255 - R) * (polarCoordinates.R - radialLimit) / (-radialLimit);
                B += (255 - B) * (polarCoordinates.R - radialLimit) / (-radialLimit);
                G += (255 - G) * (polarCoordinates.R - radialLimit) / (-radialLimit);
            }

            color.R = Convert.ToByte(R);
            color.B = Convert.ToByte(B);
            color.G = Convert.ToByte(G);
            color.A = 255;

            brush = new SolidColorBrush(color);
            ColorPicker.Fill = brush;

            if (writer != null)
            {
                if (color.R == 13) color.R = 14;
                if (color.G == 13) color.G = 14;
                if (color.B == 13) color.B = 14;

                writer.WriteByte(0x43);
                writer.WriteByte(color.R);
                writer.WriteByte(color.G);
                writer.WriteByte(color.B);
                writer.WriteByte(0x0D);
                await writer.StoreAsync();
            }

            e.Handled = true;
        }

        private RPhi XY2RPhi(double X, double Y)
        {
            RPhi tempPolarCoordinates = new RPhi(0, 0);
            double tempPhi;

            tempPolarCoordinates.R = Math.Sqrt(X * X + Y * Y);
            tempPhi = Math.Atan2(Y, X);
            if (tempPhi >= 0) tempPolarCoordinates.Phi = tempPhi;
            else tempPolarCoordinates.Phi = tempPhi + 2 * Math.PI;
            return tempPolarCoordinates;
        }

    }
}
