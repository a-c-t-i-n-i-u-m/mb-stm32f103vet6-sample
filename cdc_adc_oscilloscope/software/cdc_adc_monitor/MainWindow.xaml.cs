using System;
using System.Collections.Generic;
using System.IO.Ports;
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

using System.Windows.Forms.DataVisualization.Charting;

namespace cdc_adc_monitor
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        const int resolution = 8;
        const double vmax = 3.3;
        const int baudrate = 115200;

        byte[] command_start = new byte[] { (byte)'b' };
        byte[] command_stop = new byte[] { (byte)'e' };

        SerialPort port = new SerialPort();
        int appstate = 0;

        Series PrimarySeries;
        UInt64 PrimarySeriesIndex = 0;
        DateTime StartTime;

        public MainWindow()
        {
            InitializeComponent();

            // initialize chart view
            var chart = PrimaryChart.ChartAreas.Add("PrimaryChartArea");
            chart.AxisX.Title = "Times(ms)";
            chart.AxisX.ScaleView.Zoomable = true;
            chart.AxisX.Minimum = 0;

            chart.AxisY.Title = "Voltage(V)";
            chart.AxisY.ScaleView.Zoomable = true;
            //chart.AxisY.Minimum = 0;
            //chart.AxisY.Maximum = vmax;

            chart.CursorX.IsUserEnabled = true;
            chart.CursorX.IsUserSelectionEnabled = true;
            chart.CursorY.IsUserEnabled = true;
            chart.CursorY.IsUserSelectionEnabled = true;

            port.BaudRate = baudrate;
            port.DataReceived += port_DataReceived;
        }

        public void ConsoleWriteLine(params object[] content)
        {
            StringBuilder buf = new StringBuilder();
            foreach (var itemt in content)
            {
                buf.Append(itemt.ToString() + " ");
            }
            ConsoleTextBox.Text += "[" + DateTime.Now.ToString("O") + "] " + buf.ToString() + "\n";
            ConsoleTextBox.ScrollToEnd();
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            if (appstate == 1)
            {
                return;
            }

            if (!port.IsOpen)
            {
                try
                {
                    port.Open();
                }
                catch (Exception ex)
                {
                    ConsoleWriteLine("Error", ex.Message);
                    return;
                }
            }
            port.DiscardOutBuffer();
            port.DiscardInBuffer();

            // disbale select box
            portSelectBox.IsEnabled = false;

            // chart deactive
            PrimarySeriesIndex = 0;
            PrimaryChart.Series.Clear();

            PrimarySeries = new Series();
            PrimarySeries.ChartType = SeriesChartType.Line;

            // start command
            appstate = 1;
            port.Write(command_start, 0, 1);

            StartTime = DateTime.Now;
            ConsoleWriteLine("Start sampling, port = " + port.PortName);
        }

        private void port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (!port.IsOpen || appstate == 0)
            {
                return;
            }

            Dispatcher.BeginInvoke((Action)delegate
            {
                int len = port.BytesToRead;
                byte[] data = new byte[len];
                port.Read(data, 0, len);

                for (var i = 0; i < len; i++)
                {
                    PrimarySeries.Points.Add(new DataPoint(PrimarySeriesIndex++, data[i]));
                }
            });
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            if (port.IsOpen && appstate == 1)
            {
                appstate = 0;

                // stop command
                port.Write(command_stop, 0, 1);
                while (port.BytesToWrite > 0) ;

                // clear buffer
                port.DiscardOutBuffer();
                port.DiscardInBuffer();

                // enable select box
                portSelectBox.IsEnabled = true;

                // scale
                double duration = ((DateTime.Now - StartTime).TotalMilliseconds);
                double dp = PrimarySeriesIndex == 0 ? 0 : duration / PrimarySeriesIndex;
                double vp = 3.3 / Math.Pow(2, resolution);
                for (var i = 0; i < PrimarySeries.Points.Count; i++)
                {
                    var el = PrimarySeries.Points[i];
                    el.XValue *= dp;
                    el.YValues[0] *= vp;
                    PrimarySeries.Points[i] = el;
                }

                // show chart
                PrimaryChart.Series.Add(PrimarySeries);

                ConsoleWriteLine("Finish sampling,",
                    duration, "msec",
                    PrimarySeriesIndex, "samples,",
                    "fsample =", Math.Round(PrimarySeriesIndex / duration, 3), "kHz");
            }
        }

        private void portSelectBox_DropDownOpened(object sender, EventArgs e)
        {
            // show COM port list
            portSelectBox.ItemsSource = SerialPort.GetPortNames();
        }

        private void portSelectBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            try
            {
                port.Close();
            }
            catch
            {
            }
            port.PortName = portSelectBox.SelectedValue.ToString();
        }

        private void PrimaryChart_MouseClick(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Right)
            {
                PrimaryChart.ChartAreas[0].AxisX.ScaleView.ZoomReset();
                PrimaryChart.ChartAreas[0].AxisY.ScaleView.ZoomReset();
            }
        }
    }
}
