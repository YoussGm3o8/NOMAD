/*
 * NOMAD Mission Planner Plugin
 * MAD 2026
 * 
 * This plugin provides NOMAD control directly within Mission Planner:
 * - NOMAD menu in menu bar with all controls
 * - Right-click map menu access
 * - Floating control panel
 * 
 * To install: Copy this file to Mission Planner's plugins folder
 * e.g., C:\Program Files (x86)\Mission Planner\plugins\nomad.cs
 */
using System;
using System.IO;
using System.Net;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace nomad
{
    public class Plugin : MissionPlanner.Plugin.Plugin
    {
        string ip = "127.0.0.1";
        int port = 8000;
        Form controlPanel;

        public override string Name { get { return "NOMAD"; } }
        public override string Version { get { return "1.0"; } }
        public override string Author { get { return "MAD"; } }

        public override bool Init()
        {
            // Load saved settings
            ip = Host.config.GetString("nomad_ip", "127.0.0.1");
            port = Host.config.GetInt32("nomad_port", 8000);
            return true;
        }

        public override bool Loaded()
        {
            // === MAIN MENU BAR ===
            ToolStripMenuItem mainMenu = new ToolStripMenuItem("NOMAD");

            ToolStripMenuItem panelItem = new ToolStripMenuItem("Control Panel");
            panelItem.Click += delegate { ShowControlPanel(); };
            mainMenu.DropDownItems.Add(panelItem);

            ToolStripMenuItem settingsItem = new ToolStripMenuItem("Settings");
            settingsItem.Click += delegate { ShowSettings(); };
            mainMenu.DropDownItems.Add(settingsItem);

            mainMenu.DropDownItems.Add(new ToolStripSeparator());

            ToolStripMenuItem testItem = new ToolStripMenuItem("Test Connection");
            testItem.Click += delegate { TestConnection(); };
            mainMenu.DropDownItems.Add(testItem);

            mainMenu.DropDownItems.Add(new ToolStripSeparator());

            ToolStripMenuItem captureItem = new ToolStripMenuItem("Task 1: Capture");
            captureItem.Click += delegate { CaptureSnapshot(); };
            mainMenu.DropDownItems.Add(captureItem);

            ToolStripMenuItem landmarksItem = new ToolStripMenuItem("Task 1: View Landmarks");
            landmarksItem.Click += delegate { ViewLandmarks(); };
            mainMenu.DropDownItems.Add(landmarksItem);

            mainMenu.DropDownItems.Add(new ToolStripSeparator());

            ToolStripMenuItem resetItem = new ToolStripMenuItem("Task 2: Reset Map");
            resetItem.Click += delegate { ResetMap(); };
            mainMenu.DropDownItems.Add(resetItem);

            ToolStripMenuItem statusItem = new ToolStripMenuItem("Task 2: Status");
            statusItem.Click += delegate { ViewStatus(); };
            mainMenu.DropDownItems.Add(statusItem);

            // Insert into menu bar
            try
            {
                int idx = MainV2.instance.MainMenu.Items.IndexOfKey("MenuHelp");
                if (idx >= 0)
                    MainV2.instance.MainMenu.Items.Insert(idx, mainMenu);
                else
                    MainV2.instance.MainMenu.Items.Add(mainMenu);
            }
            catch
            {
                MainV2.instance.MainMenu.Items.Add(mainMenu);
            }

            // === RIGHT-CLICK MAP MENU ===
            ToolStripMenuItem mapMenu = new ToolStripMenuItem("NOMAD");
            Host.FDMenuMap.Items.Add(mapMenu);

            ToolStripMenuItem mapPanel = new ToolStripMenuItem("Control Panel");
            mapPanel.Click += delegate { ShowControlPanel(); };
            mapMenu.DropDownItems.Add(mapPanel);

            ToolStripMenuItem mapSettings = new ToolStripMenuItem("Settings");
            mapSettings.Click += delegate { ShowSettings(); };
            mapMenu.DropDownItems.Add(mapSettings);

            ToolStripMenuItem mapCapture = new ToolStripMenuItem("Task 1: Capture");
            mapCapture.Click += delegate { CaptureSnapshot(); };
            mapMenu.DropDownItems.Add(mapCapture);

            ToolStripMenuItem mapReset = new ToolStripMenuItem("Task 2: Reset Map");
            mapReset.Click += delegate { ResetMap(); };
            mapMenu.DropDownItems.Add(mapReset);

            CustomMessageBox.Show("NOMAD Plugin loaded!\n\nUse the NOMAD menu in the menu bar\nor right-click the map.", "NOMAD");
            return true;
        }

        public override bool Loop() { return true; }
        public override bool Exit() { return true; }

        // === CONTROL PANEL ===
        void ShowControlPanel()
        {
            if (controlPanel != null && !controlPanel.IsDisposed)
            {
                controlPanel.BringToFront();
                return;
            }

            controlPanel = new Form();
            controlPanel.Text = "NOMAD Control Panel";
            controlPanel.Size = new Size(320, 420);
            controlPanel.StartPosition = FormStartPosition.CenterScreen;
            controlPanel.BackColor = Color.FromArgb(45, 45, 48);
            controlPanel.FormBorderStyle = FormBorderStyle.FixedToolWindow;

            int y = 15;

            // Title
            Label title = new Label();
            title.Text = "NOMAD Control";
            title.ForeColor = Color.White;
            title.Font = new Font("Segoe UI", 14, FontStyle.Bold);
            title.Location = new Point(15, y);
            title.AutoSize = true;
            controlPanel.Controls.Add(title);
            y += 40;

            // Connection section
            GroupBox connGroup = new GroupBox();
            connGroup.Text = "Connection";
            connGroup.ForeColor = Color.White;
            connGroup.Location = new Point(10, y);
            connGroup.Size = new Size(290, 90);
            controlPanel.Controls.Add(connGroup);

            Label lblIP = new Label();
            lblIP.Text = "Jetson IP:";
            lblIP.ForeColor = Color.White;
            lblIP.Location = new Point(10, 25);
            lblIP.AutoSize = true;
            connGroup.Controls.Add(lblIP);

            TextBox txtIP = new TextBox();
            txtIP.Text = ip;
            txtIP.Name = "txtIP";
            txtIP.Location = new Point(80, 22);
            txtIP.Width = 120;
            connGroup.Controls.Add(txtIP);

            Label lblPort = new Label();
            lblPort.Text = "Port:";
            lblPort.ForeColor = Color.White;
            lblPort.Location = new Point(210, 25);
            lblPort.AutoSize = true;
            connGroup.Controls.Add(lblPort);

            TextBox txtPort = new TextBox();
            txtPort.Text = port.ToString();
            txtPort.Name = "txtPort";
            txtPort.Location = new Point(245, 22);
            txtPort.Width = 35;
            connGroup.Controls.Add(txtPort);

            Button btnTest = new Button();
            btnTest.Text = "Test Connection";
            btnTest.Location = new Point(10, 55);
            btnTest.Size = new Size(120, 25);
            btnTest.Click += delegate {
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                SaveSettings();
                TestConnection();
            };
            connGroup.Controls.Add(btnTest);

            Button btnSave = new Button();
            btnSave.Text = "Save";
            btnSave.Location = new Point(140, 55);
            btnSave.Size = new Size(60, 25);
            btnSave.Click += delegate {
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                SaveSettings();
                CustomMessageBox.Show("Settings saved!", "NOMAD");
            };
            connGroup.Controls.Add(btnSave);

            y += 100;

            // Task 1 section
            GroupBox task1Group = new GroupBox();
            task1Group.Text = "Task 1: Recon";
            task1Group.ForeColor = Color.Cyan;
            task1Group.Location = new Point(10, y);
            task1Group.Size = new Size(290, 60);
            controlPanel.Controls.Add(task1Group);

            Button btnCapture = new Button();
            btnCapture.Text = "Capture";
            btnCapture.Location = new Point(10, 22);
            btnCapture.Size = new Size(80, 28);
            btnCapture.Click += delegate { 
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                CaptureSnapshot(); 
            };
            task1Group.Controls.Add(btnCapture);

            Button btnLandmarks = new Button();
            btnLandmarks.Text = "Landmarks";
            btnLandmarks.Location = new Point(100, 22);
            btnLandmarks.Size = new Size(85, 28);
            btnLandmarks.Click += delegate { 
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                ViewLandmarks(); 
            };
            task1Group.Controls.Add(btnLandmarks);

            y += 70;

            // Task 2 section
            GroupBox task2Group = new GroupBox();
            task2Group.Text = "Task 2: Extinguish";
            task2Group.ForeColor = Color.Orange;
            task2Group.Location = new Point(10, y);
            task2Group.Size = new Size(290, 60);
            controlPanel.Controls.Add(task2Group);

            Button btnReset = new Button();
            btnReset.Text = "Reset Map";
            btnReset.Location = new Point(10, 22);
            btnReset.Size = new Size(90, 28);
            btnReset.Click += delegate { 
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                ResetMap(); 
            };
            task2Group.Controls.Add(btnReset);

            Button btnStatus = new Button();
            btnStatus.Text = "Status";
            btnStatus.Location = new Point(110, 22);
            btnStatus.Size = new Size(70, 28);
            btnStatus.Click += delegate { 
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                ViewStatus(); 
            };
            task2Group.Controls.Add(btnStatus);

            y += 70;

            // Video section
            GroupBox videoGroup = new GroupBox();
            videoGroup.Text = "Video Stream";
            videoGroup.ForeColor = Color.LimeGreen;
            videoGroup.Location = new Point(10, y);
            videoGroup.Size = new Size(290, 60);
            controlPanel.Controls.Add(videoGroup);

            Button btnRTSP = new Button();
            btnRTSP.Text = "Open RTSP";
            btnRTSP.Location = new Point(10, 22);
            btnRTSP.Size = new Size(90, 28);
            btnRTSP.Click += delegate { 
                ip = txtIP.Text;
                string rtsp = "rtsp://" + ip + ":8554/zed";
                try { System.Diagnostics.Process.Start("vlc", rtsp); }
                catch { CustomMessageBox.Show("Install VLC to view stream.\nRTSP: " + rtsp, "Video"); }
            };
            videoGroup.Controls.Add(btnRTSP);

            Button btnWeb = new Button();
            btnWeb.Text = "Web View";
            btnWeb.Location = new Point(110, 22);
            btnWeb.Size = new Size(80, 28);
            btnWeb.Click += delegate { 
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                string url = "http://" + ip + ":" + port + "/video";
                try { System.Diagnostics.Process.Start(url); }
                catch { CustomMessageBox.Show("Open browser to: " + url, "Video"); }
            };
            videoGroup.Controls.Add(btnWeb);

            controlPanel.Show();
        }

        // === SETTINGS DIALOG ===
        void ShowSettings()
        {
            Form settingsForm = new Form();
            settingsForm.Text = "NOMAD Settings";
            settingsForm.Size = new Size(300, 180);
            settingsForm.StartPosition = FormStartPosition.CenterParent;
            settingsForm.FormBorderStyle = FormBorderStyle.FixedDialog;
            settingsForm.MaximizeBox = false;
            settingsForm.MinimizeBox = false;

            Label lblIP = new Label();
            lblIP.Text = "Jetson IP:";
            lblIP.Location = new Point(20, 25);
            lblIP.AutoSize = true;
            settingsForm.Controls.Add(lblIP);

            TextBox txtIP = new TextBox();
            txtIP.Text = ip;
            txtIP.Location = new Point(100, 22);
            txtIP.Width = 160;
            settingsForm.Controls.Add(txtIP);

            Label lblPort = new Label();
            lblPort.Text = "Port:";
            lblPort.Location = new Point(20, 60);
            lblPort.AutoSize = true;
            settingsForm.Controls.Add(lblPort);

            TextBox txtPort = new TextBox();
            txtPort.Text = port.ToString();
            txtPort.Location = new Point(100, 57);
            txtPort.Width = 60;
            settingsForm.Controls.Add(txtPort);

            Button btnSave = new Button();
            btnSave.Text = "Save";
            btnSave.Location = new Point(100, 100);
            btnSave.Size = new Size(80, 30);
            btnSave.Click += delegate {
                ip = txtIP.Text;
                int.TryParse(txtPort.Text, out port);
                SaveSettings();
                settingsForm.Close();
                CustomMessageBox.Show("Settings saved!", "NOMAD");
            };
            settingsForm.Controls.Add(btnSave);

            Button btnCancel = new Button();
            btnCancel.Text = "Cancel";
            btnCancel.Location = new Point(190, 100);
            btnCancel.Size = new Size(80, 30);
            btnCancel.Click += delegate { settingsForm.Close(); };
            settingsForm.Controls.Add(btnCancel);

            settingsForm.ShowDialog();
        }

        void SaveSettings()
        {
            Host.config["nomad_ip"] = ip;
            Host.config["nomad_port"] = port.ToString();
        }

        // === API CALLS ===
        string GetUrl(string path)
        {
            return "http://" + ip + ":" + port + path;
        }

        string DoGet(string path)
        {
            HttpWebRequest req = (HttpWebRequest)WebRequest.Create(GetUrl(path));
            req.Timeout = 5000;
            using (HttpWebResponse resp = (HttpWebResponse)req.GetResponse())
            using (StreamReader sr = new StreamReader(resp.GetResponseStream()))
            {
                return sr.ReadToEnd();
            }
        }

        string DoPost(string path)
        {
            HttpWebRequest req = (HttpWebRequest)WebRequest.Create(GetUrl(path));
            req.Method = "POST";
            req.ContentLength = 0;
            req.Timeout = 10000;
            using (HttpWebResponse resp = (HttpWebResponse)req.GetResponse())
            using (StreamReader sr = new StreamReader(resp.GetResponseStream()))
            {
                return sr.ReadToEnd();
            }
        }

        void TestConnection()
        {
            try
            {
                string result = DoGet("/health");
                CustomMessageBox.Show("Connected to Jetson!\n\n" + result, "NOMAD - Success");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("Connection failed!\n\n" + ex.Message + "\n\nCheck IP: " + ip + ":" + port, "NOMAD - Error");
            }
        }

        void CaptureSnapshot()
        {
            try
            {
                string result = DoPost("/api/task/1/capture");
                CustomMessageBox.Show(result, "Task 1 - Capture");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("Capture failed: " + ex.Message, "Error");
            }
        }

        void ViewLandmarks()
        {
            try
            {
                string result = DoGet("/api/task/1/landmarks");
                CustomMessageBox.Show(result, "Task 1 - Landmarks");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("Failed: " + ex.Message, "Error");
            }
        }

        void ResetMap()
        {
            try
            {
                string result = DoPost("/api/task/2/reset_map");
                CustomMessageBox.Show(result, "Task 2 - Reset");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("Reset failed: " + ex.Message, "Error");
            }
        }

        void ViewStatus()
        {
            try
            {
                string result = DoGet("/status");
                CustomMessageBox.Show(result, "System Status");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("Failed: " + ex.Message, "Error");
            }
        }
    }
}
