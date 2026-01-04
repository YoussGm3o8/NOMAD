// ============================================================
// NOMAD Control Panel - Flight Data Tab UI
// ============================================================
// Custom control panel added to Mission Planner's Flight Data screen.
// Provides buttons for Task 1 and Task 2 operations.
// Includes Indoor Nudge controls for manual velocity commands.
// Includes RTSP video streaming from Jetson.
// ============================================================

using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Comms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Control panel for NOMAD operations in Mission Planner.
    /// </summary>
    public class NOMADControlPanel : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================

        private readonly DualLinkSender _sender;
        private NOMADConfig _config;

        // UI Controls
        private Label _lblTitle;
        private Label _lblStatus;
        private Label _lblMode;
        private Label _lblPosition;

        // Task 1 Controls
        private GroupBox _grpTask1;
        private Button _btnTask1Capture;
        private TextBox _txtTask1Result;

        // Task 2 Controls
        private GroupBox _grpTask2;
        private Button _btnTask2Reset;
        private Button _btnTask2Hit;
        private NumericUpDown _numX, _numY, _numZ;
        private Label _lblExclusionCount;

        // Indoor Nudge Controls
        private GroupBox _grpNudge;
        private CheckBox _chkEnableNudge;
        private Label _lblNudgeStatus;
        private NumericUpDown _numNudgeSpeed;

        // Video Stream Controls
        private GroupBox _grpVideo;
        private Button _btnOpenPrimaryVideo;
        private Button _btnOpenSecondaryVideo;
        private Label _lblVideoStatus;
        private ComboBox _cmbVideoPlayer;

        // Nudge State
        private bool _nudgeEnabled = false;
        private const float DEFAULT_NUDGE_SPEED = 0.5f;  // m/s (safe indoor speed)

        // ============================================================
        // Constructor
        // ============================================================

        public NOMADControlPanel(DualLinkSender sender, NOMADConfig config)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            _config = config ?? throw new ArgumentNullException(nameof(config));

            InitializeComponents();
            UpdateModeDisplay();

            // Enable keyboard input for nudge controls
            this.SetStyle(ControlStyles.Selectable, true);
            this.TabStop = true;
            this.KeyDown += NOMADControlPanel_KeyDown;
            this.KeyUp += NOMADControlPanel_KeyUp;
        }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Update status display with current vehicle state.
        /// </summary>
        public void UpdateStatus(bool connected, double lat, double lng, double alt)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => UpdateStatus(connected, lat, lng, alt)));
                return;
            }

            _lblStatus.Text = connected ? "[*] Connected" : "[ ] Disconnected";
            _lblStatus.ForeColor = connected ? Color.LimeGreen : Color.Red;
            _lblPosition.Text = connected
                ? $"Position: {lat:F6}, {lng:F6} @ {alt:F1}m"
                : "Position: --";
        }

        /// <summary>
        /// Update configuration reference.
        /// </summary>
        public void UpdateConfig(NOMADConfig config)
        {
            _config = config;
            UpdateModeDisplay();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeComponents()
        {
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.Padding = new Padding(10);

            int yOffset = 10;

            // Title
            _lblTitle = new Label
            {
                Text = "NOMAD Control",
                Font = new Font("Segoe UI", 16, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 122, 204),
                Location = new Point(10, yOffset),
                AutoSize = true
            };
            this.Controls.Add(_lblTitle);
            yOffset += 40;

            // Status Row
            var pnlStatus = new FlowLayoutPanel
            {
                Location = new Point(10, yOffset),
                Size = new Size(350, 25),
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent
            };

            _lblStatus = new Label
            {
                Text = "[ ] Disconnected",
                ForeColor = Color.Red,
                AutoSize = true,
                Font = new Font("Segoe UI", 10)
            };
            pnlStatus.Controls.Add(_lblStatus);

            _lblMode = new Label
            {
                Text = "| Mode: HTTP",
                ForeColor = Color.Gray,
                AutoSize = true,
                Font = new Font("Segoe UI", 10),
                Margin = new Padding(20, 0, 0, 0)
            };
            pnlStatus.Controls.Add(_lblMode);

            this.Controls.Add(pnlStatus);
            yOffset += 30;

            // Position Label
            _lblPosition = new Label
            {
                Text = "Position: --",
                ForeColor = Color.LightGray,
                Font = new Font("Consolas", 9),
                Location = new Point(10, yOffset),
                AutoSize = true
            };
            this.Controls.Add(_lblPosition);
            yOffset += 30;

            // ============================================================
            // Task 1: Recon
            // ============================================================

            _grpTask1 = new GroupBox
            {
                Text = "Task 1: Recon (Outdoor)",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 150),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _btnTask1Capture = new Button
            {
                Text = "[CAP] Capture Snapshot",
                Location = new Point(15, 25),
                Size = new Size(320, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnTask1Capture.FlatAppearance.BorderSize = 0;
            _btnTask1Capture.Click += BtnTask1Capture_Click;
            _grpTask1.Controls.Add(_btnTask1Capture);

            var lblResult = new Label
            {
                Text = "Result:",
                Location = new Point(15, 75),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpTask1.Controls.Add(lblResult);

            _txtTask1Result = new TextBox
            {
                Location = new Point(15, 95),
                Size = new Size(320, 40),
                Multiline = true,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.LimeGreen,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle
            };
            _grpTask1.Controls.Add(_txtTask1Result);

            this.Controls.Add(_grpTask1);
            yOffset += 160;

            // ============================================================
            // Task 2: Extinguish
            // ============================================================

            _grpTask2 = new GroupBox
            {
                Text = "Task 2: Extinguish (Indoor)",
                ForeColor = Color.FromArgb(255, 150, 50),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 180),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _btnTask2Reset = new Button
            {
                Text = "[CLR] Reset Exclusion Map",
                Location = new Point(15, 25),
                Size = new Size(320, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(200, 50, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnTask2Reset.FlatAppearance.BorderSize = 0;
            _btnTask2Reset.Click += BtnTask2Reset_Click;
            _grpTask2.Controls.Add(_btnTask2Reset);

            _lblExclusionCount = new Label
            {
                Text = "Targets: 0",
                Location = new Point(15, 65),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpTask2.Controls.Add(_lblExclusionCount);

            // Manual Hit Registration
            var lblManualHit = new Label
            {
                Text = "Manual Hit (X, Y, Z):",
                Location = new Point(15, 90),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpTask2.Controls.Add(lblManualHit);

            _numX = CreateNumericUpDown(15, 110);
            _numY = CreateNumericUpDown(90, 110);
            _numZ = CreateNumericUpDown(165, 110);
            _grpTask2.Controls.Add(_numX);
            _grpTask2.Controls.Add(_numY);
            _grpTask2.Controls.Add(_numZ);

            _btnTask2Hit = new Button
            {
                Text = "Hit",
                Location = new Point(245, 110),
                Size = new Size(90, 25),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(255, 150, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Cursor = Cursors.Hand
            };
            _btnTask2Hit.FlatAppearance.BorderSize = 0;
            _btnTask2Hit.Click += BtnTask2Hit_Click;
            _grpTask2.Controls.Add(_btnTask2Hit);

            this.Controls.Add(_grpTask2);
            yOffset += 190;

            // ============================================================
            // Indoor Nudge Controls
            // ============================================================

            _grpNudge = new GroupBox
            {
                Text = "Indoor Nudge (Task 2)",
                ForeColor = Color.FromArgb(100, 200, 100),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 140),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _chkEnableNudge = new CheckBox
            {
                Text = "Enable Indoor Nudge (WASD)",
                Location = new Point(15, 25),
                Size = new Size(320, 25),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10),
                Checked = false
            };
            _chkEnableNudge.CheckedChanged += ChkEnableNudge_CheckedChanged;
            _grpNudge.Controls.Add(_chkEnableNudge);

            var lblSpeed = new Label
            {
                Text = "Nudge Speed (m/s):",
                Location = new Point(15, 55),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpNudge.Controls.Add(lblSpeed);

            _numNudgeSpeed = new NumericUpDown
            {
                Location = new Point(140, 52),
                Size = new Size(70, 25),
                DecimalPlaces = 2,
                Minimum = 0.1M,
                Maximum = 2.0M,
                Increment = 0.1M,
                Value = (decimal)DEFAULT_NUDGE_SPEED,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            _grpNudge.Controls.Add(_numNudgeSpeed);

            var lblNudgeHelp = new Label
            {
                Text = "W=Forward  S=Back  A=Left  D=Right",
                Location = new Point(15, 85),
                ForeColor = Color.Gray,
                Font = new Font("Consolas", 9),
                AutoSize = true
            };
            _grpNudge.Controls.Add(lblNudgeHelp);

            _lblNudgeStatus = new Label
            {
                Text = "Status: Disabled",
                Location = new Point(15, 110),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 9),
                AutoSize = true
            };
            _grpNudge.Controls.Add(_lblNudgeStatus);

            this.Controls.Add(_grpNudge);
            yOffset += 150;

            // ============================================================
            // Video Streaming Controls
            // ============================================================

            _grpVideo = new GroupBox
            {
                Text = "Video Streams (RTSP)",
                ForeColor = Color.FromArgb(200, 100, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 130),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _btnOpenPrimaryVideo = new Button
            {
                Text = "[VID] Open Primary (ZED/Nav)",
                Location = new Point(15, 25),
                Size = new Size(155, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 50, 150),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnOpenPrimaryVideo.FlatAppearance.BorderSize = 0;
            _btnOpenPrimaryVideo.Click += BtnOpenPrimaryVideo_Click;
            _grpVideo.Controls.Add(_btnOpenPrimaryVideo);

            _btnOpenSecondaryVideo = new Button
            {
                Text = "[TGT] Open Secondary (Gimbal)",
                Location = new Point(180, 25),
                Size = new Size(155, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(150, 100, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnOpenSecondaryVideo.FlatAppearance.BorderSize = 0;
            _btnOpenSecondaryVideo.Click += BtnOpenSecondaryVideo_Click;
            _grpVideo.Controls.Add(_btnOpenSecondaryVideo);

            var lblPlayer = new Label
            {
                Text = "Video Player:",
                Location = new Point(15, 70),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpVideo.Controls.Add(lblPlayer);

            _cmbVideoPlayer = new ComboBox
            {
                Location = new Point(100, 67),
                Size = new Size(150, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            _cmbVideoPlayer.Items.AddRange(new object[] { "VLC", "FFplay", "System Default" });
            _cmbVideoPlayer.SelectedIndex = 0;
            _grpVideo.Controls.Add(_cmbVideoPlayer);

            _lblVideoStatus = new Label
            {
                Text = "Status: Ready",
                Location = new Point(15, 100),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 9),
                AutoSize = true
            };
            _grpVideo.Controls.Add(_lblVideoStatus);

            this.Controls.Add(_grpVideo);
        }

        private NumericUpDown CreateNumericUpDown(int x, int y)
        {
            return new NumericUpDown
            {
                Location = new Point(x, y),
                Size = new Size(70, 25),
                DecimalPlaces = 2,
                Minimum = -100,
                Maximum = 100,
                Value = 0,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
        }

        private void UpdateModeDisplay()
        {
            if (_lblMode == null) return;

            var mode = _config.UseELRS ? "ELRS/MAVLink" : "HTTP";
            _lblMode.Text = $"| Mode: {mode}";
            _lblMode.ForeColor = _config.UseELRS ? Color.Orange : Color.LightBlue;
        }

        // ============================================================
        // Event Handlers
        // ============================================================

        private async void BtnTask1Capture_Click(object sender, EventArgs e)
        {
            _btnTask1Capture.Enabled = false;
            _btnTask1Capture.Text = "Capturing...";
            _txtTask1Result.Text = "";

            try
            {
                var result = await _sender.SendTask1Capture();

                if (result.Success)
                {
                    // Try to extract target_text from response
                    try
                    {
                        dynamic json = Newtonsoft.Json.JsonConvert.DeserializeObject(result.Data);
                        _txtTask1Result.Text = json?.target_text ?? "Capture successful";
                        _txtTask1Result.ForeColor = Color.LimeGreen;
                    }
                    catch
                    {
                        _txtTask1Result.Text = result.Message;
                        _txtTask1Result.ForeColor = Color.LimeGreen;
                    }
                }
                else
                {
                    _txtTask1Result.Text = result.Message;
                    _txtTask1Result.ForeColor = Color.Red;
                }
            }
            catch (Exception ex)
            {
                _txtTask1Result.Text = $"Error: {ex.Message}";
                _txtTask1Result.ForeColor = Color.Red;
            }
            finally
            {
                _btnTask1Capture.Enabled = true;
                _btnTask1Capture.Text = "[CAP] Capture Snapshot";
            }
        }

        private async void BtnTask2Reset_Click(object sender, EventArgs e)
        {
            var confirm = MessageBox.Show(
                "Reset the exclusion map?\nAll stored targets will be cleared.",
                "Confirm Reset",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning
            );

            if (confirm != DialogResult.Yes) return;

            _btnTask2Reset.Enabled = false;

            try
            {
                var result = await _sender.SendTask2ResetMap();

                if (result.Success)
                {
                    _lblExclusionCount.Text = "Targets: 0";
                    _lblExclusionCount.ForeColor = Color.LimeGreen;
                    MessageBox.Show("Exclusion map cleared!", "Success", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                else
                {
                    MessageBox.Show($"Reset failed: {result.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                _btnTask2Reset.Enabled = true;
            }
        }

        private async void BtnTask2Hit_Click(object sender, EventArgs e)
        {
            _btnTask2Hit.Enabled = false;

            try
            {
                var result = await _sender.SendTask2TargetHit(
                    (float)_numX.Value,
                    (float)_numY.Value,
                    (float)_numZ.Value
                );

                if (result.Success)
                {
                    try
                    {
                        dynamic json = Newtonsoft.Json.JsonConvert.DeserializeObject(result.Data);
                        int total = json?.total_targets ?? 0;
                        _lblExclusionCount.Text = $"Targets: {total}";
                        _lblExclusionCount.ForeColor = Color.Orange;
                    }
                    catch
                    {
                        _lblExclusionCount.ForeColor = Color.LimeGreen;
                    }
                }
                else
                {
                    MessageBox.Show($"Hit registration failed: {result.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                _btnTask2Hit.Enabled = true;
            }
        }

        // ============================================================
        // Indoor Nudge Event Handlers
        // ============================================================

        private void ChkEnableNudge_CheckedChanged(object sender, EventArgs e)
        {
            _nudgeEnabled = _chkEnableNudge.Checked;

            if (_nudgeEnabled)
            {
                _lblNudgeStatus.Text = "Status: ENABLED - Press WASD to nudge";
                _lblNudgeStatus.ForeColor = Color.LimeGreen;
                _chkEnableNudge.ForeColor = Color.LimeGreen;

                // Focus this control to receive keyboard events
                this.Focus();
            }
            else
            {
                _lblNudgeStatus.Text = "Status: Disabled";
                _lblNudgeStatus.ForeColor = Color.Gray;
                _chkEnableNudge.ForeColor = Color.White;

                // Send zero velocity to stop any movement
                SendBodyVelocity(0, 0);
            }
        }

        // ============================================================
        // Video Streaming Event Handlers
        // ============================================================

        private void BtnOpenPrimaryVideo_Click(object sender, EventArgs e)
        {
            OpenVideoStream(_config.RtspUrlPrimary, "Primary (ZED/Navigation)");
        }

        private void BtnOpenSecondaryVideo_Click(object sender, EventArgs e)
        {
            OpenVideoStream(_config.RtspUrlSecondary, "Secondary (Gimbal/Targeting)");
        }

        /// <summary>
        /// Open an RTSP video stream in the selected video player.
        /// VLC or FFplay are recommended for low-latency RTSP playback.
        /// </summary>
        /// <param name="rtspUrl">RTSP stream URL from Jetson</param>
        /// <param name="streamName">Display name for status updates</param>
        private void OpenVideoStream(string rtspUrl, string streamName)
        {
            try
            {
                _lblVideoStatus.Text = $"Opening {streamName}...";
                _lblVideoStatus.ForeColor = Color.Yellow;

                string player = _cmbVideoPlayer.SelectedItem?.ToString() ?? "VLC";
                ProcessStartInfo psi = null;

                switch (player)
                {
                    case "VLC":
                        // VLC with low-latency settings for RTSP
                        psi = new ProcessStartInfo
                        {
                            FileName = "vlc",
                            Arguments = $"--network-caching=100 --rtsp-tcp \"{rtspUrl}\"",
                            UseShellExecute = true
                        };
                        break;

                    case "FFplay":
                        // FFplay with low-latency flags
                        psi = new ProcessStartInfo
                        {
                            FileName = "ffplay",
                            Arguments = $"-fflags nobuffer -flags low_delay -rtsp_transport tcp \"{rtspUrl}\"",
                            UseShellExecute = true
                        };
                        break;

                    case "System Default":
                    default:
                        // Open with system default handler
                        psi = new ProcessStartInfo
                        {
                            FileName = rtspUrl,
                            UseShellExecute = true
                        };
                        break;
                }

                Process.Start(psi);

                _lblVideoStatus.Text = $"Opened: {streamName}";
                _lblVideoStatus.ForeColor = Color.LimeGreen;
            }
            catch (Exception ex)
            {
                _lblVideoStatus.Text = $"Error: {ex.Message}";
                _lblVideoStatus.ForeColor = Color.Red;
                MessageBox.Show(
                    $"Failed to open video stream.\n\n" +
                    $"URL: {rtspUrl}\n" +
                    $"Error: {ex.Message}\n\n" +
                    "Make sure VLC or FFplay is installed and in your PATH.",
                    "Video Stream Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
        }

        private void NOMADControlPanel_KeyDown(object sender, KeyEventArgs e)
        {
            if (!_nudgeEnabled) return;

            float speed = (float)_numNudgeSpeed.Value;
            float vx = 0, vy = 0;

            // Map WASD to body-frame velocities
            // W = Forward (+X body), S = Back (-X body)
            // A = Left (-Y body), D = Right (+Y body)
            switch (e.KeyCode)
            {
                case Keys.W:
                    vx = speed;  // Forward
                    break;
                case Keys.S:
                    vx = -speed;  // Back
                    break;
                case Keys.A:
                    vy = -speed;  // Left
                    break;
                case Keys.D:
                    vy = speed;  // Right
                    break;
                default:
                    return;  // Ignore other keys
            }

            // Update status display
            _lblNudgeStatus.Text = $"Status: Nudging {e.KeyCode} ({vx:F1}, {vy:F1}) m/s";
            _lblNudgeStatus.ForeColor = Color.Orange;

            // Send velocity command
            SendBodyVelocity(vx, vy);

            e.Handled = true;
            e.SuppressKeyPress = true;
        }

        private void NOMADControlPanel_KeyUp(object sender, KeyEventArgs e)
        {
            if (!_nudgeEnabled) return;

            // Stop movement when key released
            if (e.KeyCode == Keys.W || e.KeyCode == Keys.S ||
                e.KeyCode == Keys.A || e.KeyCode == Keys.D)
            {
                SendBodyVelocity(0, 0);
                _lblNudgeStatus.Text = "Status: ENABLED - Press WASD to nudge";
                _lblNudgeStatus.ForeColor = Color.LimeGreen;

                e.Handled = true;
                e.SuppressKeyPress = true;
            }
        }

        /// <summary>
        /// Send body-frame velocity command via ELRS/MAVLink direct link.
        /// Uses SET_POSITION_TARGET_LOCAL_NED with MAV_FRAME_BODY_NED.
        /// Bypasses 4G network for low-latency indoor control.
        /// </summary>
        /// <param name="vx">Forward velocity in m/s (body X, nose direction)</param>
        /// <param name="vy">Right velocity in m/s (body Y, right wing direction)</param>
        private void SendBodyVelocity(float vx, float vy)
        {
            try
            {
                // Check if connected via direct ELRS link
                if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                {
                    _lblNudgeStatus.Text = "Status: ERROR - No ELRS connection";
                    _lblNudgeStatus.ForeColor = Color.Red;
                    return;
                }

                // ============================================================
                // SET_POSITION_TARGET_LOCAL_NED Message
                // ============================================================
                // Coordinate Frame: MAV_FRAME_BODY_NED (8) - relative to drone nose
                // Type Mask: 0x0DC7 - Ignore Position, Accel, Yaw
                //   Bits 0-2 (pos): 1 (ignore)
                //   Bits 3-5 (vel): 0 (use) - We want velocity control
                //   Bits 6-8 (acc): 1 (ignore)
                //   Bit 9 (force): 1 (ignore)
                //   Bit 10 (yaw): 1 (ignore)
                //   Bit 11 (yaw_rate): 1 (ignore)
                //   = 0b0000_1101_1100_0111 = 0x0DC7
                // ============================================================

                const ushort TYPE_MASK_VELOCITY_ONLY = 0x0DC7;
                const byte MAV_FRAME_BODY_NED = 8;

                var packet = new MAVLink.mavlink_set_position_target_local_ned_t
                {
                    time_boot_ms = 0,                    // 0 = use system time
                    target_system = MainV2.comPort.MAV.sysid,
                    target_component = MainV2.comPort.MAV.compid,
                    coordinate_frame = MAV_FRAME_BODY_NED,
                    type_mask = TYPE_MASK_VELOCITY_ONLY,
                    x = 0,                               // Position X (ignored)
                    y = 0,                               // Position Y (ignored)
                    z = 0,                               // Position Z (ignored)
                    vx = vx,                             // Velocity X (body forward)
                    vy = vy,                             // Velocity Y (body right)
                    vz = 0,                              // Velocity Z (no vertical)
                    afx = 0,                             // Accel X (ignored)
                    afy = 0,                             // Accel Y (ignored)
                    afz = 0,                             // Accel Z (ignored)
                    yaw = 0,                             // Yaw angle (ignored)
                    yaw_rate = 0                         // Yaw rate (ignored)
                };

                // Send directly via ELRS/Serial link (bypasses 4G)
                MainV2.comPort.sendPacket(packet, MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD Nudge Error: {ex.Message}");
                _lblNudgeStatus.Text = $"Status: ERROR - {ex.Message}";
                _lblNudgeStatus.ForeColor = Color.Red;
            }
        }
    }
}
