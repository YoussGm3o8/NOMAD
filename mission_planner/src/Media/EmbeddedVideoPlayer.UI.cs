// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player - UI partial
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private void InitializeUI()
        {
            BackColor = Color.Black;
            Dock = DockStyle.Fill;

            _videoBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            _videoBox.DoubleClick += (s, e) => ToggleFullscreen();
            _videoBox.Paint += OnVideoPaint;

            _lblStatus = new Label
            {
                Text = TryInitializeGStreamer() ? "Ready - Click Play" : "GStreamer not found",
                Dock = DockStyle.Bottom,
                Height = 22,
                ForeColor = Color.Gray,
                BackColor = Color.FromArgb(30, 30, 30),
                TextAlign = ContentAlignment.MiddleCenter,
                Font = new Font("Segoe UI", 8),
            };

            var ctrlPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 60,
                BackColor = Color.FromArgb(35, 35, 38),
                Padding = new Padding(5),
            };

            var btnPlay = CreateButton("Play", 10, 5, 55, Color.FromArgb(60, 120, 60));
            var btnStop = CreateButton("Stop", 70, 5, 55, Color.FromArgb(120, 60, 60));
            var btnFull = CreateButton("Full", 130, 5, 50, Color.FromArgb(70, 70, 75));
            var btnVlc = CreateButton("VLC", 185, 5, 45, Color.FromArgb(70, 70, 75));
            var btnSnap = CreateButton("Snap", 235, 5, 50, Color.FromArgb(80, 80, 120));

            btnPlay.Click += (s, e) => StartStream();
            btnStop.Click += (s, e) => StopStream();
            btnFull.Click += (s, e) => ToggleFullscreen();
            btnVlc.Click += (s, e) => OpenExternal();
            btnSnap.Click += (s, e) => TakeSnapshot();

            var lblLatency = new Label
            {
                Text = "Latency:",
                Location = new Point(295, 8),
                ForeColor = Color.Gray,
                AutoSize = true,
                Font = new Font("Segoe UI", 8),
            };
            _trkLatency = new TrackBar
            {
                Location = new Point(350, 0),
                Size = new Size(150, 25),
                Minimum = 20,
                Maximum = 500,
                Value = _latencyMs,
                TickFrequency = 50,
            };
            _trkLatency.ValueChanged += (s, e) =>
            {
                _lblLatencyValue.Text = $"{_trkLatency.Value}ms";
            };
            _lblLatencyValue = new Label
            {
                Text = $"{_latencyMs}ms",
                Location = new Point(505, 8),
                ForeColor = Color.LightGray,
                AutoSize = true,
                Font = new Font("Segoe UI", 8),
            };

            var btnApplyLatency = CreateButton("Apply", 550, 5, 55, Color.FromArgb(0, 100, 140));
            btnApplyLatency.Font = new Font("Segoe UI", 7.5f);
            btnApplyLatency.Click += (s, e) =>
            {
                int newLatency = _trkLatency.Value;
                if (newLatency == _latencyMs && _isPlaying)
                {
                    return;
                }

                _latencyMs = newLatency;
                if (!_isPlaying)
                {
                    _lblStatus.Text = $"Latency set to {_latencyMs}ms (applied on Play)";
                    _lblStatus.ForeColor = Color.DarkCyan;
                    return;
                }

                btnApplyLatency.Enabled = false;
                btnApplyLatency.Text = "...";
                UiAsync.Run(this, async () =>
                {
                    try
                    {
                        await RestartStreamAsync();
                        _lblStatus.Text = $"Latency: {_latencyMs}ms";
                        _lblStatus.ForeColor = Color.Cyan;
                    }
                    finally
                    {
                        btnApplyLatency.Enabled = true;
                        btnApplyLatency.Text = "Apply";
                    }
                }, "ApplyVideoLatency");
            };

            ctrlPanel.Controls.AddRange(new Control[]
            {
                btnPlay, btnStop, btnFull, btnVlc, btnSnap,
                lblLatency, _trkLatency, _lblLatencyValue, btnApplyLatency,
            });

            Controls.Add(_videoBox);
            if (_showControls)
            {
                Controls.Add(_lblStatus);
                Controls.Add(ctrlPanel);
            }
        }

        private Button CreateButton(string text, int x, int y, int width, Color color)
        {
            return new Button
            {
                Text = text,
                Location = new Point(x, y),
                Size = new Size(width, 24),
                FlatStyle = FlatStyle.Flat,
                BackColor = color,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
        }

        private void OnVideoPaint(object sender, PaintEventArgs e)
        {
            if (_videoBox.Image == null)
            {
                return;
            }

            int centerX = _videoBox.Width / 2;
            int centerY = _videoBox.Height / 2;
            const int arm = 18;
            const int gap = 5;
            e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            using (var outline = new Pen(Color.FromArgb(140, Color.Black), 4f))
            using (var foreground = new Pen(Color.FromArgb(220, Color.Cyan), 1.5f))
            {
                foreach (var pen in new[] { outline, foreground })
                {
                    e.Graphics.DrawLine(pen, centerX - arm, centerY, centerX - gap, centerY);
                    e.Graphics.DrawLine(pen, centerX + gap, centerY, centerX + arm, centerY);
                    e.Graphics.DrawLine(pen, centerX, centerY - arm, centerX, centerY - gap);
                    e.Graphics.DrawLine(pen, centerX, centerY + gap, centerX, centerY + arm);
                }
            }
        }

        public void ToggleFullscreen()
        {
            if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
            {
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    _fullscreenBox.Image = null;
                }
                _fullscreenForm.Close();
                _fullscreenForm = null;
                _fullscreenBox = null;
                return;
            }

            _fullscreenForm = new Form
            {
                FormBorderStyle = FormBorderStyle.None,
                WindowState = FormWindowState.Maximized,
                BackColor = Color.Black,
                KeyPreview = true,
            };
            _fullscreenForm.KeyDown += (s, e) =>
            {
                if (e.KeyCode == Keys.Escape)
                {
                    ToggleFullscreen();
                }
            };

            _fullscreenBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            _fullscreenBox.DoubleClick += (s, e) => ToggleFullscreen();

            lock (_frameBufferLock)
            {
                if (_frameBuffers != null &&
                    _displayBufferIndex >= 0 &&
                    _displayBufferIndex < _frameBuffers.Length)
                {
                    _fullscreenBox.Image = _frameBuffers[_displayBufferIndex];
                }
            }

            _fullscreenForm.Controls.Add(_fullscreenBox);
            _fullscreenForm.Show();
        }

        public void OpenExternal()
        {
            string vlcArgs = _streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase)
                ? BuildUdpVlcArguments()
                : $"--network-caching={_latencyMs} --rtsp-tcp \"{_streamUrl}\"";

            var vlcPaths = new[]
            {
                "vlc",
                @"C:\Program Files\VideoLAN\VLC\vlc.exe",
                @"C:\Program Files (x86)\VideoLAN\VLC\vlc.exe",
            };
            foreach (var path in vlcPaths)
            {
                try
                {
                    System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = path,
                        Arguments = vlcArgs,
                        UseShellExecute = true,
                    });
                    _lblStatus.Text = "Opened in VLC";
                    return;
                }
                catch
                {
                }
            }

            MessageBox.Show($"VLC not found.\n\nStream URL: {_streamUrl}", "VLC Not Found", MessageBoxButtons.OK);
        }

        private string BuildUdpVlcArguments()
        {
            var port = ExtractUdpPort(_streamUrl);
            var sdp = $"v=0\no=- 0 0 IN IP4 127.0.0.1\ns=Stream\nc=IN IP4 127.0.0.1\nt=0 0\n" +
                $"m=video {port} RTP/AVP 96\na=rtpmap:96 H264/90000";
            var sdpPath = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "nomad_stream.sdp");
            System.IO.File.WriteAllText(sdpPath, sdp);
            return $"--network-caching={_latencyMs} \"{sdpPath}\"";
        }

        public void TakeSnapshot()
        {
            Bitmap source = null;
            lock (_frameBufferLock)
            {
                if (_frameBuffers != null &&
                    _displayBufferIndex >= 0 &&
                    _displayBufferIndex < _frameBuffers.Length)
                {
                    source = _frameBuffers[_displayBufferIndex];
                }
            }
            if (source == null)
            {
                _lblStatus.Text = "No frame available";
                return;
            }

            var path = System.IO.Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.Desktop),
                $"NOMAD_{DateTime.Now:yyyyMMdd_HHmmss}.png");
            using (var snapshot = (Bitmap)source.Clone())
            {
                snapshot.Save(path);
            }
            _lblStatus.Text = $"Saved: {System.IO.Path.GetFileName(path)}";
            _lblStatus.ForeColor = Color.LimeGreen;
        }

        public void UpdateStreamUrl(string newUrl)
        {
            var wasPlaying = _isPlaying;
            StopStream();
            _streamUrl = newUrl ?? "";
            if (wasPlaying)
            {
                StartStream();
            }
        }
    }
}
