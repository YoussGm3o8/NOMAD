// ============================================================
// NOMAD Embedded Video Player Control
// ============================================================
// Provides in-app RTSP video streaming for Mission Planner
// without requiring external VLC or FFplay applications.
// Uses LibVLCSharp for embedded video playback.
// ============================================================

using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Embedded video player for RTSP streams within Mission Planner.
    /// Uses LibVLCSharp for native video playback.
    /// Falls back to external player if LibVLC is not available.
    /// </summary>
    public class EmbeddedVideoPlayer : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================
        
        private string _streamTitle;
        private string _streamUrl;
        private bool _isPlaying;
        private bool _useEmbedded;
        
        // UI Controls
        private Panel _videoPanel;
        private Panel _controlPanel;
        private Label _lblTitle;
        private Label _lblStatus;
        private Button _btnPlay;
        private Button _btnStop;
        private Button _btnFullscreen;
        private Button _btnExternal;
        private Button _btnSnapshot;
        private ComboBox _cmbQuality;
        private TrackBar _trkLatency;
        private Label _lblLatency;
        
        // LibVLC (if available)
        private dynamic _libVLC;
        private dynamic _mediaPlayer;
        private dynamic _videoView;
        private string _libVlcInitErrorMessage = null;
        
        // Playback settings
        private int _networkCaching = 100; // ms
        private string _quality = "auto";
        
        // ============================================================
        // Constructor
        // ============================================================
        
        public EmbeddedVideoPlayer(string title, string rtspUrl)
        {
            _streamTitle = title;
            _streamUrl = rtspUrl;
            _isPlaying = false;
            _useEmbedded = TryInitializeLibVLC();
            
            InitializeUI();
        }
        
        // ============================================================
        // LibVLC Initialization
        // ============================================================
        
        private bool TryInitializeLibVLC()
        {
            try
            {
                // Try to load LibVLCSharp dynamically from the AppDomain first
                System.Reflection.Assembly assembly = null;
                try
                {
                    assembly = System.Reflection.Assembly.Load("LibVLCSharp.WinForms");
                }
                catch { }

                // If not already loaded, try to find a local copy next to this assembly
                if (assembly == null)
                {
                    var baseDir = System.IO.Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location);
                    var candidate = System.IO.Path.Combine(baseDir, "LibVLCSharp.WinForms.dll");
                    if (System.IO.File.Exists(candidate))
                    {
                        try { assembly = System.Reflection.Assembly.LoadFrom(candidate); } catch { assembly = null; }
                    }
                }

                if (assembly != null)
                {
                    var vlcType = assembly.GetType("LibVLCSharp.Shared.LibVLC");
                    var mediaPlayerType = assembly.GetType("LibVLCSharp.Shared.MediaPlayer");
                    var videoViewType = assembly.GetType("LibVLCSharp.WinForms.VideoView");

                    if (vlcType != null && mediaPlayerType != null && videoViewType != null)
                    {
                        try
                        {
                            // Initialize LibVLC - pass args as string[] to match expected ctor
                            var args = new string[] { $"--network-caching={_networkCaching}" };
                            _libVLC = Activator.CreateInstance(vlcType, new object[] { args });
                            _mediaPlayer = Activator.CreateInstance(mediaPlayerType, _libVLC);
                            _videoView = Activator.CreateInstance(videoViewType);

                            // Set media player on video view
                            _videoView.MediaPlayer = _mediaPlayer;

                            System.Diagnostics.Debug.WriteLine("NOMAD: LibVLCSharp initialized successfully");
                            return true;
                        }
                        catch (System.DllNotFoundException dllEx)
                        {
                            // Native libvlc not found (common if VLC isn't installed)
                            _libVlcInitErrorMessage = "Native libVLC not found (libvlc.dll); install VLC or include libvlc redistributables." + Environment.NewLine + dllEx.Message;
                            System.Diagnostics.Debug.WriteLine("NOMAD: LibVLCSharp initialization failed - native lib missing: " + dllEx.Message);
                        }
                        catch (Exception ex)
                        {
                            _libVlcInitErrorMessage = ex.Message;
                            System.Diagnostics.Debug.WriteLine($"NOMAD: LibVLCSharp initialization failed - {ex}");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                _libVlcInitErrorMessage = ex.Message;
                System.Diagnostics.Debug.WriteLine($"NOMAD: LibVLCSharp not available - {ex}");
            }
            
            return false;
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(30, 30, 30);
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(5);
            
            var mainPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                BackColor = Color.Transparent,
            };
            
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));
            
            // Title Bar
            var titlePanel = CreateTitleBar();
            mainPanel.Controls.Add(titlePanel, 0, 0);
            
            // Video Display Area
            _videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                Margin = new Padding(0),
            };
            
            // Add placeholder or LibVLC video view
            if (_useEmbedded && _videoView != null)
            {
                _videoView.Dock = DockStyle.Fill;
                _videoPanel.Controls.Add(_videoView);
            }
            else
            {
                var placeholder = new Label
                {
                    Text = "[VIDEO] Video Feed\n\nClick Play to start stream\n\n" +
                           "(External player will open if LibVLC unavailable)",
                    Dock = DockStyle.Fill,
                    ForeColor = Color.Gray,
                    Font = new Font("Segoe UI", 11),
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                    _videoPanel.Controls.Add(placeholder);

                    // If there was a specific libVLC init error, show a small note below the control
                    if (!string.IsNullOrEmpty(_libVlcInitErrorMessage))
                    {
                        var info = new Label
                        {
                            Text = "Embedded player unavailable: " + _libVlcInitErrorMessage,
                            Dock = DockStyle.Bottom,
                            ForeColor = Color.Orange,
                            Font = new Font("Segoe UI", 8),
                            Height = 32,
                            TextAlign = ContentAlignment.MiddleLeft,
                        };
                        // place the note above control panel
                        mainPanel.Controls.Add(info, 0, 2);
                    }
            }
            
            mainPanel.Controls.Add(_videoPanel, 0, 1);
            
            // Control Bar
            _controlPanel = CreateControlBar();
            mainPanel.Controls.Add(_controlPanel, 0, 2);
            
            this.Controls.Add(mainPanel);
        }
        
        private Panel CreateTitleBar()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            _lblTitle = new Label
            {
                Text = $"[VID] {_streamTitle}",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.White,
                Location = new Point(10, 5),
                AutoSize = true,
            };
            panel.Controls.Add(_lblTitle);
            
            _lblStatus = new Label
            {
                Text = "[*] Stopped",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.Gray,
                Location = new Point(200, 7),
                AutoSize = true,
            };
            panel.Controls.Add(_lblStatus);
            
            return panel;
        }
        
        private Panel CreateControlBar()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(40, 40, 43),
                Padding = new Padding(10, 5, 10, 5),
            };
            
            // Play Button
            _btnPlay = new Button
            {
                Text = "> Play",
                Location = new Point(10, 10),
                Size = new Size(70, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 150, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _btnPlay.Click += (s, e) => StartStream();
            panel.Controls.Add(_btnPlay);
            
            // Stop Button
            _btnStop = new Button
            {
                Text = "[] Stop",
                Location = new Point(85, 10),
                Size = new Size(70, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _btnStop.Click += (s, e) => StopStream();
            panel.Controls.Add(_btnStop);
            
            // Snapshot Button
            _btnSnapshot = new Button
            {
                Text = "[S]",
                Location = new Point(160, 10),
                Size = new Size(35, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 100, 150),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10),
            };
            _btnSnapshot.Click += (s, e) => TakeSnapshot();
            panel.Controls.Add(_btnSnapshot);
            
            // External Button
            _btnExternal = new Button
            {
                Text = "[EXT] External",
                Location = new Point(200, 10),
                Size = new Size(80, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(80, 80, 85),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            _btnExternal.Click += (s, e) => OpenExternal();
            panel.Controls.Add(_btnExternal);
            
            // Fullscreen Button
            _btnFullscreen = new Button
            {
                Text = "[+]",
                Location = new Point(285, 10),
                Size = new Size(35, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(80, 80, 85),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 12),
            };
            _btnFullscreen.Click += (s, e) => ToggleFullscreen();
            panel.Controls.Add(_btnFullscreen);
            
            // Latency Slider
            var lblLatencyTitle = new Label
            {
                Text = "Latency:",
                Location = new Point(10, 48),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                AutoSize = true,
            };
            panel.Controls.Add(lblLatencyTitle);
            
            _trkLatency = new TrackBar
            {
                Location = new Point(60, 43),
                Size = new Size(150, 30),
                Minimum = 50,
                Maximum = 1000,
                Value = _networkCaching,
                TickFrequency = 100,
                SmallChange = 50,
                LargeChange = 100,
            };
            _trkLatency.ValueChanged += (s, e) =>
            {
                _networkCaching = _trkLatency.Value;
                _lblLatency.Text = $"{_networkCaching}ms";
            };
            panel.Controls.Add(_trkLatency);
            
            _lblLatency = new Label
            {
                Text = $"{_networkCaching}ms",
                Location = new Point(215, 48),
                ForeColor = Color.LightGray,
                Font = new Font("Segoe UI", 8),
                AutoSize = true,
            };
            panel.Controls.Add(_lblLatency);
            
            // Quality Selector
            var lblQuality = new Label
            {
                Text = "Quality:",
                Location = new Point(270, 48),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                AutoSize = true,
            };
            panel.Controls.Add(lblQuality);
            
            _cmbQuality = new ComboBox
            {
                Location = new Point(315, 45),
                Size = new Size(80, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            _cmbQuality.Items.AddRange(new object[] { "Auto", "720p", "480p", "360p" });
            _cmbQuality.SelectedIndex = 0;
            _cmbQuality.SelectedIndexChanged += (s, e) => _quality = _cmbQuality.SelectedItem.ToString().ToLower();
            panel.Controls.Add(_cmbQuality);
            
            return panel;
        }
        
        // ============================================================
        // Playback Methods
        // ============================================================
        
        public void StartStream()
        {
            if (_isPlaying) return;
            
            try
            {
                if (_useEmbedded && _libVLC != null && _mediaPlayer != null)
                {
                    // Use LibVLC embedded playback
                    var mediaType = _libVLC.GetType().Assembly.GetType("LibVLCSharp.Shared.Media");
                    var fromLocationType = _libVLC.GetType().Assembly.GetType("LibVLCSharp.Shared.FromType");
                    
                    var options = new string[]
                    {
                        $":network-caching={_networkCaching}",
                        ":rtsp-tcp",
                        ":clock-jitter=0",
                        ":clock-synchro=0"
                    };
                    
                    dynamic media = Activator.CreateInstance(mediaType, _libVLC, _streamUrl, 1); // FromType.FromLocation = 1
                    foreach (var opt in options)
                    {
                        media.AddOption(opt);
                    }
                    
                    _mediaPlayer.Media = media;
                    _mediaPlayer.Play();
                    
                    _isPlaying = true;
                    UpdateStatus("[*] Playing (Embedded)", Color.LimeGreen);
                }
                else
                {
                    // Fall back to external player
                    OpenExternal();
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Play error - {ex.Message}");
                UpdateStatus($"[X] Error: {ex.Message}", Color.Red);
                
                // Try external as fallback
                OpenExternal();
            }
        }
        
        public void StopStream()
        {
            try
            {
                if (_useEmbedded && _mediaPlayer != null)
                {
                    _mediaPlayer.Stop();
                }
                
                _isPlaying = false;
                UpdateStatus("[*] Stopped", Color.Gray);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Stop error - {ex.Message}");
            }
        }
        
        public void OpenExternal()
        {
            // Common VLC installation paths on Windows
            var vlcPaths = new[]
            {
                "vlc",  // PATH
                @"C:\Program Files\VideoLAN\VLC\vlc.exe",
                @"C:\Program Files (x86)\VideoLAN\VLC\vlc.exe",
                Environment.ExpandEnvironmentVariables(@"%LOCALAPPDATA%\Programs\VideoLAN\VLC\vlc.exe")
            };
            
            // Try VLC paths
            foreach (var vlcPath in vlcPaths)
            {
                try
                {
                    var psi = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = vlcPath,
                        Arguments = $"--network-caching={_networkCaching} --rtsp-tcp \"{_streamUrl}\"",
                        UseShellExecute = true,
                    };
                    System.Diagnostics.Process.Start(psi);
                    
                    UpdateStatus("[>] Opened in VLC", Color.Yellow);
                    _isPlaying = true;
                    return;
                }
                catch
                {
                    // Try next path
                }
            }
            
            // Try FFplay (common with FFmpeg)
            var ffplayPaths = new[]
            {
                "ffplay",  // PATH
                @"C:\ffmpeg\bin\ffplay.exe",
                Environment.ExpandEnvironmentVariables(@"%LOCALAPPDATA%\Programs\ffmpeg\bin\ffplay.exe")
            };
            
            foreach (var ffplayPath in ffplayPaths)
            {
                try
                {
                    var psi = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = ffplayPath,
                        Arguments = $"-fflags nobuffer -flags low_delay -rtsp_transport tcp \"{_streamUrl}\"",
                        UseShellExecute = true,
                    };
                    System.Diagnostics.Process.Start(psi);
                    
                    UpdateStatus("[>] Opened in FFplay", Color.Yellow);
                    _isPlaying = true;
                    return;
                }
                catch
                {
                    // Try next path
                }
            }
            
            // No player found - show error
            UpdateStatus("[X] No player found", Color.Red);
            MessageBox.Show(
                $"Could not open video stream.\n\n" +
                $"Please install VLC or FFplay and ensure it's in your PATH.\n\n" +
                $"VLC download: https://www.videolan.org/vlc/\n" +
                $"FFmpeg download: https://ffmpeg.org/download.html\n\n" +
                $"Stream URL: {_streamUrl}",
                "Video Player Not Found",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning
            );
        }
        
        public void TakeSnapshot()
        {
            try
            {
                if (_useEmbedded && _mediaPlayer != null)
                {
                    var desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
                    var filename = $"NOMAD_Snapshot_{DateTime.Now:yyyyMMdd_HHmmss}.png";
                    var path = System.IO.Path.Combine(desktopPath, filename);
                    
                    // Try to take snapshot via LibVLC
                    bool success = _mediaPlayer.TakeSnapshot(0, path, 0, 0);
                    
                    if (success)
                    {
                        UpdateStatus($"[S] Saved: {filename}", Color.LimeGreen);
                    }
                    else
                    {
                        UpdateStatus("[X] Snapshot failed", Color.Red);
                    }
                }
                else
                {
                    UpdateStatus("[!] No embedded player", Color.Yellow);
                }
            }
            catch (Exception ex)
            {
                UpdateStatus($"[X] Error: {ex.Message}", Color.Red);
            }
        }
        
        public void ToggleFullscreen()
        {
            try
            {
                if (_useEmbedded && _mediaPlayer != null)
                {
                    _mediaPlayer.ToggleFullscreen();
                }
                else
                {
                    // Create fullscreen form with video panel
                    var fullscreenForm = new Form
                    {
                        FormBorderStyle = FormBorderStyle.None,
                        WindowState = FormWindowState.Maximized,
                        BackColor = Color.Black,
                        KeyPreview = true,
                    };
                    
                    fullscreenForm.KeyDown += (s, e) =>
                    {
                        if (e.KeyCode == Keys.Escape)
                        {
                            fullscreenForm.Close();
                        }
                    };
                    
                    var infoLabel = new Label
                    {
                        Text = $"Fullscreen Mode\n\n{_streamTitle}\n{_streamUrl}\n\nPress ESC to exit",
                        Dock = DockStyle.Fill,
                        ForeColor = Color.White,
                        Font = new Font("Segoe UI", 14),
                        TextAlign = ContentAlignment.MiddleCenter,
                    };
                    fullscreenForm.Controls.Add(infoLabel);
                    
                    fullscreenForm.Show();
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Fullscreen error - {ex.Message}");
            }
        }
        
        // ============================================================
        // Helper Methods
        // ============================================================
        
        private void UpdateStatus(string status, Color color)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => UpdateStatus(status, color)));
                return;
            }
            
            _lblStatus.Text = status;
            _lblStatus.ForeColor = color;
        }
        
        public void UpdateStreamUrl(string newUrl)
        {
            var wasPlaying = _isPlaying;
            
            if (_isPlaying)
            {
                StopStream();
            }
            
            _streamUrl = newUrl;
            
            if (wasPlaying)
            {
                StartStream();
            }
        }
        
        // ============================================================
        // Cleanup
        // ============================================================
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                try
                {
                    StopStream();
                    
                    if (_mediaPlayer != null)
                    {
                        _mediaPlayer.Dispose();
                        _mediaPlayer = null;
                    }
                    
                    if (_libVLC != null)
                    {
                        _libVLC.Dispose();
                        _libVLC = null;
                    }
                }
                catch { }
            }
            
            base.Dispose(disposing);
        }
    }
}
