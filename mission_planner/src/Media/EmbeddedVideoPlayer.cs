// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player - GStreamer Implementation
// ============================================================
// Uses Mission Planner's built-in GStreamer for direct RTSP/UDP video.
// Topic and overlay controls belonged to the removed Python REST service and
// are intentionally not part of this player.
// ============================================================

using System;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;
using MissionPlanner.Utilities;
using MPBitmap = MPDrawing::System.Drawing.Bitmap;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Embedded video player using Mission Planner's GStreamer wrapper.
    /// </summary>
    public partial class EmbeddedVideoPlayer : UserControl
    {
        private string _streamUrl;
        private int _latencyMs = 100;
        private bool _isPlaying;

        private GStreamer _gst;
        private PictureBox _videoBox;
        private Label _lblStatus;
        private TrackBar _trkLatency;
        private Label _lblLatencyValue;
        private Form _fullscreenForm;
        private PictureBox _fullscreenBox;

        private readonly SemaphoreSlim _lifecycleLock = new SemaphoreSlim(1, 1);
        private int _streamGeneration;
        private volatile bool _stopping;

        private const int FrameBufferCount = 3;
        private readonly object _frameBufferLock = new object();
        private Bitmap[] _frameBuffers;
        private int _frameBufferWidth;
        private int _frameBufferHeight;
        private int _displayBufferIndex = -1;
        private int _pendingBufferIndex = -1;
        private int _nextBufferIndex;
        private int _frameCount;

        private readonly bool _showControls;

        /// <summary>
        /// Creates an embedded player for a direct RTSP or UDP stream.
        /// </summary>
        public EmbeddedVideoPlayer(string title, string streamUrl, bool showControls = true)
        {
            _streamUrl = streamUrl ?? "";
            _showControls = showControls;
            InitializeUI();

            this.HandleCreated += (s, e) => UiAsync.Run(this, async () =>
            {
                if (!_showControls && !_isPlaying && !IsDisposed)
                {
                    StartStream();
                }
                await System.Threading.Tasks.Task.CompletedTask;
            }, "EmbeddedVideoHandleCreated");
        }

        private int ExtractUdpPort(string url)
        {
            if (string.IsNullOrEmpty(url)) return 5600;
            string cleaned = url;
            if (cleaned.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
                cleaned = cleaned.Substring(6);
            cleaned = cleaned.Replace("@", "").TrimStart(':');
            return int.TryParse(cleaned, out int port) ? port : 5600;
        }


        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                StopStream();
                _lifecycleLock.Dispose();
                if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
                    _fullscreenForm.Close();
            }
            base.Dispose(disposing);
        }
    }
}
