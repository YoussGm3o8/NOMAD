// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player - Playback partial
// ============================================================

using System;
using System.Drawing;
using MissionPlanner.Utilities;
using MPBitmap = MPDrawing::System.Drawing.Bitmap;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private bool TryInitializeGStreamer()
        {
            try
            {
                var gstPath = GStreamer.LookForGstreamer();
                return !string.IsNullOrWhiteSpace(gstPath) && GStreamer.GstLaunchExists;
            }
            catch
            {
                return false;
            }
        }

        private string BuildGStreamerPipeline()
        {
            int queueBuffers = _latencyMs <= 100 ? 1 : Math.Min(_latencyMs / 50, 10);
            string leaky = _latencyMs <= 100 ? "leaky=2" : "leaky=0";
            string syncValue = _latencyMs <= 100 ? "false" : "true";

            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                var port = ExtractUdpPort(_streamUrl);
                return $"udpsrc port={port} buffer-size=90000 ! " +
                       "application/x-rtp,media=(string)video,clock-rate=(int)90000," +
                       "encoding-name=(string)H264 ! decodebin3 ! " +
                       $"queue max-size-buffers={queueBuffers} {leaky} ! " +
                       "videoconvert ! video/x-raw,format=BGRA ! " +
                       $"appsink name=outsink sync={syncValue}";
            }

            return $"rtspsrc location={_streamUrl} protocols=tcp latency={_latencyMs} " +
                   "do-retransmission=false ! " +
                   "application/x-rtp,media=video,encoding-name=H264 ! " +
                   "rtph264depay ! h264parse disable-passthrough=true ! avdec_h264 ! " +
                   $"queue max-size-buffers={queueBuffers} {leaky} ! " +
                   "videoconvert ! video/x-raw,format=BGRA ! " +
                   $"appsink name=outsink sync={syncValue}";
        }

        private async System.Threading.Tasks.Task RestartStreamAsync()
        {
            await _lifecycleLock.WaitAsync();
            try
            {
                StopStream();
                await System.Threading.Tasks.Task.Delay(500);
                if (!IsDisposed)
                {
                    StartStream();
                }
            }
            finally
            {
                _lifecycleLock.Release();
            }
        }

        public void StartStream()
        {
            if (_isPlaying)
            {
                return;
            }

            if (!TryInitializeGStreamer())
            {
                _lblStatus.Text = "GStreamer not available - use VLC";
                _lblStatus.ForeColor = Color.Orange;
                return;
            }

            GStreamer gst = null;
            try
            {
                _stopping = false;
                _streamGeneration++;
                _frameCount = 0;

                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Starting stream to {_streamUrl}");
                gst = new GStreamer();
                gst.OnNewImage += OnGstNewImage;
                gst.Start(BuildGStreamerPipeline());
                _gst = gst;
                _isPlaying = true;
                gst = null;
                _lblStatus.Text = "Connecting...";
                _lblStatus.ForeColor = Color.Yellow;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Start error - {ex}");
                _lblStatus.Text = $"Error: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
            finally
            {
                if (gst != null)
                {
                    try { gst.OnNewImage -= OnGstNewImage; } catch { }
                    try { gst.Stop(); } catch { }
                    try { (gst as IDisposable)?.Dispose(); } catch { }
                }
            }
        }

        public void StopStream()
        {
            if (!_isPlaying && _gst == null)
            {
                return;
            }

            _stopping = true;
            _streamGeneration++;

            var gst = _gst;
            _gst = null;
            _isPlaying = false;

            try
            {
                if (gst != null)
                {
                    try { gst.OnNewImage -= OnGstNewImage; } catch { }
                    try { gst.Stop(); } catch { }
                    System.Threading.Thread.Sleep(300);
                    try { (gst as IDisposable)?.Dispose(); } catch { }
                }
            }
            catch
            {
            }

            ClearVideoDisplayAndDisposeBuffers();
            _lblStatus.Text = "Stopped";
            _lblStatus.ForeColor = Color.Gray;
        }
    }
}
