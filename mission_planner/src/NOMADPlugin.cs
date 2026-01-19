// ============================================================
// NOMAD Mission Planner Plugin
// ============================================================
// Target: Mission Planner 1.3.x
// Competition: AEAC 2026
//
// This plugin adds NOMAD-specific controls to Mission Planner
// for Task 1 (Recon) and Task 2 (Extinguish) operations.
//
// Features:
// - Full-page NOMAD control interface with tabs
// - Embedded video streaming
// - Jetson terminal access
// - System health monitoring
// - Dual-link communication (HTTP or MAVLink/ELRS)
// - Task 1: Capture snapshot command
// - Task 2: Reset exclusion map, WASD indoor control
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Main NOMAD plugin class implementing Mission Planner Plugin interface.
    /// </summary>
    public class NOMADPlugin : Plugin
    {
        // Plugin metadata
        public override string Name => "NOMAD Control";
        public override string Version => "2.0.0";
        public override string Author => "McGill Aerial Design";

        // Plugin state
        private NOMADConfig _config;
        private DualLinkSender _sender;
        private NOMADControlPanel _controlPanelTab;
        private NOMADControlPanel _controlPanelWindow;
        private NOMADFullPage _fullPage;
        private TabPage _tabPage;
        private Form _panelForm;
        private Form _fullPageForm;
        private bool _tabAdded;

        // ============================================================
        // Plugin Lifecycle
        // ============================================================

        /// <summary>
        /// Called when plugin is loaded.
        /// </summary>
        public override bool Init()
        {
            try
            {
                // Load configuration
                _config = NOMADConfig.Load();
                
                // Initialize dual-link sender
                _sender = new DualLinkSender(_config);

                // Add menu items to FlightData right-click menu (if available)
                try
                {
                    if (Host?.FDMenuMap?.Items != null)
                    {
                        var openFullPageItem = new ToolStripMenuItem("NOMAD Full Control (New!)");
                        openFullPageItem.Click += (s, e) => ShowFullPage();
                        Host.FDMenuMap.Items.Add(openFullPageItem);
                        
                        var openPanelItem = new ToolStripMenuItem("NOMAD Quick Panel");
                        openPanelItem.Click += (s, e) => ShowControlPanel();
                        Host.FDMenuMap.Items.Add(openPanelItem);

                        var settingsItem = new ToolStripMenuItem("NOMAD Settings");
                        settingsItem.Click += (s, e) => ShowSettings();
                        Host.FDMenuMap.Items.Add(settingsItem);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"NOMAD: Could not add FlightData menu - {ex.Message}");
                }

                // Also add to main menu bar under Help (common location for plugins)
                try
                {
                    var menuStrip = Host.MainForm.MainMenuStrip;
                    if (menuStrip != null)
                    {
                        // Find or create NOMAD menu
                        ToolStripMenuItem nomadMenu = null;
                        foreach (ToolStripItem existing in menuStrip.Items)
                        {
                            if (existing is ToolStripMenuItem item && string.Equals(item.Text, "NOMAD", StringComparison.OrdinalIgnoreCase))
                            {
                                nomadMenu = item;
                                break;
                            }
                        }
                        if (nomadMenu == null)
                        {
                            nomadMenu = new ToolStripMenuItem("NOMAD")
                            {
                                ForeColor = Color.White,
                                BackColor = Color.FromArgb(0, 122, 204)
                            };
                            // Insert before Help menu (usually last)
                            int insertIndex = menuStrip.Items.Count - 1;
                            if (insertIndex < 0) insertIndex = 0;
                            menuStrip.Items.Insert(insertIndex, nomadMenu);
                        }

                        // Avoid duplicate items if plugin reloads
                        nomadMenu.DropDownItems.Clear();

                        // NEW: Full Page Control (Primary)
                        var openFullPageItem = new ToolStripMenuItem(">> Open Full Control Page");
                        openFullPageItem.Font = new Font(openFullPageItem.Font, FontStyle.Bold);
                        openFullPageItem.Click += (s, e) => ShowFullPage();
                        nomadMenu.DropDownItems.Add(openFullPageItem);
                        
                        nomadMenu.DropDownItems.Add(new ToolStripSeparator());
                        
                        var openPanelItem = new ToolStripMenuItem("Quick Control Panel");
                        openPanelItem.Click += (s, e) => ShowControlPanel();
                        nomadMenu.DropDownItems.Add(openPanelItem);
                        
                        nomadMenu.DropDownItems.Add(new ToolStripSeparator());
                        
                        var settingsItem2 = new ToolStripMenuItem("[Settings...]");
                        settingsItem2.Click += (s, e) => ShowSettings();
                        nomadMenu.DropDownItems.Add(settingsItem2);
                        
                        var aboutItem = new ToolStripMenuItem("[i] About NOMAD");
                        aboutItem.Click += (s, e) => CustomMessageBox.Show(
                            $"NOMAD Plugin v{Version}\n" +
                            $"McGill Aerial Design - AEAC 2026\n\n" +
                            $"Features:\n" +
                            $"• Full-page control interface\n" +
                            $"• Embedded video streaming\n" +
                            $"• Jetson terminal access\n" +
                            $"• Real-time health monitoring\n" +
                            $"• Task 1 (Outdoor) & Task 2 (Indoor) support\n\n" +
                            $"Jetson: {_config.EffectiveIP}:{_config.JetsonPort}\n" +
                            $"Mode: {(_config.UseELRS ? "ELRS/MAVLink" : "HTTP")}",
                            "About NOMAD"
                        );
                        nomadMenu.DropDownItems.Add(aboutItem);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"NOMAD: Could not add main menu - {ex.Message}");
                }

                // Keep startup non-blocking; show popup only in DebugMode
                if (_config.DebugMode)
                {
                    Host?.MainForm?.BeginInvoke((MethodInvoker)delegate
                    {
                        CustomMessageBox.Show(
                            $"NOMAD Plugin v{Version} loaded.\n\n" +
                            $"Use the NOMAD menu → Open Full Control Page\n" +
                            $"for the complete NOMAD interface.\n\n" +
                            $"Mode: {(_config.UseELRS ? "ELRS/MAVLink" : "HTTP")}\n" +
                            $"Jetson IP: {_config.EffectiveIP}",
                            "NOMAD"
                        );
                    });
                }

                return true;
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"NOMAD Plugin failed to load: {ex.Message}", "Error");
                return false;
            }
        }

        /// <summary>
        /// Called when FlightData tab is first shown.
        /// Adds NOMAD control panel to the UI.
        /// </summary>
        public override bool Loaded()
        {
            try
            {
                // Ensure UI setup runs on the UI thread
                if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
                {
                    Host.MainForm.BeginInvoke((MethodInvoker)delegate { Loaded(); });
                    return true;
                }

                if (_controlPanelTab == null)
                {
                    _controlPanelTab = new NOMADControlPanel(_sender, _config);
                }

                if (_tabPage == null)
                {
                    _tabPage = new TabPage("NOMAD")
                    {
                        BackColor = Color.FromArgb(45, 45, 48)
                    };
                    _tabPage.Controls.Add(_controlPanelTab);
                    _controlPanelTab.Dock = DockStyle.Fill;
                }

                // Attempt tab injection; if it fails, use the control-panel window.
                if (!_tabAdded)
                {
                    _tabAdded = AddTabToFlightData(_tabPage);
                    if (!_tabAdded)
                    {
                        Console.WriteLine("NOMAD: Could not add FlightData tab; using control panel window");
                        // Don't auto-show - let user choose from menu
                    }
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to create UI - {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Called periodically during FlightData updates.
        /// </summary>
        public override bool Loop()
        {
            // Update status display if needed
            var connected = Host.cs.connected;
            var lat = Host.cs.lat;
            var lng = Host.cs.lng;
            var alt = Host.cs.alt;

            _controlPanelTab?.UpdateStatus(connected, lat, lng, alt);
            _controlPanelWindow?.UpdateStatus(connected, lat, lng, alt);
            return true;
        }

        /// <summary>
        /// Called when plugin is unloaded.
        /// </summary>
        public override bool Exit()
        {
            try
            {
                // Force close forms (bypass FormClosing hide behavior)
                if (_fullPageForm != null && !_fullPageForm.IsDisposed)
                {
                    _fullPage?.Dispose();
                    _fullPage = null;
                    _fullPageForm.Dispose();
                    _fullPageForm = null;
                }
                
                if (_panelForm != null && !_panelForm.IsDisposed)
                {
                    _controlPanelWindow?.Dispose();
                    _controlPanelWindow = null;
                    _panelForm.Dispose();
                    _panelForm = null;
                }
            }
            catch
            {
                // ignore disposal errors
            }

            _fullPage = null;
            _controlPanelWindow = null;
            _sender?.Dispose();
            return true;
        }

        // ============================================================
        // Private Methods
        // ============================================================

        private bool AddTabToFlightData(TabPage tab)
        {
            // Access FlightData tab control
            // Note: This depends on Mission Planner internals and may
            // need adjustment for different MP versions
            try
            {
                var flightData = Host.MainForm.Controls.Find("FlightData", true);
                if (flightData.Length > 0 && flightData[0] is UserControl fd)
                {
                    var tabControl = fd.Controls.Find("tabControlactions", true);
                    if (tabControl.Length > 0 && tabControl[0] is TabControl tc)
                    {
                        if (!tc.TabPages.Contains(tab))
                        {
                            tc.TabPages.Add(tab);
                        }
                        return true;
                    }
                }
                
                return false;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Tab insertion failed - {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Shows the full-page NOMAD control interface.
        /// This is the primary interface with all features.
        /// </summary>
        private void ShowFullPage()
        {
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowFullPage(); });
                return;
            }

            // Always create a fresh instance to avoid disposed object errors
            if (_fullPageForm == null || _fullPageForm.IsDisposed)
            {
                // Create new full page instance
                _fullPage = new NOMADFullPage(_sender, _config);
                
                _fullPageForm = new Form
                {
                    Text = "NOMAD Control Center - AEAC 2026",
                    StartPosition = FormStartPosition.CenterScreen,
                    Width = 1200,
                    Height = 800,
                    MinimumSize = new Size(900, 600),
                    Icon = null, // Could add custom icon
                    BackColor = Color.FromArgb(30, 30, 30),
                };
                
                // Handle FormClosing to prevent crash - hide instead of close when X clicked
                _fullPageForm.FormClosing += (s, e) =>
                {
                    if (e.CloseReason == CloseReason.UserClosing)
                    {
                        // Hide instead of close to prevent disposal issues
                        e.Cancel = true;
                        _fullPageForm.Hide();
                    }
                };
                
                _fullPageForm.FormClosed += (s, e) =>
                {
                    // Safely dispose control when form is actually closed
                    try
                    {
                        _fullPage?.Dispose();
                    }
                    catch { }
                    _fullPage = null;
                    _fullPageForm = null;
                };
                
                _fullPageForm.Controls.Add(_fullPage);
                _fullPage.Dock = DockStyle.Fill;
            }

            if (!_fullPageForm.Visible)
            {
                _fullPageForm.Show(Host?.MainForm);
            }
            else
            {
                _fullPageForm.BringToFront();
                _fullPageForm.Activate();
            }
        }

        private void ShowControlPanel()
        {
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowControlPanel(); });
                return;
            }

            // Always create a fresh instance to avoid disposed object errors
            if (_panelForm == null || _panelForm.IsDisposed)
            {
                // Create new panel instance
                _controlPanelWindow = new NOMADControlPanel(_sender, _config);
                
                _panelForm = new Form
                {
                    Text = "NOMAD Quick Panel",
                    StartPosition = FormStartPosition.CenterParent,
                    Width = 420,
                    Height = 800,
                    AutoScroll = true
                };
                
                // Handle FormClosing to prevent crash - hide instead of close when X clicked
                _panelForm.FormClosing += (s, e) =>
                {
                    if (e.CloseReason == CloseReason.UserClosing)
                    {
                        e.Cancel = true;
                        _panelForm.Hide();
                    }
                };
                
                _panelForm.FormClosed += (s, e) =>
                {
                    // Safely dispose control when form is actually closed
                    try
                    {
                        _controlPanelWindow?.Dispose();
                    }
                    catch { }
                    _controlPanelWindow = null;
                    _panelForm = null;
                };
                _panelForm.Controls.Add(_controlPanelWindow);
                _controlPanelWindow.Dock = DockStyle.Fill;
                _controlPanelWindow.AutoScroll = true;
            }

            if (!_panelForm.Visible)
            {
                _panelForm.Show(Host?.MainForm);
            }
            else
            {
                _panelForm.BringToFront();
                _panelForm.Activate();
            }
        }

        private void ShowSettings()
        {
            using (var form = new NOMADSettingsForm(_config))
            {
                if (form.ShowDialog() == DialogResult.OK)
                {
                    _config = form.Config;
                    _config.Save();
                    _sender.UpdateConfig(_config);
                    _controlPanelTab?.UpdateConfig(_config);
                    _controlPanelWindow?.UpdateConfig(_config);
                    _fullPage?.UpdateConfig(_config);
                }
            }
        }
    }
}
