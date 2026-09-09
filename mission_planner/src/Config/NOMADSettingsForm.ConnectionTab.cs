// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateConnectionTab()
        {
            var tab = CreateTabPage("Core");
            int y = 15;

            AddSectionLabel(tab, "C++ Core Command Boundary", ref y);

            AddLabel(tab, "Core executable:", 20, y);
            _txtCoreExePath = AddTextBox(tab, 170, y, 360);
            y += 30;

            AddLabel(tab, "MAVLink endpoint:", 20, y);
            _txtCoreEndpoint = AddTextBox(tab, 170, y, 360);
            y += 30;

            AddLabel(tab, "Core API key:", 20, y);
            _txtCoreApiKey = AddTextBox(tab, 170, y, 360);
            _txtCoreApiKey.UseSystemPasswordChar = true;
            y += 35;

            var hint = new Label
            {
                Text = "Vehicle commands are validated and acknowledged by the C++ core. " +
                       "The core endpoint is a local MAVLink transport, not a REST service.",
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
                ForeColor = Color.FromArgb(170, 170, 170),
                Location = new Point(20, y),
                AutoSize = true,
                MaximumSize = new Size(560, 0),
            };
            tab.Controls.Add(hint);

            return tab;
        }

        private TabPage CreateDualLinkTab()
        {
            var tab = CreateTabPage("Dual Link");
            int y = 15;

            AddSectionLabel(tab, "MAVLink Dual Link (LTE + RadioMaster)", ref y);

            _chkDualLinkEnabled = AddCheckBox(tab, "Enable NOMAD dual-link router", 20, y, Color.LimeGreen);
            _chkDualLinkEnabled.CheckedChanged += (s, e) => UpdateDualLinkControlsState();
            y += 35;

            AddLabel(tab, "RadioMaster Type:", 40, y);
            _cmbRadioMasterConnType = AddComboBox(tab, 170, y, 80, new[] { "UDP", "COM", "TCP" });
            _cmbRadioMasterConnType.SelectedIndexChanged += (s, e) => UpdateRadioMasterConnTypeState();
            y += 30;

            _lblRadioMasterPort = AddLabel(tab, "UDP Port:", 40, y);
            _numRadioMasterPort = AddNumericUpDown(tab, 170, y, 80, 1, 65535, 14550);
            y += 30;

            _lblRadioTcpHost = AddLabel(tab, "TCP Host:", 40, y);
            _txtRadioTcpHost = AddTextBox(tab, 170, y, 110);
            y += 30;

            AddLabel(tab, "COM Port:", 40, y);
            _cmbRadioMasterComPort = AddComboBox(tab, 170, y, 100, SerialPort.GetPortNames());
            y += 30;

            AddLabel(tab, "Baud Rate:", 40, y);
            _cmbRadioMasterBaudRate = AddComboBox(tab, 170, y, 100, new[] { "115200", "420000", "460800", "921600" });
            y += 30;

            AddLabel(tab, "LTE MAVLink Port:", 40, y);
            _numLteMavlinkPort = AddNumericUpDown(tab, 170, y, 80, 1, 65535, 14560);
            y += 35;

            AddSectionLabel(tab, "Failover Settings", ref y);

            _chkAutoFailover = AddCheckBox(tab, "Enable Automatic Failover", 40, y);
            y += 30;

            AddLabel(tab, "Preferred Link:", 40, y);
            _cmbPreferredLink = AddComboBox(tab, 170, y, 130, new[] { "LTE", "RadioMaster", "None" });
            y += 30;

            _chkAutoReconnectPreferred = AddCheckBox(tab, "Auto-reconnect to preferred link", 40, y);
            y += 30;

            AddLabel(tab, "Reconnect Delay (s):", 40, y);
            _numPreferredReconnectDelay = AddNumericUpDown(tab, 170, y, 60, 1, 120, 10);
            y += 30;

            AddLabel(tab, "Heartbeat Timeout (s):", 40, y);
            _numHeartbeatTimeout = AddNumericUpDown(tab, 170, y, 60, 1, 30, 3, 1);
            y += 30;

            AddLabel(tab, "Monitor Interval (ms):", 40, y);
            _numLinkMonitorInterval = AddNumericUpDown(tab, 170, y, 80, 100, 5000, 500);
            y += 35;

            AddSectionLabel(tab, "Local Router", ref y);

            AddLabel(tab, "Bind Address:", 40, y);
            _txtRouterBindAddress = AddTextBox(tab, 170, y, 130);
            y += 30;

            AddLabel(tab, "Local UDP Port:", 40, y);
            _numRouterLocalPort = AddNumericUpDown(tab, 170, y, 80, 1024, 65535, 14600);
            y += 30;

            _chkRouterDedup = AddCheckBox(tab, "Deduplicate cross-link packets", 40, y);
            y += 30;

            var routerHint = new Label
            {
                Text = "When enabled, connect Mission Planner to the merged loopback endpoint " +
                       "shown above. When disabled, use the direct MAVLink link configured in Mission Planner.",
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
                ForeColor = Color.FromArgb(150, 150, 150),
                Location = new Point(40, y),
                AutoSize = true,
                MaximumSize = new Size(500, 0),
            };
            tab.Controls.Add(routerHint);

            return tab;
        }

        private void UpdateDualLinkControlsState()
        {
            bool enabled = _chkDualLinkEnabled.Checked;
            _cmbRadioMasterConnType.Enabled = enabled;
            _numRadioMasterPort.Enabled = enabled;
            _cmbRadioMasterComPort.Enabled = enabled;
            _cmbRadioMasterBaudRate.Enabled = enabled;
            _numLteMavlinkPort.Enabled = enabled;
            _chkAutoFailover.Enabled = enabled;
            _cmbPreferredLink.Enabled = enabled;
            _chkAutoReconnectPreferred.Enabled = enabled;
            _numPreferredReconnectDelay.Enabled = enabled;
            _numHeartbeatTimeout.Enabled = enabled;
            _numLinkMonitorInterval.Enabled = enabled;
            if (_txtRouterBindAddress != null) _txtRouterBindAddress.Enabled = enabled;
            if (_numRouterLocalPort != null) _numRouterLocalPort.Enabled = enabled;
            if (_chkRouterDedup != null) _chkRouterDedup.Enabled = enabled;

            if (enabled)
            {
                UpdateRadioMasterConnTypeState();
            }
        }

        private void UpdateRadioMasterConnTypeState()
        {
            int idx = _cmbRadioMasterConnType.SelectedIndex;
            bool isUDP = idx == 0;
            bool isCOM = idx == 1;
            bool isTCP = idx == 2;
            _numRadioMasterPort.Visible = isUDP || isTCP;
            if (_lblRadioTcpHost != null) _lblRadioTcpHost.Visible = isTCP;
            if (_txtRadioTcpHost != null) _txtRadioTcpHost.Visible = isTCP;
            _cmbRadioMasterComPort.Visible = isCOM;
            _cmbRadioMasterBaudRate.Visible = isCOM;
            if (_lblRadioMasterPort != null)
            {
                _lblRadioMasterPort.Text = isUDP ? "UDP Port:" : (isTCP ? "TCP Port:" : "COM Port:");
            }
        }
    }
}
