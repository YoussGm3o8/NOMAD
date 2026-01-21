# NOMAD Mission Planner Plugin - New Features Integration Guide

## Build Complete

The NOMAD Mission Planner plugin has been successfully built with three new features:

1. **Telemetry Injection** - Status messages to HUD
2. **WASD Nudge Control** - Indoor keyboard control
3. **Jetson Health Tab** - Real-time health monitoring

---

## Integrating New Features

### Option A: Update Existing Plugin Code

To use the new features in the existing NOMADControlPanel, add the following:

#### 1. Add Private Fields

Add to `NOMADControlPanel.cs`:

```csharp
// New feature instances
private TelemetryInjector _telemetryInjector;
private WASDNudgeControl _wasdControl;
private JetsonHealthTab _healthTab;
```

#### 2. Initialize in Constructor

Add to the constructor after existing initialization:

```csharp
// Initialize new features
_telemetryInjector = new TelemetryInjector(MainV2.comPort?.MAV);
_wasdControl = new WASDNudgeControl(MainV2.comPort?.MAV);
_wasdControl.NudgeSpeed = 0.5f; // 0.5 m/s default

// Add health tab to panel
_healthTab = new JetsonHealthTab();
_healthTab.Location = new Point(10, 520); // Adjust as needed
_healthTab.SetJetsonUrl(_config.JetsonBaseUrl);
this.Controls.Add(_healthTab);
```

#### 3. Add WASD Control Toggle

Add a checkbox to enable/disable WASD control:

```csharp
var chkWASD = new CheckBox
{
    Text = "Enable WASD Indoor Control",
    Location = new Point(15, 480),
    AutoSize = true,
    ForeColor = Color.White
};
chkWASD.CheckedChanged += (s, e) =>
{
    _wasdControl.Enabled = chkWASD.Checked;
    if (chkWASD.Checked)
    {
        MessageBox.Show(
            "WASD Control Enabled\n\n" +
            "W/S = Forward/Back\n" +
            "A/D = Left/Right\n" +
            "Q/E = Up/Down\n\n" +
            "Use for indoor nudging only!",
            "WASD Control",
            MessageBoxButtons.OK,
            MessageBoxIcon.Information
        );
    }
};
this.Controls.Add(chkWASD);
```

#### 4. Handle Keyboard Events

Override keyboard handlers in `NOMADControlPanel`:

```csharp
protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
{
    if (_wasdControl?.Enabled == true)
    {
        if (keyData == Keys.W || keyData == Keys.A || 
            keyData == Keys.S || keyData == Keys.D || 
            keyData == Keys.Q || keyData == Keys.E)
        {
            _wasdControl.HandleKeyDown(keyData);
            return true; // Handled
        }
    }
    return base.ProcessCmdKey(ref msg, keyData);
}

protected override void OnKeyUp(KeyEventArgs e)
{
    if (_wasdControl?.Enabled == true)
    {
        _wasdControl.HandleKeyUp(e.KeyCode);
    }
    base.OnKeyUp(e);
}
```

#### 5. Use Telemetry Injection

Replace existing status updates with telemetry injection:

```csharp
// Example: After snapshot capture
_telemetryInjector.SendTaskStatus(1, "Snapshot Captured");

// Example: Vision status update
_telemetryInjector.SendVisionStatus(true, "YOLO Running");

// Example: Target detection
_telemetryInjector.SendTargetStatus(true, "Fire");
```

---

### Option B: Standalone Health Monitor Tab

If you just want the health monitor tab without modifying existing code:

1. Open Mission Planner
2. Go to **Flight Data** screen
3. Right-click on the left panel
4. Select **Add User Control**
5. Browse to: `C:\Users\Youssef\AppData\Local\Mission Planner\plugins\NOMADPlugin.dll`
6. Select `JetsonHealthTab` control

---

## Testing the New Features

### 1. Test Telemetry Injection

```csharp
// In NOMADControlPanel or test harness
_telemetryInjector.SendCustomStatus("Test Message");
```

Check the Mission Planner HUD and console for "NOMAD: Test Message"

### 2. Test WASD Control

1. Enable WASD checkbox
2. Ensure drone is in Guided mode
3. Press W key - verify `SET_POSITION_TARGET_LOCAL_NED` is sent
4. Check MAVLink inspector (Ctrl+F) for velocity commands

### 3. Test Health Monitor

1. Ensure Jetson is accessible at configured URL
2. Health tab should auto-update every 2 seconds
3. Verify CPU/GPU load and temperature readings
4. Change URL if using Tailscale IP

---

## Configuration Updates

Update `NOMADConfig.cs` to store WASD settings:

```csharp
public class NOMADConfig
{
    // Existing properties...
    
    // WASD Control Settings
    public bool WasdEnabled { get; set; } = false;
    public float WasdNudgeSpeed { get; set; } = 0.5f; // m/s
}
```

---

## Mission Planner Setup

### Enable Plugin

1. Launch Mission Planner
2. Go to **Tools** → **NOMAD Settings**
3. Set Jetson URL (e.g., `http://100.100.10.5:8000` for Tailscale)
4. Click **Save**

### Access Features

**Health Monitor:**
- Automatically visible in NOMAD Control Panel (if integrated)
- Or add as standalone tab (see Option B above)

**WASD Control:**
- Check "Enable WASD Indoor Control"
- Use W/A/S/D/Q/E keys for nudging
- **CRITICAL:** Only use in Guided mode during Task 2 (indoor)

**Telemetry:**
- Automatic - messages appear in HUD
- Check Mission Planner console (Messages tab)

---

## Safety Notes

### WASD Control Safety

**WARNING:** WASD control sends velocity commands directly to the drone

- **Only use in controlled indoor environment**
- **Ensure Guided mode is active**
- **Have RC transmitter ready for manual takeover**
- **Test thoroughly in simulation first**
- **Disable when not in use**

### Recommended Workflow

1. **Pre-flight:** Keep WASD disabled
2. **Task 1 (Outdoor):** WASD disabled - use GPS waypoints
3. **Task 2 (Indoor):** Enable WASD for fine adjustments only
4. **Post-flight:** Disable WASD immediately

---

## Troubleshooting

### Issue: WASD Not Responding

**Check:**
- WASD is enabled (checkbox checked)
- Mission Planner has focus (click on window)
- MAVLink connection is active
- Drone is in Guided mode

### Issue: Health Tab Shows "Error"

**Check:**
- Jetson URL is correct
- Tailscale is connected (if using remote IP)
- Jetson service is running: `sudo systemctl status nomad`
- Firewall allows port 8000

### Issue: Telemetry Not Appearing

**Check:**
- MAVLink connection is active
- Messages appear in console (Ctrl+F → Messages)
- HUD is visible in Mission Planner

---

## Next Steps

1. **Rebuild plugin** if you integrated features (Option A)
2. **Test in simulation** with SITL
3. **Test on bench** with Jetson connected
4. **Field test** with Tailscale over 4G/LTE
5. **Competition deployment** with full system integration

---

## Quick Reference

**Build Command:**
```powershell
.\build_and_deploy.ps1
```

**Deploy Locations:**
- User: `%LOCALAPPDATA%\Mission Planner\plugins\NOMADPlugin.dll`
- System: `C:\Program Files (x86)\Mission Planner\plugins\NOMADPlugin.dll`

**Feature Classes:**
- `TelemetryInjector` - HUD status messages
- `WASDNudgeControl` - Keyboard velocity control
- `JetsonHealthTab` - Real-time health monitoring

---

## Support

For issues or questions:
- Check [Mission Planner README](README.md)
- Review [Implementation Summary](../docs/IMPLEMENTATION_SUMMARY.md)
- Verify [Tailscale Setup](../docs/TAILSCALE_SETUP.md)
