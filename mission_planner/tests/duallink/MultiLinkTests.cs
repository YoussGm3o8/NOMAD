// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private static GroundLinkRouter.RouterConfig MultiConfig(int port)
    {
        return new GroundLinkRouter.RouterConfig
        {
            PreferredLink = "cell", HeartbeatTimeoutSec = .6, StatsTickMs = 20,
            FailoverCooldownSec = .1, PreferredLinkReconnectDelaySec = 1,
            Links = new List<LinkConfig>
            {
                new LinkConfig { Id = "cell", Port = port + 1, Priority = 100 },
                new LinkConfig { Id = "radio", Port = port + 2, Priority = 80 },
                new LinkConfig { Id = "wifi", Port = port + 3, Priority = 60 },
                new LinkConfig { Id = "disabled", Port = port + 4, Enabled = false }
            },
            Consumers = new List<ConsumerConfig>
            {
                new ConsumerConfig { Id = "mp", RouterPort = port },
                new ConsumerConfig { Id = "core", RouterPort = port + 5 },
                new ConsumerConfig { Id = "absent", RouterPort = port + 6, ClientPort = port + 7 }
            }
        };
    }

    private static void MultiConfigValidation()
    {
        Action<Action<GroundLinkRouter.RouterConfig>> rejects = change =>
        {
            var config = MultiConfig(31000);
            change(config);
            bool rejected = false;
            try { using (var router = new GroundLinkRouter(config)) { } }
            catch (ArgumentException) { rejected = true; }
            Check(rejected, "invalid configuration rejected before opening sockets");
        };
        rejects(c => c.Links[1].Id = "cell");
        rejects(c => c.Links[1].Port = c.Links[0].Port);
        rejects(c => c.Links[0].Transport = "BOGUS");
        rejects(c => c.Links[0].Port = 65536);
        rejects(c => c.HeartbeatTimeoutSec = double.NaN);
        rejects(c => c.PreferredLink = "disabled");
        rejects(c => c.Consumers[0].RouterPort = c.Links[0].Port);
        rejects(c => c.Consumers[0].ClientPort = c.Consumers[1].RouterPort);
        rejects(c => c.Consumers[0].ClientPort = c.Links[0].Port);
        rejects(c => { c.Links[0].RemoteHost = "127.0.0.1"; c.Links[0].RemotePort = c.Consumers[0].RouterPort; });
        rejects(c => c.BindAddress = "0.0.0.0");
    }

    private static async Task MultiLinkRouting()
    {
        const int port = 31000;
        using (var router = new GroundLinkRouter(MultiConfig(port)))
        using (var cell = UdpSink.ConnectedTo(port + 1))
        using (var radio = UdpSink.ConnectedTo(port + 2))
        using (var wifi = UdpSink.ConnectedTo(port + 3))
        using (var mp = UdpSink.ConnectedTo(port))
        using (var core = UdpSink.ConnectedTo(port + 5))
        {
            router.Start();
            mp.Send(Frames.Heartbeat(255, 190, 0));
            core.Send(Frames.Heartbeat(254, 191, 0));
            using (var pump = new Pump(cell.Send, 10))
            using (var backup = new Pump(radio.Send, 10))
            using (var third = new Pump(wifi.Send, 10))
            {
                Check(await WaitUntil(() => router.Links.Count(s => s.IsConnected) == 3, 3000), "three links live");
                Check(!router.SetManualOverride("unknown"), "unknown override rejected");
                Check(!router.SetManualOverride("disabled"), "disabled override rejected");
                Check(router.ManualOverride == "", "rejected override leaves auto unchanged");
                await CheckSequenceIsolation(router, cell, radio, wifi);
                await CheckHandoverDedup(router, cell, radio, wifi, mp, core);
                await CheckConsumerIsolation(router, cell, radio, wifi, mp, core);
                await CheckCommandNoFanout(router, cell, radio, wifi, core);
                await CheckParameterPin(router, cell, radio, wifi, mp);
            }
            router.Stop();
            foreach (var number in new[] { port, port + 1, port + 2, port + 3, port + 5, port + 6 })
            { using (var rebound = new UdpClient(new IPEndPoint(IPAddress.Loopback, number))) { } }
            router.Start();
            Check(router.Links.All(s => !s.IsConnected), "restart discards old liveness");
            router.Stop();
        }
    }

    private static async Task CheckSequenceIsolation(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi)
    {
        router.ResetCounters();
        foreach (var link in new[] { cell, radio, wifi })
        {
            for (int i = 0; i < 20; i++)
            {
                link.Send(Frames.Marker((uint)i, (byte)(250 + i), 1, 70));
                link.Send(Frames.Marker((uint)i, (byte)(100 + i), 2, 70));
            }
        }
        Check(await WaitUntil(() => router.Links.Take(3).All(l => l.FramesReceived >= 40), 2000),
            "every physical link observes independent system/component streams");
        Check(router.Links.Take(3).All(l => l.PacketLossPercent < .5), "sequence wrap has no phantom loss");
        // A partial frame on one source must not consume another source's bytes.
        cell.Send(Frames.Marker(88111, 1).Take(5).ToArray());
        var before = router.Links[2].FramesReceived;
        wifi.Send(Frames.Marker(88112, 1));
        Check(await WaitUntil(() => router.Links[2].FramesReceived > before, 1000),
            "one partial parser cannot corrupt another link");
        cell.Send(Frames.Marker(88111, 1).Skip(5).ToArray());
    }

    private static async Task CheckHandoverDedup(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink mp, UdpSink core)
    {
        router.SetManualOverride("cell");
        mp.Drain(); core.Drain();
        var frame = Frames.Marker(88001, 8);
        cell.Send(frame);
        var first = new List<uint>();
        Check(await WaitUntil(() => { DrainMarkers(mp, first); return first.Contains(88001); }, 2000), "first delivery");
        router.SetManualOverride("radio");
        radio.Send(frame);
        await Task.Delay(30);
        router.SetManualOverride("wifi");
        wifi.Send(frame);
        await Task.Delay(150);
        DrainMarkers(mp, first);
        CheckEq(first.Count(m => m == 88001), 1, "three-link handover delivers one copy to MP");
        CheckEq(DrainMarkers(core).Count(m => m == 88001), 1, "three-link handover delivers one copy to core");
        // Standby-first arrival must not hide the later active copy.
        var next = Frames.Marker(88002, 9);
        cell.Send(next); radio.Send(next); await Task.Delay(30); wifi.Send(next);
        var received = new List<uint>();
        Check(await WaitUntil(() => { DrainMarkers(core, received); return received.Contains(88002); }, 2000),
            "standby-first copy cannot suppress active delivery");
    }

    private static async Task CheckConsumerIsolation(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink mp, UdpSink core)
    {
        router.SetManualOverride("wifi");
        cell.Drain(); radio.Drain(); wifi.Drain(); mp.Drain(); core.Drain();
        var command = Frames.Marker(89001, 10, 255, 190);
        mp.Send(command.Take(7).ToArray());
        core.Send(Frames.Marker(89002, 11, 254, 191));
        mp.Send(command.Skip(7).ToArray());
        var received = new List<uint>();
        Check(await WaitUntil(() => { DrainMarkers(wifi, received); return received.Count >= 2; }, 2000),
            "both local parsers complete independently");
        await Task.Delay(100);
        DrainMarkers(wifi, received);
        CheckEq(received.Count(m => m == 89001), 1, "MP command emitted once");
        CheckEq(received.Count(m => m == 89002), 1, "core command emitted once");
        CheckEq(DrainMarkers(cell).Count, 0, "command never reaches cell standby");
        CheckEq(DrainMarkers(radio).Count, 0, "command never reaches radio standby");
        CheckEq(DrainMarkers(core).Count, 0, "local command not reflected to another consumer");
        var telemetry = Frames.Marker(89003, 12);
        wifi.Send(telemetry);
        var downlink = new List<uint>();
        Check(await WaitUntil(() => { DrainMarkers(mp, downlink); return downlink.Contains(89003); }, 2000),
            "absent third consumer does not block telemetry");
        mp.Send(telemetry); core.Send(telemetry);
        await Task.Delay(100);
        CheckEq(DrainMarkers(wifi).Count, 0, "reflected downlink is not sent back to aircraft");
    }

    private static async Task CheckCommandNoFanout(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink core)
    {
        router.SetManualOverride("radio");
        cell.Drain(); radio.Drain(); wifi.Drain();
        // COMMAND_LONG: command field 400 (arm/disarm), all other payload fields zero.
        var payload = new byte[33];
        payload[28] = 144; payload[29] = 1;
        var command = Frames.V2(255, 190, 76, 33, payload);
        core.Send(command);
        var received = new List<byte[]>();
        Check(await WaitUntil(() => { received.AddRange(radio.Drain());
            return received.Any(f => f.SequenceEqual(command)); }, 2000), "ARM command reaches selected radio");
        await Task.Delay(150);
        received.AddRange(radio.Drain());
        CheckEq(received.Count(f => f.SequenceEqual(command)), 1, "ARM appears once on selected radio");
        CheckEq(cell.Drain().Count(f => Frames.MsgIdOf(f) == 76), 0, "ARM never reaches cell standby");
        CheckEq(wifi.Drain().Count(f => Frames.MsgIdOf(f) == 76), 0, "ARM never reaches Wi-Fi standby");
    }

    private static async Task CheckParameterPin(GroundLinkRouter router, UdpSink cell, UdpSink radio,
        UdpSink wifi, UdpSink mp)
    {
        router.SetManualOverride("cell");
        cell.Drain(); radio.Drain(); wifi.Drain();
        mp.Send(Frames.ParamRequestList(20));
        var frames = new List<byte[]>();
        Check(await WaitUntil(() => { frames.AddRange(cell.Drain()); return frames.Any(f => Frames.MsgIdOf(f) == 21); },
            2000), "parameter transaction starts on cell");
        router.SetManualOverride("wifi");
        mp.Send(Frames.V2(255, 190, 23, 21, new byte[23]));
        frames.Clear();
        Check(await WaitUntil(() => { frames.AddRange(cell.Drain()); return frames.Any(f => Frames.MsgIdOf(f) == 23); },
            2000), "PARAM_SET stays pinned despite manual selection change");
        CheckEq(wifi.Drain().Count(f => Frames.MsgIdOf(f) == 23), 0, "parameter write not copied to new active link");
        CheckEq(radio.Drain().Count(f => Frames.MsgIdOf(f) == 23), 0, "parameter write not copied to standby");
    }

    private static async Task TcpIsolation()
    {
        const int port = 31100;
        var config = MultiConfig(port);
        config.Links[0].Transport = "TCP";
        config.Links[0].RemoteHost = "127.0.0.1";
        config.Links[0].ReconnectSeconds = .1;
        var listener = new TcpListener(IPAddress.Loopback, port + 1);
        listener.Start();
        try
        {
            using (var router = new GroundLinkRouter(config))
            using (var wifi = UdpSink.ConnectedTo(port + 3))
            using (var mp = UdpSink.ConnectedTo(port))
            using (var pump = new Pump(wifi.Send, 10))
            {
                router.Start();
                using (var peer = await listener.AcceptTcpClientAsync())
                {
                    mp.Send(Frames.Heartbeat(255, 190, 0));
                    var heartbeat = Frames.Heartbeat(1, 1, 1);
                    await peer.GetStream().WriteAsync(heartbeat, 0, 4);
                    await peer.GetStream().WriteAsync(heartbeat, 4, heartbeat.Length - 4);
                    Check(await WaitUntil(() => router.Links[0].FramesReceived > 0, 2000), "TCP split frame parsed");
                }
                Check(await WaitUntil(() => router.ActiveLink == "wifi", 3000), "TCP close leaves UDP routing live");
                using (var reconnected = await listener.AcceptTcpClientAsync())
                { Check(reconnected.Connected, "TCP reconnect opens a fresh connection"); }
            }
        }
        finally { listener.Stop(); }
    }
}
