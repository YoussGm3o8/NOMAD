// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO;
using System.Threading;
using System.Web.Script.Serialization;
using NOMAD.MissionPlanner;

internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length != 1)
        {
            Console.Error.WriteLine("Usage: nomad-link-router config.json (enter 'stop' or Ctrl+C to stop)");
            return 2;
        }
        try
        {
            var json = File.ReadAllText(args[0]);
            var config = new JavaScriptSerializer().Deserialize<GroundLinkRouter.RouterConfig>(json);
            using (var router = new GroundLinkRouter(config))
            using (var management = new RouterManagementServer(
                router,
                config.ManagementBindAddress,
                config.ManagementPort,
                "nomad-link-router-1"))
            using (var stop = new ManualResetEvent(false))
            {
                ConsoleCancelEventHandler cancel = (sender, e) => { e.Cancel = true; stop.Set(); };
                Console.CancelKeyPress += cancel;
                router.LogMessage += (sender, message) => Console.Error.WriteLine(message);
                router.ActiveLinkChanged += (sender, id) => Console.WriteLine("Active: " + id);
                router.Start();
                management.Start();
                Console.WriteLine("READY");
                var input = new Thread(() =>
                {
                    while (true)
                    {
                        var line = Console.ReadLine();
                        if (line == null)
                        {
                            return;
                        }
                        if (line == "stop")
                        {
                            stop.Set(); return;
                        }
                        if (line == "status")
                        {
                            Console.WriteLine(router.GetStatusSummary());
                        }
                        if (line == "auto")
                        {
                            router.SetManualOverride("");
                        }
                        if (line.StartsWith("select ", StringComparison.Ordinal))
                        {
                            Console.WriteLine(router.SetManualOverride(line.Substring(7)) ? "SELECTED" : "REJECTED");
                        }
                    }
                }) { IsBackground = true };
                input.Start();
                stop.WaitOne();
                router.Stop();
                Console.CancelKeyPress -= cancel;
            }
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex.Message); return 1;
        }
    }
}
