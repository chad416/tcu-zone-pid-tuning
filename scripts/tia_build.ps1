<#
  Build and compile the SCL blocks in TIA Portal through TIA Openness (SPEC V02).

  Last run 9 Nov 2026: 0 errors, 0 warnings (results\tia_compile.txt).

  Needs: TIA Portal V18 with STEP 7 Professional (licence), the Windows user in the
  "Siemens TIA Openness" group, Windows PowerShell 5.1.

  Usage:  powershell -ExecutionPolicy Bypass -File scripts\tia_build.ps1 -OutDir C:\TIA_tcu
  Result: a TIA project with FB_ZonePID, FB_ZonePlant, FB_Scenario, FB_ZoneTest on an
          S7-1513 (6ES7 513-1AL02-0AB0), compiled; the compile messages are printed and
          written to results\tia_compile.txt.  OB30 + DBs from plc\tia\ are added by
          scripts\tia_ob30.ps1.
#>
param(
    [string]$OutDir = "C:\TIA_tcu",
    [string]$Version = "V18",
    [string]$Cpu = "OrderNumber:6ES7 513-1AL02-0AB0/V2.9"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$api = "C:\Program Files\Siemens\Automation\Portal $Version\PublicAPI\$Version\Siemens.Engineering.dll"
if (-not (Test-Path $api)) { throw "Openness API not found: $api" }

$code = @"
using System;
using System.IO;
using System.Linq;
using System.Text;
using Siemens.Engineering;
using Siemens.Engineering.HW;
using Siemens.Engineering.HW.Features;
using Siemens.Engineering.SW;
using Siemens.Engineering.SW.ExternalSources;
using Siemens.Engineering.Compiler;

public static class TcuBuild {
    static void Walk(CompilerResultMessage m, StringBuilder sb, int depth) {
        sb.AppendLine(new string(' ', 2 * depth) + m.State + "  " + m.Path + "  " + m.Description);
        foreach (var c in m.Messages) Walk(c, sb, depth + 1);
    }
    public static string Run(string outDir, string plcDir, string cpu) {
        var sb = new StringBuilder();
        using (var tia = new TiaPortal(TiaPortalMode.WithoutUserInterface)) {
            var dir = new DirectoryInfo(outDir);
            var project = tia.Projects.Create(dir, "tcu_zone_" + DateTime.Now.ToString("yyyyMMdd_HHmmss"));
            var dev = project.Devices.CreateWithItem(cpu, "PLC_1", "PLC_1");
            PlcSoftware plc = null;
            foreach (var item in dev.DeviceItems) {
                var sc = item.GetService<SoftwareContainer>();
                if (sc != null && sc.Software is PlcSoftware) { plc = (PlcSoftware)sc.Software; break; }
            }
            if (plc == null) throw new Exception("no PLC software in " + cpu);
            foreach (var name in new[] { "FB_Scenario", "FB_ZonePID", "FB_ZonePlant", "FB_ZoneTest" }) {
                var src = plc.ExternalSourceGroup.ExternalSources.CreateFromFile(
                    name, Path.Combine(plcDir, name + ".scl"));
                try {
                    src.GenerateBlocksFromSource();
                    sb.AppendLine("generated " + name);
                } catch (Exception ex) {
                    sb.AppendLine("GENERATE FAILED " + name + ": " + (ex.InnerException ?? ex).Message);
                }
            }
            var result = plc.GetService<ICompilable>().Compile();
            sb.AppendLine("COMPILE " + result.State + "  errors=" + result.ErrorCount + "  warnings=" + result.WarningCount);
            foreach (var m in result.Messages) Walk(m, sb, 1);
            project.Save();
            sb.AppendLine("project: " + project.Path.FullName);
            project.Close();
        }
        return sb.ToString();
    }
}
"@

# Assembly resolver in compiled C# (a PowerShell script-block handler recursed into a
# StackOverflowException on the first run).
$resolver = @"
using System; using System.IO; using System.Reflection;
public static class OpnResolve {
    static string[] dirs;
    public static void Init(string[] d) {
        dirs = d;
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) => {
            var n = new AssemblyName(e.Name).Name;
            if (n.EndsWith(".resources")) return null;
            foreach (var dir in dirs) { var p = Path.Combine(dir, n + ".dll"); if (File.Exists(p)) return Assembly.LoadFrom(p); }
            return null;
        };
    }
}
"@
Add-Type -TypeDefinition $resolver
$portal = "C:\Program Files\Siemens\Automation\Portal $Version"
[OpnResolve]::Init(@((Split-Path -Parent $api), "$portal\Bin\PublicAPI", "$portal\Bin"))
New-Item -ItemType Directory -Force $OutDir | Out-Null
[void][System.Reflection.Assembly]::LoadFrom($api)
Add-Type -TypeDefinition $code -ReferencedAssemblies $api
$text = [TcuBuild]::Run($OutDir, (Join-Path $root "plc"), $Cpu)
$text
$text | Out-File -Encoding utf8 (Join-Path $root "results\tia_compile.txt")
