# Capture the RViz (WSLg) window to a PNG from the Windows side.
#   powershell -ExecutionPolicy Bypass -File scripts\capture_rviz_window.ps1 -Out assets\screenshots\x.png [-Count 3 -IntervalS 4]
# Brings the RViz window to the foreground, captures only its rectangle
# (clipped to the visible screen), nothing else on the desktop.
param([Parameter(Mandatory = $true)][string]$Out, [int]$Count = 1, [double]$IntervalS = 3, [string]$TitleRegex = ' - RViz( |$)')
Add-Type -AssemblyName System.Drawing, System.Windows.Forms
Add-Type @'
using System; using System.Runtime.InteropServices; using System.Text;
public class ExWin {
  public delegate bool EnumProc(IntPtr h, IntPtr l);
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc f, IntPtr l);
  [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  public struct RECT { public int L, T, R, B; }
}
'@
[ExWin]::SetProcessDPIAware() | Out-Null
$script:hw = [IntPtr]::Zero
[ExWin]::EnumWindows({ param($h, $l)
    if ([ExWin]::IsWindowVisible($h)) {
      $sb = New-Object System.Text.StringBuilder 512; [void][ExWin]::GetWindowText($h, $sb, 512)
      if ($sb.ToString() -cmatch $TitleRegex) { $script:hw = $h } }
    $true }, [IntPtr]::Zero) | Out-Null
if ($script:hw -eq [IntPtr]::Zero) { Write-Error "no window matching '$TitleRegex'"; exit 1 }
[ExWin]::ShowWindow($script:hw, 9) | Out-Null
[ExWin]::SetForegroundWindow($script:hw) | Out-Null
Start-Sleep -Seconds 2
$rect = New-Object ExWin+RECT; [ExWin]::GetWindowRect($script:hw, [ref]$rect) | Out-Null
$scr = [System.Windows.Forms.SystemInformation]::VirtualScreen
$x0 = [Math]::Max($rect.L, $scr.Left); $y0 = [Math]::Max($rect.T, $scr.Top)
$x1 = [Math]::Min($rect.R, $scr.Right); $y1 = [Math]::Min($rect.B, $scr.Bottom)
$w = $x1 - $x0; $hgt = $y1 - $y0
$dir = Split-Path -Parent $Out; if ($dir) { New-Item -ItemType Directory -Force $dir | Out-Null }
for ($i = 1; $i -le $Count; $i++) {
  $path = if ($Count -gt 1) { $Out -replace '\.png$', "_$i.png" } else { $Out }
  $bmp = New-Object System.Drawing.Bitmap $w, $hgt
  $g = [System.Drawing.Graphics]::FromImage($bmp); $g.CopyFromScreen($x0, $y0, 0, 0, $bmp.Size)
  $bmp.Save($path, [System.Drawing.Imaging.ImageFormat]::Png); $g.Dispose(); $bmp.Dispose()
  Write-Output "saved $path (${w}x${hgt})"
  if ($i -lt $Count) { Start-Sleep -Seconds $IntervalS }
}
