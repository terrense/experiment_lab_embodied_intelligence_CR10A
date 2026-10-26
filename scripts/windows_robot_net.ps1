# Configure the wired (USB-Ethernet) adapter for the Dobot controller network.
#   Show only (no changes):   powershell -ExecutionPolicy Bypass -File scripts\windows_robot_net.ps1
#   Apply (ADMIN PowerShell): powershell -ExecutionPolicy Bypass -File scripts\windows_robot_net.ps1 -Apply
# Sets <adapter> = 192.168.5.100/24 with NO default gateway, so internet keeps using WLAN.
# Undo: Remove-NetIPAddress -InterfaceAlias "<adapter>" -IPAddress 192.168.5.100 ; Set-NetIPInterface -InterfaceAlias "<adapter>" -Dhcp Enabled
param([switch]$Apply, [string]$Ip = '192.168.5.100', [int]$Prefix = 24, [string]$Robot = '192.168.5.1')

$virtual = 'Hyper-V|Meta Tunnel|TAP|Wintun|WireGuard|VPN|Loopback|Bluetooth|Wi-Fi|WiFi|Wireless|802\.11'
$eth = Get-NetAdapter -Physical -ErrorAction SilentlyContinue | Where-Object { $_.InterfaceDescription -notmatch $virtual -and $_.Name -notmatch 'WLAN' }
"== candidate wired adapters"
$eth | Select-Object Name, InterfaceDescription, Status, LinkSpeed | Format-Table -AutoSize
if (-not $eth) { "No wired adapter found. Plug in the USB-Ethernet adapter and re-run."; exit 1 }
if (@($eth).Count -gt 1) { "More than one wired adapter; unplug extras or edit the script."; exit 1 }
$name = $eth.Name
"== current IPv4 on '$name'"
Get-NetIPAddress -InterfaceAlias $name -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object IPAddress, PrefixLength, PrefixOrigin | Format-Table -AutoSize

if ($Apply) {
  $admin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
  if (-not $admin) { "Run this in an ADMINISTRATOR PowerShell for -Apply."; exit 1 }
  Set-NetIPInterface -InterfaceAlias $name -Dhcp Disabled
  Get-NetIPAddress -InterfaceAlias $name -AddressFamily IPv4 -ErrorAction SilentlyContinue | Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
  New-NetIPAddress -InterfaceAlias $name -IPAddress $Ip -PrefixLength $Prefix | Out-Null   # no -DefaultGateway on purpose
  "set $name -> $Ip/$Prefix (no gateway)"
}
"== route to $Robot"
Find-NetRoute -RemoteIPAddress $Robot -ErrorAction SilentlyContinue | Select-Object -First 1 InterfaceAlias, IPAddress, DestinationPrefix | Format-List
"== ping $Robot"
Test-Connection -ComputerName $Robot -Count 2 -ErrorAction SilentlyContinue | Select-Object Address, ResponseTime
