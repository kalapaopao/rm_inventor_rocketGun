<#
.SYNOPSIS
  镖架串口协议 v1 解码监视器（模拟电控接收端），用于自测视觉端发出的帧。

.DESCRIPTION
  连接指定串口，按协议 docs/串口通讯协议_电控对接.md 解码：
    SOF 0xA5 | TARGET(0/1) | int16 YAW 小端 | CRC16-Modbus 小端
  逐帧打印人类可读结果，并统计帧数、CRC 错误、丢弃字节。
  失败/校验错时按字节滑动重新同步，与电控解码要求一致。

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1 -Port COM12 -Seconds 60
#>
param(
    [string]$Port = "COM12",
    [int]$Baud = 115200,
    [int]$Seconds = 30
)

$ErrorActionPreference = "Stop"

function Get-Crc16Modbus([byte[]]$data) {
    $crc = 0xFFFF
    foreach ($b in $data) {
        $crc = $crc -bxor $b
        for ($i = 0; $i -lt 8; $i++) {
            if ($crc -band 1) { $crc = (($crc -shr 1) -bxor 0xA001) -band 0xFFFF }
            else { $crc = ($crc -shr 1) -band 0xFFFF }
        }
    }
    return [int]$crc
}

$serial = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 200
try { $serial.Open() } catch {
    Write-Host "打开 $Port 失败: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
Write-Host "已连接 $Port @ $Baud 8N1，运行 $Seconds 秒（Ctrl+C 提前结束）" -ForegroundColor Green

$buffer = New-Object System.Collections.Generic.List[byte]
$frames = 0; $crcErrors = 0; $junkBytes = 0
$sw = [Diagnostics.Stopwatch]::StartNew()

try {
    while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
        $n = $serial.BytesToRead
        if ($n -gt 0) {
            $tmp = New-Object byte[] $n
            [void]$serial.Read($tmp, 0, $n)
            $buffer.AddRange($tmp)
        } else {
            Start-Sleep -Milliseconds 20
        }

        while ($true) {
            # 找帧头
            $sof = -1
            for ($i = 0; $i -lt $buffer.Count; $i++) {
                if ($buffer[$i] -eq 0xA5) { $sof = $i; break }
            }
            if ($sof -lt 0) {
                if ($buffer.Count -gt 0) { $junkBytes += $buffer.Count; $buffer.Clear() }
                break
            }
            if ($sof -gt 0) { $junkBytes += $sof; $buffer.RemoveRange(0, $sof) }
            if ($buffer.Count -lt 6) { break }   # 数据不足，等下一批

            $head = $buffer.GetRange(0, 4).ToArray()
            $crcCalc = Get-Crc16Modbus $head
            $crcGot = $buffer[4] -bor ($buffer[5] -shl 8)
            $target = $buffer[1]
            if ($crcCalc -ne $crcGot -or $target -gt 1) {
                $crcErrors++
                $buffer.RemoveAt(0)   # 跳过该帧头重新同步
                continue
            }

            $yaw = [BitConverter]::ToInt16($buffer.GetRange(2, 2).ToArray(), 0)
            $frames++
            if ($target -eq 1) {
                $sign = if ($yaw -gt 0) { "偏右" } elseif ($yaw -lt 0) { "偏左" } else { "已对准" }
                Write-Host ("[{0,4}] TARGET=1  yaw={1,6}px  ({2})" -f $frames, $yaw, $sign)
            } else {
                Write-Host ("[{0,4}] TARGET=0  无目标" -f $frames) -ForegroundColor DarkGray
            }
            $buffer.RemoveRange(0, 6)
        }
    }
} finally {
    $serial.Close()
}

Write-Host ""
Write-Host ("统计: 有效帧 {0}  CRC错误 {1}  丢弃字节 {2}" -f $frames, $crcErrors, $junkBytes) -ForegroundColor Cyan
if ($frames -eq 0) { exit 2 }
exit 0
