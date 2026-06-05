$port = New-Object System.IO.Ports.SerialPort COM8, 115200, None, 8, one
try {
    $port.Open()
    Write-Output "Resetting ESP32 into normal running mode..."
    $port.DtrEnable = $false
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 100
    $port.RtsEnable = $false
    Start-Sleep -Milliseconds 200
    Write-Output "Reset complete."
} catch {
    Write-Output "Error: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}
