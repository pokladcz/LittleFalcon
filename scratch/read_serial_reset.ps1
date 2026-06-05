$port = New-Object System.IO.Ports.SerialPort COM8, 921600, None, 8, one
$port.ReadTimeout = 2000
try {
    $port.Open()
    Write-Output "Opened COM8. Resetting ESP32..."
    
    # Toggle EN (RTS) and IO0 (DTR) to reset the chip
    $port.DtrEnable = $false
    $port.RtsEnable = $false
    Start-Sleep -Milliseconds 100
    $port.DtrEnable = $true
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 200
    
    Write-Output "ESP32 reset. Reading lines for 10 seconds..."
    $endTime = (Get-Date).AddSeconds(10)
    while ((Get-Date) -lt $endTime) {
        try {
            $line = $port.ReadLine()
            Write-Output $line
        } catch {
            # Timeout
        }
    }
} catch {
    Write-Output "Failed to open port: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Output "Closed COM8."
    }
}
