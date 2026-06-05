$port = New-Object System.IO.Ports.SerialPort COM17, 115200, None, 8, one
$port.ReadTimeout = 2000
try {
    $port.Open()
    Write-Output "Opened COM17. Resetting ESP32..."
    
    # Toggle EN (RTS) and IO0 (DTR) to reset the chip
    $port.DtrEnable = $false
    $port.RtsEnable = $false
    Start-Sleep -Milliseconds 100
    $port.DtrEnable = $true
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 200
    
    Write-Output "ESP32 reset. Reading lines for 25 seconds..."
    $endTime = (Get-Date).AddSeconds(25)
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
        Write-Output "Closed COM17."
    }
}
