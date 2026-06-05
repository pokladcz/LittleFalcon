$port = New-Object System.IO.Ports.SerialPort COM8, 921600, None, 8, one
$port.ReadTimeout = 2000
try {
    $port.Open()
    Write-Output "Opened COM8. Waiting 3 seconds for boot..."
    Start-Sleep -Seconds 3
    Write-Output "Reading lines..."
    $endTime = (Get-Date).AddSeconds(10)
    while ((Get-Date) -lt $endTime) {
        try {
            $line = $port.ReadLine()
            Write-Output $line
        } catch {
            # Write-Output "Timeout or no data."
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
