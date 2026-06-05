Get-CimInstance Win32_Process -Filter "name = 'python.exe' or name = 'node.exe'" | Select-Object ProcessId, Name, CommandLine | Format-List
