Import("env")
import subprocess

def kill_com_monitor(source, target, env):
    port = env.GetProjectOption("upload_port", "COM11")
    print(f"Releasing {port} before upload...")
    try:
        # Only kill Python processes that are running 'device monitor' (serial monitor)
        # These have both "monitor" AND the COM port in their command line
        ps_cmd = (
            f'Get-WmiObject Win32_Process | '
            f'Where-Object {{ $_.Name -eq "python.exe" -and '
            f'$_.CommandLine -like "*monitor*" -and '
            f'$_.CommandLine -like "*{port}*" }} | '
            f'ForEach-Object {{ Write-Host "Killing monitor PID $($_.ProcessId)"; '
            f'Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }}'
        )
        result = subprocess.run(
            ["powershell", "-Command", ps_cmd],
            capture_output=True, text=True, timeout=10
        )
        if result.stdout.strip():
            print(result.stdout.strip())
        else:
            print(f"  No monitor holding {port}.")
    except Exception as e:
        print(f"  Note: {e}")

env.AddPreAction("upload", kill_com_monitor)
