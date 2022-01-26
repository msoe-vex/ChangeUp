$docker_command = "docker run -it"
$docker_vol_location = -join(" -v ", $PSScriptRoot.Replace("/scripts", ""), ":/root/ChangeUp raiderrobotics/container-registry:rr-noetic-base")
$docker_container_command = -join(" sh -c 'cd /root/ChangeUp;", $args[0], "'")

# $IsWindows doesn't work because powershell 5.1 is too old
if ([System.Environment]::OSVersion.Platform -eq "Win32NT") {
    $is_not_first
    $is_exited
    $joined_cmd = -join($docker_command, $docker_vol_location, $docker_container_command)
    while(!$is_exited) {
        # run docker container
        Invoke-Expression -Command $joined_cmd
        # check if container has exited to see if it ran successfully (will fail if docker desktop is not running)
        $container_status = docker ps -a -q -f ancestor=raiderrobotics/container-registry:rr-noetic-base -f status=exited
        if (!$container_status) {
	        # attempt to launch docker desktop only once
            if(!($is_not_first)) {
                C:/Program` Files/Docker/Docker/Docker` Desktop.exe -WindowsStyle Minimized
                $is_not_first = "True"
            }
            Write-Host "Docker Desktop Not Running"
            Start-Sleep -Seconds 1
        }
        else {
	        # remove container after it exits
            docker rm $container_status
            $is_exited = "True"
        }
    }
}
elseif ($IsLinux) {
    # rootless podman permission workaround
    $is_podman
    if (Get-Command podman -erroraction 'silentlycontinue') {
        $is_podman = " --privileged --net=host"
    }

    # docker command with environment variables that enable X11 programs to work
    $joined_cmd = -join($docker_command, " --rm", $is_podman, " --env DISPLAY='", `
    [System.Environment]::GetEnvironmentVariable('DISPLAY'), "' --env='QT_X11_NO_MITSHM=1'", $docker_vol_location, $docker_container_command)
    Invoke-Expression -Command $joined_cmd
}
elseif ($IsMacOS) {
    Write-Host "macOS not supported"
}
