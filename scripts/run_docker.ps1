$is_exited
$is_not_first
while(!$is_exited) {
    $docker_command = -join("docker run -it -v $HOME/ChangeUp:/root/ChangeUp raiderrobotics/container-registry:rr-noetic-base sh -c ", "'cd /root/ChangeUp;", $args[0], "'");
    Invoke-Expression -Command $docker_command
    if (!(docker ps -a -q -f ancestor=raiderrobotics/container-registry:rr-noetic-base -f status=exited)) {
        if(!($is_not_first)) {
            C:/Program` Files/Docker/Docker/Docker` Desktop.exe -WindowsStyle Minimized
            $is_not_first = "True"
        }
        Write-Output "Docker Desktop Not Running"
        Start-Sleep -Seconds 1
    }
    else {
        docker rm $(docker ps --filter status=exited -q)
        $is_exited = "True"
    }
}