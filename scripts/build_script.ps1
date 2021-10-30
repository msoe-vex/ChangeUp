if(!((wsl.exe -l --running) -contains("docker-desktop"))){
    C:/Program` Files/Docker/Docker/Docker` Desktop.exe -WindowsStyle Minimized
    Write-Output "Started Docker Desktop"
    Write-Output "Checking if Docker is Running"
    while(!((wsl.exe -l --running) -contains("docker-desktop"))){
        Write-Output "Checking if Docker is Running"
        Start-Sleep -Seconds 1
    }  
}
Write-Output "Docker Desktop is Running"
docker run --rm -it -v $HOME/ChangeUp:/root/ChangeUp raiderrobotics/container-registry:rr-noetic-base sh -c 'cd /root/ChangeUp; catkin_make v5_hal_firmware_build'