#!bin/bash
docker ps -a | grep vslam > /dev/null 2>&1
if [ $? -eq 0 ]; then
    docker stop vslam
    sleep 3
fi

#sudo python3 -m pip install git+https://github.com/TriOrb-Inc/triorb-core.git@dev/202411
ls /dev/sd*1 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo '/dev/sd*1 is found'
else
    sudo python3 -c "import triorb_core, time; r = triorb_core.robot(); print(r.get_info()); time.sleep(1); r.ota_reboot()"
    sleep 3
fi

ls /dev/sd*1 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    ls /dev/sd*1 | xargs -i sudo mount -t vfat {} /media
    sleep 3
    sudo cp ./drive.uf2 /media/
    sleep 3
    sudo umount /media
else
    echo '/dev/sd*1 is not found'
fi
