
SWITCH=$1
case $SWITCH in
    shell)
    sshpass -p "root" ssh root@192.168.100.115
    ;;
    mount)
    sshpass -p "root" sshfs root@192.168.100.115:/root/ ~/ssh_mnt
    ;;
    umount)
    umount ~/ssh_mnt
    ;;
    push)
    sshpass -p "root" scp -r $2 root@192.168.100.115:/root/
    ;;
    pull)
    sshpass -p "root" scp -r root@192.168.100.115:/root/$2 ./
    ;;
    build)
    maixcdk build --no-gen
    ;;
    *)
    echo "usage:{$0 shell|mount|umount|push|pull}"
    ;;
esac
