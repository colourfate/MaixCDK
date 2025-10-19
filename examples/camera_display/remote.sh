
SWITCH=$1
case $SWITCH in
    shell)
    ssh root@192.168.100.178
    ;;
    mount)
    sshfs root@192.168.100.178:/root/ ~/ssh_mnt
    ;;
    umount)
    umount ~/ssh_mnt
    ;;
    push)
    cp $2 ~/ssh_mnt -rf
    ;;
    pull)
    cp ~/ssh_mnt/$2 ./ -rf
    ;;
    *)
    echo "usage:{$0 shell|mount|umount|push|pull}"
    ;;
esac
