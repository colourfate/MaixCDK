SWITCH=$1
DEV=sde
case $SWITCH in
    probe)
    modprobe vhci_hcd
    ;;
    list)
    fdisk -l
    ;;
    mount)
    mount /dev/${DEV}2 /mnt/rootfs
    mount /dev/${DEV}1 /mnt/boot
    ;;
    umount)
    umount /mnt/rootfs
    umount /mnt/boot
    ;;
    push)
    if [ -z "$2" ]; then
        echo "Please specify the file to push."
        exit 1
    fi
    cp "$2" /mnt/rootfs/root -rf
    ;;
    pull)
    if [ -z "$2" ]; then
        echo "Please specify the file to pull."
        exit 1
    fi
    cp /mnt/rootfs/root/"$2" ./ -rf
    ;;
    all)
    mount /dev/${DEV}2 /mnt/rootfs
    cp "$2" /mnt/rootfs/root -rf
    umount /mnt/rootfs
    ;;
    *)
    echo "usage:{$0 probe|list|mount|umount|push|pull}"
    ;;
esac
