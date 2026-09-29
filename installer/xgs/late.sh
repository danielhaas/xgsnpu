#!/bin/sh
# preseed/late_command: runs in the installer with the new system at /target
X=/cdrom/xgs; T=/target
KV=$(cat $X/xgsnpu.kver)
log() { echo "xgs-late: $*" >&2; }
if [ -d $T/lib/modules/$KV ]; then
	mkdir -p $T/lib/modules/$KV/extra && cp $X/xgsnpu.ko $T/lib/modules/$KV/extra/
	in-target depmod -a $KV
else
	log "installed kernel is not $KV - prebuilt module skipped, firstboot will build it"
fi
mkdir -p $T/usr/local/lib/xgsnpu $T/usr/src/xgsnpu-0.1 $T/root/.ssh $T/etc/network/interfaces.d
cp $X/mcp2210.py $T/usr/local/lib/xgsnpu/
cp $X/xgsnpu-up $X/xgsnpu-firstboot $T/usr/local/sbin/
chmod 755 $T/usr/local/sbin/xgsnpu-up $T/usr/local/sbin/xgsnpu-firstboot
cp $X/xgsnpu.service $X/xgsnpu-firstboot.service $T/etc/systemd/system/
cp $X/src/* $T/usr/src/xgsnpu-0.1/
printf '# loaded by xgsnpu.service after the NPU reset pulse - never autoload\nblacklist xgsnpu\n' > $T/etc/modprobe.d/xgsnpu.conf
grep -q interfaces.d $T/etc/network/interfaces || echo 'source /etc/network/interfaces.d/*' >> $T/etc/network/interfaces
printf 'allow-hotplug port1\niface port1 inet dhcp\n' > $T/etc/network/interfaces.d/port1
cat $X/authorized_keys >> $T/root/.ssh/authorized_keys
chmod 700 $T/root/.ssh; chmod 600 $T/root/.ssh/authorized_keys
cat > $T/etc/apt/sources.list <<SRC
deb http://deb.debian.org/debian trixie main contrib non-free-firmware
deb http://deb.debian.org/debian trixie-updates main contrib non-free-firmware
deb http://security.debian.org/debian-security trixie-security main contrib non-free-firmware
SRC
in-target systemctl enable xgsnpu.service xgsnpu-firstboot.service
log done
