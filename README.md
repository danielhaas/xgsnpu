# xgsnpu — Sophos XGS 136 front ports under Debian

Linux port of the FreeBSD `npuep` driver from https://github.com/AbdelmonemAwad/os-xgs-npu
(BSD-2-Clause). Talks to the Marvell CN9131 NPU (PCI 11ab:7080) running its **stock** Sophos
firmware; exposes the 14 front ports as `port1`..`port12`, `portf1`, `portf2`.

## Layout
- `driver/` — kernel module source, Makefile, dkms.conf (installed on the XGS as `/usr/src/xgsnpu-0.1`, DKMS)
- `tools/xgsnpu-up` — pulses the NPU reset via the MCP2210 (`/dev/hidraw0`), waits for the NPU
  console (`/dev/ttyS2`, 115200) to report the handshake wait, restores BARs (PCI remove+rescan),
  then `modprobe xgsnpu`. Installed as `/usr/local/sbin/xgsnpu-up`.
- `tools/mcp2210.py` — MCP2210 GPIO tool from os-xgs-npu, patched for Linux hidraw (leading report-ID byte)
- `tools/barmap.py` — read-only dump of the NPU facility table
- `systemd/xgsnpu.service` — runs `xgsnpu-up` before `network-pre.target`

## On the XGS
- `/etc/modprobe.d/xgsnpu.conf` blacklists autoload (the NPU is held in reset at power-on; touching its
  BARs then can hang the host). Only `xgsnpu-up` loads it.
- The NPU accepts one host session per NPU boot: reloading the module needs a fresh reset pulse —
  just run `xgsnpu-up` again.
- `/etc/network/interfaces.d/port1`: DHCP on port1.
- NPU console log: `/var/log/xgsnpu-npu-console.log`.

## Status (2026-09-29)
All 14 ports programmed (RPC tables verified, NetAgent 14/14 up). port1 verified end to end:
DHCP, internet, apt, SSH; iperf3 ~360 Mbit/s each way (limited by the test client's USB 2.0 NIC).
Other ports not yet cable-tested. MACs: 02:<crc32(DMI product serial)>:00:<port>.

## Warning
Experimental. It drives an undocumented coprocessor; a mistake can hang the appliance until
power-cycled. Use a serial console and don't run it on anything you rely on.

## Install (on the XGS, Debian 13)
```sh
apt install dkms linux-headers-amd64 build-essential python3
cp -r driver /usr/src/xgsnpu-0.1 && dkms install xgsnpu/0.1
install -D tools/mcp2210.py /usr/local/lib/xgsnpu/mcp2210.py
install tools/xgsnpu-up /usr/local/sbin/ && install -m644 systemd/xgsnpu.service /etc/systemd/system/
echo "blacklist xgsnpu" > /etc/modprobe.d/xgsnpu.conf
systemctl daemon-reload && systemctl enable --now xgsnpu
```

## License
BSD-2-Clause (see LICENSE). Derived from os-xgs-npu by Abdelmonem Awad.
