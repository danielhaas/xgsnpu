# Unattended reinstall stick

Debian 13 netinst copied onto a FAT32 GPT USB stick (UEFI boot), plus:

- `grub.cfg` → `boot/grub/grub.cfg`: serial console, **default entry boots the internal SSD**
  (so a stick left in, with USB first in the boot order, is harmless). The automated reinstall
  must be chosen explicitly.
- `preseed.cfg` → stick root. Replace `CHANGEME_SHA512_HASH` (`openssl passwd -6`). Picks the
  first non-removable, non-USB disk and erases it.
- `xgs/` → stick `/xgs/`, plus: `xgsnpu.ko` built for the installer's kernel, `xgsnpu.kver`
  (that kernel's version), `src/` (= `../driver`), `xgsnpu-up`, `mcp2210.py`, `xgsnpu.service`
  (from `../tools`, `../systemd`), and `authorized_keys` (your SSH public key).

The installer has no network (the front ports need xgsnpu), so it installs offline; `late.sh`
drops in the prebuilt module and services. On first boot port1 comes up with DHCP and
`xgsnpu-firstboot` sets up apt and builds the module with DKMS for this and future kernels.

## Stick boot loader
Debian's signed GRUB looks for `/EFI/debian/grub.cfg` and, on this firmware, finds the **SSD's**
copy once Debian is installed - so the stick would just boot the SSD. The stick therefore uses a
standalone GRUB as `EFI/boot/bootx64.efi` (Secure Boot is off on the XGS), built with:

```sh
grub-mkstandalone -O x86_64-efi -o bootx64.efi \
  --modules="part_gpt part_msdos fat search search_fs_file configfile normal linux chain serial terminal efi_gop all_video echo test regexp ls" \
  "boot/grub/grub.cfg=stick-grub-embedded.cfg"
```
Replace the `/.disk/id/<uuid>` in `stick-grub-embedded.cfg` with the one on your stick.

To reinstall: pick "AUTOMATED REINSTALL" in the menu (over serial use Ctrl-N/Ctrl-P, not the
arrow keys - their ESC byte can leave the menu). Or from the running system, one-shot boot the
stick with `efibootmgr -n <stick entry>`.
