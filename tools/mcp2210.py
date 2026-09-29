#!/usr/bin/python3
#-
# SPDX-License-Identifier: BSD-2-Clause
#
"""
Talk to the XGS 136's MCP2210 USB-to-SPI bridge from FreeBSD, the way Sophos does.

WHY THIS EXISTS. The NPU's serial console (uart2, 0x3E8) is silent - not one receive
interrupt in a quiet window - and the PCIe endpoint enumerating proves only that the NPU's
boot ROM ran, not that its Linux did. Sophos's own host-side tool `xgs-usb-spi-flash` has a
`-t  Reset the NPU` option, and it reaches the NPU's reset line over this bridge. So the
bridge is the host's one control path to the NPU that does not require the NPU to be alive.

EVERYTHING BELOW IS READ OUT OF SOPHOS'S BINARY, not out of the datasheet. The command bytes,
the field offsets and the per-board GPIO masks were all recovered by disassembling
`xgs-usb-spi-flash` (NPU-BSP-slot1, x86-64, stripped) - see docs/npu-bring-up.md.
Where something is a datasheet guess instead, it says so at the call site.

Transport is a plain 64-byte write() then a 64-byte read() on the HID device - no report ID
byte, which is what Sophos's code does and what FreeBSD's hidraw(4) gives you.

    ./mcp2210.py status                  read and decode; changes nothing
    ./mcp2210.py raw 20                  send one command byte, print the 64-byte reply
    ./mcp2210.py set-gpio 0xffff 0x0029  value, mask - THIS DRIVES THE NPU RESET LINE
"""
import os
import select
import time
import sys

DEV = '/dev/hidraw0'
REPORT = 64
TIMEOUT = float(os.environ.get('MCP_TIMEOUT', '2.0'))
ATTEMPTS = 4

# Commands that only read. Everything outside this set either writes the chip's volatile
# settings, writes its NVRAM, or starts an SPI transaction on a bus that is wired to the NPU's
# boot flash, so `probe` will not send them.
SAFE_PROBE = [0x10, 0x11, 0x20, 0x22, 0x31, 0x32, 0x33, 0x41, 0x50, 0x61, 0x76]

# Pin designation values, from the MCP2210's chip-settings block.
DESIG = {0: 'gpio', 1: 'cs', 2: 'dedicated'}

# The three pins Sophos touches on every board: GP0, GP3, GP5. It always passes this exact
# mask, so these are the only lines in play.
NPU_MASK = 0x0029

# Per-board GPIO values, from the two jump tables in xgs-usb-spi-flash. The index is the
# numeric part of the assembly number (AMDA0201 -> 201), and the entry decides what
# npu_reset(1) and npu_reset(0) write. npu_reset(1) is what the tool calls "getting control
# of the SPI bus"; npu_reset(0) is on every exit path, so it is the release.
#
# Note the polarity is NOT the same across boards - AMDA0200 is the exact inverse of ours.
# That is why Sophos carries a table rather than a constant, and why guessing would be wrong.
NPU_VALUES = {
    200: {'hold': 0xFFD7, 'release': 0xFFFF},   # XGS desktop gen 1
    201: {'hold': 0xFFFF, 'release': 0xFFD7},   # XGS 126 / 136   <- this board
    202: {'hold': 0xFFDF, 'release': 0xFFF7},   # XGS 1U small
    203: {'hold': 0xFFDF, 'release': 0xFFF7},
    204: {'hold': 0xFFDF, 'release': 0xFFF7},
    205: {'hold': 0xFFDF, 'release': 0xFFF7},
    208: {'hold': 0xFFFF, 'release': 0xFFD7},   # XGS 116
    224: {'hold': 0xFFFF, 'release': 0xFFD7},   # XGS 138
}
THIS_BOARD = 201


def drain(fh):
    """Throw away any reply still sitting in the HID queue from a previous command.

    THIS IS NOT OPTIONAL, and leaving it out produced a full hour of wrong conclusions. The
    queue desynchronises easily: one stale report and every later read returns the PREVIOUS
    command's answer, which looks like real data because the fields are in the right places;
    one missing report and the next read blocks until the timeout, which looks exactly like
    "the chip does not implement this command". Both happened, and both were believed.
    Draining first makes the reply to command N actually be the reply to command N.
    """
    dropped = 0
    while select.select([fh], [], [], 0)[0]:
        try:
            if not fh.read(REPORT):
                break
        except (OSError, IOError):
            break
        dropped += 1
        if dropped > 16:
            break
    return dropped


def xfer(fh, tx):
    """One 64-byte command, one 64-byte reply.

    The select() timeout is not politeness either. This runs over the serial console, and a
    bare read() with nothing coming blocks forever with the console stuck behind it. That
    already happened, and cost a Ctrl+C rescue.

    Note the device only rewrites the bytes a given reply needs and leaves the rest of its
    64-byte buffer holding the PREVIOUS reply. So never read a field a command does not
    document - it will be stale data that looks entirely plausible.
    """
    # The retry is measured, not defensive habit: the first command sent after opening the
    # device is dropped often enough to be the norm rather than the exception, which is what
    # made 0x20 look unimplemented on one run and fine on the next. A resend always fixes it.
    last = None
    for attempt in range(ATTEMPTS):
        drain(fh)
        buf = bytearray(REPORT)
        buf[:len(tx)] = tx
        # Linux hidraw: leading report-ID byte (0 = unnumbered reports)
        fh.write((b'\x00' if sys.platform.startswith('linux') else b'') + bytes(buf))
        fh.flush()
        ready, _, _ = select.select([fh], [], [], TIMEOUT)
        if not ready:
            last = 'no reply to command 0x%02x within %.1fs' % (tx[0], TIMEOUT)
            continue
        rx = fh.read(REPORT)
        if len(rx) != REPORT:
            last = 'short read: %d bytes' % len(rx)
            continue
        if rx[0] != tx[0]:
            last = 'reply is for 0x%02x, not 0x%02x' % (rx[0], tx[0])
            continue
        if rx[1] == 0xF9:
            raise IOError('command 0x%02x: chip says unknown command (0xf9)' % tx[0])
        return rx
    raise IOError('%s (after %d attempts)' % (last, ATTEMPTS))


def get_chip_settings(fh):
    """Command 0x20. Reply[1] is the status byte; 0 means it answered."""
    rx = xfer(fh, bytes([0x20]))
    if rx[1] != 0:
        raise IOError('get chip settings returned status 0x%02x' % rx[1])
    return {
        'desig': list(rx[4:13]),                        # GP0..GP8
        'value': rx[13] | (rx[14] << 8),                # power-up output value
        'dir': rx[15] | (rx[16] << 8),                  # 1 = input, 0 = output
        'other': rx[17],
        'raw': rx,
    }


def set_chip_settings(fh, st):
    """Command 0x21. Same field layout as 0x20's reply, shifted to the request."""
    tx = bytearray(REPORT)
    tx[0] = 0x21
    tx[4:13] = bytes(st['desig'])
    tx[13] = st['value'] & 0xFF
    tx[14] = (st['value'] >> 8) & 0xFF
    tx[15] = st['dir'] & 0xFF
    tx[16] = (st['dir'] >> 8) & 0xFF
    tx[17] = st['other']
    rx = xfer(fh, bytes(tx))
    if rx[1] != 0:
        raise IOError('set chip settings returned status 0x%02x' % rx[1])


def set_gpio_value(fh, value):
    """Command 0x30, the live pin values. 0xFB back means another master owns the bus."""
    rx = xfer(fh, bytes([0x30, 0, 0, 0, value & 0xFF, (value >> 8) & 0xFF]))
    if rx[1] == 0xFB:
        raise IOError('the SPI bus is owned by an external master (status 0xfb)')
    if rx[1] != 0:
        raise IOError('set gpio value returned status 0x%02x' % rx[1])
    return rx


def gpio_write(fh, value, mask):
    """Sophos's gpio_write(), step for step.

    For every pin in the mask: make it a GPIO, make it an output, and put the requested bit
    in the power-up value. Write that back as chip settings, then set the live pin values.
    Pins outside the mask keep whatever they had.
    """
    st = get_chip_settings(fh)
    for i in range(9):
        if mask & (1 << i):
            st['desig'][i] = 0                     # GPIO operation
            st['dir'] &= ~(1 << i)                 # output
            st['value'] = (st['value'] & ~(1 << i)) | (value & (1 << i))
    set_chip_settings(fh, st)
    set_gpio_value(fh, value & 0xFFFF)


def live_gpio(fh):
    """Command 0x32, the levels the pins are actually at. Only bytes 4 and 5 are this
    command's own - everything past them is the previous reply still sitting in the buffer."""
    rx = xfer(fh, bytes([0x32]))
    return rx[4] | (rx[5] << 8)


def show(st):
    print('pin  designation  dir     value')
    for i, d in enumerate(st['desig']):
        direction = 'input ' if st['dir'] & (1 << i) else 'output'
        level = 1 if st['value'] & (1 << i) else 0
        note = '   <- NPU control line' if (NPU_MASK >> i) & 1 else ''
        print('GP%d  %-11s  %s  %d%s' % (i, DESIG.get(d, '0x%02x' % d), direction, level, note))
    print()
    print('value 0x%04x   dir 0x%04x   other 0x%02x' % (st['value'], st['dir'], st['other']))

    want = NPU_VALUES[THIS_BOARD]
    got = st['value'] & NPU_MASK
    for name in ('hold', 'release'):
        if got == (want[name] & NPU_MASK):
            print('GP0/GP3/GP5 = 0x%02x, which is this board\'s "%s" state' % (got, name))
            break
    else:
        print('GP0/GP3/GP5 = 0x%02x, which matches NEITHER hold (0x%02x) nor release (0x%02x)'
              % (got, want['hold'] & NPU_MASK, want['release'] & NPU_MASK))


def main(argv):
    if len(argv) < 2:
        print(__doc__.strip())
        return 2
    cmd = argv[1]

    # Buffered I/O would merge or split the 64-byte reports, so open unbuffered.
    try:
        fh = open(DEV, 'r+b', buffering=0)
    except OSError as exc:
        print('cannot open %s: %s' % (DEV, exc))
        print('if the node is missing: kldload hidraw')
        return 1

    with fh:
        if cmd == 'status':
            show(get_chip_settings(fh))

        elif cmd == 'raw':
            if len(argv) < 3:
                print('usage: raw <hex command byte> [more hex bytes]')
                return 2
            tx = bytes(int(a, 16) for a in argv[2:])
            try:
                rx = xfer(fh, tx)
            except IOError as exc:
                print('tx %s -> %s' % (tx.hex(), exc))
                return 1
            print('tx', tx.hex())
            for off in range(0, REPORT, 16):
                print('%02x  %s' % (off, rx[off:off + 16].hex(' ')))

        elif cmd == 'release':
            # What the boot hook calls. Idempotent: if the pins are already released it says so
            # and writes nothing, so running it twice cannot disturb a working NPU.
            want = NPU_VALUES[THIS_BOARD]['release']
            now = live_gpio(fh)
            if (now & NPU_MASK) == (want & NPU_MASK):
                print('already released (live 0x%04x)' % now)
                return 0
            gpio_write(fh, want, NPU_MASK)
            after = live_gpio(fh)
            print('released: live 0x%04x -> 0x%04x' % (now, after))
            if (after & NPU_MASK) != (want & NPU_MASK):
                print('WARNING: pins did not take the requested state')
                return 1

        elif cmd == 'pulse':
            # Sophos's `-t  Reset the NPU`, reproduced: take the SPI bus (which holds the NPU),
            # then release it. `xgs-usb-spi-flash` does the release on its exit path, so a run
            # of -t is exactly this pair. One serial round trip, because the interesting thing
            # is what comes out of uart2 while it happens.
            hold = NPU_VALUES[THIS_BOARD]['hold']
            release = NPU_VALUES[THIS_BOARD]['release']
            print('start   live 0x%04x' % live_gpio(fh))
            gpio_write(fh, hold, NPU_MASK)
            print('hold    wrote 0x%04x mask 0x%04x -> live 0x%04x' % (hold, NPU_MASK, live_gpio(fh)))
            time.sleep(0.5)
            gpio_write(fh, release, NPU_MASK)
            print('release wrote 0x%04x mask 0x%04x -> live 0x%04x' % (release, NPU_MASK, live_gpio(fh)))
            time.sleep(1.0)
            print('settled live 0x%04x' % live_gpio(fh))

        elif cmd == 'nvram':
            # 0x61 sub 0x20: the power-up chip settings the chip loads from its own NVRAM.
            # This is the factory state, written at manufacture by `xgs-usb-spi-flash -m`.
            rx = xfer(fh, bytes([0x61, 0x20]))
            print('designations', list(rx[4:13]))
            print('value 0x%04x   dir 0x%04x   other 0x%02x'
                  % (rx[13] | (rx[14] << 8), rx[15] | (rx[16] << 8), rx[17]))

        elif cmd == 'restore':
            # Put the volatile settings back to the factory power-up values, without needing a
            # power cycle. Use this after any experiment that drove the NPU control pins.
            rx = xfer(fh, bytes([0x61, 0x20]))
            st = {'desig': list(rx[4:13]), 'value': rx[13] | (rx[14] << 8),
                  'dir': rx[15] | (rx[16] << 8), 'other': rx[17]}
            print('nvram : value 0x%04x dir 0x%04x' % (st['value'], st['dir']))
            set_chip_settings(fh, st)
            now = get_chip_settings(fh)
            print('now   : value 0x%04x dir 0x%04x' % (now['value'], now['dir']))

        elif cmd == 'probe':
            # Which commands does THIS chip actually implement? It answers nothing at all to
            # one it does not, so this is the only way to find out, and guessing from the
            # datasheet has already been wrong twice here.
            for c in SAFE_PROBE:
                try:
                    rx = xfer(fh, bytes([c]))
                    print('0x%02x  %s' % (c, rx[:8].hex(' ')))
                except IOError:
                    print('0x%02x  -- no reply' % c)

        elif cmd == 'set-gpio':
            if len(argv) != 4:
                print('usage: set-gpio <value> <mask>')
                return 2
            value, mask = int(argv[2], 0), int(argv[3], 0)
            before = get_chip_settings(fh)
            print('before: value 0x%04x dir 0x%04x' % (before['value'], before['dir']))
            gpio_write(fh, value, mask)
            after = get_chip_settings(fh)
            print('after : value 0x%04x dir 0x%04x' % (after['value'], after['dir']))

        else:
            print('unknown command %r' % cmd)
            return 2
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
