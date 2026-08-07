#!/bin/sh
# Migrate a running stock Raspberry Pi OS install to OpenMower OS, in place,
# over the network -- no SD card removal.
#
# This file is the HEADER of a self-extracting installer: `make image-rescue`
# (see external/board/openmower-cm4-rescue/post-image.sh) appends a tar
# payload (kernel, dtbs, rescue initramfs, and the boot/rootfs images) after
# the marker line at the bottom, producing a single runnable
# openmower-migrate-<version>.sh that carries everything it needs -- no
# server, no separate downloads. Run *this* checked-in copy directly and it
# will just fail cleanly at the "no embedded payload found" check below.
#
# Mechanism: NOT kexec -- kexec_load is either outright disabled in stock
# Raspberry Pi OS kernels, or hangs on real hardware even when enabled
# ("CPUs are stuck in the kernel"): Raspberry Pi's boot firmware has no PSCI,
# so Linux has no way to park and re-wake the secondary cores across the
# jump. Long-standing, unresolved upstream. Instead this drops the rescue
# kernel/dtbs/initramfs/cmdline into a rescue/ subdirectory of the EXISTING
# stock boot partition (purely additive, nothing existing is touched) plus a
# tryboot.txt at its root (os_prefix=rescue/), then triggers Raspberry Pi
# firmware's own one-shot `tryboot` reboot -- a real cold reset, so no PSCI
# involved at all. Firmware loads tryboot.txt instead of config.txt for
# exactly one boot attempt; the rescue initramfs then repartitions this same
# disk into the openmower-cm4 A/B layout and writes the first slot. See
# external/board/openmower-cm4-rescue/rootfs-overlay/init for exactly what
# happens to the disk.
#
# Safety net, same one openmower-updater/rauc-mark-good rely on elsewhere in
# this project: the tryboot flag is one-shot and self-clears the moment it's
# consumed. If the rescue boot never gets far enough to trigger its own
# (ordinary, non-tryboot) reboot -- crash, hang, power loss -- any
# subsequent reset boots config.txt/the stock OS again, automatically. And
# per Raspberry Pi's own firmware behavior, if tryboot.txt/rescue/ turns out
# to be misconfigured (wrong filename, missing file), the os_prefix is
# silently dropped and firmware just boots normally -- not a hang.
#
# THIS IS DESTRUCTIVE AND IRREVERSIBLE: the whole disk is repartitioned and
# overwritten. Everything on the current Raspberry Pi OS install is gone
# once it commits (see "point of no return" below) -- the DISK itself isn't
# touched until then, and staging up to that point is fully reversible. The
# first confirmation prompt, earlier, does let the script stop the running
# openmower/docker stack and prune docker images (to free up space) before
# that -- reversible in spirit (nothing on disk is destroyed, services just
# restart on next boot) but real, so it's still gated behind its own yes.
#
# Usage:
#   sudo ./openmower-migrate-<version>.sh [--yes] [--dry-run]
#
#   --yes      skip the interactive confirmations (fleet automation)
#   --dry-run  verify + extract the payload, stage user data + the rescue
#              boot files (all reversible, no stopping of the running
#              stack), stop before the reboot that commits to it
#
# Must be run as a real file (`sh $0` needs to read its own embedded payload
# off disk) -- `curl ... | sh` will not work, download it first.
set -eu

PAYLOAD_SHA256="@@PAYLOAD_SHA256@@"

log() { echo "migrate-to-openmower: $*"; }
die() { echo "migrate-to-openmower: ERROR: $*" >&2; exit 1; }

ASSUME_YES=0 DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --yes | -y) ASSUME_YES=1 ;;
        --dry-run) DRY_RUN=1 ;;
        *) die "unknown option: $arg (usage: $0 [--yes] [--dry-run])" ;;
    esac
done

[ "$(id -u)" = "0" ] || die "must run as root"

# --- Detect source OS: REFUSES to run without real evidence this is an ----
# --- OpenMower device, doesn't just warn ------------------------------------
# The board-model check below (Pi 4 / CM4) only tells us this is SOME
# Raspberry Pi -- plenty of those run stock Raspberry Pi OS as a homeserver,
# NAS, whatever, with nothing to do with OpenMower. That's not a safe
# enough signal to repartition on. Require actual evidence instead: either
# OpenMowerOS itself, or -- the real "stock Raspberry Pi OS" migration path
# this tool documents -- concrete OpenMower artifacts on a stock install
# (the compose stack, the openmower home dir, or the CLI). A bare stock Pi
# with none of that is refused outright, not warned-and-continued-past.
if [ -f /usr/share/openmoweros/version.txt ]; then
    log "source OS looks like OpenMowerOS: $(head -n1 /usr/share/openmoweros/version.txt 2>/dev/null)"
elif [ -f /etc/rpi-issue ] && grep -qi openmower /etc/rpi-issue 2>/dev/null; then
    log "source OS looks like OpenMowerOS: $(head -n1 /etc/rpi-issue 2>/dev/null)"
elif [ -f /opt/stacks/openmower/compose.yaml ] || [ -d /home/openmower ] || command -v openmower >/dev/null 2>&1; then
    log "no OpenMowerOS version marker, but found an OpenMower stack on this system (stock Raspberry Pi OS) -- proceeding"
else
    die "this doesn't look like an OpenMower device -- no OpenMowerOS version marker (/usr/share/openmoweros/version.txt, /etc/rpi-issue) and no OpenMower stack found (/opt/stacks/openmower/compose.yaml, /home/openmower, openmower command). Refusing to repartition a system with no evidence it's actually a mower."
fi

SELF="$(readlink -f "$0" 2>/dev/null || true)"
[ -n "$SELF" ] && [ -f "$SELF" ] || die "cannot resolve my own path as a regular file -- save this script to disk and run that copy, don't pipe it into sh"

# --- Identify the disk/partition we're running from -----------------------
ROOT_SRC="$(findmnt -n -o SOURCE /)" || die "cannot determine root device"
ROOT_FSTYPE="$(findmnt -n -o FSTYPE /)" || die "cannot determine root fstype"
case "$ROOT_SRC" in
    /dev/mmcblk*p[0-9]*) DISK="${ROOT_SRC%p[0-9]*}" ;;
    *) die "unsupported root device '$ROOT_SRC' -- this tool only supports SD/eMMC (/dev/mmcblkN), matching the openmower-cm4 target hardware" ;;
esac
log "root=$ROOT_SRC ($ROOT_FSTYPE) on disk=$DISK"

MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
case "$MODEL" in
    *"Compute Module 4"*) DTB="bcm2711-rpi-cm4.dtb" ;;
    *"Raspberry Pi 4"*) DTB="bcm2711-rpi-4-b.dtb" ;;
    *) die "unrecognized board '$MODEL' -- this tool only supports Pi 4 / CM4 (openmower-cm4)" ;;
esac
log "board: $MODEL -> $DTB"

# --- Data-loss + experimental-migration warning -----------------------------
# Deliberately BEFORE anything below touches the running system (stopping
# services, freeing space) -- if you're not ready, nothing has happened yet.
cat << WARN

##############################################################################
# BACK UP YOUR MAP AND SETTINGS NOW if you haven't already. This tool makes
# a best-effort attempt to carry over /home/openmower and your stack's
# .env (see below) -- anything it misses, or that only ever existed
# somewhere else (e.g. only in the app, or on a different device), is gone
# once you continue. There is no automatic backup step.
#
# This migration is EXPERIMENTAL and may fail partway. If it does, the
# fallback is physical access to the mower: pull the SD card (or put the
# CM4 IO board into rpiboot mode) and reflash it directly. There is no
# remote recovery path from a failed migration.
##############################################################################

WARN

if [ "$ASSUME_YES" != "1" ]; then
    printf 'Backed up your map/settings and understand this is experimental? Type "yes" to continue: '
    read -r ans
    [ "$ans" = "yes" ] || die "aborted -- nothing has been touched yet"
fi

# --- Stop the running stack, reclaim disk space -----------------------------
# Best-effort, non-fatal throughout: the migration should proceed even if a
# given stack was never running, docker isn't installed at all (stock
# Raspberry Pi OS), or a step here fails oddly -- none of this is required
# for the migration itself. It just frees up space (the preflight check
# below needs it) and avoids leaving containers running against a
# filesystem that's about to be repartitioned out from under them.
#
# Skipped under --dry-run: unlike staging (pure read + copy), this actually
# stops the running mower and deletes docker images -- a real, disruptive
# side effect that doesn't belong in a "verify without touching anything
# that matters" run.
if [ "$DRY_RUN" = "1" ]; then
    log "--dry-run: skipping openmower/docker stop+prune (would stop the running stack)"
else
    if command -v openmower >/dev/null 2>&1; then
        log "openmower CLI found, stopping the ROS stack..."
        openmower stop || log "WARNING: 'openmower stop' failed, continuing anyway"
    fi

    if command -v docker >/dev/null 2>&1; then
        for compose_file in /opt/stacks/openmower/compose.yaml /opt/stacks/webterminal/compose.yaml /opt/dockge/compose.yaml; do
            if [ -f "$compose_file" ]; then
                log "stopping stack: $compose_file"
                docker compose -f "$compose_file" down || log "WARNING: 'docker compose -f $compose_file down' failed, continuing anyway"
            fi
        done
        log "removing unused docker images to free up disk space..."
        docker image prune -a -f >/dev/null || log "WARNING: 'docker image prune' failed, continuing anyway"
    fi
fi

# --- Preflight checks ------------------------------------------------------
AVAILKB="$(df -Pk / | awk 'NR==2{print $4}')"
[ "$AVAILKB" -ge 1572864 ] || die "need >=1.5GiB free on / to extract the embedded payload (have $((AVAILKB / 1024))MiB)"

# --- Locate the boot (firmware) partition ----------------------------------
# Bookworm-and-later Raspberry Pi OS mounts it at /boot/firmware; older
# releases mount it directly at /boot. Ask the kernel instead of guessing by
# path convention -- it's whatever /dev/mmcblk0p1 (partition 1, same disk as
# root) is actually mounted on right now.
BOOT_SRC="${DISK}p1"
BOOT_MNT="$(findmnt -n -o TARGET "$BOOT_SRC" 2>/dev/null || true)"
[ -n "$BOOT_MNT" ] || die "cannot find where $BOOT_SRC (boot/firmware partition) is mounted"
[ -f "$BOOT_MNT/config.txt" ] || die "$BOOT_MNT doesn't look like the boot partition (no config.txt)"
log "boot partition: $BOOT_SRC on $BOOT_MNT"

BOOT_AVAILKB="$(df -Pk "$BOOT_MNT" | awk 'NR==2{print $4}')"
[ "$BOOT_AVAILKB" -ge 65536 ] || die "need >=64MiB free on $BOOT_MNT for the rescue kernel/dtbs/initramfs (have $((BOOT_AVAILKB / 1024))MiB)"

# --- Locate + verify + extract the embedded payload ------------------------
# The marker string must appear as a CONTIGUOUS literal exactly once in this
# header -- the real one, alone on the last line -- since it's matched with
# -m1 (first hit) against the script's own bytes. Built from two halves at
# runtime so the search pattern itself never spells out the full string
# in the source (that would be a second contiguous occurrence, earlier in
# the file than the real one, and grep would find that instead).
MARKER="#__OPENMOWER_MIGRATE""_PAYLOAD__"
PAYLOAD_LINE="$(grep -aFn -m1 "$MARKER" "$SELF" | cut -d: -f1)"
[ -n "$PAYLOAD_LINE" ] || die "no embedded payload found in $SELF -- this is the bare template, not a build output (run 'make image-rescue' to produce openmower-migrate-<version>.sh)"

# Deliberately NOT a literal match against the "@@PAYLOAD_SHA256@@"
# placeholder text: post-image.sh's sed substitutes that placeholder
# wherever it appears, and an exact-string check here would itself contain
# the placeholder as a second occurrence, which sed would then *also*
# substitute -- making this check compare the real hash against itself
# and always pass, silently disabling it (this shipped broken once
# already: every build died claiming an unsubstituted placeholder because
# the placeholder in THIS check got replaced too). Check the shape instead.
case "$PAYLOAD_SHA256" in
    *[!0-9a-f]* | "") die "payload checksum missing or malformed ('$PAYLOAD_SHA256') -- this is the bare template, not a build output" ;;
esac
[ ${#PAYLOAD_SHA256} -eq 64 ] || die "payload checksum has the wrong length ('$PAYLOAD_SHA256') -- this is the bare template, not a build output"

log "verifying embedded payload checksum..."
ACTUAL_SHA256="$(tail -n +$((PAYLOAD_LINE + 1)) "$SELF" | sha256sum | cut -d' ' -f1)"
[ "$ACTUAL_SHA256" = "$PAYLOAD_SHA256" ] || die "payload checksum mismatch (got $ACTUAL_SHA256, expected $PAYLOAD_SHA256) -- this file is corrupted or was truncated in transfer, re-copy it"

STAGE_ROOT=/var/lib/openmower-migrate
log "extracting payload to $STAGE_ROOT..."
rm -rf "$STAGE_ROOT"
mkdir -p "$STAGE_ROOT"
tail -n +$((PAYLOAD_LINE + 1)) "$SELF" | tar -x -C "$STAGE_ROOT" || die "payload extraction failed"

for f in Image "$DTB" rootfs.cpio.gz config.vfat boot-a.vfat boot-b.vfat rootfs.squashfs; do
    [ -f "$STAGE_ROOT/$f" ] || die "extracted payload is missing $f"
done
log "payload extracted and verified."

# --- Best-effort: stage user data next to the new OS image ------------------
# Copied into $STAGE_ROOT -- same filesystem as the payload just extracted
# above (i.e. this running system's own root disk, plenty of room), NOT the
# small FAT32 boot partition -- so the rescue initramfs can find it via the
# same /mnt/old/$STAGE mount it already uses for config.vfat etc., and copy
# it onto the freshly-formatted data partition before the final reboot (see
# external/board/openmower-cm4-rescue/rootfs-overlay/init).
#
# Best-effort and non-fatal throughout: this is a bonus, not the point of
# the migration -- a missing/failed copy here must never abort the OS
# migration itself. Whatever isn't picked up here, back it up yourself (see
# the warning above).
MIGRATE_DIR="$STAGE_ROOT/migrated-data"
mkdir -p "$MIGRATE_DIR"

if [ -d /home/openmower ]; then
    log "staging /home/openmower (params/maps/recordings/...) for migration..."
    # This installer is commonly downloaded/run from inside /home/openmower
    # itself -- and since it carries its own embedded tar payload
    # (kernel/dtbs/rescue initramfs/boot+rootfs images, see the header
    # comment), it's easily 1GB+. A plain `cp -a` would sweep it up along
    # with everything else and dwarf the actual data being migrated, so
    # exclude it (and any other openmower-migrate-*.sh -- an older download
    # left sitting there, say) at copy time via tar rather than copying it
    # and cleaning up after. POSIX sh has no pipefail, so this goes through
    # an intermediate tar file to keep each step's exit status checkable,
    # instead of a `tar ... | tar ...` pipe that would silently ignore the
    # first tar failing.
    mkdir -p "$MIGRATE_DIR/home-openmower"
    HOME_OPENMOWER_TAR="$STAGE_ROOT/home-openmower.tar"
    if tar --exclude="$(basename "$SELF")" --exclude='openmower-migrate-*.sh' \
           -C /home/openmower -cf "$HOME_OPENMOWER_TAR" . \
       && tar -C "$MIGRATE_DIR/home-openmower" -xf "$HOME_OPENMOWER_TAR"; then
        rm -f "$HOME_OPENMOWER_TAR"
        log "staged $(du -sh "$MIGRATE_DIR/home-openmower" 2>/dev/null | cut -f1) of data"
    else
        log "WARNING: failed to stage /home/openmower -- back it up yourself before continuing"
        rm -rf "$MIGRATE_DIR/home-openmower" "$HOME_OPENMOWER_TAR"
    fi
else
    log "no /home/openmower found, nothing to stage there"
fi

if [ -f /opt/stacks/openmower/.env ]; then
    log "staging /opt/stacks/openmower/.env (HARDWARE_PLATFORM/MOWER/... settings)..."
    cp -a /opt/stacks/openmower/.env "$MIGRATE_DIR/openmower.env" \
        || log "WARNING: failed to stage /opt/stacks/openmower/.env"
fi

# --- Stage the rescue boot files onto the EXISTING boot partition ----------
# Purely additive: a new rescue/ subdirectory plus one new top-level file
# (tryboot.txt). Nothing already on the boot partition is read, modified, or
# removed, so this step alone is fully and trivially reversible (delete
# those two paths) regardless of what happens next.
#
# Serial-only console, deliberately (no console=tty1): a USB keyboard is
# not usable here regardless. This board's onboard USB3 host controller is
# a Renesas xHCI chip (drivers/usb/host/xhci-pci-renesas.c) that needs a
# firmware blob (renesas_usb_fw.mem via request_firmware) neither this
# project's rootfs nor rpi-firmware ships anywhere -- confirmed missing
# even from the full prod build, not something the rescue image regressed.
# The driver is also only built as a module (CONFIG_USB_XHCI_PCI_RENESAS=m)
# and this initramfs has no modules at all. So HDMI+USB keyboard doesn't
# work in EITHER image; serial (GPIO14/15, 115200 8N1, this project's own
# documented debug method -- see README's "Flashing" section) is what
# actually lets you interact with the fallback shell if something goes
# wrong.
CMDLINE="console=serial0,115200 omrescue.disk=$DISK omrescue.src=$ROOT_SRC omrescue.srcfs=$ROOT_FSTYPE omrescue.stage=$STAGE_ROOT"
RESCUE_DIR="$BOOT_MNT/rescue"
log "writing rescue boot files to $RESCUE_DIR..."
rm -rf "$RESCUE_DIR"
mkdir -p "$RESCUE_DIR"
cp "$STAGE_ROOT/Image" "$STAGE_ROOT/bcm2711-rpi-cm4.dtb" "$STAGE_ROOT/bcm2711-rpi-4-b.dtb" "$STAGE_ROOT/rootfs.cpio.gz" "$RESCUE_DIR/"
printf '%s\n' "$CMDLINE" > "$RESCUE_DIR/cmdline.txt"
# Raspberry Pi firmware loads tryboot.txt instead of config.txt for exactly
# one boot, when rebooted with the "0 tryboot" one-shot flag (below) --
# os_prefix prefixes every OS file it loads (kernel, initramfs, dtb,
# cmdline.txt) with rescue/, so this is a complete, self-contained config
# for that one boot; it does NOT inherit anything from the real config.txt,
# including enable_uart=1 -- without it here too, the mini-UART (GPIO14/15,
# what console=serial0 above actually needs) never gets enabled/clocked by
# firmware and stays silent (confirmed against real hardware: prod's own
# config.txt has this exact line, tryboot.txt didn't, UART was dead).
# start_file/fixup_file/gpu_mem mirror prod's config.txt for the same
# "don't rely on firmware's own defaults" reason.
cat > "$BOOT_MNT/tryboot.txt" << EOF
start_file=start4.elf
fixup_file=fixup4.dat
kernel=Image
arm_64bit=1
gpu_mem=16
disable_splash=1
enable_uart=1
initramfs rootfs.cpio.gz followkernel
os_prefix=rescue/
EOF
sync
log "rescue boot files staged."

if [ "$DRY_RUN" = "1" ]; then
    log "--dry-run: stopping here, before the reboot. $BOOT_MNT/tryboot.txt and"
    log "$RESCUE_DIR are staged; re-run without --dry-run to reboot into them."
    exit 0
fi

# --- Point of no return -----------------------------------------------------
cat << WARN

##############################################################################
# This will REPARTITION AND OVERWRITE $DISK and replace this Raspberry Pi OS
# install with OpenMower OS. This CANNOT be undone. The device reboots into
# the new system when done.
##############################################################################

WARN

if [ "$ASSUME_YES" != "1" ]; then
    printf 'Type "yes" to continue: '
    read -r ans
    [ "$ans" = "yes" ] || die "aborted -- $RESCUE_DIR and $BOOT_MNT/tryboot.txt are staged but inert; delete them to fully undo"
fi

log "rebooting into the rescue system now (tryboot) -- this console goes quiet"
log "until it either comes back up as OpenMower OS, or (if tryboot isn't"
log "supported, or rescue/ wasn't found) boots this same Raspberry Pi OS again."
mkdir -p /run/systemd
printf '0 tryboot' > /run/systemd/reboot-param
systemctl reboot

# Everything from here on is unreachable when embedded in a real installer
# (systemctl reboot above tears down the running system) -- the marker + tar
# payload below is appended as raw bytes, never executed as shell.
exit 0
#__OPENMOWER_MIGRATE_PAYLOAD__
