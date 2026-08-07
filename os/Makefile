# Host-side convenience wrapper. Real work happens in the container (build.sh).

.PHONY: image image-dev image-rescue shell menuconfig menuconfig-dev \
	menuconfig-rescue linux-menuconfig \
	savedefconfig savedefconfig-dev savedefconfig-rescue clean distclean

image:
	./build.sh image

# Dev variant: gdbserver + host gdb + openssh, see external/configs/openmower_cm4_dev_defconfig.
# Shares the same output/ dir as prod; switch back with `make image`.
image-dev:
	DEFCONFIG=openmower_cm4_dev_defconfig ./build.sh image

# Rescue/migration installer, see external/configs/openmower_cm4_rescue_defconfig
# and scripts/migrate-to-openmower.sh. Needs a prior `make image` (pulls the
# kernel/dtbs/boot-*/rootfs images from output/images/); own output-rescue/
# dir otherwise. Result: output-rescue/images/openmower-migrate-<version>.sh
image-rescue:
	./build.sh image-rescue

shell:
	./build.sh shell

menuconfig:
	./build.sh menuconfig

menuconfig-dev:
	DEFCONFIG=openmower_cm4_dev_defconfig ./build.sh menuconfig

menuconfig-rescue:
	./build.sh menuconfig-rescue

linux-menuconfig:
	./build.sh linux-menuconfig

# Write the current .config back to external/configs/openmower_cm4_defconfig
savedefconfig:
	./build.sh savedefconfig BR2_DEFCONFIG=/work/os/external/configs/openmower_cm4_defconfig

# Write the current .config back to external/configs/openmower_cm4_dev_defconfig
savedefconfig-dev:
	./build.sh savedefconfig BR2_DEFCONFIG=/work/os/external/configs/openmower_cm4_dev_defconfig

# Write the current .config back to external/configs/openmower_cm4_rescue_defconfig
savedefconfig-rescue:
	./build.sh savedefconfig-rescue

clean:
	./build.sh clean

distclean:
	rm -rf output output-rescue
