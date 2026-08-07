################################################################################
#
# openmower-updater
#
################################################################################

OPENMOWER_UPDATER_VERSION = 1.0
OPENMOWER_UPDATER_SITE = $(BR2_EXTERNAL_OPENMOWER_PATH)/package/openmower-updater/files
OPENMOWER_UPDATER_SITE_METHOD = local
OPENMOWER_UPDATER_LICENSE = Proprietary

define OPENMOWER_UPDATER_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/openmower-updater \
		$(TARGET_DIR)/usr/bin/openmower-updater
	$(INSTALL) -D -m 0644 $(@D)/openmower-updater.conf \
		$(TARGET_DIR)/etc/openmower-updater.conf
	$(INSTALL) -D -m 0644 $(@D)/openmower-updater.service \
		$(TARGET_DIR)/usr/lib/systemd/system/openmower-updater.service
	$(INSTALL) -D -m 0644 $(@D)/openmower-updater.timer \
		$(TARGET_DIR)/usr/lib/systemd/system/openmower-updater.timer
endef

$(eval $(generic-package))
