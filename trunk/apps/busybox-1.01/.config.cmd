deps_config := \
	util-linux/Config.in \
	sysklogd/Config.in \
	shell/Config.in \
	procps/Config.in \
	networking/udhcp/Config.in \
	networking/Config.in \
	modutils/Config.in \
	miscutils/Config.in \
	loginutils/Config.in \
	init/Config.in \
	findutils/Config.in \
	editors/Config.in \
	debianutils/Config.in \
	console-tools/Config.in \
	coreutils/Config.in \
	archival/Config.in \
	/home/fanjianqi/atheros_work/reference/trunk_11_6_22/apps/busybox-1.01/sysdeps/linux/Config.in

.config include/config.h: $(deps_config)

$(deps_config):
