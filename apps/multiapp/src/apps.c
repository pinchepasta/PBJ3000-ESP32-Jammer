/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Runs one app per boot as an llext extension (doc/multiapp.md): loads it
 * into PSRAM, runs its constructors (include/multiapp/ext.h) and starts its
 * main() in a thread. Starting another app reboots into it.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/app_version.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/llext/buf_loader.h>
#include <zephyr/llext/llext.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/usb/usbd.h>

#include <esp_attr.h>
#include <esp_rom_sys.h>
#include <rom/cache.h>

#include <esp_sdr/esp_sdr.h>
#if defined(CONFIG_ESP_SDR_IEEE802154)
#include <esp_sdr/ieee154_esp_sdr.h>
#endif

#include <multiapp/multiapp.h>

#include "multiapp_build_id.h"

#include "app_crash.h"
#include "app_usb.h"
#include "apps.h"

LOG_MODULE_REGISTER(multiapp, LOG_LEVEL_INF);

/* One MMU table maps both buses: PSRAM at 0x3c000000 + x executes at 0x42000000 + x. */
#define IBUS_DBUS_OFFSET 0x06000000

#define ROOTS_MAX       4
#define REQUEST_MAGIC   0x6d617070U /* "mapp" */
#define TRIED_MAGIC     0x74726964U /* "trid": the boot that tried it has started */
#define REBOOT_DELAY_MS 100

static EXT_RAM_BSS_ATTR uint8_t instr_heap[KB(CONFIG_MULTIAPP_INSTR_HEAP_KB)] __aligned(64);
static EXT_RAM_BSS_ATTR uint8_t data_heap[KB(CONFIG_MULTIAPP_DATA_HEAP_KB)] __aligned(64);

/* PSRAM like the app's data: internal RAM is kept for the dump banks. */
static Z_KERNEL_STACK_DEFINE_IN(app_stack, CONFIG_MULTIAPP_STACK_SIZE, EXT_RAM_BSS_ATTR);
static struct k_thread app_thread;

static EXT_RAM_BSS_ATTR struct llext_buf_loader loader;
static struct llext *loaded;
static const struct multiapp_info *loaded_info;
static const char *loaded_source; /* "file" or "built-in" */

/* What this host's apps carry in their trailers (buildid.py). */
static const uint8_t build_id[] = MULTIAPP_BUILD_ID;

/* The app's file, read whole before llext sees it (drive.c). */
static EXT_RAM_BSS_ATTR uint8_t file_buf[KB(CONFIG_MULTIAPP_FILE_MAX_KB)] __aligned(4);
static const struct shell_static_entry *roots[ROOTS_MAX];
static size_t root_count;

/* The app for the next boot and how far its boot got; survives a software reset. */
static RTC_NOINIT_ATTR struct {
	uint32_t magic;
	char name[24];
	uint32_t step;
} request;

enum boot_step {
	STEP_REQUESTED,
	STEP_LOADING,
	STEP_CONSTRUCTORS,
	STEP_MAIN,
	STEP_RUNNING,
};

static const char *const step_names[] = {"requested", "loading", "constructors", "main",
					 "running"};

/* What the previous boot left: the request it tried and the step it reached. */
static char tried_name[sizeof(request.name)];
static uint32_t tried_step;
static bool tried;

/* This boot's app, from a request or CONFIG_MULTIAPP_AUTOSTART, and its USB profile. */
static char boot_name[sizeof(request.name)];
static uint32_t usb_profile;
static int boot_ret;

/*
 * Internal RAM for one user per boot: the boot's app if it has buffers there
 * (ESP_SDR_HIGH_BUF_DEFINE(), exports.py), else the 802.15.4 driver.
 */
uint8_t multiapp_area[CONFIG_MULTIAPP_AREA_SIZE] __aligned(16) ESP_SDR_HIGH_RAM;
static const struct multiapp_embedded *area_app;

#if defined(CONFIG_ESP_SDR_IEEE802154_HIGH_RAM_APP)
void *ieee154_esp_sdr_high_ram(size_t size)
{
	if (area_app != NULL || size > sizeof(multiapp_area)) {
		return NULL;
	}
	return multiapp_area;
}
#endif

/* Before the devices: the area's user and the USB profile follow the boot's app. */
static int boot_pick(void)
{
	const struct multiapp_embedded *app;

	if (request.magic == TRIED_MAGIC) {
		/* The boot that tried an app ended before "app run" or a reset: report it. */
		tried = true;
		tried_step = request.step;
		memcpy(tried_name, request.name, sizeof(tried_name));
		tried_name[sizeof(tried_name) - 1] = '\0';
		request.magic = 0;
	}
	if (request.magic == REQUEST_MAGIC) {
		request.magic = TRIED_MAGIC;
		request.step = STEP_REQUESTED;
		memcpy(boot_name, request.name, sizeof(boot_name));
		boot_name[sizeof(boot_name) - 1] = '\0';
	} else if (!(tried && tried_step < STEP_RUNNING &&
		     strcmp(tried_name, CONFIG_MULTIAPP_AUTOSTART) == 0)) {
		/* Tried like a request, so one that never gets running is skipped once. */
		strncpy(boot_name, CONFIG_MULTIAPP_AUTOSTART, sizeof(boot_name) - 1);
		if (boot_name[0] != '\0') {
			request.magic = TRIED_MAGIC;
			request.step = STEP_REQUESTED;
			memcpy(request.name, boot_name, sizeof(request.name));
		}
	}
	app = multiapp_embedded_find(boot_name);
	usb_profile = app != NULL ? app->usb : MULTIAPP_USB_MSC;
	area_app = app != NULL && app->area ? app : NULL;
	return 0;
}
SYS_INIT(boot_pick, PRE_KERNEL_1, 0);

static int heaps_init(void)
{
	return llext_heap_init_harvard_exec(instr_heap, sizeof(instr_heap), data_heap,
					    sizeof(data_heap), IBUS_DBUS_OFFSET);
}
SYS_INIT(heaps_init, APPLICATION, 0);

/* Only the boot's app's classes: the OTG controller has too few IN endpoints for all. */
void app_usb_classes(struct usbd_context *ctx)
{
	if (IS_ENABLED(CONFIG_USBD_CDC_NCM_CLASS) && (usb_profile & MULTIAPP_USB_NCM) == 0U) {
		(void)usbd_unregister_class(ctx, "cdc_ncm_0", USBD_SPEED_FS, 1);
	}
	if ((usb_profile & MULTIAPP_USB_VENDOR) == 0U) {
		(void)usbd_unregister_class(ctx, "esp_sdr_usb", USBD_SPEED_FS, 1);
	}
	if (IS_ENABLED(CONFIG_USBD_MSC_CLASS) && (usb_profile & MULTIAPP_USB_MSC) == 0U) {
		(void)usbd_unregister_class(ctx, "msc_0", USBD_SPEED_FS, 1);
	}
}

void multiapp_shell_register(const struct shell_static_entry *root)
{
	if (root_count == ARRAY_SIZE(roots)) {
		LOG_ERR("no room for root command %s", root->syntax);
		return;
	}
	roots[root_count++] = root;
}

static const struct shell_static_entry *root_find(const char *name)
{
	for (size_t i = 0; i < root_count; i++) {
		if (strcmp(roots[i]->syntax, name) == 0) {
			return roots[i];
		}
	}
	return NULL;
}

const struct multiapp_embedded *multiapp_embedded_find(const char *name)
{
	for (size_t i = 0; i < multiapp_embedded_count; i++) {
		if (strcmp(multiapp_embedded[i].name, name) == 0) {
			return &multiapp_embedded[i];
		}
	}
	return NULL;
}

/* llext wrote the code through the data bus, and the ESP cache driver has no icache ops. */
static void sync_text(const struct llext *ext)
{
	uintptr_t text = (uintptr_t)ext->mem[LLEXT_MEM_TEXT];
	size_t size = ext->mem_size[LLEXT_MEM_TEXT];

	Cache_WriteBack_Addr(text, size);
	Cache_Invalidate_Addr(text + IBUS_DBUS_OFFSET, size);
}

/* Record how far this boot's app got, until the next "app run" or reset. */
static void boot_step(enum boot_step step)
{
	if (request.magic == TRIED_MAGIC) {
		request.step = step;
	}
}

static void app_main(void *p1, void *p2, void *p3)
{
	int (*entry)(void) = p1;
	int ret;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	ret = entry();
	boot_step(STEP_RUNNING);
	LOG_INF("%s: main returned %d", loaded_info->name, ret);
}

/* The ELF size and stack of an app file whose trailer holds (apptrail.py); else -errno. */
static int check_trailer(const char *name, const char *source, const uint8_t *data, size_t len,
			 size_t *elf_size, uint32_t *stack_need)
{
	struct multiapp_trailer t;

	if (len < sizeof(t)) {
		LOG_ERR("%s (%s): %zu bytes, no trailer", name, source, len);
		return -ENOEXEC;
	}
	memcpy(&t, data + len - sizeof(t), sizeof(t));
	if (t.magic != MULTIAPP_TRAILER_MAGIC || t.elf_size != len - sizeof(t)) {
		LOG_ERR("%s (%s): no trailer for %zu bytes", name, source, len);
		return -ENOEXEC;
	}
	if (crc32_ieee(data, t.elf_size) != t.crc32) {
		LOG_ERR("%s (%s): CRC mismatch", name, source);
		return -EBADMSG;
	}
	BUILD_ASSERT(sizeof(build_id) == sizeof(t.build_id));
	if (memcmp(t.build_id, build_id, sizeof(build_id)) != 0) {
		LOG_ERR("%s (%s): built for host %02x%02x%02x%02x, this is %02x%02x%02x%02x", name,
			source, t.build_id[0], t.build_id[1], t.build_id[2], t.build_id[3],
			build_id[0], build_id[1], build_id[2], build_id[3]);
		return -ENOEXEC;
	}
	*elf_size = t.elf_size;
	*stack_need = t.stack_size;
	return 0;
}

int multiapp_start(const char *name)
{
	struct llext_load_param param = LLEXT_LOAD_PARAM_DEFAULT;
	const struct multiapp_embedded *app = multiapp_embedded_find(name);
	const struct multiapp_info *info;
	const uint8_t *data;
	int (*entry)(void);
	size_t stack, elf_size;
	uint32_t stack_need;
	ssize_t n;
	int ret;

	if (app == NULL) {
		return -ENOENT;
	}
	if (loaded != NULL) {
		return -EBUSY;
	}
	boot_step(STEP_LOADING);
	/* Its file if the volume has a good one, else the built-in copy. */
	n = multiapp_drive_status() == 0 ? multiapp_drive_read(name, file_buf, sizeof(file_buf))
					 : multiapp_drive_status();
	if (n >= 0 && check_trailer(name, "file", file_buf, n, &elf_size, &stack_need) == 0) {
		data = file_buf;
		loaded_source = "file";
	} else if (app->elf != NULL &&
		   check_trailer(name, "built-in", app->elf, app->size, &elf_size, &stack_need) == 0) {
		if (n != -ENOENT && n != -ENODEV) {
			LOG_WRN("%s: the file is not usable (%d), the built-in copy runs", name,
				(int)n);
		}
		data = app->elf;
		loaded_source = "built-in";
	} else {
		LOG_ERR("%s: no usable file (%d)%s", name, (int)n,
			app->elf != NULL ? " or built-in copy" : "");
		return n < 0 ? (int)n : -ENOEXEC;
	}
	stack = K_THREAD_STACK_SIZEOF(app_stack);
	if (stack_need > stack) {
		LOG_ERR("%s: needs a %u byte stack, the host has %zu (CONFIG_MULTIAPP_STACK_SIZE)",
			name, stack_need, stack);
		return -ENOMEM;
	}
	/*
	 * Temporary, so llext copies every region: in place, it would not relocate
	 * .got.loc, as .rela.dyn names no target section.
	 */
	loader = (struct llext_buf_loader)LLEXT_TEMPORARY_BUF_LOADER(data, elf_size);
	ret = llext_load(&loader.loader, app->name, &loaded, &param);
	if (ret != 0) {
		LOG_ERR("%s: load failed (%d)", name, ret);
		loaded = NULL;
		return ret;
	}
	sync_text(loaded);
	info = llext_find_sym(&loaded->exp_tab, "multiapp_info");
	if (info == NULL || info->abi != MULTIAPP_ABI) {
		LOG_ERR("%s: built for ABI %d, the host has %u", name,
			info != NULL ? (int)info->abi : -1, MULTIAPP_ABI);
		ret = -ENOEXEC;
		goto unload;
	}
	entry = info->main;
	if (entry == NULL) {
		LOG_ERR("%s: no main()", name);
		ret = -ENOEXEC;
		goto unload;
	}
	loaded_info = info;
	boot_step(STEP_CONSTRUCTORS);
	ret = llext_bringup(loaded);
	if (ret != 0) {
		LOG_ERR("%s: constructors failed (%d)", name, ret);
		goto unload;
	}
	LOG_INF("%s (%s): %zu bytes of code at %p, starting main()", name, loaded_source,
		loaded->mem_size[LLEXT_MEM_TEXT],
		(void *)((uintptr_t)loaded->mem[LLEXT_MEM_TEXT] + IBUS_DBUS_OFFSET));
	boot_step(STEP_MAIN);
	k_thread_create(&app_thread, app_stack, stack, app_main, entry, NULL, NULL,
			CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&app_thread, info->name);
	return 0;

unload:
	/* Nothing of it has run yet, so it can go. */
	loaded_info = NULL;
	root_count = 0;
	llext_unload(&loaded);
	loaded = NULL;
	return ret;
}

#if defined(CONFIG_APP_USB_DFU)
/*
 * Before a DFU download writes flash: the loaded app's own hook, by name
 * (wpan stops its radio, which keeps a CPU from taking flash stall requests).
 */
void app_usb_dfu_prepare(void)
{
	void (*prepare)(void);

	if (loaded == NULL) {
		return;
	}
	prepare = llext_find_sym(&loaded->exp_tab, "app_usb_dfu_prepare");
	if (prepare != NULL) {
		prepare();
	}
}
#endif

static void reboot_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	sys_reboot(SYS_REBOOT_WARM);
}
static K_WORK_DELAYABLE_DEFINE(reboot_work, reboot_handler);

int multiapp_request(const char *name)
{
	const struct multiapp_embedded *app = multiapp_embedded_find(name);

	if (app == NULL) {
		return -ENOENT;
	}
	if (loaded == NULL && app->usb == usb_profile && (!app->area || area_app == app)) {
		return multiapp_start(name);
	}
	/* An app cannot stop, USB enumerates once and the area has one user per boot. */
	request.magic = REQUEST_MAGIC;
	strncpy(request.name, name, sizeof(request.name) - 1);
	request.name[sizeof(request.name) - 1] = '\0';
	k_work_schedule(&reboot_work, K_MSEC(REBOOT_DELAY_MS));
	return 1;
}

bool multiapp_boot(void)
{
	int ret;

	if (boot_name[0] == '\0') {
		return false;
	}
	ret = multiapp_start(boot_name);
	if (ret != 0) {
		LOG_ERR("%s: not started (%d)", boot_name, ret);
		/* USB has not started: it comes up without the app's classes, with the drive. */
		usb_profile = MULTIAPP_USB_MSC;
	}
	/* Before the shell is open: "app info" tells. */
	boot_ret = ret;
	return ret == 0;
}

void *multiapp_app_fn(const char *name)
{
	return loaded != NULL ? (void *)llext_find_sym(&loaded->exp_tab, name) : NULL;
}

/* The host main()'s stack use, measured as it returns. */
static size_t main_used, main_size;

void multiapp_note_main_stack(void)
{
#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_THREAD_STACK_INFO)
	size_t unused;

	if (k_thread_stack_space_get(k_current_get(), &unused) == 0) {
		main_size = k_current_get()->stack_info.size;
		main_used = main_size - unused;
	}
#endif
}

/* An app's app_usb_init() (include/multiapp/ext.h): the host's main() starts USB. */
int multiapp_ext_usb_init(void)
{
	return 0;
}

/*
 * Proxy roots: the apps' root commands by name, so the shell looks the same
 * as in the standalone app. The subcommands come from the loaded app's table.
 */
static void proxy_get(const char *name, size_t idx, struct shell_static_entry *entry)
{
	const struct shell_static_entry *root = root_find(name);
	const struct shell_static_entry *sub;

	entry->syntax = NULL;
	if (root == NULL || root->subcmd == NULL) {
		return;
	}
	sub = root->subcmd->entry;
	for (size_t i = 0; i <= idx; i++) {
		if (sub[i].syntax == NULL) {
			return;
		}
	}
	*entry = sub[idx];
}

static int proxy_cmd(const char *name, const struct shell *sh, size_t argc, char **argv)
{
	const struct shell_static_entry *root = root_find(name);

	if (root == NULL) {
		shell_error(sh, "%s: no loaded app has this command (app list)", name);
		return -ENOEXEC;
	}
	if (root->handler == NULL) {
		if (argc > 1) {
			shell_error(sh, "%s: unknown parameter: %s", name, argv[1]);
		}
		shell_help(sh);
		return argc > 1 ? -EINVAL : SHELL_CMD_HELP_PRINTED;
	}
	/* As the shell checks a static root: mandatory counts the command, 0 checks nothing. */
	if (root->args.mandatory != 0 &&
	    (argc < root->args.mandatory ||
	     (root->args.optional != SHELL_OPT_ARG_CHECK_SKIP &&
	      argc > (size_t)root->args.mandatory + root->args.optional))) {
		shell_error(sh, "%s: wrong parameter count", name);
		return -EINVAL;
	}
	return root->handler(sh, argc, argv);
}

#define PROXY(_name)                                                                               \
	static void UTIL_CAT(proxy_get_, _name)(size_t idx, struct shell_static_entry *entry)      \
	{                                                                                          \
		proxy_get(STRINGIFY(_name), idx, entry);                                           \
	}                                                                                          \
	SHELL_DYNAMIC_CMD_CREATE(UTIL_CAT(proxy_sub_, _name), UTIL_CAT(proxy_get_, _name));        \
	static int UTIL_CAT(proxy_cmd_, _name)(const struct shell *sh, size_t argc, char **argv)  \
	{                                                                                          \
		return proxy_cmd(STRINGIFY(_name), sh, argc, argv);                                \
	}                                                                                          \
	SHELL_CMD_ARG_REGISTER(_name, &UTIL_CAT(proxy_sub_, _name),                                \
			       "Command of the loaded app (app list)", UTIL_CAT(proxy_cmd_, _name), \
			       1, SHELL_OPT_ARG_CHECK_SKIP)

/* Every root command the apps register: hello, rfprobe, wpan, osmosdr and stream, link. */
PROXY(hello);
PROXY(probe);
PROXY(wpan);
PROXY(sdr);
PROXY(link);
PROXY(ofdm);

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (size_t i = 0; i < multiapp_embedded_count; i++) {
		const struct multiapp_embedded *app = &multiapp_embedded[i];
		char built_in[24] = "     -";
		char file[40] = "";
		ssize_t size;
		bool same;

		if (multiapp_drive_file(app->name, &size, &same) == 0) {
			if (size < 0) {
				(void)snprintf(file, sizeof(file), ", no file (%d)", (int)size);
			} else {
				(void)snprintf(file, sizeof(file), ", file %zd bytes%s", size,
					       same || app->elf == NULL ? "" : " (differs)");
			}
		}
		if (app->elf != NULL) {
			(void)snprintf(built_in, sizeof(built_in), "%6zu bytes", app->size);
		}
		shell_print(sh, "%-12s %s%s%s", app->name, built_in, file,
			    loaded_info != NULL && strcmp(loaded_info->name, app->name) == 0
				    ? ", running"
				    : "");
	}
	if (multiapp_drive_status() != 0) {
		shell_print(sh, "no FAT volume (%d)", multiapp_drive_status());
	}
	return 0;
}

static int cmd_run(const struct shell *sh, size_t argc, char **argv)
{
	int ret = multiapp_request(argv[1]);

	ARG_UNUSED(argc);
	if (ret == -ENOENT) {
		shell_error(sh, "%s: no such app (app list)", argv[1]);
	} else if (ret == 1) {
		shell_print(sh, "%s: rebooting into it%s", argv[1],
			    (usb_profile & MULTIAPP_USB_MSC) != 0U
				    ? " (eject the drive first: the PC may not have written it out)"
				    : "");
		ret = 0;
	} else if (ret != 0) {
		shell_error(sh, "%s: failed (%d)", argv[1], ret);
	} else {
		shell_print(sh, "%s: started", argv[1]);
	}
	return ret;
}

static int cmd_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "host %s, build %02x%02x%02x%02x", APP_VERSION_STRING, build_id[0],
		    build_id[1], build_id[2], build_id[3]);
	if (main_size != 0U) {
		shell_print(sh, "host main: %zu of %zu bytes of stack used", main_used, main_size);
	}
	if (boot_name[0] != '\0') {
		shell_print(sh, "boot: %s, start %d, USB profile 0x%x", boot_name, boot_ret,
			    usb_profile);
	}
	if (loaded == NULL) {
		shell_print(sh, "no app loaded");
		return 0;
	}
	shell_print(sh, "%s (%s): text %zu at %p, data %zu, bss %zu, rodata %zu, %zu bytes in heaps",
		    loaded_info->name, loaded_source, loaded->mem_size[LLEXT_MEM_TEXT],
		    (void *)((uintptr_t)loaded->mem[LLEXT_MEM_TEXT] + IBUS_DBUS_OFFSET),
		    loaded->mem_size[LLEXT_MEM_DATA], loaded->mem_size[LLEXT_MEM_BSS],
		    loaded->mem_size[LLEXT_MEM_RODATA], loaded->alloc_size);
	for (size_t i = 0; i < root_count; i++) {
		shell_print(sh, "  command %s", roots[i]->syntax);
	}
	return 0;
}

/* What ended the previous boot, if it ended badly (apps/common app_crash.h). */
static int cmd_crash(const struct shell *sh, size_t argc, char **argv)
{
	struct app_crash_dx dx;
	struct app_crash c;
	size_t used, size;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "reset reason cpu0 %d cpu1 %d (esp_rom_get_reset_reason())",
		    (int)esp_rom_get_reset_reason(0), (int)esp_rom_get_reset_reason(1));
	if (tried) {
		shell_print(sh, "the boot into %s ended at step %s", tried_name,
			    tried_step < ARRAY_SIZE(step_names) ? step_names[tried_step] : "?");
	}
	if (app_usb_dfu_stack(&used, &size)) {
		shell_print(sh, "last DFU download: %zu of %zu bytes of the USB thread's stack used",
			    used, size);
	}
	if (app_crash_dx_last(&dx)) {
		shell_print(sh, "double exception: depc 0x%08x exccause %u excvaddr 0x%08x"
			    " epc1 0x%08x ps 0x%08x a1 0x%08x prid 0x%x",
			    dx.depc, dx.exccause, dx.excvaddr, dx.epc1, dx.ps, dx.a1, dx.prid);
	}
	if (!app_crash_last(&c)) {
		shell_print(sh, "no fatal error or watchdog reset recorded");
		return 0;
	}
	if (c.reason == APP_CRASH_WATCHDOG) {
		shell_print(sh, "watchdog reset: thread %s cpu %u uptime %u ms", c.thread, c.cpu,
			    c.uptime_ms);
	} else {
		shell_print(sh, "fatal error %u: thread %s cpu %u uptime %u ms pc 0x%08x a0 0x%08x"
			    " ps 0x%08x exccause %u excvaddr 0x%08x",
			    c.reason, c.thread, c.cpu, c.uptime_ms, c.pc, c.a0, c.ps, c.exccause,
			    c.excvaddr);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(app_cmds,
	SHELL_CMD(list, NULL, "The apps in this image", cmd_list),
	SHELL_CMD_ARG(run, NULL, "<name>: start an app (reboots if one runs)", cmd_run, 2, 0),
	SHELL_CMD(info, NULL, "The loaded app", cmd_info),
	SHELL_CMD(crash, NULL, "What ended the previous boot", cmd_crash),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(app, &app_cmds, "Apps run as extensions", NULL);
