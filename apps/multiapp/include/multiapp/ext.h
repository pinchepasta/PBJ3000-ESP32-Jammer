/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Put in front of every source of a multiapp extension, so the apps build
 * unchanged. An extension cannot add to the host's linker sections, so:
 *
 * - SHELL_CMD_REGISTER(): a constructor hands the root command to the
 *   host's proxy root of the same name (src/apps.c).
 * - SYS_INIT(): a constructor, run by llext_bringup() before main().
 * - LOG_MODULE_REGISTER(): a declaration; the host registers the module
 *   (exports.py).
 * - .noinit and the PSRAM attributes join .bss: llext loads one NOBITS
 *   section, and the data heap is PSRAM anyway.
 * - K_MEM_SLAB_DEFINE(): initialized by a constructor instead of at boot.
 * - NET_BUF_POOL_DEFINE(), ESP_SDR_HIGH_BUF_DEFINE() and
 *   ESP_SDR_HIGH_STACK_DEFINE(): declared under a host name and described
 *   in .multiapp_host, from which exports.py defines them in the host
 *   (net_buf finds a pool by its index in a host section; internal RAM
 *   comes from the host's area).
 * - app_usb_init() does nothing: the host starts USB, and the app's PSRAM
 *   stack must not run the watchdog's flash access.
 */

#ifndef MULTIAPP_EXT_H_
#define MULTIAPP_EXT_H_

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/shell/shell.h>
#include <zephyr/toolchain.h>

#include <esp_attr.h>

#include <multiapp/multiapp.h>

#undef SHELL_CMD_ARG_REGISTER
#define SHELL_CMD_ARG_REGISTER(syntax, subcmd, help, handler, mandatory, optional)                \
	static const struct shell_static_entry UTIL_CAT(_shell_, syntax) =                        \
		SHELL_CMD_ARG(syntax, subcmd, help, handler, mandatory, optional);                 \
	static void __attribute__((constructor)) UTIL_CAT(multiapp_shell_, syntax)(void)          \
	{                                                                                          \
		multiapp_shell_register(&UTIL_CAT(_shell_, syntax));                               \
	}

/* The shell tells these apart by their address in a host section. */
#undef SHELL_DYNAMIC_CMD_CREATE
#define SHELL_DYNAMIC_CMD_CREATE(name, get)                                                        \
	BUILD_ASSERT(0, "multiapp: no dynamic shell commands in an extension")
#undef SHELL_SUBCMD_SET_CREATE
#define SHELL_SUBCMD_SET_CREATE(_name, _parent)                                                    \
	BUILD_ASSERT(0, "multiapp: no section-based subcommand sets in an extension")

#undef SYS_INIT_NAMED
#define SYS_INIT_NAMED(name, init_fn, level, prio)                                                 \
	static void __attribute__((constructor)) UTIL_CAT(multiapp_init_, name)(void)             \
	{                                                                                          \
		(void)init_fn();                                                                   \
	}

#undef LOG_MODULE_REGISTER
#define LOG_MODULE_REGISTER(...) LOG_MODULE_DECLARE(__VA_ARGS__)

#define app_usb_init multiapp_ext_usb_init

/* The host's name for an object it defines for this extension. */
#define Z_MULTIAPP_HOST_NAME(_name) "multiapp_" STRINGIFY(MULTIAPP_NAME) "_" #_name

#define Z_MULTIAPP_HOST_DESC(_name, _kind, ...)                                                    \
	static const struct multiapp_host_object __used                                          \
		__attribute__((section(".multiapp_host"))) UTIL_CAT(multiapp_host_, _name) = {    \
		Z_MULTIAPP_HOST_NAME(_name), _kind, {__VA_ARGS__}}

#define ESP_SDR_HIGH_BUF_DEFINE(type, name, dims, align)                                          \
	extern type name dims __asm__(Z_MULTIAPP_HOST_NAME(name));                                \
	Z_MULTIAPP_HOST_DESC(name, MULTIAPP_HOST_AREA, sizeof(name), align)

#define ESP_SDR_HIGH_STACK_DEFINE(name, size)                                                     \
	extern k_thread_stack_t name[K_KERNEL_STACK_LEN(size)]                                    \
		__asm__(Z_MULTIAPP_HOST_NAME(name));                                               \
	Z_MULTIAPP_HOST_DESC(name, MULTIAPP_HOST_AREA, sizeof(name), Z_KERNEL_STACK_OBJ_ALIGN)

#undef NET_BUF_POOL_FIXED_DEFINE
#define NET_BUF_POOL_FIXED_DEFINE(_name, _count, _data_size, _ud_size, _destroy)                 \
	BUILD_ASSERT(__builtin_types_compatible_p(__typeof__(_destroy), void *),                 \
		     "multiapp: a host pool takes no destroy callback");                          \
	extern struct net_buf_pool _name __asm__(Z_MULTIAPP_HOST_NAME(_name));                    \
	Z_MULTIAPP_HOST_DESC(_name, MULTIAPP_HOST_POOL, _count, _data_size, _ud_size)

#define Z_MULTIAPP_SLAB(scope, name, size, num, align)                                         \
	BUILD_ASSERT(((size) % (align)) == 0, "slab_block_size must be a multiple of slab_align"); \
	BUILD_ASSERT((((align) & ((align) - 1)) == 0), "slab_align must be a power of 2");         \
	static char __aligned(WB_UP(align)) _k_mem_slab_buf_##name[(num) * WB_UP(size)];         \
	scope struct k_mem_slab name;                                                              \
	static void __attribute__((constructor)) UTIL_CAT(multiapp_slab_, name)(void)             \
	{                                                                                          \
		(void)k_mem_slab_init(&name, _k_mem_slab_buf_##name, WB_UP(size), num);            \
	}

#undef K_MEM_SLAB_DEFINE_IN_SECT
#define K_MEM_SLAB_DEFINE_IN_SECT(name, in_section, slab_block_size, slab_num_blocks, slab_align)  \
	Z_MULTIAPP_SLAB(, name, slab_block_size, slab_num_blocks, slab_align)
#undef K_MEM_SLAB_DEFINE_IN_SECT_STATIC
#define K_MEM_SLAB_DEFINE_IN_SECT_STATIC(name, in_section, slab_block_size, slab_num_blocks,       \
					 slab_align)                                               \
	Z_MULTIAPP_SLAB(static, name, slab_block_size, slab_num_blocks, slab_align)

#undef __noinit
#define __noinit _SECTION_ATTR_IMPL(".bss.noinit", __COUNTER__)
/* The extension's data heap is PSRAM: the high region is the host's (esp_sdr.h). */
#define ESP_SDR_HIGH_RAM _SECTION_ATTR_IMPL(".bss.esp_sdr_high", __COUNTER__)
#undef EXT_RAM_BSS_ATTR
#define EXT_RAM_BSS_ATTR _SECTION_ATTR_IMPL(".bss.ext_ram", __COUNTER__)
#undef EXT_RAM_NOINIT_ATTR
#define EXT_RAM_NOINIT_ATTR _SECTION_ATTR_IMPL(".bss.ext_ram_noinit", __COUNTER__)

#endif /* MULTIAPP_EXT_H_ */
