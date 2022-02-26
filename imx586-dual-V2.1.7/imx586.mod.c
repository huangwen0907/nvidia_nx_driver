#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xc2996440, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x2de3052, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x5069b1b7, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0xf95eb840, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0x6f657e8a, __VMLINUX_SYMBOL_STR(tegracam_v4l2subdev_register) },
	{ 0xb5aac65b, __VMLINUX_SYMBOL_STR(camera_common_mclk_disable) },
	{ 0xb08a7a58, __VMLINUX_SYMBOL_STR(camera_common_mclk_enable) },
	{ 0x40af5072, __VMLINUX_SYMBOL_STR(i2c_unregister_device) },
	{ 0x2049f30f, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0x1f6d53ea, __VMLINUX_SYMBOL_STR(__devm_regmap_init_i2c) },
	{ 0xf9661c03, __VMLINUX_SYMBOL_STR(i2c_new_device) },
	{ 0xeea0e2af, __VMLINUX_SYMBOL_STR(i2c_get_adapter) },
	{ 0xf6bce128, __VMLINUX_SYMBOL_STR(tegracam_set_privdata) },
	{ 0xe40b0345, __VMLINUX_SYMBOL_STR(tegracam_device_register) },
	{ 0x9166fada, __VMLINUX_SYMBOL_STR(strncpy) },
	{ 0xfbe87172, __VMLINUX_SYMBOL_STR(of_find_property) },
	{ 0xae71a4ac, __VMLINUX_SYMBOL_STR(tegracam_get_privdata) },
	{ 0xef6b4279, __VMLINUX_SYMBOL_STR(regmap_util_write_table_8) },
	{ 0xde863f76, __VMLINUX_SYMBOL_STR(devm_kfree) },
	{ 0xef70ba75, __VMLINUX_SYMBOL_STR(of_get_named_gpio_flags) },
	{ 0x805153f3, __VMLINUX_SYMBOL_STR(of_property_read_variable_u32_array) },
	{ 0x62194974, __VMLINUX_SYMBOL_STR(of_property_read_string) },
	{ 0xcd579665, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x41464f2a, __VMLINUX_SYMBOL_STR(of_match_device) },
	{ 0x6e1c56e8, __VMLINUX_SYMBOL_STR(gpiod_set_raw_value) },
	{ 0x99c25d19, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0x2396c7f0, __VMLINUX_SYMBOL_STR(clk_set_parent) },
	{ 0x89214349, __VMLINUX_SYMBOL_STR(devm_clk_get) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x86b0be11, __VMLINUX_SYMBOL_STR(camera_common_g_ctrl) },
	{ 0xbf7ca6fe, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xbc64fe4e, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xff886cea, __VMLINUX_SYMBOL_STR(regmap_write) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0xedfa018, __VMLINUX_SYMBOL_STR(tegracam_device_unregister) },
	{ 0xb799f8aa, __VMLINUX_SYMBOL_STR(tegracam_v4l2subdev_unregister) },
	{ 0x66687a94, __VMLINUX_SYMBOL_STR(sensor_common_parse_num_modes) },
	{ 0xd8d394c7, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0x1fdc7df2, __VMLINUX_SYMBOL_STR(_mcount) },
	{ 0xd2b4f854, __VMLINUX_SYMBOL_STR(regmap_read) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Cnvidia,imx586");
MODULE_ALIAS("of:N*T*Cnvidia,imx586C*");
