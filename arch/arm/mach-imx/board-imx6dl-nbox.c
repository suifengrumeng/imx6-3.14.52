/*
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_net.h>
#include <linux/fec.h>
#include <linux/netdevice.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

/* cyttsp */
#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_platform.h>



static struct fec_platform_data fec_pdata;
static struct flexcan_platform_data flexcan_pdata[2];
static int flexcan_en_gpio;
static int flexcan_stby_gpio;
static int flexcan0_en;
static int flexcan1_en;

static void imx6q_fec_sleep_enable(int enabled)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (enabled)
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ,
					   IMX6Q_GPR13_ENET_STOP_REQ);
		else
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ, 0);
	} else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

static void __init imx6q_enet_plt_init(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02100000/ethernet@02188000");
	if (np && of_get_property(np, "fsl,magic-packet", NULL))
		fec_pdata.sleep_mode_enable = imx6q_fec_sleep_enable;
}

static void mx6q_flexcan_switch(void)
{
	if (flexcan0_en || flexcan1_en) {
		/*
		 * The transceiver TJA1041A on sabreauto RevE baseboard will
		 * fail to transit to Normal state if EN/STBY is high by default
		 * after board power up. So we set the EN/STBY initial state to low
		 * first then to high to guarantee the state transition successfully.
		 */
		gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);

		gpio_set_value_cansleep(flexcan_en_gpio, 1);
		gpio_set_value_cansleep(flexcan_stby_gpio, 1);
	} else {
		/*
		 * avoid to disable CAN xcvr if any of the CAN interfaces
		 * are down. XCRV will be disabled only if both CAN2
		 * interfaces are DOWN.
		*/
		gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);
	}
}

static void imx6q_flexcan0_switch_auto(int enable)
{
	flexcan0_en = enable;
	mx6q_flexcan_switch();
}

static void imx6q_flexcan1_switch_auto(int enable)
{
	flexcan1_en = enable;
	mx6q_flexcan_switch();
}

static int __init imx6q_flexcan_fixup_auto(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
	if (!np)
		return -ENODEV;

	flexcan_en_gpio = of_get_named_gpio(np, "trx-en-gpio", 0);
	flexcan_stby_gpio = of_get_named_gpio(np, "trx-stby-gpio", 0);
	if (gpio_is_valid(flexcan_en_gpio) && gpio_is_valid(flexcan_stby_gpio) &&
		!gpio_request_one(flexcan_en_gpio, GPIOF_DIR_OUT, "flexcan-trx-en") &&
		!gpio_request_one(flexcan_stby_gpio, GPIOF_DIR_OUT, "flexcan-trx-stby")) {
		/* flexcan 0 & 1 are using the same GPIOs for transceiver */
		flexcan_pdata[0].transceiver_switch = imx6q_flexcan0_switch_auto;
		flexcan_pdata[1].transceiver_switch = imx6q_flexcan1_switch_auto;
	}

	return 0;
}

/* For imx6q sabrelite board: set KSZ9021RN RGMII pad skew */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* min rx data delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0xf0f0);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
	}

	return 0;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);

	return 0;
}

/*
 * fixup for PLX PEX8909 bridge to configure GPIO1-7 as output High
 * as they are used for slots1-7 PERST#
 */
static void ventana_pciesw_early_fixup(struct pci_dev *dev)
{
	u32 dw;

	if (!of_machine_is_compatible("gw,ventana"))
		return;

	if (dev->devfn != 0)
		return;

	pci_read_config_dword(dev, 0x62c, &dw);
	dw |= 0xaaa8; // GPIO1-7 outputs
	pci_write_config_dword(dev, 0x62c, dw);

	pci_read_config_dword(dev, 0x644, &dw);
	dw |= 0xfe;   // GPIO1-7 output high
	pci_write_config_dword(dev, 0x644, dw);

	msleep(100);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8609, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8606, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8604, ventana_pciesw_early_fixup);

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Set RGMII IO voltage to 1.8V */
	phy_write(dev, 0x1d, 0x1f);
	phy_write(dev, 0x1e, 0x8);

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	phy_write(dev, 0xd, 0x7);
	phy_write(dev, 0xe, 0x8016);
	phy_write(dev, 0xd, 0x4007);

	val = phy_read(dev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

#define PHY_ID_AR8031	0x004dd074

static int ar8035_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);

	val = phy_read(dev, 0xe);
	phy_write(dev, 0xe, val & ~(1 << 8));

	/*
	 * Enable 125MHz clock from CLK_25M on the AR8031.  This
	 * is fed in to the IMX6 on the ENET_REF_CLK (V22) pad.
	 * Also, introduce a tx clock delay.
	 *
	 * This is the same as is the AR8031 fixup.
	 */
	ar8031_phy_fixup(dev);

	/*check phy power*/
	val = phy_read(dev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(dev, 0x0, val & ~BMCR_PDOWN);

	return 0;
}

#define PHY_ID_AR8035 0x004dd072

static void __init imx6q_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
				ksz9021rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
				ksz9031rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
				ar8031_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8035, 0xffffffef,
				ar8035_phy_fixup);
	}
}

static void __init imx6q_1588_init(void)
{
	struct device_node *np;
	struct clk *ptp_clk;
	struct regmap *gpr;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-fec");
	if (!np) {
		pr_warn("%s: failed to find fec node\n", __func__);
		return;
	}

	ptp_clk = of_clk_get(np, 2);
	if (IS_ERR(ptp_clk)) {
		pr_warn("%s: failed to get ptp clock\n", __func__);
		goto put_node;
	}

	/*
	 * If enet_ref from ANATOP/CCM is the PTP clock source, we need to
	 * set bit IOMUXC_GPR1[21].  Or the PTP clock must be from pad
	 * (external OSC), and we need to clear the bit.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

	clk_put(ptp_clk);
put_node:
	of_node_put(np);
}

static void __init imx6q_csi_mux_init(void)
{
	/*
	 * MX6Q SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR1 bit 19 to 0x1.
	 *
	 * MX6DL SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR13 bit 0-2 to 0x4.
	 * IPU1 CSI1 connects to MIPI CSI2 virtual channel 1.
	 * Set GPR13 bit 3-5 to 0x1.
	 */
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (of_machine_is_compatible("fsl,imx6q-sabresd") ||
			of_machine_is_compatible("fsl,imx6q-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR1, 1 << 19, 1 << 19);
		else if (of_machine_is_compatible("fsl,imx6dl-sabresd") ||
			 of_machine_is_compatible("fsl,imx6dl-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR13, 0x3F, 0x0C);
	} else {
		pr_err("%s(): failed to find fsl,imx6q-iomux-gpr regmap\n",
		       __func__);
	}
}

static void __init imx6q_enet_clk_sel(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR5,
				   IMX6Q_GPR5_ENET_TX_CLK_SEL, IMX6Q_GPR5_ENET_TX_CLK_SEL);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

static inline void imx6q_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx6q-fec");
	imx6q_enet_phy_init();
	imx6q_1588_init();
	if (cpu_is_imx6q() && imx_get_soc_revision() == IMX_CHIP_REVISION_2_0)
		imx6q_enet_clk_sel();
	imx6q_enet_plt_init();
}

/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6q_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02090000, NULL, &flexcan_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02094000, NULL, &flexcan_pdata[1]),
	OF_DEV_AUXDATA("fsl,imx6q-fec", 0x02188000, NULL, &fec_pdata),
	{ /* sentinel */ }
};

static void __init imx6q_init_machine(void)
{
    int status;
	struct device *parent;

	if (cpu_is_imx6q() && imx_get_soc_revision() == IMX_CHIP_REVISION_2_0)
		imx_print_silicon_rev("i.MX6QP", IMX_CHIP_REVISION_1_0);
	else
		imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
				 imx_get_soc_revision());

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table,
					imx6q_auxdata_lookup, parent);

	imx6q_enet_init();
	imx_anatop_init();
	imx6q_csi_mux_init();
	cpu_is_imx6q() ?  imx6q_pm_init() : imx6dl_pm_init();
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_996MHZ		0x2
#define OCOTP_CFG3_SPEED_852MHZ		0x1

static void __init imx6q_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz;
	 * 2b'10: 996000000Hz;
	 * 2b'01: 852000000Hz; -- i.MX6Q Only, exclusive with 996MHz.
	 * 2b'00: 792000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	val &= 0x3;

	if (val != OCOTP_CFG3_SPEED_1P2GHZ)
		if (dev_pm_opp_disable(cpu_dev, 1200000000))
			pr_warn("failed to disable 1.2 GHz OPP\n");
	if (val < OCOTP_CFG3_SPEED_996MHZ)
		if (dev_pm_opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 996 MHz OPP\n");
	if (cpu_is_imx6q()) {
		if (val != OCOTP_CFG3_SPEED_852MHZ)
			if (dev_pm_opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 852 MHz OPP\n");
	}

	if (IS_ENABLED(CONFIG_MX6_VPU_352M)) {
		if (dev_pm_opp_disable(cpu_dev, 396000000))
			pr_warn("failed to disable 396MHz OPP\n");
		pr_info("remove 396MHz OPP for VPU running at 352MHz!\n");
	}

put_node:
	of_node_put(np);
}

static void __init imx6q_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6q_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device imx6q_cpufreq_pdev = {
	.name = "imx6q-cpufreq",
};







/********usb reset*************************/
#define USBHUB_RESET 	IMX_GPIO_NR(1, 15)
#define USB_POWER 	    IMX_GPIO_NR(1, 11)

void usb_reset(void)
{
	int status = 0;
	
    printk("usbhub_reset____________________________________________!\n");

	status = gpio_request(USBHUB_RESET, "lcd spi clk\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi clk");
		return;
	}
	gpio_direction_output(USBHUB_RESET, 0);
	
    gpio_set_value(USBHUB_RESET, 1);	
    mdelay(100);
	gpio_set_value(USBHUB_RESET, 0);	
    mdelay(100);
    gpio_set_value(USBHUB_RESET, 1);	


	status = gpio_request(USB_POWER, "lcd spi clk\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi clk");
		return;
	}
	gpio_direction_output(USB_POWER, 0);
	   
    printk("usb_power____________________________________________!\n");
    gpio_set_value(USB_POWER, 1);	
    
}


/*******lcd_spi****************************************/
#define ILI9488_LG35_GPIO_RESET 	IMX_GPIO_NR(1, 18)
//#define ILI9488_LG35_GPIO_BACKLIGHT IMX_GPIO_NR(4, 10)
#define ILI9488_LG35_GPIO_SPI_CS	IMX_GPIO_NR(4, 9)
#define ILI9488_LG35_GPIO_SPI_CLK	IMX_GPIO_NR(5, 22)
#define ILI9488_LG35_GPIO_SPI_DATA	IMX_GPIO_NR(5, 23)


static void lcd_gpio_init(void)
{
	int status = 0;
	
	status = gpio_request(ILI9488_LG35_GPIO_SPI_CLK, "lcd spi clk\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi clk");
		return;
	}
	gpio_direction_output(ILI9488_LG35_GPIO_SPI_CLK, 0);
	
	status = gpio_request(ILI9488_LG35_GPIO_SPI_DATA, "lcd spi data\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi data");
		return;
	}
	gpio_direction_output(ILI9488_LG35_GPIO_SPI_DATA, 0);	
	
	
	status = gpio_request(ILI9488_LG35_GPIO_SPI_CS, "lcd spi cs\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for lcd spi cs");
		return;
	}
	gpio_direction_output(ILI9488_LG35_GPIO_SPI_CS, 0);

}


static void lcd_gpio_clear(void)
{
	gpio_free(ILI9488_LG35_GPIO_SPI_CLK);	
	gpio_free(ILI9488_LG35_GPIO_SPI_DATA);	
	gpio_free(ILI9488_LG35_GPIO_SPI_CS);	
	//gpio_free(ILI9488_LG35_GPIO_RESET);	//cannot free, or else will reset lcd
}

static void fn043_spi_write_cmd_byte(unsigned int i)
{ 
    unsigned char n,m;
    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
     
    m=0x80;
    for(n=0; n<8; n++)
    {           
           if(i&m)

            {    
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
            else
            {     
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }            
            m=m>>1;

           
     }  
     
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
} 


static void fn043_spi_write_cmd_double_byte(unsigned int i)
{ 
    unsigned short n,m;
    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
     
    m=0x8000;
    for(n=0; n<16; n++)
    {           
           if(i&m)
            {    
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
            else
            {     
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }            
            m=m>>1;

           
     }  
     
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
} 




static void fn043_spi_write_data_byte(unsigned int i)
{ 
    unsigned char n,m;
    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);    
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
     
    m=0x80;
    for(n=0; n<8; n++)
    {           
           if(i&m)

            {    
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
            else
            {     
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
                gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }            
            m=m>>1;

           
     }  
     
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
} 

static void TFT_FN043_Reset(void)
{

   gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);

    
    if (gpio_request(ILI9488_LG35_GPIO_RESET, "lcd reset\n") < 0)
    {
        pr_err("Failed to request gpio for lcd reset");
        return;
    }
    gpio_direction_output(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(100); // Delay 1ms 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 0);
    mdelay(800); //  Delay 10ms // This delay time is necessary 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(800); // Delay 120 ms 

   gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
}




static void TFT_STD4802_Reset(void)
{
    // VCI=2.8V 
    /************* Reset LCD Driver ***************
      548     LCD_nRESET = 1; 
      549     mdelay(1); // Delay 1ms 
      550     LCD_nRESET = 0; 
      551     mdelay(10); // Delay 10ms // This delay time is necessary 
      552     LCD_nRESET = 1; 
      553     mdelay(120); // Delay 120 ms 
      554      */

    if (gpio_request(ILI9488_LG35_GPIO_RESET, "lcd reset\n") < 0)
    {
        pr_err("Failed to request gpio for lcd reset");
        return;
    }
    gpio_direction_output(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(100); // Delay 1ms 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 0);
    mdelay(200); //  Delay 10ms // This delay time is necessary 
    gpio_set_value(ILI9488_LG35_GPIO_RESET, 1);
    mdelay(200); // Delay 120 ms 
}


static void lcd_spi_send_byte(unsigned char i){
    uint8_t n;
    uint8_t m;
    m = 0x80;
    for (n = 0; n < 8; n++) {
    if (i & m) {
            gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
            gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 1);
            gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
        } else {
            gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 0);
            gpio_set_value(ILI9488_LG35_GPIO_SPI_DATA, 0);
            gpio_set_value(ILI9488_LG35_GPIO_SPI_CLK, 1);
            }
        m = m >> 1;
       
    }
}

static void lcd_spi_write_cmd_byte(unsigned short i){
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);
    lcd_spi_send_byte(0x20);
    lcd_spi_send_byte(i>>8);
    lcd_spi_send_byte(0x00);
    lcd_spi_send_byte(i);
    gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
}

static void lcd_spi_write_data_byte(unsigned char i)
{
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 0);
     lcd_spi_send_byte(0x40);
     lcd_spi_send_byte(i);
     gpio_set_value(ILI9488_LG35_GPIO_SPI_CS, 1);
}




static void TFT_FN043_init_code(void)
{
 
	TFT_FN043_Reset();   //reset
//************* Start Initial Sequence **********//

lcd_spi_write_cmd_byte(0xff00); lcd_spi_write_data_byte(0x80);
lcd_spi_write_cmd_byte(0xff01); lcd_spi_write_data_byte(0x09);
lcd_spi_write_cmd_byte(0xff02); lcd_spi_write_data_byte(0x01);
lcd_spi_write_cmd_byte(0xff80); lcd_spi_write_data_byte(0x80);
lcd_spi_write_cmd_byte(0xff81); lcd_spi_write_data_byte(0x09);
lcd_spi_write_cmd_byte(0xff03); lcd_spi_write_data_byte(0x01);
lcd_spi_write_cmd_byte(0xc5b1); lcd_spi_write_data_byte(0xA9);	
lcd_spi_write_cmd_byte(0xc591); lcd_spi_write_data_byte(0x0F);    
lcd_spi_write_cmd_byte(0xc0B4); lcd_spi_write_data_byte(0x50);
mdelay(500);	
lcd_spi_write_cmd_byte(0xE100); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xE101); lcd_spi_write_data_byte(0x09);
lcd_spi_write_cmd_byte(0xE102); lcd_spi_write_data_byte(0x0F);
lcd_spi_write_cmd_byte(0xE103); lcd_spi_write_data_byte(0x0E);
lcd_spi_write_cmd_byte(0xE104); lcd_spi_write_data_byte(0x07);
lcd_spi_write_cmd_byte(0xE105); lcd_spi_write_data_byte(0x10);
lcd_spi_write_cmd_byte(0xE106); lcd_spi_write_data_byte(0x0B);
lcd_spi_write_cmd_byte(0xE107); lcd_spi_write_data_byte(0x0A);
lcd_spi_write_cmd_byte(0xE108); lcd_spi_write_data_byte(0x04);
lcd_spi_write_cmd_byte(0xE109); lcd_spi_write_data_byte(0x07);
lcd_spi_write_cmd_byte(0xE10A); lcd_spi_write_data_byte(0x0B);
lcd_spi_write_cmd_byte(0xE10B); lcd_spi_write_data_byte(0x08);
lcd_spi_write_cmd_byte(0xE10C); lcd_spi_write_data_byte(0x0F);
lcd_spi_write_cmd_byte(0xE10D); lcd_spi_write_data_byte(0x10);
lcd_spi_write_cmd_byte(0xE10E); lcd_spi_write_data_byte(0x0A);
lcd_spi_write_cmd_byte(0xE10F); lcd_spi_write_data_byte(0x01);
mdelay(500);	
lcd_spi_write_cmd_byte(0xE200); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xE201); lcd_spi_write_data_byte(0x09);
lcd_spi_write_cmd_byte(0xE202); lcd_spi_write_data_byte(0x0F);
lcd_spi_write_cmd_byte(0xE203); lcd_spi_write_data_byte(0x0E);
lcd_spi_write_cmd_byte(0xE204); lcd_spi_write_data_byte(0x07);
lcd_spi_write_cmd_byte(0xE205); lcd_spi_write_data_byte(0x10);
lcd_spi_write_cmd_byte(0xE206); lcd_spi_write_data_byte(0x0B);
lcd_spi_write_cmd_byte(0xE207); lcd_spi_write_data_byte(0x0A);
lcd_spi_write_cmd_byte(0xE208); lcd_spi_write_data_byte(0x04);
lcd_spi_write_cmd_byte(0xE209); lcd_spi_write_data_byte(0x07);
lcd_spi_write_cmd_byte(0xE20A); lcd_spi_write_data_byte(0x0B);
lcd_spi_write_cmd_byte(0xE20B); lcd_spi_write_data_byte(0x08);
lcd_spi_write_cmd_byte(0xE20C); lcd_spi_write_data_byte(0x0F);
lcd_spi_write_cmd_byte(0xE20D); lcd_spi_write_data_byte(0x10);
lcd_spi_write_cmd_byte(0xE20E); lcd_spi_write_data_byte(0x0A);
lcd_spi_write_cmd_byte(0xE20F); lcd_spi_write_data_byte(0x01);
mdelay(500);	
lcd_spi_write_cmd_byte(0xD900); lcd_spi_write_data_byte(0x4E);
lcd_spi_write_cmd_byte(0xc181); lcd_spi_write_data_byte(0x66);
lcd_spi_write_cmd_byte(0xc1a1); lcd_spi_write_data_byte(0x08);
lcd_spi_write_cmd_byte(0xc592); lcd_spi_write_data_byte(0x01);
lcd_spi_write_cmd_byte(0xc595); lcd_spi_write_data_byte(0x34);
lcd_spi_write_cmd_byte(0xd800); lcd_spi_write_data_byte(0x79);	
lcd_spi_write_cmd_byte(0xd801); lcd_spi_write_data_byte(0x79);	
lcd_spi_write_cmd_byte(0xc594); lcd_spi_write_data_byte(0x33);
lcd_spi_write_cmd_byte(0xc0a3); lcd_spi_write_data_byte(0x1B);    
lcd_spi_write_cmd_byte(0xc582); lcd_spi_write_data_byte(0x83);
lcd_spi_write_cmd_byte(0xc481); lcd_spi_write_data_byte(0x83);
lcd_spi_write_cmd_byte(0xc1a1); lcd_spi_write_data_byte(0x0E);
lcd_spi_write_cmd_byte(0xb3a6); lcd_spi_write_data_byte(0x20);
lcd_spi_write_cmd_byte(0xb3a7); lcd_spi_write_data_byte(0x01);
lcd_spi_write_cmd_byte(0xce80); lcd_spi_write_data_byte(0x85);
lcd_spi_write_cmd_byte(0xce81); lcd_spi_write_data_byte(0x01); 
lcd_spi_write_cmd_byte(0xce82); lcd_spi_write_data_byte(0x00);	
lcd_spi_write_cmd_byte(0xce83); lcd_spi_write_data_byte(0x84); 
lcd_spi_write_cmd_byte(0xce84); lcd_spi_write_data_byte(0x01); 
lcd_spi_write_cmd_byte(0xce85); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcea0); lcd_spi_write_data_byte(0x18); 
lcd_spi_write_cmd_byte(0xcea1); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcea2); lcd_spi_write_data_byte(0x03); 
lcd_spi_write_cmd_byte(0xcea3); lcd_spi_write_data_byte(0x39); 
lcd_spi_write_cmd_byte(0xcea4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcea5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcea6); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcea7); lcd_spi_write_data_byte(0x18); 
lcd_spi_write_cmd_byte(0xcea8); lcd_spi_write_data_byte(0x03);
lcd_spi_write_cmd_byte(0xcea9); lcd_spi_write_data_byte(0x03); 
lcd_spi_write_cmd_byte(0xceaa); lcd_spi_write_data_byte(0x3a);
lcd_spi_write_cmd_byte(0xceab); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xceac); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcead); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xceb0); lcd_spi_write_data_byte(0x18);
lcd_spi_write_cmd_byte(0xceb1); lcd_spi_write_data_byte(0x02); 
lcd_spi_write_cmd_byte(0xceb2); lcd_spi_write_data_byte(0x03); 
lcd_spi_write_cmd_byte(0xceb3); lcd_spi_write_data_byte(0x3b); 
lcd_spi_write_cmd_byte(0xceb4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xceb5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xceb6); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xceb7); lcd_spi_write_data_byte(0x18);
lcd_spi_write_cmd_byte(0xceb8); lcd_spi_write_data_byte(0x01); 
lcd_spi_write_cmd_byte(0xceb9); lcd_spi_write_data_byte(0x03); 
lcd_spi_write_cmd_byte(0xceba); lcd_spi_write_data_byte(0x3c); 
lcd_spi_write_cmd_byte(0xcebb); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcebc); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcebd); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcfc0); lcd_spi_write_data_byte(0x01); 
lcd_spi_write_cmd_byte(0xcfc1); lcd_spi_write_data_byte(0x01); 
lcd_spi_write_cmd_byte(0xcfc2); lcd_spi_write_data_byte(0x20); 
lcd_spi_write_cmd_byte(0xcfc3); lcd_spi_write_data_byte(0x20); 
lcd_spi_write_cmd_byte(0xcfc4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcfc5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcfc6); lcd_spi_write_data_byte(0x01); 
lcd_spi_write_cmd_byte(0xcfc7); lcd_spi_write_data_byte(0x00);    
lcd_spi_write_cmd_byte(0xcfc8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcfc9); lcd_spi_write_data_byte(0x00);    
lcd_spi_write_cmd_byte(0xcfd0); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb80); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb81); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb82); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb83); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb84); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb85); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb86); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb87); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb88); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb89); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcb90); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb91); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb92); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb93); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb94); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb95); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb96); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb97); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb98); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb99); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb9a); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb9b); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb9c); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb9d); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcb9e); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcba0); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba1); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba2); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba3); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcba9); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbaa); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbab); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbac); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbad); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbae); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcbb0); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb1); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb2); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb3); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbb9); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcbc0); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbc1); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbc2); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbc3); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbc4); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbc5); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbc6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbc7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbc8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbc9); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbca); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbcb); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbcc); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbcd); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbce); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcbd0); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbd1); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbd2); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbd3); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbd4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbd5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbd6); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbd7); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbd8); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbd9); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbda); lcd_spi_write_data_byte(0x04); 
lcd_spi_write_cmd_byte(0xcbdb); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbdc); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbdd); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbde); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcbe0); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe1); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe2); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe3); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcbe9); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcbf0); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf1); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf2); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf3); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf4); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf5); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf6); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf7); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf8); lcd_spi_write_data_byte(0xFF); 
lcd_spi_write_cmd_byte(0xcbf9); lcd_spi_write_data_byte(0xFF);
lcd_spi_write_cmd_byte(0xcc80); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc81); lcd_spi_write_data_byte(0x26); 
lcd_spi_write_cmd_byte(0xcc82); lcd_spi_write_data_byte(0x09); 
lcd_spi_write_cmd_byte(0xcc83); lcd_spi_write_data_byte(0x0B); 
lcd_spi_write_cmd_byte(0xcc84); lcd_spi_write_data_byte(0x01); 
lcd_spi_write_cmd_byte(0xcc85); lcd_spi_write_data_byte(0x25); 
lcd_spi_write_cmd_byte(0xcc86); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc87); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc88); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc89); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xcc90); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc91); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc92); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc93); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc94); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc95); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc96); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc97); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc98); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc99); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc9a); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcc9b); lcd_spi_write_data_byte(0x26); 
lcd_spi_write_cmd_byte(0xcc9c); lcd_spi_write_data_byte(0x0A); 
lcd_spi_write_cmd_byte(0xcc9d); lcd_spi_write_data_byte(0x0C); 
lcd_spi_write_cmd_byte(0xcc9e); lcd_spi_write_data_byte(0x02);
lcd_spi_write_cmd_byte(0xcca0); lcd_spi_write_data_byte(0x25); 
lcd_spi_write_cmd_byte(0xcca1); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca2); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca3); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcca9); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccaa); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccab); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccac); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccad); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccae); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xccb0); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccb1); lcd_spi_write_data_byte(0x25); 
lcd_spi_write_cmd_byte(0xccb2); lcd_spi_write_data_byte(0x0C); 
lcd_spi_write_cmd_byte(0xccb3); lcd_spi_write_data_byte(0x0A); 
lcd_spi_write_cmd_byte(0xccb4); lcd_spi_write_data_byte(0x02); 
lcd_spi_write_cmd_byte(0xccb5); lcd_spi_write_data_byte(0x26); 
lcd_spi_write_cmd_byte(0xccb6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccb7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccb8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccb9); lcd_spi_write_data_byte(0x00);
lcd_spi_write_cmd_byte(0xccc0); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc1); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc2); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc3); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccc9); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccca); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xcccb); lcd_spi_write_data_byte(0x25); 
lcd_spi_write_cmd_byte(0xcccc); lcd_spi_write_data_byte(0x0B); 
lcd_spi_write_cmd_byte(0xcccd); lcd_spi_write_data_byte(0x09); 
lcd_spi_write_cmd_byte(0xccce); lcd_spi_write_data_byte(0x01);
lcd_spi_write_cmd_byte(0xccd0); lcd_spi_write_data_byte(0x26); 
lcd_spi_write_cmd_byte(0xccd1); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd2); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd3); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd4); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd5); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd6); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd7); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd8); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccd9); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccda); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccdb); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccdc); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccdd); lcd_spi_write_data_byte(0x00); 
lcd_spi_write_cmd_byte(0xccde); lcd_spi_write_data_byte(0x00); 


lcd_spi_write_cmd_byte(0xC480); lcd_spi_write_data_byte(0x30);
mdelay(50);
lcd_spi_write_cmd_byte(0xC48A); lcd_spi_write_data_byte(0x40);
mdelay(50);
lcd_spi_write_cmd_byte(0xC181); lcd_spi_write_data_byte(0x66);
lcd_spi_write_cmd_byte(0xF5B6); lcd_spi_write_data_byte(0x06);
lcd_spi_write_cmd_byte(0xC1A1); lcd_spi_write_data_byte(0x0E);
lcd_spi_write_cmd_byte(0xC481); lcd_spi_write_data_byte(0x83);//source bias //V02
lcd_spi_write_cmd_byte(0xC592); lcd_spi_write_data_byte(0x01);//(01)
lcd_spi_write_cmd_byte(0xC481); lcd_spi_write_data_byte(0x83);
lcd_spi_write_cmd_byte(0xC592); lcd_spi_write_data_byte(0x01);//Pump45
lcd_spi_write_cmd_byte(0xC5B1); lcd_spi_write_data_byte(0xA9);//DC voltage setting ;[0]GVDD output, default: 0xa8

lcd_spi_write_cmd_byte(0xd900); lcd_spi_write_data_byte(0x25);//DC voltage setting ;[0]GVDD output, default: 0xa8
mdelay(500);


lcd_spi_write_cmd_byte(0xff00); lcd_spi_write_data_byte(0xff); 
lcd_spi_write_cmd_byte(0xff01); lcd_spi_write_data_byte(0xff); 
lcd_spi_write_cmd_byte(0xff02); lcd_spi_write_data_byte(0xff); 

///============================================================================
 
  lcd_spi_write_cmd_byte(0x3A00);  //  RGB 18bits D[17:0]
  lcd_spi_write_data_byte(0x77);			
  lcd_spi_write_cmd_byte(0x1100);
  mdelay(2000);
  lcd_spi_write_cmd_byte(0x2900);
  mdelay(2000);

  lcd_spi_write_cmd_byte(0x2A00);
  lcd_spi_write_data_byte(0x00);
  lcd_spi_write_cmd_byte(0x2A01);
  lcd_spi_write_data_byte(0x00);
  lcd_spi_write_cmd_byte(0x2A02);
  lcd_spi_write_data_byte(0x01);
  lcd_spi_write_cmd_byte(0x2A03);
  lcd_spi_write_data_byte(0xDF);

  lcd_spi_write_cmd_byte(0x2B00);
  lcd_spi_write_data_byte(0x00);
  lcd_spi_write_cmd_byte(0x2B01);
  lcd_spi_write_data_byte(0x00);
  lcd_spi_write_cmd_byte(0x2B02);
  lcd_spi_write_data_byte(0x03);  //03
  lcd_spi_write_cmd_byte(0x2B03);
  lcd_spi_write_data_byte(0x1f); //55

  lcd_spi_write_cmd_byte(0x2C00);

}






 
static void TFT_STD4802_init_code(void) 
{
     TFT_STD4802_Reset();
/* 
     ILI9488_LG35_spi_write_cmd_byte(0x11);    //Sleep out
     mdelay(120);
     ILI9488_LG35_spi_write_cmd_byte(0x29);    //Sleep out
     mdelay(25);
*/   
     
    lcd_spi_write_cmd_byte(0xFF00); lcd_spi_write_data_byte(0x48);
    lcd_spi_write_cmd_byte(0xFF01); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xFF02); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xFF80); lcd_spi_write_data_byte(0x48);
    lcd_spi_write_cmd_byte(0xFF81); lcd_spi_write_data_byte(0x02);
 
    lcd_spi_write_cmd_byte(0x5100); lcd_spi_write_data_byte(0xF0);
 
    lcd_spi_write_cmd_byte(0xC5B1); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0xC4B0); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xC4B1); lcd_spi_write_data_byte(0x08);
    lcd_spi_write_cmd_byte(0xC4B2); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xC4B3); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0xC090); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xC091); lcd_spi_write_data_byte(0x0F);
    lcd_spi_write_cmd_byte(0xC092); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xC093); lcd_spi_write_data_byte(0x15);
    lcd_spi_write_cmd_byte(0xC094); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xC095); lcd_spi_write_data_byte(0x17);
 
    lcd_spi_write_cmd_byte(0xC582); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xC590); lcd_spi_write_data_byte(0x47);
 
    lcd_spi_write_cmd_byte(0xD800); lcd_spi_write_data_byte(0x48);
    lcd_spi_write_cmd_byte(0xD801); lcd_spi_write_data_byte(0x48);
 
    lcd_spi_write_cmd_byte(0xD900); lcd_spi_write_data_byte(0xB0);
 
    lcd_spi_write_cmd_byte(0xB391); lcd_spi_write_data_byte(0x10);//C0
    lcd_spi_write_cmd_byte(0xB392); lcd_spi_write_data_byte(0x25);//25
 
    lcd_spi_write_cmd_byte(0xC181); lcd_spi_write_data_byte(0x77);
 
    lcd_spi_write_cmd_byte(0x3A81); lcd_spi_write_data_byte(0x55);
 
    lcd_spi_write_cmd_byte(0xFF90); lcd_spi_write_data_byte(0x02);
 
    lcd_spi_write_cmd_byte(0xFF93); lcd_spi_write_data_byte(0x20);
    lcd_spi_write_cmd_byte(0xE100); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xE101); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE102); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE103); lcd_spi_write_data_byte(0x04);
    lcd_spi_write_cmd_byte(0xE104); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xE105); lcd_spi_write_data_byte(0x0B);
    lcd_spi_write_cmd_byte(0xE106); lcd_spi_write_data_byte(0x0A);
    lcd_spi_write_cmd_byte(0xE107); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE108); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE109); lcd_spi_write_data_byte(0x08);
    lcd_spi_write_cmd_byte(0xE10A); lcd_spi_write_data_byte(0x10);
    lcd_spi_write_cmd_byte(0xE10B); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE10C); lcd_spi_write_data_byte(0x06);
    lcd_spi_write_cmd_byte(0xE10D); lcd_spi_write_data_byte(0x11);
    lcd_spi_write_cmd_byte(0xE10E); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE10F); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xE200); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xE201); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE202); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE203); lcd_spi_write_data_byte(0x04);
    lcd_spi_write_cmd_byte(0xE204); lcd_spi_write_data_byte(0x02);
    lcd_spi_write_cmd_byte(0xE205); lcd_spi_write_data_byte(0x0B);
    lcd_spi_write_cmd_byte(0xE206); lcd_spi_write_data_byte(0x0A);
    lcd_spi_write_cmd_byte(0xE207); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE208); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE209); lcd_spi_write_data_byte(0x08);
    lcd_spi_write_cmd_byte(0xE20A); lcd_spi_write_data_byte(0x10);
    lcd_spi_write_cmd_byte(0xE20B); lcd_spi_write_data_byte(0x05);
    lcd_spi_write_cmd_byte(0xE20C); lcd_spi_write_data_byte(0x06);
    lcd_spi_write_cmd_byte(0xE20D); lcd_spi_write_data_byte(0x11);
    lcd_spi_write_cmd_byte(0xE20E); lcd_spi_write_data_byte(0x09);
    lcd_spi_write_cmd_byte(0xE20F); lcd_spi_write_data_byte(0x01);
 
    lcd_spi_write_cmd_byte(0xFF80); lcd_spi_write_data_byte(0x00);
    lcd_spi_write_cmd_byte(0xFF81); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0xFF00); lcd_spi_write_data_byte(0xFF);
    lcd_spi_write_cmd_byte(0xFF01); lcd_spi_write_data_byte(0xFF);
    lcd_spi_write_cmd_byte(0xFF02); lcd_spi_write_data_byte(0xFF);
 
    lcd_spi_write_cmd_byte(0x3500); lcd_spi_write_data_byte(0x00);
 
    lcd_spi_write_cmd_byte(0x3600); lcd_spi_write_data_byte(0x00);     //00
 
    lcd_spi_write_cmd_byte(0x1100);
    mdelay(120);
 
    lcd_spi_write_cmd_byte(0x2900);
    mdelay(20);

} 












static void ILI9488_LG35_init(void)
{
	lcd_gpio_init();
   
    TFT_FN043_init_code();
	
//    TFT_STD4802_init_code();
//
    
    //lcd_gpio_clear();
	
	printk("LDSH------------------------------------------------:ILI9488_LG35_init\n");
	return;
}



/**********************NBOX-INPUT-TS***************************/
#define CYTTSP5_I2C_TCH_ADR 0x24
#define CYTTSP5_LDR_TCH_ADR 0x24
#define CYTTSP5_I2C_IRQ_GPIO	IMX_GPIO_NR(1, 0)
#define CYTTSP5_I2C_RST_GPIO	IMX_GPIO_NR(1, 1)

void ts_gpio_init(void)
{   
    int status;
    status = gpio_request(CYTTSP5_I2C_IRQ_GPIO, "input ts irq\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for input ts irq");
		return;
	}
	gpio_direction_output(CYTTSP5_I2C_IRQ_GPIO, 0);

    status = gpio_request(CYTTSP5_I2C_RST_GPIO, "input ts reset\n");
	if (status < 0)
	{
		pr_err("Failed to request gpio for input ts reset");
		return;
	}
	gpio_direction_output(CYTTSP5_I2C_RST_GPIO, 0);
}


/***************late*******************************/
static void __init imx6q_init_late(void)
{
	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */


    ILI9488_LG35_init();   //lcd
    usb_reset();    //usb_reset
    ts_gpio_init();  //ts_input

	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		imx6q_opp_init();
		platform_device_register(&imx6q_cpufreq_pdev);
	}

	if (of_machine_is_compatible("fsl,imx6q-sabreauto")
		|| of_machine_is_compatible("fsl,imx6dl-sabreauto"))
		imx6q_flexcan_fixup_auto();
}

static void __init imx6q_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
#ifdef CONFIG_CPU_FREQ
	imx_busfreq_map_io();
#endif
}

static void __init imx6q_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static const char *imx6q_dt_compat[] __initdata = {
	"fsl,imx6dl",
	"fsl,imx6q",
	NULL,
};

DT_MACHINE_START(IMX6Q, "Freescale i.MX6 Quad/DualLite (Device Tree)")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6q_map_io,
	.init_irq	= imx6q_init_irq,
	.init_machine	= imx6q_init_machine,
	.init_late      = imx6q_init_late,
	.dt_compat	= imx6q_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
