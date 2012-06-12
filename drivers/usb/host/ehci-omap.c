/*
 * ehci-omap.c - driver for USBHOST on OMAP3/4 processors
 *
 * Bus Glue for the EHCI controllers in OMAP3/4
 * Tested on several OMAP3 boards, and OMAP4 Pandaboard
 *
 * Copyright (C) 2007-2011 Texas Instruments, Inc.
 *	Author: Vikram Pandita <vikram.pandita@ti.com>
 *	Author: Anand Gadiyar <gadiyar@ti.com>
 *	Author: Keshava Munegowda <keshava_mgowda@ti.com>
 *
 * Copyright (C) 2009 Nokia Corporation
 *	Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * Based on "ehci-fsl.c" and "ehci-au1xxx.c" ehci glue layers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * TODO (last updated Feb 27, 2010):
 *	- add kernel-doc
 *	- enable AUTOIDLE
 *	- add suspend/resume
 *	- add HSIC and TLL support
 *	- convert to use hwmod and runtime PM
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb/ulpi.h>
#include <plat/usb.h>
#include <linux/regulator/consumer.h>

enum ehci_hcd_omap_mode {
	EHCI_HCD_OMAP_MODE_UNKNOWN,
	EHCI_HCD_OMAP_MODE_PHY,
	EHCI_HCD_OMAP_MODE_TLL,
};

struct usb_hcd *ghcd;

/* EHCI Register Set */
#define EHCI_INSNREG04					(0xA0)
#define EHCI_INSNREG04_DISABLE_UNSUSPEND		(1 << 5)
#define	EHCI_INSNREG05_ULPI				(0xA4)
#define	EHCI_INSNREG05_ULPI_CONTROL_SHIFT		31
#define	EHCI_INSNREG05_ULPI_PORTSEL_SHIFT		24
#define	EHCI_INSNREG05_ULPI_OPSEL_SHIFT			22
#define	EHCI_INSNREG05_ULPI_REGADD_SHIFT		16
#define	EHCI_INSNREG05_ULPI_EXTREGADD_SHIFT		8
#define	EHCI_INSNREG05_ULPI_WRDATA_SHIFT		0

/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver;


static inline void ehci_write(void __iomem *base, u32 reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 ehci_read(void __iomem *base, u32 reg)
{
	return __raw_readl(base + reg);
}

/* Electrical test support */
static void host_write_port(u8 port, const char *buf)
{
       struct usb_bus *bus;
       struct ehci_hcd *ehci = hcd_to_ehci(ghcd);
       u32 port_status;
       u32 cmd;

       /* Reset Device */
       if (!strncmp(buf, "reset", 5)) {
               printk(KERN_INFO "\n RESET PORT \n");
               bus = hcd_to_bus(ghcd);
               if (bus->root_hub->children[port])
                       usb_reset_device(bus->root_hub->children[port]);
       }

       if (!strncmp(buf, "t-j", 3)) {
               printk(KERN_INFO "\n TEST_J \n");

#ifdef CONFIG_PM
               /* Suspend bus first */
               ehci_bus_suspend(ghcd);
#endif
               port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
               cmd = ehci_readl(ehci, &ehci->regs->command);

               port_status |= 1<<16; /* Test_Packet on Port2 */
               ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

               cmd |= CMD_RUN;
               ehci_writel(ehci, cmd, &ehci->regs->command);
       }

       if (!strncmp(buf, "t-k", 3)) {
               printk(KERN_INFO "\n TEST_K \n");

#ifdef CONFIG_PM
               /* Suspend bus first */
               ehci_bus_suspend(ghcd);
#endif
               port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
               cmd = ehci_readl(ehci, &ehci->regs->command);

               port_status |= 2<<16; /* Test_Packet on Port2 */
               ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

               cmd |= CMD_RUN;
               ehci_writel(ehci, cmd, &ehci->regs->command);
       }

       if (!strncmp(buf, "t-se0", 5)) {
               printk(KERN_INFO "\n TEST_SE0_NAK \n");

#ifdef CONFIG_PM
               /* Suspend bus first */
               ehci_bus_suspend(ghcd);
#endif
               port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
               cmd = ehci_readl(ehci, &ehci->regs->command);

               port_status |= 3<<16; /* Test_Packet on Port2 */
               ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

               cmd |= CMD_RUN;
               ehci_writel(ehci, cmd, &ehci->regs->command);
       }

       /* Send test packet on suspended port */
       if (!strncmp(buf, "t-pkt", 5)) {
               printk(KERN_INFO "\n TEST_PACKET \n");

#ifdef CONFIG_PM
               /* Suspend bus first */
               ehci_bus_suspend(ghcd);
#endif
               port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
               cmd = ehci_readl(ehci, &ehci->regs->command);

               /* Set Test packet bit */
               port_status |= 4<<16; /* Test_Packet on Port2 */
               ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

               cmd |= CMD_RUN;
               ehci_writel(ehci, cmd, &ehci->regs->command);
       }

       if (!strncmp(buf, "t-force", 7)) {
               printk(KERN_INFO "\n TEST_FORCE \n");

#ifdef CONFIG_PM
               /* Suspend bus first */
               ehci_bus_suspend(ghcd);
#endif
               port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
               cmd = ehci_readl(ehci, &ehci->regs->command);

               port_status |= 5<<16; /* Test_Packet on Port2 */
               ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

               cmd |= CMD_RUN;
               ehci_writel(ehci, cmd, &ehci->regs->command);
       }

}

static ssize_t
host_show_port(struct device *dev,
               struct device_attribute *attr, char *buf)
{
       return sprintf(buf, "\nOptions\t--> Description\n"
                       "\nreset\t-->Reset Device"
                       "\nt-j\t-->Send TEST_J on suspended port"
                       "\nt-k\t-->Send TEST_K on suspended port"
                       "\nt-pkt\t-->Send TEST_PACKET[53] on suspended port"
                       "\nt-force\t-->Send TEST_FORCE_ENABLE on suspended port"
                       "\nt-se0\t-->Send TEST_SE0_NAK on suspended port\n\n"
                       );
}

/* Port 1 */
static ssize_t
host_write_port1(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t n)
{
       struct usb_device       *udev = to_usb_device(dev);

       usb_lock_device(udev);
       host_write_port(0, buf);
       usb_unlock_device(udev);
       return n;
}
static DEVICE_ATTR(port1, S_IRUGO | S_IWUSR, host_show_port, host_write_port1);

/* Port 2 */
static ssize_t
host_write_port2(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t n)
{
       struct usb_device       *udev = to_usb_device(dev);

       usb_lock_device(udev);
       host_write_port(1, buf);
       usb_unlock_device(udev);
       return n;
}
static DEVICE_ATTR(port2, S_IRUGO | S_IWUSR, host_show_port, host_write_port2);

/* Port 3 */
static ssize_t
host_write_port3(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t n)
{
       struct usb_device       *udev = to_usb_device(dev);

       usb_lock_device(udev);
       host_write_port(2, buf);
       usb_unlock_device(udev);
       return n;
}
static DEVICE_ATTR(port3, S_IRUGO | S_IWUSR, host_show_port, host_write_port3);

static void omap_ehci_soft_phy_reset(struct platform_device *pdev, u8 port)
{
	struct usb_hcd	*hcd = dev_get_drvdata(&pdev->dev);
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	unsigned reg = 0;

	reg = ULPI_FUNC_CTRL_RESET
		/* FUNCTION_CTRL_SET register */
		| (ULPI_SET(ULPI_FUNC_CTRL) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
		/* Write */
		| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT)
		/* PORTn */
		| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
		/* start ULPI access*/
		| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	ehci_write(hcd->regs, EHCI_INSNREG05_ULPI, reg);

	/* Wait for ULPI access completion */
	while ((ehci_read(hcd->regs, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_dbg(&pdev->dev, "phy reset operation timed out\n");
			break;
		}
	}
}

static void disable_put_regulator(
		struct ehci_hcd_omap_platform_data *pdata)
{
	int i;

	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		if (pdata->regulator[i]) {
			regulator_disable(pdata->regulator[i]);
			regulator_put(pdata->regulator[i]);
		}
	}
}

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * ehci_hcd_omap_probe - initialize TI-based HCDs
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int ehci_hcd_omap_probe(struct platform_device *pdev)
{
	struct device				*dev = &pdev->dev;
	struct ehci_hcd_omap_platform_data	*pdata = dev->platform_data;
	struct resource				*res;
	struct usb_hcd				*hcd;
	void __iomem				*regs;
	struct ehci_hcd				*omap_ehci;
	int					ret = -ENODEV;
	int					irq;
	int					i;
	char					supply[7];

	if (usb_disabled())
		return -ENODEV;

	if (!dev->parent) {
		dev_err(dev, "Missing parent device\n");
		return -ENODEV;
	}

	irq = platform_get_irq_byname(pdev, "ehci-irq");
	if (irq < 0) {
		dev_err(dev, "EHCI irq failed\n");
		return -ENODEV;
	}

	res =  platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "ehci");
	if (!res) {
		dev_err(dev, "UHH EHCI get resource failed\n");
		return -ENODEV;
	}

	regs = ioremap(res->start, resource_size(res));
	if (!regs) {
		dev_err(dev, "UHH EHCI ioremap failed\n");
		return -ENOMEM;
	}

	/* Electrical test support
	* Interface is /sys/devices/platform/usbhs-omap.0/ehci-omap.0/portn
	*/
	if (pdata->port_mode[0] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		   ret = device_create_file(&pdev->dev, &dev_attr_port1);
	if (pdata->port_mode[1] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		   ret = device_create_file(&pdev->dev, &dev_attr_port2);
	if (pdata->port_mode[2] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		   ret = device_create_file(&pdev->dev, &dev_attr_port3);

	ghcd = hcd = usb_create_hcd(&ehci_omap_hc_driver, &pdev->dev,
		dev_name(&pdev->dev));

	if (!hcd) {
		dev_err(dev, "failed to create hcd with err %d\n", ret);
		ret = -ENOMEM;
		goto err_io;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = regs;

	/* get ehci regulator and enable */
	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		if (pdata->port_mode[i] != OMAP_EHCI_PORT_MODE_PHY) {
			pdata->regulator[i] = NULL;
			continue;
		}
		snprintf(supply, sizeof(supply), "hsusb%d", i);
		pdata->regulator[i] = regulator_get(dev, supply);
		if (IS_ERR(pdata->regulator[i])) {
			pdata->regulator[i] = NULL;
			dev_dbg(dev,
			"failed to get ehci port%d regulator\n", i);
		} else {
			regulator_enable(pdata->regulator[i]);
		}
	}

	ret = omap_usbhs_enable(dev);
	if (ret) {
		dev_err(dev, "failed to start usbhs with err %d\n", ret);
		goto err_enable;
	}

	/*
	 * An undocumented "feature" in the OMAP3 EHCI controller,
	 * causes suspended ports to be taken out of suspend when
	 * the USBCMD.Run/Stop bit is cleared (for example when
	 * we do ehci_bus_suspend).
	 * This breaks suspend-resume if the root-hub is allowed
	 * to suspend. Writing 1 to this undocumented register bit
	 * disables this feature and restores normal behavior.
	 */
	ehci_write(regs, EHCI_INSNREG04,
				EHCI_INSNREG04_DISABLE_UNSUSPEND);

	/* Soft reset the PHY using PHY reset command over ULPI */
	if (pdata->port_mode[0] == OMAP_EHCI_PORT_MODE_PHY)
		omap_ehci_soft_phy_reset(pdev, 0);
	if (pdata->port_mode[1] == OMAP_EHCI_PORT_MODE_PHY)
		omap_ehci_soft_phy_reset(pdev, 1);

	omap_ehci = hcd_to_ehci(hcd);
	omap_ehci->sbrn = 0x20;

	/* we know this is the memory we want, no need to ioremap again */
	omap_ehci->caps = hcd->regs;
	omap_ehci->regs = hcd->regs
		+ HC_LENGTH(ehci, readl(&omap_ehci->caps->hc_capbase));

	dbg_hcs_params(omap_ehci, "reset");
	dbg_hcc_params(omap_ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	omap_ehci->hcs_params = readl(&omap_ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret) {
		dev_err(dev, "failed to add hcd with err %d\n", ret);
		goto err_add_hcd;
	}

	/* root ports should always stay powered */
	ehci_port_power(omap_ehci, 1);

	return 0;

err_add_hcd:
	omap_usbhs_disable(dev);

err_enable:
	disable_put_regulator(pdata);
	usb_put_hcd(hcd);

err_io:
	iounmap(regs);
	return ret;
}


/**
 * ehci_hcd_omap_remove - shutdown processing for EHCI HCDs
 * @pdev: USB Host Controller being removed
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static int ehci_hcd_omap_remove(struct platform_device *pdev)
{
	struct device *dev	= &pdev->dev;
	struct usb_hcd *hcd	= dev_get_drvdata(dev);
	struct ehci_hcd_omap_platform_data      *pdata = dev->platform_data;

	if (pdata->port_mode[0] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		   device_remove_file(&pdev->dev, &dev_attr_port1);
	if (pdata->port_mode[1] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		   device_remove_file(&pdev->dev, &dev_attr_port2);
	if (pdata->port_mode[2] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		   device_remove_file(&pdev->dev, &dev_attr_port3);

	usb_remove_hcd(hcd);
	omap_usbhs_disable(dev);
	disable_put_regulator(dev->platform_data);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);
	return 0;
}

static void ehci_hcd_omap_shutdown(struct platform_device *pdev)
{
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static struct platform_driver ehci_hcd_omap_driver = {
	.probe			= ehci_hcd_omap_probe,
	.remove			= ehci_hcd_omap_remove,
	.shutdown		= ehci_hcd_omap_shutdown,
	/*.suspend		= ehci_hcd_omap_suspend, */
	/*.resume		= ehci_hcd_omap_resume, */
	.driver = {
		.name		= "ehci-omap",
	}
};

/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "OMAP-EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset			= ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

MODULE_ALIAS("platform:omap-ehci");
MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

