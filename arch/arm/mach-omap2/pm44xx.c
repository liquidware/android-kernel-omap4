/*
 * OMAP4 Power Management Routines
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <plat/serial.h>

#include "pm.h"
#include "powerdomain.h"
#include "clockdomain.h"
#include <mach/omap4-common.h>

#include "clock.h"
#include "cminst44xx.h"
#include "prcm44xx.h"
#include "cm1_44xx.h"
#include "cm2_44xx.h"
#include "prm44xx.h"
#include "prminst44xx.h"

#include <mach/omap4-common.h>
#include <plat/common.h>

#include "prm-regbits-44xx.h"
#include "cm-regbits-44xx.h"

struct power_state {
	struct powerdomain *pwrdm;
	u32 next_state;
#ifdef CONFIG_SUSPEND
	u32 saved_state;
	u32 saved_logic_state;
#endif
	struct list_head node;
};

static LIST_HEAD(pwrst_list);

#define MAX_IOPAD_LATCH_TIME 1000
void omap4_trigger_ioctrl(void)
{
	int i = 0;

	/* Trigger WUCLKIN enable */
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_WUCLK_CTRL_MASK, OMAP4430_WUCLK_CTRL_MASK,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_IO_PMCTRL_OFFSET);
	omap_test_timeout(
		((omap4_prminst_read_inst_reg(OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_IO_PMCTRL_OFFSET)
		>> OMAP4430_WUCLK_STATUS_SHIFT) == 1),
		MAX_IOPAD_LATCH_TIME, i);
	/* Trigger WUCLKIN disable */
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_WUCLK_CTRL_MASK, 0x0,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_IO_PMCTRL_OFFSET);
	return;
}

#ifdef CONFIG_SUSPEND
/* This is a common low power function called from suspend and
 * cpuidle
 */
void omap4_enter_sleep(void)
{
	u32 cpu_id = smp_processor_id();

	omap_uart_prepare_idle(0);
	omap_uart_prepare_idle(1);
	omap_uart_prepare_idle(2);
	omap_uart_prepare_idle(3);
	omap2_gpio_prepare_for_idle(0);
	omap4_trigger_ioctrl();

	omap4_enter_lowpower(cpu_id, PWRDM_POWER_OFF);

	omap2_gpio_resume_after_idle();
	omap_uart_resume_idle(0);
	omap_uart_resume_idle(1);
	omap_uart_resume_idle(2);
	omap_uart_resume_idle(3);

	return;
}

static int omap4_pm_suspend(void)
{
	omap_do_wfi();
	return 0;
}

static int omap4_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap4_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap4_pm_begin(suspend_state_t state)
{
	disable_hlt();
	return 0;
}

static void omap4_pm_end(void)
{
	enable_hlt();
	return;
}

static const struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap4_pm_begin,
	.end		= omap4_pm_end,
	.enter		= omap4_pm_enter,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */

/*
 * Enable hardware supervised mode for all clockdomains if it's
 * supported. Initiate sleep transition for other clockdomains, if
 * they are not used
 */
static int __init clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	if (clkdm->flags & CLKDM_CAN_ENABLE_AUTO)
		clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
			atomic_read(&clkdm->usecount) == 0)
		clkdm_sleep(clkdm);
	return 0;
}


static int __init pwrdms_setup(struct powerdomain *pwrdm, void *unused)
{
	struct power_state *pwrst;

	if (!pwrdm->pwrsts)
		return 0;

	pwrst = kmalloc(sizeof(struct power_state), GFP_ATOMIC);
	if (!pwrst)
		return -ENOMEM;
	pwrst->pwrdm = pwrdm;
	pwrst->next_state = PWRDM_POWER_ON;
	list_add(&pwrst->node, &pwrst_list);

	return pwrdm_set_next_pwrst(pwrst->pwrdm, pwrst->next_state);
}

/**
 * omap_default_idle -
 * Implements OMAP4 memory, IO ordering requirements which can't be addressed
 * with default arch_idle() hook. Used by all CPUs with !CONFIG_CPUIDLE and
 * by secondary CPU with CONFIG_CPUIDLE.
 */
static void omap_default_idle(void)
{
	local_irq_disable();
	local_fiq_disable();

	omap_do_wfi();

	local_fiq_enable();
	local_irq_enable();
}

static void __init prcm_setup_regs(void)
{
	struct clk *dpll_abe_ck, *dpll_core_ck, *dpll_iva_ck;
	struct clk *dpll_mpu_ck, *dpll_per_ck, *dpll_usb_ck;

	/*Enable all the DPLL autoidle */

	dpll_abe_ck = clk_get(NULL, "dpll_abe_ck");
	omap3_dpll_allow_idle(dpll_abe_ck);
	dpll_core_ck = clk_get(NULL, "dpll_core_ck");
	omap3_dpll_allow_idle(dpll_core_ck);
	dpll_iva_ck = clk_get(NULL, "dpll_iva_ck");
	omap3_dpll_allow_idle(dpll_iva_ck);
	if (cpu_is_omap446x())
		dpll_mpu_ck = clk_get(NULL, "virt_dpll_mpu_ck");
	else
		dpll_mpu_ck = clk_get(NULL, "dpll_mpu_ck");
	if (dpll_mpu_ck)
		omap3_dpll_allow_idle(dpll_mpu_ck);
	dpll_per_ck = clk_get(NULL, "dpll_per_ck");
	omap3_dpll_allow_idle(dpll_per_ck);
	dpll_usb_ck = clk_get(NULL, "dpll_usb_ck");
	omap3_dpll_allow_idle(dpll_usb_ck);

	/* Enable autogating for all DPLL post dividers */
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_MPU_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M4_DPLL_IVA_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M5_DPLL_IVA_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_CORE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M3_DPLL_CORE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M4_DPLL_CORE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M5_DPLL_CORE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M6_DPLL_CORE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT4_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M7_DPLL_CORE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_PER_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_PER_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M3_DPLL_PER_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M4_DPLL_PER_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M5_DPLL_PER_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M6_DPLL_PER_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_HSDIVIDER_CLKOUT4_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M7_DPLL_PER_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_ABE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_ABE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_PARTITION, OMAP4430_CM1_CKGEN_INST, OMAP4_CM_DIV_M3_DPLL_ABE_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_USB_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKDCOLDO_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_CLKDCOLDO_DPLL_USB_OFFSET);
	omap4_cminst_rmw_inst_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_PARTITION, OMAP4430_CM2_CKGEN_INST, OMAP4_CM_DIV_M2_DPLL_UNIPRO_OFFSET);
	/* Enable IO_ST interrupt */
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_IO_ST_MASK, OMAP4430_IO_ST_MASK,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_OCP_SOCKET_INST, OMAP4_PRM_IRQENABLE_MPU_OFFSET);

	/* Enable GLOBAL_WUEN */
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_GLOBAL_WUEN_MASK, OMAP4430_GLOBAL_WUEN_MASK,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_IO_PMCTRL_OFFSET);
	/*
	 * Errata ID: i608 Impacted OMAP4430 ES 1.0,2.0,2.1,2.2
	 * On OMAP4, Retention-Till-Access Memory feature is not working
	 * reliably and hardware recommondation is keep it disabled by
	 * default
	 */
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_SRAM_WKUP_SETUP_OFFSET);
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_LDO_SRAM_CORE_SETUP_OFFSET);
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_LDO_SRAM_MPU_SETUP_OFFSET);
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_LDO_SRAM_IVA_SETUP_OFFSET);
	/* Toggle CLKREQ in RET and OFF states */
	omap4_prminst_write_inst_reg(0x2, OMAP4430_PRM_PARTITION,
		OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_CLKREQCTRL_OFFSET);
	/*
	 * De-assert PWRREQ signal in Device OFF state
	 *	0x3: PWRREQ is de-asserted if all voltage domain are in
	 *	OFF state. Conversely, PWRREQ is asserted upon any
	 *	voltage domain entering or staying in ON or SLEEP or
	 *	RET state.
	 */
	omap4_prminst_write_inst_reg(0x3, OMAP4430_PRM_PARTITION,
		OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_PWRREQCTRL_OFFSET);
}

int omap4_can_sleep(void)
{
	if (!omap_uart_can_sleep())
		return -1;
	return 0;
}

static irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 irqenable_mpu, irqstatus_mpu;

	irqenable_mpu = omap4_prm_read_inst_reg(OMAP4430_PRM_OCP_SOCKET_INST,
					 OMAP4_PRM_IRQENABLE_MPU_OFFSET);
	irqstatus_mpu = omap4_prm_read_inst_reg(OMAP4430_PRM_OCP_SOCKET_INST,
					 OMAP4_PRM_IRQSTATUS_MPU_OFFSET);

	/* Check if a IO_ST interrupt */
	if (irqstatus_mpu & OMAP4430_IO_ST_MASK) {
		omap4_trigger_ioctrl();
	}

	/* Clear the interrupt */
	irqstatus_mpu &= irqenable_mpu;
	omap4_prm_write_inst_reg(irqstatus_mpu, OMAP4430_PRM_OCP_SOCKET_INST,
					OMAP4_PRM_IRQSTATUS_MPU_OFFSET);

	return IRQ_HANDLED;
}


/**
 * omap4_pm_init - Init routine for OMAP4 PM
 *
 * Initializes all powerdomain and clockdomain target states
 * and all PRCM settings.
 */
static int __init omap4_pm_init(void)
{
	int ret;
	struct clockdomain *emif_clkdm, *mpuss_clkdm, *l3_1_clkdm;
	struct clockdomain *ducati_clkdm, *l3_2_clkdm;

	if (!cpu_is_omap44xx())
		return -ENODEV;

	if (omap_rev() == OMAP4430_REV_ES1_0) {
		WARN(1, "Power Management not supported on OMAP4430 ES1.0\n");
		return -ENODEV;
	}

	pr_err("Power Management for TI OMAP4.\n");

	/* Enable IO_ST interrupt */
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_IO_ST_MASK, OMAP4430_IO_ST_MASK,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_OCP_SOCKET_INST, OMAP4_PRM_IRQENABLE_MPU_OFFSET);

	/* Enable GLOBAL_WUEN */
	omap4_prminst_rmw_inst_reg_bits(OMAP4430_GLOBAL_WUEN_MASK, OMAP4430_GLOBAL_WUEN_MASK,
		OMAP4430_PRM_PARTITION, OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_IO_PMCTRL_OFFSET);

	ret = request_irq(OMAP44XX_IRQ_PRCM,
			  (irq_handler_t)prcm_interrupt_handler,
			  IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
		       OMAP44XX_IRQ_PRCM);
		goto err2;
	}
	ret = pwrdm_for_each(pwrdms_setup, NULL);
	if (ret) {
		pr_err("Failed to setup powerdomains\n");
		goto err2;
	}

	/*
	 * The dynamic dependency between MPUSS -> MEMIF and
	 * MPUSS -> L3_* and DUCATI -> doesn't work as expected.
	 * The hardware recommendation is to keep above dependencies.
	 * Without this system locks up or randomly crashesh.
	 */
	mpuss_clkdm = clkdm_lookup("mpuss_clkdm");
	emif_clkdm = clkdm_lookup("l3_emif_clkdm");
	l3_1_clkdm = clkdm_lookup("l3_1_clkdm");
	l3_2_clkdm = clkdm_lookup("l3_2_clkdm");
	ducati_clkdm = clkdm_lookup("ducati_clkdm");
	if ((!mpuss_clkdm) || (!emif_clkdm) || (!l3_1_clkdm) ||
			(!l3_2_clkdm) || (!ducati_clkdm))
		goto err2;

	ret = clkdm_add_wkdep(mpuss_clkdm, emif_clkdm);
	ret |= clkdm_add_wkdep(mpuss_clkdm, l3_1_clkdm);
	ret |= clkdm_add_wkdep(mpuss_clkdm, l3_2_clkdm);
	ret |= clkdm_add_wkdep(ducati_clkdm, l3_1_clkdm);
	ret |= clkdm_add_wkdep(ducati_clkdm, l3_2_clkdm);
	if (ret) {
		pr_err("Failed to add MPUSS -> L3/EMIF, DUCATI -> L3 "
				"wakeup dependency\n");
		goto err2;
	}

	prcm_setup_regs();

	ret = omap4_mpuss_init();
	if (ret) {
		pr_err("Failed to initialise OMAP4 MPUSS\n");
		goto err2;
	}

	(void) clkdm_for_each(clkdms_setup, NULL);

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif /* CONFIG_SUSPEND */

	/* Overwrite the default arch_idle() */
	pm_idle = omap_default_idle;

	omap4_idle_init();

err2:
	return ret;
}
late_initcall(omap4_pm_init);
