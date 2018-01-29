/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/* This file is generated by GenLP_setting.pl v1.5.7 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

const unsigned int MT6355_PMIC_REG_gs_suspend_32kless_data[] = {
/* Address	 Mask		Golden Setting Value */
	0x0400, 0x0008, 0x0008,/* TOP_CKPDN_CON0 */
	0x040C, 0x0032, 0x0032,/* TOP_CKPDN_CON2 */
	0x0412, 0x0024, 0x0024,/* TOP_CKPDN_CON3 */
	0x1024, 0x0002, 0x0002,/* BUCK_VPROC11_OP_EN */
	0x102A, 0x0002, 0x0002,/* BUCK_VPROC11_OP_CFG */
	0x1046, 0x0002, 0x0002,/* BUCK_VPROC12_OP_EN */
	0x104C, 0x0002, 0x0002,/* BUCK_VPROC12_OP_CFG */
	0x1068, 0x0002, 0x0002,/* BUCK_VCORE_OP_EN */
	0x106E, 0x0002, 0x0002,/* BUCK_VCORE_OP_CFG */
	0x1082, 0x0001, 0x0000,/* BUCK_VGPU_CON0 */
	0x108C, 0x0002, 0x0000,/* BUCK_VGPU_OP_EN */
	0x10AE, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_EN */
	0x10B4, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_CFG */
	0x10D0, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_EN */
	0x10D6, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_CFG */
	0x10E8, 0x0001, 0x0000,/* BUCK_VMODEM_CON0 */
	0x10F2, 0x0002, 0x0000,/* BUCK_VMODEM_OP_EN */
	0x111C, 0x0002, 0x0002,/* BUCK_VS1_OP_EN */
	0x1122, 0x0002, 0x0002,/* BUCK_VS1_OP_CFG */
	0x1146, 0x0002, 0x0002,/* BUCK_VS2_OP_EN */
	0x114C, 0x0002, 0x0002,/* BUCK_VS2_OP_CFG */
	0x1166, 0x0001, 0x0000,/* BUCK_VPA_CON0 */
	0x1602, 0x0002, 0x0002,/* LDO_VIO28_OP_EN */
	0x1608, 0x0002, 0x0002,/* LDO_VIO28_OP_CFG */
	0x1616, 0x0002, 0x0002,/* LDO_VIO18_OP_EN */
	0x161C, 0x0002, 0x0002,/* LDO_VIO18_OP_CFG */
	0x162A, 0x0002, 0x0002,/* LDO_VUFS18_OP_EN */
	0x1630, 0x0002, 0x0002,/* LDO_VUFS18_OP_CFG */
	0x1640, 0x0002, 0x0002,/* LDO_VA10_OP_EN */
	0x1646, 0x0002, 0x0002,/* LDO_VA10_OP_CFG */
	0x1654, 0x0002, 0x0002,/* LDO_VA12_OP_EN */
	0x165A, 0x0002, 0x0002,/* LDO_VA12_OP_CFG */
	0x1668, 0x0002, 0x0002,/* LDO_VA18_OP_EN */
	0x166E, 0x0002, 0x0002,/* LDO_VA18_OP_CFG */
	0x167C, 0x0002, 0x0002,/* LDO_VUSB33_OP_EN */
	0x1682, 0x0002, 0x0002,/* LDO_VUSB33_OP_CFG */
	0x1690, 0x0001, 0x0000,/* LDO_VEMC_CON0 */
	0x1692, 0x0002, 0x0000,/* LDO_VEMC_OP_EN */
	0x16A6, 0x0002, 0x0002,/* LDO_VXO22_OP_EN */
	0x16AC, 0x0002, 0x0002,/* LDO_VXO22_OP_CFG */
	0x16BA, 0x0002, 0x0002,/* LDO_VXO18_OP_EN */
	0x16C0, 0x0002, 0x0000,/* LDO_VXO18_OP_CFG */
	0x16CC, 0x0001, 0x0000,/* LDO_VSIM1_CON0 */
	0x16CE, 0x0002, 0x0000,/* LDO_VSIM1_OP_EN */
	0x16E0, 0x0001, 0x0000,/* LDO_VSIM2_CON0 */
	0x16E2, 0x0002, 0x0000,/* LDO_VSIM2_OP_EN */
	0x16F4, 0x0001, 0x0000,/* LDO_VCAMD1_CON0 */
	0x16F6, 0x0002, 0x0000,/* LDO_VCAMD1_OP_EN */
	0x1708, 0x0001, 0x0000,/* LDO_VCAMD2_CON0 */
	0x170A, 0x0002, 0x0000,/* LDO_VCAMD2_OP_EN */
	0x171C, 0x0001, 0x0000,/* LDO_VCAMIO_CON0 */
	0x171E, 0x0002, 0x0000,/* LDO_VCAMIO_OP_EN */
	0x1732, 0x0004, 0x0004,/* LDO_VMIPI_OP_EN */
	0x1738, 0x0004, 0x0000,/* LDO_VMIPI_OP_CFG */
	0x1744, 0x0001, 0x0000,/* LDO_VGP_CON0 */
	0x1746, 0x0002, 0x0000,/* LDO_VGP_OP_EN */
	0x1758, 0x0001, 0x0000,/* LDO_VCN33_CON0_BT */
	0x175A, 0x0002, 0x0000,/* LDO_VCN33_OP_EN */
	0x1766, 0x0001, 0x0000,/* LDO_VCN33_CON0_WIFI */
	0x176E, 0x0001, 0x0000,/* LDO_VCN18_CON0 */
	0x1770, 0x0002, 0x0000,/* LDO_VCN18_OP_EN */
	0x1782, 0x0001, 0x0000,/* LDO_VCN28_CON0 */
	0x1784, 0x0002, 0x0000,/* LDO_VCN28_OP_EN */
	0x1796, 0x0001, 0x0000,/* LDO_VBIF28_CON0 */
	0x1798, 0x0002, 0x0000,/* LDO_VBIF28_OP_EN */
	0x17AC, 0x0004, 0x0004,/* LDO_VTCXO24_OP_EN */
	0x17B2, 0x0004, 0x0000,/* LDO_VTCXO24_OP_CFG */
	0x17C0, 0x0002, 0x0002,/* LDO_VLDO28_OP_EN */
	0x17C6, 0x0002, 0x0002,/* LDO_VLDO28_OP_CFG */
	0x17D4, 0x0001, 0x0000,/* LDO_VGP2_CON0 */
	0x17D6, 0x0002, 0x0000,/* LDO_VGP2_OP_EN */
	0x1802, 0x0004, 0x0004,/* LDO_VFE28_OP_EN */
	0x1808, 0x0004, 0x0000,/* LDO_VFE28_OP_CFG */
	0x1814, 0x0001, 0x0000,/* LDO_VMCH_CON0 */
	0x1816, 0x0002, 0x0000,/* LDO_VMCH_OP_EN */
	0x1828, 0x0001, 0x0000,/* LDO_VMC_CON0 */
	0x182A, 0x0002, 0x0000,/* LDO_VMC_OP_EN */
	0x183E, 0x0004, 0x0004,/* LDO_VRF18_1_OP_EN */
	0x1844, 0x0004, 0x0000,/* LDO_VRF18_1_OP_CFG */
	0x1852, 0x0004, 0x0004,/* LDO_VRF18_2_OP_EN */
	0x1858, 0x0004, 0x0000,/* LDO_VRF18_2_OP_CFG */
	0x1866, 0x0004, 0x0004,/* LDO_VRF12_OP_EN */
	0x186C, 0x0004, 0x0000,/* LDO_VRF12_OP_CFG */
	0x1878, 0x0001, 0x0000,/* LDO_VCAMA1_CON0 */
	0x188C, 0x0001, 0x0000,/* LDO_VCAMA2_CON0 */
	0x18B2, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_EN */
	0x18B8, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_CFG */
	0x18D2, 0x0002, 0x0002,/* LDO_VSRAM_CORE_OP_EN */
	0x18D8, 0x0002, 0x0002,/* LDO_VSRAM_CORE_OP_CFG */
	0x18E8, 0x0001, 0x0000,/* LDO_VSRAM_GPU_CON0 */
	0x18F2, 0x0002, 0x0000,/* LDO_VSRAM_GPU_OP_EN */
	0x1908, 0x0001, 0x0000,/* LDO_VSRAM_MD_CON0 */
	0x1912, 0x0002, 0x0000/* LDO_VSRAM_MD_OP_EN */
};

const unsigned int *MT6355_PMIC_REG_gs_suspend_32kless = MT6355_PMIC_REG_gs_suspend_32kless_data;

unsigned int MT6355_PMIC_REG_gs_suspend_32kless_len = 279;

const unsigned int MT6355_PMIC_REG_gs_sodi3_0_32kless_data[] = {
/* Address	 Mask		Golden Setting Value */
	0x0400, 0x0008, 0x0008,/* TOP_CKPDN_CON0 */
	0x040C, 0x0032, 0x0032,/* TOP_CKPDN_CON2 */
	0x0412, 0x0024, 0x0024,/* TOP_CKPDN_CON3 */
	0x1024, 0x0002, 0x0002,/* BUCK_VPROC11_OP_EN */
	0x102A, 0x0002, 0x0002,/* BUCK_VPROC11_OP_CFG */
	0x1046, 0x0002, 0x0002,/* BUCK_VPROC12_OP_EN */
	0x104C, 0x0002, 0x0002,/* BUCK_VPROC12_OP_CFG */
	0x105E, 0x0001, 0x0001,/* BUCK_VCORE_CON0 */
	0x1068, 0x0002, 0x0000,/* BUCK_VCORE_OP_EN */
	0x1082, 0x0001, 0x0000,/* BUCK_VGPU_CON0 */
	0x108C, 0x0002, 0x0000,/* BUCK_VGPU_OP_EN */
	0x10AE, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_EN */
	0x10B4, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_CFG */
	0x10D0, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_EN */
	0x10D6, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_CFG */
	0x10E8, 0x0001, 0x0000,/* BUCK_VMODEM_CON0 */
	0x10F2, 0x0002, 0x0000,/* BUCK_VMODEM_OP_EN */
	0x111C, 0x0002, 0x0002,/* BUCK_VS1_OP_EN */
	0x1122, 0x0002, 0x0002,/* BUCK_VS1_OP_CFG */
	0x1146, 0x0002, 0x0002,/* BUCK_VS2_OP_EN */
	0x114C, 0x0002, 0x0002,/* BUCK_VS2_OP_CFG */
	0x1166, 0x0001, 0x0000,/* BUCK_VPA_CON0 */
	0x1602, 0x0002, 0x0002,/* LDO_VIO28_OP_EN */
	0x1608, 0x0002, 0x0002,/* LDO_VIO28_OP_CFG */
	0x1616, 0x0002, 0x0002,/* LDO_VIO18_OP_EN */
	0x161C, 0x0002, 0x0002,/* LDO_VIO18_OP_CFG */
	0x162A, 0x0002, 0x0002,/* LDO_VUFS18_OP_EN */
	0x1630, 0x0002, 0x0002,/* LDO_VUFS18_OP_CFG */
	0x1640, 0x0002, 0x0002,/* LDO_VA10_OP_EN */
	0x1646, 0x0002, 0x0002,/* LDO_VA10_OP_CFG */
	0x1654, 0x0002, 0x0002,/* LDO_VA12_OP_EN */
	0x165A, 0x0002, 0x0002,/* LDO_VA12_OP_CFG */
	0x1668, 0x0002, 0x0002,/* LDO_VA18_OP_EN */
	0x166E, 0x0002, 0x0002,/* LDO_VA18_OP_CFG */
	0x167C, 0x0002, 0x0002,/* LDO_VUSB33_OP_EN */
	0x1682, 0x0002, 0x0002,/* LDO_VUSB33_OP_CFG */
	0x1690, 0x0001, 0x0000,/* LDO_VEMC_CON0 */
	0x1692, 0x0002, 0x0000,/* LDO_VEMC_OP_EN */
	0x16A6, 0x0002, 0x0002,/* LDO_VXO22_OP_EN */
	0x16AC, 0x0002, 0x0002,/* LDO_VXO22_OP_CFG */
	0x16BA, 0x0002, 0x0002,/* LDO_VXO18_OP_EN */
	0x16C0, 0x0002, 0x0000,/* LDO_VXO18_OP_CFG */
	0x16CC, 0x0001, 0x0000,/* LDO_VSIM1_CON0 */
	0x16CE, 0x0002, 0x0000,/* LDO_VSIM1_OP_EN */
	0x16E0, 0x0001, 0x0000,/* LDO_VSIM2_CON0 */
	0x16E2, 0x0002, 0x0000,/* LDO_VSIM2_OP_EN */
	0x16F4, 0x0001, 0x0000,/* LDO_VCAMD1_CON0 */
	0x16F6, 0x0002, 0x0000,/* LDO_VCAMD1_OP_EN */
	0x1708, 0x0001, 0x0000,/* LDO_VCAMD2_CON0 */
	0x170A, 0x0002, 0x0000,/* LDO_VCAMD2_OP_EN */
	0x171C, 0x0001, 0x0000,/* LDO_VCAMIO_CON0 */
	0x171E, 0x0002, 0x0000,/* LDO_VCAMIO_OP_EN */
	0x1732, 0x0004, 0x0004,/* LDO_VMIPI_OP_EN */
	0x1738, 0x0004, 0x0000,/* LDO_VMIPI_OP_CFG */
	0x1744, 0x0001, 0x0000,/* LDO_VGP_CON0 */
	0x1746, 0x0002, 0x0000,/* LDO_VGP_OP_EN */
	0x1758, 0x0001, 0x0000,/* LDO_VCN33_CON0_BT */
	0x175A, 0x0002, 0x0000,/* LDO_VCN33_OP_EN */
	0x1766, 0x0001, 0x0000,/* LDO_VCN33_CON0_WIFI */
	0x176E, 0x0001, 0x0000,/* LDO_VCN18_CON0 */
	0x1770, 0x0002, 0x0000,/* LDO_VCN18_OP_EN */
	0x1782, 0x0001, 0x0000,/* LDO_VCN28_CON0 */
	0x1784, 0x0002, 0x0000,/* LDO_VCN28_OP_EN */
	0x1796, 0x0001, 0x0000,/* LDO_VBIF28_CON0 */
	0x1798, 0x0002, 0x0000,/* LDO_VBIF28_OP_EN */
	0x17AC, 0x0004, 0x0004,/* LDO_VTCXO24_OP_EN */
	0x17B2, 0x0004, 0x0000,/* LDO_VTCXO24_OP_CFG */
	0x17BE, 0x0001, 0x0001,/* LDO_VLDO28_CON0_AF */
	0x17C0, 0x0002, 0x0000,/* LDO_VLDO28_OP_EN */
	0x17CC, 0x0001, 0x0001,/* LDO_VLDO28_CON0_TP */
	0x17D4, 0x0001, 0x0000,/* LDO_VGP2_CON0 */
	0x17D6, 0x0002, 0x0000,/* LDO_VGP2_OP_EN */
	0x1802, 0x0004, 0x0004,/* LDO_VFE28_OP_EN */
	0x1808, 0x0004, 0x0000,/* LDO_VFE28_OP_CFG */
	0x1814, 0x0001, 0x0000,/* LDO_VMCH_CON0 */
	0x1816, 0x0002, 0x0000,/* LDO_VMCH_OP_EN */
	0x1828, 0x0001, 0x0000,/* LDO_VMC_CON0 */
	0x182A, 0x0002, 0x0000,/* LDO_VMC_OP_EN */
	0x183E, 0x0004, 0x0004,/* LDO_VRF18_1_OP_EN */
	0x1844, 0x0004, 0x0000,/* LDO_VRF18_1_OP_CFG */
	0x1852, 0x0004, 0x0004,/* LDO_VRF18_2_OP_EN */
	0x1858, 0x0004, 0x0000,/* LDO_VRF18_2_OP_CFG */
	0x1866, 0x0004, 0x0004,/* LDO_VRF12_OP_EN */
	0x186C, 0x0004, 0x0000,/* LDO_VRF12_OP_CFG */
	0x1878, 0x0001, 0x0000,/* LDO_VCAMA1_CON0 */
	0x188C, 0x0001, 0x0000,/* LDO_VCAMA2_CON0 */
	0x18B2, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_EN */
	0x18B8, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_CFG */
	0x18C8, 0x0001, 0x0001,/* LDO_VSRAM_CORE_CON0 */
	0x18D2, 0x0002, 0x0000,/* LDO_VSRAM_CORE_OP_EN */
	0x18E8, 0x0001, 0x0000,/* LDO_VSRAM_GPU_CON0 */
	0x18F2, 0x0002, 0x0000,/* LDO_VSRAM_GPU_OP_EN */
	0x1908, 0x0001, 0x0000,/* LDO_VSRAM_MD_CON0 */
	0x1912, 0x0002, 0x0000/* LDO_VSRAM_MD_OP_EN */
};

const unsigned int *MT6355_PMIC_REG_gs_sodi3_0_32kless = MT6355_PMIC_REG_gs_sodi3_0_32kless_data;

unsigned int MT6355_PMIC_REG_gs_sodi3_0_32kless_len = 282;

const unsigned int MT6355_PMIC_REG_gs_deepidle_lp_mp3_data[] = {
/* Address	 Mask		Golden Setting Value */
	0x0400, 0x0008, 0x0008,/* TOP_CKPDN_CON0 */
	0x040C, 0x0032, 0x0032,/* TOP_CKPDN_CON2 */
	0x0412, 0x0024, 0x0024,/* TOP_CKPDN_CON3 */
	0x1024, 0x0008, 0x0008,/* BUCK_VPROC11_OP_EN */
	0x102A, 0x0008, 0x0008,/* BUCK_VPROC11_OP_CFG */
	0x1046, 0x0008, 0x0008,/* BUCK_VPROC12_OP_EN */
	0x104C, 0x0008, 0x0008,/* BUCK_VPROC12_OP_CFG */
	0x1068, 0x0008, 0x0008,/* BUCK_VCORE_OP_EN */
	0x106E, 0x0008, 0x0008,/* BUCK_VCORE_OP_CFG */
	0x1082, 0x0001, 0x0000,/* BUCK_VGPU_CON0 */
	0x108C, 0x0008, 0x0000,/* BUCK_VGPU_OP_EN */
	0x10AE, 0x0008, 0x0008,/* BUCK_VDRAM1_OP_EN */
	0x10B4, 0x0008, 0x0008,/* BUCK_VDRAM1_OP_CFG */
	0x10D0, 0x0008, 0x0008,/* BUCK_VDRAM2_OP_EN */
	0x10D6, 0x0008, 0x0008,/* BUCK_VDRAM2_OP_CFG */
	0x10E8, 0x0001, 0x0000,/* BUCK_VMODEM_CON0 */
	0x10F2, 0x0008, 0x0000,/* BUCK_VMODEM_OP_EN */
	0x111C, 0x0008, 0x0008,/* BUCK_VS1_OP_EN */
	0x1122, 0x0008, 0x0008,/* BUCK_VS1_OP_CFG */
	0x1146, 0x0008, 0x0008,/* BUCK_VS2_OP_EN */
	0x114C, 0x0008, 0x0008,/* BUCK_VS2_OP_CFG */
	0x1166, 0x0001, 0x0000,/* BUCK_VPA_CON0 */
	0x1602, 0x0008, 0x0008,/* LDO_VIO28_OP_EN */
	0x1608, 0x0008, 0x0008,/* LDO_VIO28_OP_CFG */
	0x1616, 0x0008, 0x0008,/* LDO_VIO18_OP_EN */
	0x161C, 0x0008, 0x0008,/* LDO_VIO18_OP_CFG */
	0x162A, 0x0008, 0x0008,/* LDO_VUFS18_OP_EN */
	0x1630, 0x0008, 0x0008,/* LDO_VUFS18_OP_CFG */
	0x1640, 0x0008, 0x0008,/* LDO_VA10_OP_EN */
	0x1646, 0x0008, 0x0008,/* LDO_VA10_OP_CFG */
	0x1654, 0x0008, 0x0008,/* LDO_VA12_OP_EN */
	0x165A, 0x0008, 0x0008,/* LDO_VA12_OP_CFG */
	0x1668, 0x0008, 0x0008,/* LDO_VA18_OP_EN */
	0x166E, 0x0008, 0x0008,/* LDO_VA18_OP_CFG */
	0x167C, 0x0008, 0x0008,/* LDO_VUSB33_OP_EN */
	0x1682, 0x0008, 0x0008,/* LDO_VUSB33_OP_CFG */
	0x1690, 0x0001, 0x0000,/* LDO_VEMC_CON0 */
	0x1692, 0x0008, 0x0000,/* LDO_VEMC_OP_EN */
	0x16A6, 0x0008, 0x0008,/* LDO_VXO22_OP_EN */
	0x16AC, 0x0008, 0x0008,/* LDO_VXO22_OP_CFG */
	0x16BA, 0x0008, 0x0008,/* LDO_VXO18_OP_EN */
	0x16C0, 0x0008, 0x0008,/* LDO_VXO18_OP_CFG */
	0x16CC, 0x0001, 0x0000,/* LDO_VSIM1_CON0 */
	0x16CE, 0x0008, 0x0000,/* LDO_VSIM1_OP_EN */
	0x16E0, 0x0001, 0x0000,/* LDO_VSIM2_CON0 */
	0x16E2, 0x0008, 0x0000,/* LDO_VSIM2_OP_EN */
	0x16F4, 0x0001, 0x0000,/* LDO_VCAMD1_CON0 */
	0x16F6, 0x0008, 0x0000,/* LDO_VCAMD1_OP_EN */
	0x1708, 0x0001, 0x0000,/* LDO_VCAMD2_CON0 */
	0x170A, 0x0008, 0x0000,/* LDO_VCAMD2_OP_EN */
	0x171C, 0x0001, 0x0000,/* LDO_VCAMIO_CON0 */
	0x171E, 0x0008, 0x0000,/* LDO_VCAMIO_OP_EN */
	0x1732, 0x0004, 0x0004,/* LDO_VMIPI_OP_EN */
	0x1738, 0x0004, 0x0000,/* LDO_VMIPI_OP_CFG */
	0x1744, 0x0001, 0x0000,/* LDO_VGP_CON0 */
	0x1746, 0x0008, 0x0000,/* LDO_VGP_OP_EN */
	0x1758, 0x0001, 0x0000,/* LDO_VCN33_CON0_BT */
	0x175A, 0x0008, 0x0000,/* LDO_VCN33_OP_EN */
	0x1766, 0x0001, 0x0000,/* LDO_VCN33_CON0_WIFI */
	0x176E, 0x0001, 0x0000,/* LDO_VCN18_CON0 */
	0x1770, 0x0008, 0x0000,/* LDO_VCN18_OP_EN */
	0x1782, 0x0001, 0x0000,/* LDO_VCN28_CON0 */
	0x1784, 0x0008, 0x0000,/* LDO_VCN28_OP_EN */
	0x1796, 0x0001, 0x0000,/* LDO_VBIF28_CON0 */
	0x1798, 0x0008, 0x0000,/* LDO_VBIF28_OP_EN */
	0x17AC, 0x0004, 0x0004,/* LDO_VTCXO24_OP_EN */
	0x17B2, 0x0004, 0x0000,/* LDO_VTCXO24_OP_CFG */
	0x17C0, 0x0008, 0x0008,/* LDO_VLDO28_OP_EN */
	0x17C6, 0x0008, 0x0008,/* LDO_VLDO28_OP_CFG */
	0x17D4, 0x0001, 0x0000,/* LDO_VGP2_CON0 */
	0x17D6, 0x0008, 0x0000,/* LDO_VGP2_OP_EN */
	0x1802, 0x0004, 0x0004,/* LDO_VFE28_OP_EN */
	0x1808, 0x0004, 0x0000,/* LDO_VFE28_OP_CFG */
	0x1814, 0x0001, 0x0000,/* LDO_VMCH_CON0 */
	0x1816, 0x0008, 0x0000,/* LDO_VMCH_OP_EN */
	0x1828, 0x0001, 0x0000,/* LDO_VMC_CON0 */
	0x182A, 0x0008, 0x0000,/* LDO_VMC_OP_EN */
	0x183E, 0x0004, 0x0004,/* LDO_VRF18_1_OP_EN */
	0x1844, 0x0004, 0x0000,/* LDO_VRF18_1_OP_CFG */
	0x1852, 0x0004, 0x0004,/* LDO_VRF18_2_OP_EN */
	0x1858, 0x0004, 0x0000,/* LDO_VRF18_2_OP_CFG */
	0x1866, 0x0004, 0x0004,/* LDO_VRF12_OP_EN */
	0x186C, 0x0004, 0x0000,/* LDO_VRF12_OP_CFG */
	0x1878, 0x0001, 0x0000,/* LDO_VCAMA1_CON0 */
	0x188C, 0x0001, 0x0000,/* LDO_VCAMA2_CON0 */
	0x18B2, 0x0008, 0x0008,/* LDO_VSRAM_PROC_OP_EN */
	0x18B8, 0x0008, 0x0008,/* LDO_VSRAM_PROC_OP_CFG */
	0x18D2, 0x0008, 0x0008,/* LDO_VSRAM_CORE_OP_EN */
	0x18D8, 0x0008, 0x0008,/* LDO_VSRAM_CORE_OP_CFG */
	0x18E8, 0x0001, 0x0000,/* LDO_VSRAM_GPU_CON0 */
	0x18F2, 0x0008, 0x0000,/* LDO_VSRAM_GPU_OP_EN */
	0x1908, 0x0001, 0x0000,/* LDO_VSRAM_MD_CON0 */
	0x1912, 0x0008, 0x0000/* LDO_VSRAM_MD_OP_EN */
};

const unsigned int *MT6355_PMIC_REG_gs_deepidle_lp_mp3 = MT6355_PMIC_REG_gs_deepidle_lp_mp3_data;

unsigned int MT6355_PMIC_REG_gs_deepidle_lp_mp3_len = 279;

const unsigned int MT6355_PMIC_REG_gs_suspend_data[] = {
/* Address	 Mask		Golden Setting Value */
	0x0400, 0x0008, 0x0008,/* TOP_CKPDN_CON0 */
	0x040C, 0x0032, 0x0032,/* TOP_CKPDN_CON2 */
	0x0412, 0x0024, 0x0024,/* TOP_CKPDN_CON3 */
	0x1024, 0x0002, 0x0002,/* BUCK_VPROC11_OP_EN */
	0x102A, 0x0002, 0x0002,/* BUCK_VPROC11_OP_CFG */
	0x1046, 0x0002, 0x0002,/* BUCK_VPROC12_OP_EN */
	0x104C, 0x0002, 0x0002,/* BUCK_VPROC12_OP_CFG */
	0x1068, 0x0002, 0x0002,/* BUCK_VCORE_OP_EN */
	0x106E, 0x0002, 0x0002,/* BUCK_VCORE_OP_CFG */
	0x1082, 0x0001, 0x0000,/* BUCK_VGPU_CON0 */
	0x108C, 0x0002, 0x0000,/* BUCK_VGPU_OP_EN */
	0x10AE, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_EN */
	0x10B4, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_CFG */
	0x10D0, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_EN */
	0x10D6, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_CFG */
	0x10E8, 0x0001, 0x0000,/* BUCK_VMODEM_CON0 */
	0x10F2, 0x0002, 0x0000,/* BUCK_VMODEM_OP_EN */
	0x111C, 0x0002, 0x0002,/* BUCK_VS1_OP_EN */
	0x1122, 0x0002, 0x0002,/* BUCK_VS1_OP_CFG */
	0x1146, 0x0002, 0x0002,/* BUCK_VS2_OP_EN */
	0x114C, 0x0002, 0x0002,/* BUCK_VS2_OP_CFG */
	0x1166, 0x0001, 0x0000,/* BUCK_VPA_CON0 */
	0x1602, 0x0002, 0x0002,/* LDO_VIO28_OP_EN */
	0x1608, 0x0002, 0x0002,/* LDO_VIO28_OP_CFG */
	0x1616, 0x0002, 0x0002,/* LDO_VIO18_OP_EN */
	0x161C, 0x0002, 0x0002,/* LDO_VIO18_OP_CFG */
	0x162A, 0x0002, 0x0002,/* LDO_VUFS18_OP_EN */
	0x1630, 0x0002, 0x0002,/* LDO_VUFS18_OP_CFG */
	0x1640, 0x0002, 0x0002,/* LDO_VA10_OP_EN */
	0x1646, 0x0002, 0x0002,/* LDO_VA10_OP_CFG */
	0x1654, 0x0002, 0x0002,/* LDO_VA12_OP_EN */
	0x165A, 0x0002, 0x0002,/* LDO_VA12_OP_CFG */
	0x1668, 0x0002, 0x0002,/* LDO_VA18_OP_EN */
	0x166E, 0x0002, 0x0002,/* LDO_VA18_OP_CFG */
	0x167C, 0x0002, 0x0002,/* LDO_VUSB33_OP_EN */
	0x1682, 0x0002, 0x0002,/* LDO_VUSB33_OP_CFG */
	0x1690, 0x0001, 0x0000,/* LDO_VEMC_CON0 */
	0x1692, 0x0002, 0x0000,/* LDO_VEMC_OP_EN */
	0x16A6, 0x0002, 0x0002,/* LDO_VXO22_OP_EN */
	0x16AC, 0x0002, 0x0000,/* LDO_VXO22_OP_CFG */
	0x16BA, 0x0002, 0x0002,/* LDO_VXO18_OP_EN */
	0x16C0, 0x0002, 0x0000,/* LDO_VXO18_OP_CFG */
	0x16CC, 0x0001, 0x0000,/* LDO_VSIM1_CON0 */
	0x16CE, 0x0002, 0x0000,/* LDO_VSIM1_OP_EN */
	0x16E0, 0x0001, 0x0000,/* LDO_VSIM2_CON0 */
	0x16E2, 0x0002, 0x0000,/* LDO_VSIM2_OP_EN */
	0x16F4, 0x0001, 0x0000,/* LDO_VCAMD1_CON0 */
	0x16F6, 0x0002, 0x0000,/* LDO_VCAMD1_OP_EN */
	0x1708, 0x0001, 0x0000,/* LDO_VCAMD2_CON0 */
	0x170A, 0x0002, 0x0000,/* LDO_VCAMD2_OP_EN */
	0x171C, 0x0001, 0x0000,/* LDO_VCAMIO_CON0 */
	0x171E, 0x0002, 0x0000,/* LDO_VCAMIO_OP_EN */
	0x1732, 0x0004, 0x0004,/* LDO_VMIPI_OP_EN */
	0x1738, 0x0004, 0x0000,/* LDO_VMIPI_OP_CFG */
	0x1744, 0x0001, 0x0000,/* LDO_VGP_CON0 */
	0x1746, 0x0002, 0x0000,/* LDO_VGP_OP_EN */
	0x1758, 0x0001, 0x0000,/* LDO_VCN33_CON0_BT */
	0x175A, 0x0002, 0x0000,/* LDO_VCN33_OP_EN */
	0x1766, 0x0001, 0x0000,/* LDO_VCN33_CON0_WIFI */
	0x176E, 0x0001, 0x0000,/* LDO_VCN18_CON0 */
	0x1770, 0x0002, 0x0000,/* LDO_VCN18_OP_EN */
	0x1782, 0x0001, 0x0000,/* LDO_VCN28_CON0 */
	0x1784, 0x0002, 0x0000,/* LDO_VCN28_OP_EN */
	0x1796, 0x0001, 0x0000,/* LDO_VBIF28_CON0 */
	0x1798, 0x0002, 0x0000,/* LDO_VBIF28_OP_EN */
	0x17AC, 0x0004, 0x0004,/* LDO_VTCXO24_OP_EN */
	0x17B2, 0x0004, 0x0000,/* LDO_VTCXO24_OP_CFG */
	0x17C0, 0x0002, 0x0002,/* LDO_VLDO28_OP_EN */
	0x17C6, 0x0002, 0x0002,/* LDO_VLDO28_OP_CFG */
	0x17D4, 0x0001, 0x0000,/* LDO_VGP2_CON0 */
	0x17D6, 0x0002, 0x0000,/* LDO_VGP2_OP_EN */
	0x1802, 0x0004, 0x0004,/* LDO_VFE28_OP_EN */
	0x1808, 0x0004, 0x0000,/* LDO_VFE28_OP_CFG */
	0x1814, 0x0001, 0x0000,/* LDO_VMCH_CON0 */
	0x1816, 0x0002, 0x0000,/* LDO_VMCH_OP_EN */
	0x1828, 0x0001, 0x0000,/* LDO_VMC_CON0 */
	0x182A, 0x0002, 0x0000,/* LDO_VMC_OP_EN */
	0x183E, 0x0004, 0x0004,/* LDO_VRF18_1_OP_EN */
	0x1844, 0x0004, 0x0000,/* LDO_VRF18_1_OP_CFG */
	0x1852, 0x0004, 0x0004,/* LDO_VRF18_2_OP_EN */
	0x1858, 0x0004, 0x0000,/* LDO_VRF18_2_OP_CFG */
	0x1866, 0x0004, 0x0004,/* LDO_VRF12_OP_EN */
	0x186C, 0x0004, 0x0000,/* LDO_VRF12_OP_CFG */
	0x1878, 0x0001, 0x0000,/* LDO_VCAMA1_CON0 */
	0x188C, 0x0001, 0x0000,/* LDO_VCAMA2_CON0 */
	0x18B2, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_EN */
	0x18B8, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_CFG */
	0x18D2, 0x0002, 0x0002,/* LDO_VSRAM_CORE_OP_EN */
	0x18D8, 0x0002, 0x0002,/* LDO_VSRAM_CORE_OP_CFG */
	0x18E8, 0x0001, 0x0000,/* LDO_VSRAM_GPU_CON0 */
	0x18F2, 0x0002, 0x0000,/* LDO_VSRAM_GPU_OP_EN */
	0x1908, 0x0001, 0x0000,/* LDO_VSRAM_MD_CON0 */
	0x1912, 0x0002, 0x0000/* LDO_VSRAM_MD_OP_EN */
};

const unsigned int *MT6355_PMIC_REG_gs_suspend = MT6355_PMIC_REG_gs_suspend_data;

unsigned int MT6355_PMIC_REG_gs_suspend_len = 279;

const unsigned int MT6355_PMIC_REG_gs_deepidle_lp_mp3_32kless_data[] = {
/* Address	 Mask		Golden Setting Value */
	0x0400, 0x0008, 0x0008,/* TOP_CKPDN_CON0 */
	0x040C, 0x0032, 0x0032,/* TOP_CKPDN_CON2 */
	0x0412, 0x0024, 0x0024,/* TOP_CKPDN_CON3 */
	0x1024, 0x0008, 0x0008,/* BUCK_VPROC11_OP_EN */
	0x102A, 0x0008, 0x0008,/* BUCK_VPROC11_OP_CFG */
	0x1046, 0x0008, 0x0008,/* BUCK_VPROC12_OP_EN */
	0x104C, 0x0008, 0x0008,/* BUCK_VPROC12_OP_CFG */
	0x1068, 0x0008, 0x0008,/* BUCK_VCORE_OP_EN */
	0x106E, 0x0008, 0x0008,/* BUCK_VCORE_OP_CFG */
	0x1082, 0x0001, 0x0000,/* BUCK_VGPU_CON0 */
	0x108C, 0x0008, 0x0000,/* BUCK_VGPU_OP_EN */
	0x10AE, 0x0008, 0x0008,/* BUCK_VDRAM1_OP_EN */
	0x10B4, 0x0008, 0x0008,/* BUCK_VDRAM1_OP_CFG */
	0x10D0, 0x0008, 0x0008,/* BUCK_VDRAM2_OP_EN */
	0x10D6, 0x0008, 0x0008,/* BUCK_VDRAM2_OP_CFG */
	0x10E8, 0x0001, 0x0000,/* BUCK_VMODEM_CON0 */
	0x10F2, 0x0008, 0x0000,/* BUCK_VMODEM_OP_EN */
	0x111C, 0x0008, 0x0008,/* BUCK_VS1_OP_EN */
	0x1122, 0x0008, 0x0008,/* BUCK_VS1_OP_CFG */
	0x1146, 0x0008, 0x0008,/* BUCK_VS2_OP_EN */
	0x114C, 0x0008, 0x0008,/* BUCK_VS2_OP_CFG */
	0x1166, 0x0001, 0x0000,/* BUCK_VPA_CON0 */
	0x1602, 0x0008, 0x0008,/* LDO_VIO28_OP_EN */
	0x1608, 0x0008, 0x0008,/* LDO_VIO28_OP_CFG */
	0x1616, 0x0008, 0x0008,/* LDO_VIO18_OP_EN */
	0x161C, 0x0008, 0x0008,/* LDO_VIO18_OP_CFG */
	0x162A, 0x0008, 0x0008,/* LDO_VUFS18_OP_EN */
	0x1630, 0x0008, 0x0008,/* LDO_VUFS18_OP_CFG */
	0x1640, 0x0008, 0x0008,/* LDO_VA10_OP_EN */
	0x1646, 0x0008, 0x0008,/* LDO_VA10_OP_CFG */
	0x1654, 0x0008, 0x0008,/* LDO_VA12_OP_EN */
	0x165A, 0x0008, 0x0008,/* LDO_VA12_OP_CFG */
	0x1668, 0x0008, 0x0008,/* LDO_VA18_OP_EN */
	0x166E, 0x0008, 0x0008,/* LDO_VA18_OP_CFG */
	0x167C, 0x0008, 0x0008,/* LDO_VUSB33_OP_EN */
	0x1682, 0x0008, 0x0008,/* LDO_VUSB33_OP_CFG */
	0x1690, 0x0001, 0x0000,/* LDO_VEMC_CON0 */
	0x1692, 0x0008, 0x0000,/* LDO_VEMC_OP_EN */
	0x16A6, 0x0008, 0x0008,/* LDO_VXO22_OP_EN */
	0x16AC, 0x0008, 0x0008,/* LDO_VXO22_OP_CFG */
	0x16BA, 0x0008, 0x0008,/* LDO_VXO18_OP_EN */
	0x16C0, 0x0008, 0x0008,/* LDO_VXO18_OP_CFG */
	0x16CC, 0x0001, 0x0000,/* LDO_VSIM1_CON0 */
	0x16CE, 0x0008, 0x0000,/* LDO_VSIM1_OP_EN */
	0x16E0, 0x0001, 0x0000,/* LDO_VSIM2_CON0 */
	0x16E2, 0x0008, 0x0000,/* LDO_VSIM2_OP_EN */
	0x16F4, 0x0001, 0x0000,/* LDO_VCAMD1_CON0 */
	0x16F6, 0x0008, 0x0000,/* LDO_VCAMD1_OP_EN */
	0x1708, 0x0001, 0x0000,/* LDO_VCAMD2_CON0 */
	0x170A, 0x0008, 0x0000,/* LDO_VCAMD2_OP_EN */
	0x171C, 0x0001, 0x0000,/* LDO_VCAMIO_CON0 */
	0x171E, 0x0008, 0x0000,/* LDO_VCAMIO_OP_EN */
	0x1732, 0x0004, 0x0004,/* LDO_VMIPI_OP_EN */
	0x1738, 0x0004, 0x0000,/* LDO_VMIPI_OP_CFG */
	0x1744, 0x0001, 0x0000,/* LDO_VGP_CON0 */
	0x1746, 0x0008, 0x0000,/* LDO_VGP_OP_EN */
	0x1758, 0x0001, 0x0000,/* LDO_VCN33_CON0_BT */
	0x175A, 0x0008, 0x0000,/* LDO_VCN33_OP_EN */
	0x1766, 0x0001, 0x0000,/* LDO_VCN33_CON0_WIFI */
	0x176E, 0x0001, 0x0000,/* LDO_VCN18_CON0 */
	0x1770, 0x0008, 0x0000,/* LDO_VCN18_OP_EN */
	0x1782, 0x0001, 0x0000,/* LDO_VCN28_CON0 */
	0x1784, 0x0008, 0x0000,/* LDO_VCN28_OP_EN */
	0x1796, 0x0001, 0x0000,/* LDO_VBIF28_CON0 */
	0x1798, 0x0008, 0x0000,/* LDO_VBIF28_OP_EN */
	0x17AC, 0x0004, 0x0004,/* LDO_VTCXO24_OP_EN */
	0x17B2, 0x0004, 0x0000,/* LDO_VTCXO24_OP_CFG */
	0x17C0, 0x0008, 0x0008,/* LDO_VLDO28_OP_EN */
	0x17C6, 0x0008, 0x0008,/* LDO_VLDO28_OP_CFG */
	0x17D4, 0x0001, 0x0000,/* LDO_VGP2_CON0 */
	0x17D6, 0x0008, 0x0000,/* LDO_VGP2_OP_EN */
	0x1802, 0x0004, 0x0004,/* LDO_VFE28_OP_EN */
	0x1808, 0x0004, 0x0000,/* LDO_VFE28_OP_CFG */
	0x1814, 0x0001, 0x0000,/* LDO_VMCH_CON0 */
	0x1816, 0x0008, 0x0000,/* LDO_VMCH_OP_EN */
	0x1828, 0x0001, 0x0000,/* LDO_VMC_CON0 */
	0x182A, 0x0008, 0x0000,/* LDO_VMC_OP_EN */
	0x183E, 0x0004, 0x0004,/* LDO_VRF18_1_OP_EN */
	0x1844, 0x0004, 0x0000,/* LDO_VRF18_1_OP_CFG */
	0x1852, 0x0004, 0x0004,/* LDO_VRF18_2_OP_EN */
	0x1858, 0x0004, 0x0000,/* LDO_VRF18_2_OP_CFG */
	0x1866, 0x0004, 0x0004,/* LDO_VRF12_OP_EN */
	0x186C, 0x0004, 0x0000,/* LDO_VRF12_OP_CFG */
	0x1878, 0x0001, 0x0000,/* LDO_VCAMA1_CON0 */
	0x188C, 0x0001, 0x0000,/* LDO_VCAMA2_CON0 */
	0x18B2, 0x0008, 0x0008,/* LDO_VSRAM_PROC_OP_EN */
	0x18B8, 0x0008, 0x0008,/* LDO_VSRAM_PROC_OP_CFG */
	0x18D2, 0x0008, 0x0008,/* LDO_VSRAM_CORE_OP_EN */
	0x18D8, 0x0008, 0x0008,/* LDO_VSRAM_CORE_OP_CFG */
	0x18E8, 0x0001, 0x0000,/* LDO_VSRAM_GPU_CON0 */
	0x18F2, 0x0008, 0x0000,/* LDO_VSRAM_GPU_OP_EN */
	0x1908, 0x0001, 0x0000,/* LDO_VSRAM_MD_CON0 */
	0x1912, 0x0008, 0x0000/* LDO_VSRAM_MD_OP_EN */
};

const unsigned int *MT6355_PMIC_REG_gs_deepidle_lp_mp3_32kless = MT6355_PMIC_REG_gs_deepidle_lp_mp3_32kless_data;

unsigned int MT6355_PMIC_REG_gs_deepidle_lp_mp3_32kless_len = 279;

const unsigned int MT6355_PMIC_REG_gs_sodi3_0_data[] = {
/* Address	 Mask		Golden Setting Value */
	0x0400, 0x0008, 0x0008,/* TOP_CKPDN_CON0 */
	0x040C, 0x0032, 0x0032,/* TOP_CKPDN_CON2 */
	0x0412, 0x0024, 0x0024,/* TOP_CKPDN_CON3 */
	0x1024, 0x0002, 0x0002,/* BUCK_VPROC11_OP_EN */
	0x102A, 0x0002, 0x0002,/* BUCK_VPROC11_OP_CFG */
	0x1046, 0x0002, 0x0002,/* BUCK_VPROC12_OP_EN */
	0x104C, 0x0002, 0x0002,/* BUCK_VPROC12_OP_CFG */
	0x105E, 0x0001, 0x0001,/* BUCK_VCORE_CON0 */
	0x1068, 0x0002, 0x0000,/* BUCK_VCORE_OP_EN */
	0x1082, 0x0001, 0x0000,/* BUCK_VGPU_CON0 */
	0x108C, 0x0002, 0x0000,/* BUCK_VGPU_OP_EN */
	0x10AE, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_EN */
	0x10B4, 0x0002, 0x0002,/* BUCK_VDRAM1_OP_CFG */
	0x10D0, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_EN */
	0x10D6, 0x0002, 0x0002,/* BUCK_VDRAM2_OP_CFG */
	0x10E8, 0x0001, 0x0000,/* BUCK_VMODEM_CON0 */
	0x10F2, 0x0002, 0x0000,/* BUCK_VMODEM_OP_EN */
	0x111C, 0x0002, 0x0002,/* BUCK_VS1_OP_EN */
	0x1122, 0x0002, 0x0002,/* BUCK_VS1_OP_CFG */
	0x1146, 0x0002, 0x0002,/* BUCK_VS2_OP_EN */
	0x114C, 0x0002, 0x0002,/* BUCK_VS2_OP_CFG */
	0x1166, 0x0001, 0x0000,/* BUCK_VPA_CON0 */
	0x1602, 0x0002, 0x0002,/* LDO_VIO28_OP_EN */
	0x1608, 0x0002, 0x0002,/* LDO_VIO28_OP_CFG */
	0x1616, 0x0002, 0x0002,/* LDO_VIO18_OP_EN */
	0x161C, 0x0002, 0x0002,/* LDO_VIO18_OP_CFG */
	0x162A, 0x0002, 0x0002,/* LDO_VUFS18_OP_EN */
	0x1630, 0x0002, 0x0002,/* LDO_VUFS18_OP_CFG */
	0x1640, 0x0002, 0x0002,/* LDO_VA10_OP_EN */
	0x1646, 0x0002, 0x0002,/* LDO_VA10_OP_CFG */
	0x1654, 0x0002, 0x0002,/* LDO_VA12_OP_EN */
	0x165A, 0x0002, 0x0002,/* LDO_VA12_OP_CFG */
	0x1668, 0x0002, 0x0002,/* LDO_VA18_OP_EN */
	0x166E, 0x0002, 0x0002,/* LDO_VA18_OP_CFG */
	0x167C, 0x0002, 0x0002,/* LDO_VUSB33_OP_EN */
	0x1682, 0x0002, 0x0002,/* LDO_VUSB33_OP_CFG */
	0x1690, 0x0001, 0x0000,/* LDO_VEMC_CON0 */
	0x1692, 0x0002, 0x0000,/* LDO_VEMC_OP_EN */
	0x16A6, 0x0002, 0x0002,/* LDO_VXO22_OP_EN */
	0x16AC, 0x0002, 0x0000,/* LDO_VXO22_OP_CFG */
	0x16BA, 0x0002, 0x0002,/* LDO_VXO18_OP_EN */
	0x16C0, 0x0002, 0x0000,/* LDO_VXO18_OP_CFG */
	0x16CC, 0x0001, 0x0000,/* LDO_VSIM1_CON0 */
	0x16CE, 0x0002, 0x0000,/* LDO_VSIM1_OP_EN */
	0x16E0, 0x0001, 0x0000,/* LDO_VSIM2_CON0 */
	0x16E2, 0x0002, 0x0000,/* LDO_VSIM2_OP_EN */
	0x16F4, 0x0001, 0x0000,/* LDO_VCAMD1_CON0 */
	0x16F6, 0x0002, 0x0000,/* LDO_VCAMD1_OP_EN */
	0x1708, 0x0001, 0x0000,/* LDO_VCAMD2_CON0 */
	0x170A, 0x0002, 0x0000,/* LDO_VCAMD2_OP_EN */
	0x171C, 0x0001, 0x0000,/* LDO_VCAMIO_CON0 */
	0x171E, 0x0002, 0x0000,/* LDO_VCAMIO_OP_EN */
	0x1732, 0x0004, 0x0004,/* LDO_VMIPI_OP_EN */
	0x1738, 0x0004, 0x0000,/* LDO_VMIPI_OP_CFG */
	0x1744, 0x0001, 0x0000,/* LDO_VGP_CON0 */
	0x1746, 0x0002, 0x0000,/* LDO_VGP_OP_EN */
	0x1758, 0x0001, 0x0000,/* LDO_VCN33_CON0_BT */
	0x175A, 0x0002, 0x0000,/* LDO_VCN33_OP_EN */
	0x1766, 0x0001, 0x0000,/* LDO_VCN33_CON0_WIFI */
	0x176E, 0x0001, 0x0000,/* LDO_VCN18_CON0 */
	0x1770, 0x0002, 0x0000,/* LDO_VCN18_OP_EN */
	0x1782, 0x0001, 0x0000,/* LDO_VCN28_CON0 */
	0x1784, 0x0002, 0x0000,/* LDO_VCN28_OP_EN */
	0x1796, 0x0001, 0x0000,/* LDO_VBIF28_CON0 */
	0x1798, 0x0002, 0x0000,/* LDO_VBIF28_OP_EN */
	0x17AC, 0x0004, 0x0004,/* LDO_VTCXO24_OP_EN */
	0x17B2, 0x0004, 0x0000,/* LDO_VTCXO24_OP_CFG */
	0x17BE, 0x0001, 0x0001,/* LDO_VLDO28_CON0_AF */
	0x17C0, 0x0002, 0x0000,/* LDO_VLDO28_OP_EN */
	0x17CC, 0x0001, 0x0001,/* LDO_VLDO28_CON0_TP */
	0x17D4, 0x0001, 0x0000,/* LDO_VGP2_CON0 */
	0x17D6, 0x0002, 0x0000,/* LDO_VGP2_OP_EN */
	0x1802, 0x0004, 0x0004,/* LDO_VFE28_OP_EN */
	0x1808, 0x0004, 0x0000,/* LDO_VFE28_OP_CFG */
	0x1814, 0x0001, 0x0000,/* LDO_VMCH_CON0 */
	0x1816, 0x0002, 0x0000,/* LDO_VMCH_OP_EN */
	0x1828, 0x0001, 0x0000,/* LDO_VMC_CON0 */
	0x182A, 0x0002, 0x0000,/* LDO_VMC_OP_EN */
	0x183E, 0x0004, 0x0004,/* LDO_VRF18_1_OP_EN */
	0x1844, 0x0004, 0x0000,/* LDO_VRF18_1_OP_CFG */
	0x1852, 0x0004, 0x0004,/* LDO_VRF18_2_OP_EN */
	0x1858, 0x0004, 0x0000,/* LDO_VRF18_2_OP_CFG */
	0x1866, 0x0004, 0x0004,/* LDO_VRF12_OP_EN */
	0x186C, 0x0004, 0x0000,/* LDO_VRF12_OP_CFG */
	0x1878, 0x0001, 0x0000,/* LDO_VCAMA1_CON0 */
	0x188C, 0x0001, 0x0000,/* LDO_VCAMA2_CON0 */
	0x18B2, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_EN */
	0x18B8, 0x0002, 0x0002,/* LDO_VSRAM_PROC_OP_CFG */
	0x18C8, 0x0001, 0x0001,/* LDO_VSRAM_CORE_CON0 */
	0x18D2, 0x0002, 0x0000,/* LDO_VSRAM_CORE_OP_EN */
	0x18E8, 0x0001, 0x0000,/* LDO_VSRAM_GPU_CON0 */
	0x18F2, 0x0002, 0x0000,/* LDO_VSRAM_GPU_OP_EN */
	0x1908, 0x0001, 0x0000,/* LDO_VSRAM_MD_CON0 */
	0x1912, 0x0002, 0x0000/* LDO_VSRAM_MD_OP_EN */
};

const unsigned int *MT6355_PMIC_REG_gs_sodi3_0 = MT6355_PMIC_REG_gs_sodi3_0_data;

unsigned int MT6355_PMIC_REG_gs_sodi3_0_len = 282;

const unsigned int MT6355_PMIC_REG_gs_w_key_data[] = {
/* Address	 Mask		Golden Setting Value */
	0x0E3E, 0x1F03, 0x0000,/* CPSCFG0 */
	0x0E40, 0x1F03, 0x0000,/* CPSCFG1 */
	0x0E42, 0x1F1F, 0x0000,/* CPSPSA0 */
	0x0E44, 0x1F1F, 0x0000,/* CPSPSA1 */
	0x0E46, 0x1F1F, 0x0000,/* CPSPSA2 */
	0x0E48, 0x1F1F, 0x0000,/* CPSPSA3 */
	0x0E4A, 0x1F1F, 0x0000,/* CPSPSA4 */
	0x0E4C, 0x1F1F, 0x0000,/* CPSPSA5 */
	0x0E4E, 0x1F1F, 0x0000,/* CPSPSA6 */
	0x0E50, 0x1F1F, 0x0000,/* CPSPSA7 */
	0x0E52, 0x1F1F, 0x0000,/* CPSPSA8 */
	0x0E54, 0x1F1F, 0x0000,/* CPSPSA9 */
	0x0E56, 0x1F1F, 0x0000,/* CPSPSA10 */
	0x0E58, 0x1F1F, 0x0000,/* CPSPSA11 */
	0x0E5A, 0x1F1F, 0x0000,/* CPSDSA0 */
	0x0E5C, 0x1F1F, 0x0000,/* CPSDSA1 */
	0x0E5E, 0x1F1F, 0x0000,/* CPSDSA2 */
	0x0E60, 0x1F1F, 0x0000,/* CPSDSA3 */
	0x0E62, 0x1F1F, 0x0000,/* CPSDSA4 */
	0x0E64, 0x1F1F, 0x0000,/* CPSDSA5 */
	0x0E66, 0x1F1F, 0x0000,/* CPSDSA6 */
	0x0E68, 0x1F1F, 0x0000,/* CPSDSA7 */
	0x0E6A, 0x1F1F, 0x0000,/* CPSDSA8 */
	0x0E6C, 0x1F1F, 0x0000,/* CPSDSA9 */
	0x0E6E, 0x1F1F, 0x0000,/* CPSDSA10 */
	0x0E70, 0x1F1F, 0x0000,/* CPSDSA11 */
	0x0E72, 0x0001, 0x0000,/* PORFLAG */
};

const unsigned int *MT6355_PMIC_REG_gs_w_key = MT6355_PMIC_REG_gs_w_key_data;

unsigned int MT6355_PMIC_REG_gs_w_key_len = 81;
