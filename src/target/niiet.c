/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2025 Kits-Lab-Dev
 * Written based on K1921VG015 OpenOCD driver.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "riscv_debug.h"
#include "jep106.h"
#include "gdb_packet.h"

#define FLASH_DRIVER_VER 0x00010000
/*==============================================================================
 *                    K1921VG015 CONTROL REGS
 *==============================================================================
 */
/*-- PMUSYS ---------------------------------------------------------------------*/
#define PMUSYS_CHIPID_K1921VG015 0xdeadbee0
#define PMUSYS_BASE              0x3000F000
#define PMUSYS_SERVCTL           (PMUSYS_BASE + 0x104)
#define PMUSYS_CHIPID            (PMUSYS_BASE + 0x100)
/*---- PMUSYS->SERVCTL: Service mode control register */
#define PMUSYS_SERVCTL_DONE   (1 << 8) /* Full clear done flag */
#define PMUSYS_SERVCTL_SERVEN (1 << 0) /* Service mode enable flag */
#define MAIN_REGION           0
#define NVR_REGION            1
/*-- MFLASH ------------------------------------------------------------------*/
#define MFLASH_PAGE_SIZE  4096
#define MFLASH_PAGE_TOTAL 256
#define MFLASH_WORD_WIDTH 4
#define MFLASH_BASE       0x3000D000
#define MFLASH_BANK_ADDR  0x80000000
#define MFLASH_ADDR       (MFLASH_BASE + 0x00)
#define MFLASH_DATA0      (MFLASH_BASE + 0x04)
#define MFLASH_DATA1      (MFLASH_BASE + 0x08)
#define MFLASH_DATA2      (MFLASH_BASE + 0x0C)
#define MFLASH_DATA3      (MFLASH_BASE + 0x10)
#define MFLASH_CMD        (MFLASH_BASE + 0x44)
#define MFLASH_STAT       (MFLASH_BASE + 0x48)
/*---- MFLASH->CMD: Command register */
#define MFLASH_CMD_RD    (1 << 0)       /* Read data in region */
#define MFLASH_CMD_WR    (1 << 1)       /* Write data in region */
#define MFLASH_CMD_ERSEC (1 << 2)       /* Sector erase in region */
#define MFLASH_CMD_ERALL (1 << 3)       /* Erase all sectors in region */
#define MFLASH_CMD_NVRON (1 << 8)       /* Select NVR region for command operation */
#define MFLASH_CMD_KEY   (0xC0DE << 16) /* Command enable key */
/*---- MFLASH->STAT: Status register */
#define MFLASH_STAT_BUSY (1 << 0) /* Flag operation busy */
/*---- CFGWORD (in MFLASH NVR)----------------------------------------------- */
#define CFGWORD_PAGE        1
#define CFGWORD_ADDR_OFFSET 0xFF0
#define CFGWORD_ADDR        (MFLASH_PAGE_SIZE * CFGWORD_PAGE + CFGWORD_ADDR_OFFSET)
#define CFGWORD_JTAGEN      (1 << 2) /* Enable JTAG interface */
#define CFGWORD_CFGWE       (1 << 1) /* MFLASH NVR region write enable */
#define CFGWORD_FLASHWE     (1 << 0) /* MFLASH main region write enable */

#define RSTSYS 0x3000E0C0
#define RSTKEY 0xA55A0001

#define K1921VG015_RAM_BASE 0x40000000
#define K1921VG015_RAM_SIZE 0x40000

static bool k1921vg015_flash_erase(target_flash_s *flash, target_addr_t addr, size_t len);
static bool k1921vg015_flash_write(target_flash_s *flash, target_addr_t dest, const void *src, size_t len);

static bool k1921vg015_mass_erase(target_s *target, platform_timeout_s *timeout);

static bool k1921vg015_flash_waitdone(target_flash_s *flash);

static bool k1921vg015_attach(target_s *target);
static void k1921vg015_detach(target_s *target);

static void k1921vg015_add_flash(target_s *target)
{
	target_flash_s *flash = calloc(1, sizeof(*flash));
	if (!flash) {
		DEBUG_ERROR("calloc failed in %s\n", __func__);
		return;
	}
	memset(flash, 0, sizeof(*flash));
	flash->start = MFLASH_BANK_ADDR;
	flash->length = MFLASH_PAGE_SIZE * MFLASH_PAGE_TOTAL;
	flash->blocksize = MFLASH_PAGE_SIZE;
	flash->writesize = 16U; /* Can write 4 words (16 bytes) at a time */
	flash->erase = k1921vg015_flash_erase;
	flash->write = k1921vg015_flash_write;
	flash->done = k1921vg015_flash_waitdone;
	flash->erased = 0xffU;
	target_add_flash(target, flash);
}

static bool k1921vg015_flash_waitdone(target_flash_s *flash)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, 5000);
	target_s *target = flash->t;
	uint32_t status;
	do {
		status = target_mem32_read32(target, MFLASH_STAT);
		if (target_check_error(target))
			return false;

		if (platform_timeout_is_expired(&timeout)) {
			DEBUG_ERROR("Flash timeout\n");
			return false;
		}
	} while (status & MFLASH_STAT_BUSY);
	return true;
}

static bool k1921vg015_flash_erase(target_flash_s *flash, target_addr_t addr, size_t len)
{
	(void)len;
	target_s *target = flash->t;
	const uint32_t page = (addr - MFLASH_BANK_ADDR) / MFLASH_PAGE_SIZE;

	uint32_t cmd = MFLASH_CMD_KEY | MFLASH_CMD_ERSEC;
	target_mem32_write32(target, MFLASH_ADDR, page * MFLASH_PAGE_SIZE);
	target_mem32_write32(target, MFLASH_CMD, cmd);

	return k1921vg015_flash_waitdone(flash);
}

static bool k1921vg015_mass_erase(target_s *target, platform_timeout_s *timeout)
{
	(void)timeout;

	uint32_t servctl = target_mem32_read32(target, PMUSYS_SERVCTL);
	if (target_check_error(target)) {
		DEBUG_ERROR("K1921VG015: Failed to read PMUSYS_SERVCTL\n");
		return false;
	}

	if (servctl & PMUSYS_SERVCTL_SERVEN) {
		DEBUG_INFO("K1921VG015: Service mode detected — performing service erase\n");

		target_mem32_write32(target, PMUSYS_SERVCTL, PMUSYS_SERVCTL_DONE);

		platform_timeout_s local_timeout;
		platform_timeout_set(&local_timeout, 5000); /* 5 секунд */

		servctl = target_mem32_read32(target, PMUSYS_SERVCTL);
		while ((servctl & PMUSYS_SERVCTL_DONE) == 0) {
			if (platform_timeout_is_expired(&local_timeout)) {
				DEBUG_ERROR("K1921VG015: Service erase timeout\n");
				return false;
			}
			platform_delay(1);
			servctl = target_mem32_read32(target, PMUSYS_SERVCTL);
			if (target_check_error(target))
				return false;
		}
		return true;
	} else {
		DEBUG_INFO("K1921VG015: Normal mode — performing standard mass erase\n");

		uint32_t cmd = MFLASH_CMD_KEY | MFLASH_CMD_ERALL | MFLASH_CMD_ERSEC;
		target_mem32_write32(target, MFLASH_CMD, cmd);

		return k1921vg015_flash_waitdone(target->flash);
	}
}

static bool k1921vg015_flash_write(target_flash_s *flash, target_addr_t dest, const void *src, size_t len)
{
	target_s *target = flash->t;
	target_mem32_write32(target, MFLASH_ADDR, dest);
	target_mem32_write(target, MFLASH_DATA0, src, len);
	uint32_t cmd = MFLASH_CMD_KEY | MFLASH_CMD_WR;
	target_mem32_write32(target, MFLASH_CMD, cmd);

	return k1921vg015_flash_waitdone(flash);
}

static bool k1921vg015_nvr_erase_sector(target_s *target, uint32_t page)
{
	uint32_t cmd = MFLASH_CMD_KEY | MFLASH_CMD_ERSEC | MFLASH_CMD_NVRON;
	target_mem32_write32(target, MFLASH_ADDR, page * MFLASH_PAGE_SIZE);
	target_mem32_write32(target, MFLASH_CMD, cmd);

	return k1921vg015_flash_waitdone(target->flash);
}

static bool k1921vg015_flash_unlock(target_s *target)
{
	/* Считываем текущий CFGWORD */
	uint32_t flash_cmd = MFLASH_CMD_KEY | MFLASH_CMD_RD | MFLASH_CMD_NVRON;
	target_mem32_write32(target, MFLASH_ADDR, CFGWORD_ADDR);
	target_mem32_write32(target, MFLASH_CMD, flash_cmd);

	k1921vg015_flash_waitdone(target->flash);

	uint32_t cfgword = target_mem32_read32(target, MFLASH_DATA0);
	if (target_check_error(target)) {
		DEBUG_ERROR("Failed to read CFGWORD\n");
		return false;
	}

	DEBUG_INFO("CFGWORD = 0x%08" PRIx32 "\n", cfgword);

	/* Если FLASHWE уже разрешён — ничего не делаем */
	if (cfgword & CFGWORD_FLASHWE)
		return true;

	DEBUG_WARN("Flash write protection enabled. Erasing CFGWORD sector to unlock...\n");

	/* Стираем сектор 1 (NVR), где находится CFGWORD */
	if (!k1921vg015_nvr_erase_sector(target, CFGWORD_PAGE)) {
		DEBUG_ERROR("Failed to erase NVR sector %u\n", CFGWORD_PAGE);
		return false;
	}

	/* После стирания CFGWORD = 0xFFFFFFFF → FLASHWE = 1 */
	DEBUG_INFO("Flash unlocked successfully.\n");
	return true;
}

static bool k1921vg015_software_reset(target_s *target)
{
	uint32_t rst_key = RSTKEY;
	target_mem32_write(target, RSTSYS, &rst_key, sizeof(rst_key));
	return true;
}

static bool sw_reset(target_s *target, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	k1921vg015_software_reset(target);
	target->attach(target);
	return true;
}

static bool hw_reset(target_s *target, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	target->reset(target);
	return true;
}

const command_s cmd[] = {
	{"swreset", sw_reset, "Software reset"}, {"hwreset", hw_reset, "Hardware reset"}, {NULL, NULL, NULL}};

void (*reset)(target_s *target);

void niiet_reset(target_s *const target)
{
	k1921vg015_software_reset(target);
	reset(target);
}

bool niiet_probe(target_s *target)
{
	uint32_t chipid = target_mem32_read32(target, PMUSYS_CHIPID);
	chipid &= 0xFFFFFFF0U; /* Mask revision bits */
	if (chipid != PMUSYS_CHIPID_K1921VG015)
		return false;

	target->driver = "K1921VG015";
	// reset = target->reset;
	// target->reset = niiet_reset;
	target->mass_erase = k1921vg015_mass_erase;
	target->attach = k1921vg015_attach;
	target->detach = k1921vg015_detach;

	target_mem_map_free(target);
	target_add_ram32(target, K1921VG015_RAM_BASE, K1921VG015_RAM_SIZE);
	k1921vg015_add_flash(target);
	target_add_commands(target, cmd, target->driver);

	uint32_t service_mode = target_mem32_read32(target, PMUSYS_SERVCTL);
	service_mode = service_mode & PMUSYS_SERVCTL_SERVEN ? 1 : 0;
	
	if (service_mode) {
		gdb_out("K1921VG015 service mode enabled!\n");
	} else {
		if (!k1921vg015_flash_unlock(target))
			gdb_out("WARNING: error unlock flash!\n");

		k1921vg015_software_reset(target);
	}

	// platform_delay(50);

	return true;
}

static bool k1921vg015_attach(target_s *target)
{
	return riscv_attach(target);
}

static void k1921vg015_detach(target_s *target)
{
	riscv_detach(target);
}