/*
 * Copyright (C) 2016 Alexey Bolshakov <ua3mqj@gmail.com>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/* Linker script for STM32F100x8, 64K flash, 8K RAM. */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08002000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f1.ld

