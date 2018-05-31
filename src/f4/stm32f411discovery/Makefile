##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = stm32-roboclaw

OBJS = roboclaw.o exti.o timers.o usart.o systick.o encoder.o motor.o filter.o

OOCD_INTERFACE = stlink-v2

LDSCRIPT = ../../../libopencm3-plus/lib/libopencm3_plus_stm32f411discovery.ld

LDLIBS += -lopencm3_plus_stm32f411discovery 
LDLIBS += -lm # for math.h

include ../Makefile.include
