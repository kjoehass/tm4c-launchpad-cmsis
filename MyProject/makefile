PROJ    := main

COBJS  := $(patsubst %.c,%.o,$(wildcard src/*.c))
SOBJS  := $(patsubst %.s,%.o,$(wildcard src/*.s))
INCS   := $(wildcard inc/*.h)

TOOLDIR = arm-none-eabi-
AS      = $(TOOLDIR)as
CC      = $(TOOLDIR)gcc
LD      = $(TOOLDIR)ld
OBJDUMP = $(TOOLDIR)objdump

PROCARCH := -mcpu=cortex-m4 -mthumb
FPU      := -mfloat-abi=hard -mfpu=fpv4-sp-d16

ASFLAGS  := -g $(PROCARCH)
CFLAGS   := -g3 -ggdb -std=gnu99 -Wall -fmessage-length=0 $(PROCARCH) $(FPU)
LDFLAGS  := -marmelf -nostartfiles -nostdlibs -Ttm4c123xh6xx.ld -Map=$(PROJ).map
OBJFLAGS := -S
INCLUDES := -I"inc" $(INCDIRS)

RM := rm -rf

LIBS :=

# All Target
all: $(PROJ).elf

# Tool invocations
$(PROJ).elf: $(COBJS) $(SOBJS)
	@echo "  Building target: $@"
	@$(LD) $(LDFLAGS) $(LIBDIRS) -o"$@" $(COBJS) $(SOBJS) $(LIBS)
	@$(OBJDUMP) $(OBJFLAGS) $(PROJ).elf > $(PROJ).lst
	
$(COBJS): %.o: %.c ${INCS} makefile
	@echo "  Compiling file: $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c -o"$@" "$<"

$(SOBJS) : %.o : %.s makefile
	@echo "  Assembling $<"
	@$(AS) $(ASFLAGS) "$<" -o "$@"

clean:
	-$(RM) src/*.o $(PROJ).elf $(PROJ).lst $(PROJ).map

.PHONY: all clean
