# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Extended and rewritten by B. Mouritsen <bnmguy@gmail.com>
# For GC32F303RE Arduino core using the MFL C++ library

"""
Arduino

Arduino Wiring-based Framework allows writing cross-platform software to
control devices attached to a wide range of Arduino boards to create all
kinds of creative coding, interactive objects, spaces or physical experiences.

"""

from os.path import isfile, isdir, join
from SCons.Script import COMMAND_LINE_TARGETS, DefaultEnvironment

# get environment
env = DefaultEnvironment()
platform = env.PioPlatform()
board_config = env.BoardConfig()

# check framework is installed
FRAMEWORK_DIR = platform.get_package_dir("framework-arduino-mfl")
assert isdir(FRAMEWORK_DIR)

# get mcu and board variant
mcu = board_config.get("build.mcu", "")
board_name = env.subst("$BOARD")
mcu_type = mcu[:-2]
variant = board_config.get("build.variant")
series = mcu_type[:7].upper() + "x"
variants_dir = (
	join("$PROJECT_DIR", board_config.get("build.variants_dir"))
	if board_config.get("build.variants_dir", "")
	else join(FRAMEWORK_DIR, "variants")
)
variant_dir = join(variants_dir, variant)

def process_standard_library_configuration(cpp_defines):
	if "PIO_FRAMEWORK_ARDUINO_STANDARD_LIB" in cpp_defines:
		env["LINKFLAGS"].remove("--specs=nano.specs")
	if "PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_PRINTF" in cpp_defines:
		env.Append(LINKFLAGS=["-u_printf_float"])
	if "PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF" in cpp_defines:
		env.Append(LINKFLAGS=["-u_scanf_float"])

def process_usart_configuration(cpp_defines):
	if "PIO_FRAMEWORK_ARDUINO_SERIAL_DISABLED" in cpp_defines:
		env["CPPDEFINES"].remove("MFL_USART_ENABLE")

	elif "PIO_FRAMEWORK_ARDUINO_SERIAL_WITHOUT_GENERIC" in cpp_defines:
		env.Append(CPPDEFINES=["HWSERIAL_NONE"])

def get_arm_math_lib(cpu_core):
	cpu_core = board_config.get("build.cpu")
	if "m4" in cpu_core:
		return "arm_cortexM4lf_math"

	return "arm_cortex%sl_math" % cpu_core[7:9].upper()

env.Append(
	CPPDEFINES=[
		("VECT_TAB_OFFSET", board_config.get("build.offset", ""))
	],
)

# LD_FLASH_OFFSET is mandatory even if there is no offset
env.Append(
	LINKFLAGS=[
		"-Wl,--defsym=LD_FLASH_OFFSET=%s"
		% board_config.get("build.offset", "")
	]
)

machine_flags = [
	"-mcpu=%s" % board_config.get("build.cpu"),
	"-mthumb",
]

#
# Some GD32F303RET6 chips shipped without an FPU and there doesn't
# seem to be an easy way to check
#
# TODO:
#	Do a check somehow?
#
# For now, if you use this chip and are certain it has an FPU,
# you may uncomment the following line of code (Note: Enabling this when no FPU exists is undefined)
#
if board_config.get("build.cpu") == "cortex-m4":
    machine_flags.extend(["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"])

maximum_ram_size = board_config.get("upload.maximum_ram_size")

env.Append(
	ASFLAGS=machine_flags,
	ASPPFLAGS=[
	],
	CFLAGS=[
		"-std=gnu11",
	],
	CXXFLAGS=[
		"-std=gnu++23",
		"-fno-threadsafe-statics",
		"-fno-rtti",
		"-fno-exceptions",
		"-fno-use-cxa-atexit",
	],
	CCFLAGS=machine_flags + [
		"-Os",
		"--specs=nosys.specs",
		"-ffunction-sections",
		"-fdata-sections",
		"-Wall",
		"-nostdlib",
		"--param", "max-inline-insns-single=500",
		"--specs=nano.specs",
	],
	CPPDEFINES=[
		series,
		("ARDUINO", 10000),
		"ARDUINO_ARCH_MFL",
		"ARDUINO_%s" % board_name.upper(),
		("BOARD_NAME", '\\"%s\\"' % board_name.upper()),
		("F_CPU", "$BOARD_F_CPU"),
		("ARDUINO_UPLOAD_MAXIMUM_SIZE", board_config.get("upload.maximum_size")),
	],
	CPPPATH=[
		join(FRAMEWORK_DIR, "cores", "arduino", "api", "deprecated"),
		join(FRAMEWORK_DIR, "cores", "arduino", "api", "deprecated-avr-comp"),
		join(FRAMEWORK_DIR, "system", "MFL", "CMSIS"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL"),
		join(FRAMEWORK_DIR, "system", "MFL"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "ADC"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "AFIO"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "BKP"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "CEE"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "COMMON"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "CORTEX"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "CRC"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "CTC"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "DAC"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "DBG"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "DMA"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "EXMC"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "EXTI"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "FMC"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "FWDGT"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "GPIO"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "I2C"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "OB"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "PMU"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "RCU"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "RTC"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "SDIO"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "SPI"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "STARTUP"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "STUBS"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "TIMER"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "USART"),
		join(FRAMEWORK_DIR, "system", "MFL", "MFL", "Source", "WWDGT"),
		join(FRAMEWORK_DIR, "cores", "arduino"),
		variant_dir,
	],
	LINKFLAGS=machine_flags + [
		"-Os",
		"--specs=nosys.specs",
		"-Wl,--gc-sections",
		"-Wl,--check-sections",
		"-Wl,--entry=Reset_Handler",
		"-Wl,--unresolved-symbols=report-all",
		"-Wl,--warn-common",
		"-Wl,--defsym=LD_MAX_SIZE=%d" % board_config.get("upload.maximum_size"),
		"-Wl,--defsym=LD_MAX_DATA_SIZE=%d" % board_config.get("upload.maximum_ram_size"),
		'-Wl,-Map="%s"' % join("${BUILD_DIR}", "${PROGNAME}.map"),
		"--specs=nano.specs",
	],
	LIBS=[
		"c",
		"m",
		"gcc",
		"stdc++",
		get_arm_math_lib(board_config.get("build.cpu")),
	],
	LIBPATH=[
		variant_dir, join(FRAMEWORK_DIR, "system", "MFL", "CMSIS", "DSP", "Lib", "GCC")
	],
)

env.ProcessFlags(board_config.get("build.framework_extra_flags.arduino", ""))

#configure_application_offset(mcu, upload_protocol)

#
# Linker requires preprocessing with correct RAM|ROM sizes
#

if not board_config.get("build.ldscript", ""):
	if not isfile(join(env.subst(variant_dir), "F303RE.ld")):
		print("Warning! Cannot find linker script for the current target!\n")
	env.Replace(LDSCRIPT_PATH=join(variant_dir, "F303RE.ld"))

#
# Process configuration flags
#
cpp_defines = env.Flatten(env.get("CPPDEFINES", []))

process_standard_library_configuration(cpp_defines)
process_usart_configuration(cpp_defines)

# copy CCFLAGS to ASFLAGS (-x assembler-with-cpp mode)
env.Append(ASFLAGS=env.get("CCFLAGS", [])[:])

env.Append(
	LIBSOURCE_DIRS=[
		join(FRAMEWORK_DIR, "libraries", "__cores__", "arduino"),
		join(FRAMEWORK_DIR, "libraries")
	]
)

#
# Target: Build Core Library
#
libs = []

if "build.variant" in env.BoardConfig():
	env.Append(CPPPATH=[variant_dir], LIBPATH=[variant_dir])
	env.BuildSources(join("$BUILD_DIR", "FrameworkArduinoVariant"), variant_dir)

env.BuildSources(
	join("$BUILD_DIR", "FrameworkArduino"), join(FRAMEWORK_DIR, "cores", "arduino")
)

env.Prepend(LIBS=libs)
