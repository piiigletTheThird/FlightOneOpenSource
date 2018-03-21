#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Cubic spline fitter

from __future__ import print_function

import re
import sys
import threading
import subprocess
import glob
import sys
import ntpath
import os
import platform
import argparse
import ctypes as c

from collections import namedtuple

try:
    # Python 3 name
    import queue
except ImportError:
    # Python 2 name
    import Queue as queue

class ColorFallback(object):
    def __getattr__(self, attr):
        return ""

try:
    from colorama import Fore, Style
except ImportError:
    Fore = Style = ColorFallback()



class PyObject_HEAD(c.Structure):
    _fields_ = [
        ('HEAD', c.c_ubyte * (object.__basicsize__ - c.sizeof(c.c_void_p))),
        ('ob_type', c.c_void_p)
    ]

_get_dict = c.pythonapi._PyObject_GetDictPtr
_get_dict.restype = c.POINTER(c.py_object)
_get_dict.argtypes = [c.py_object]

def get_dict(object):
    return _get_dict(object).contents.value

@property
def get_path_method(self):
    if platform.system() == 'Windows':
        return self.replace("/", "\\")
    return self

get_dict(str)['path'] = get_path_method



this_dir = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(this_dir, "output")

parser = argparse.ArgumentParser(description='')
parser.add_argument('-C', "--clean", help="clean up output folder", action='store_true')
parser.add_argument('-D', "--debug", help="build debug target", action='store_true')
parser.add_argument('-j', "--threads", help="number of threads to run", default=10, type=int)
parser.add_argument('-v', "--verbose", help="print compiler calls", action='store_true')
parser.add_argument("-T", "--target", help="target controller to build", default="", nargs='*')
args = parser.parse_args()

IS_CLEANUP = args.clean

#if IS_CLEANUP:
if os.path.exists(OUTPUT_PATH):
    for root, dirs, files in os.walk(OUTPUT_PATH, topdown=False):
        for name in files:
            os.remove(os.path.join(root, name))
        for name in dirs:
            os.rmdir(os.path.join(root, name))
#    sys.exit(0)

TargetConfig = namedtuple('TargetConfig', [
    'target',
    'sourcefiles', 'sourcedirs',
    'cflags', 'asmflags', 'ldflags', 'useColor'
])

excluded_files = [
    ".*_template.c",
]

def configure_target(TARGET):
    STM32F4_ARCH_FLAGS_ADD = ""
    # required features
    FEATURES = []

    ################################################################################
    # Determine target variables and features

   
    if TARGET == "REVOLT":
        if args.debug:
            os.system("PID=\"$(ps -elf | grep  openocd | grep -v 'grep' | sed -e 's/    / /g' | sed -e 's/   / /g' | sed -e 's/  / /g' | cut -d ' ' -f 3)\";kill $PID")
            os.system("openocd -s ~/dev -s /usr/local/share/openocd/scripts -f /usr/local/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/local/share/openocd/scripts/target/stm32f4x.cfg &> redirection &")
        FC_NAME = "REVOLT"
        TARGET_DEVICE_LC = "stm32f405xx"
        PROJECT = "flight_controller"
        TARGET_DEVICE = "STM32F405xx"
        TARGET_SCRIPT = "stm32_flash_f405.ld"
        TARGET_PROCESSOR_TYPE = "f4"
        FEATURES.extend(["usb_otg_fs"])
        OPTIMIZE_FLAGS = "-O3"
        STM32F4_ARCH_FLAGS_ADD = ""
        #STM32F4_ARCH_FLAGS_ADD = "-s -fdata-sections -ffunction-sections -flto"

  
    else:
        print("ERROR: NOT VALID TARGET: ", TARGET, file=sys.stderr)
        sys.exit(1)

    ################################################################################
    # Set per target compilation options


    if TARGET_DEVICE == "STM32F446xx":
        STM32F4_DEF_FLAGS = "-DUSE_HAL_DRIVER -DHSE_VALUE=12000000 -DPROJECT=" + PROJECT + " -D" + PROJECT + " -D" + FC_NAME +" -D" + TARGET_DEVICE + " -DARM_MATH_CM4 -D" + TARGET + " -D" + TARGET_DEVICE_LC + " -D" + TARGET_PROCESSOR_TYPE
        STM32F4_ARCH_FLAGS = "-mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -mtune=cortex-m4"
    else:
        STM32F4_DEF_FLAGS = "-DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -DPROJECT=" + PROJECT + " -D" + PROJECT + " -D" + FC_NAME +" -D" + TARGET_DEVICE + " -DARM_MATH_CM4 -D" + TARGET + " -D" + TARGET_DEVICE_LC + " -D" + TARGET_PROCESSOR_TYPE
        STM32F4_ARCH_FLAGS = "-mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -mtune=cortex-m4" + " " + STM32F4_ARCH_FLAGS_ADD

    if TARGET_PROCESSOR_TYPE == "f4":
        DEF_FLAGS = STM32F4_DEF_FLAGS
        ARCH_FLAGS = STM32F4_ARCH_FLAGS
        MCU_FAMILY = "stm32"

    else:
        print("ERROR: NOT VALID PROCESSOR TYPE FOR TARGET %s, CHECK MAKE FILE CODE" % TARGET, file=sys.stderr)
        sys.exit(1)

    MCU_DIR    = "src/low_level_driver/stm32%s" % TARGET_PROCESSOR_TYPE
    CMSIS_DIR  = "lib/CMSIS/Device/ST/STM32%sxx/Include" % TARGET_PROCESSOR_TYPE.upper()
    HAL_DIR    = "lib/STM32%sxx_HAL_Driver" % TARGET_PROCESSOR_TYPE.upper()


    ################################################################################
    # Set source and includes directories

    # common directories

    INCLUDE_DIRS = [
        "src/low_level_driver/" + MCU_FAMILY,
        MCU_DIR,
        "src/%s/inc" % PROJECT,
        "lib/CMSIS/Include",
        "lib/CMSIS/DSP_Lib/Include",
        CMSIS_DIR,
        HAL_DIR + "/Inc",
    ]

    SOURCE_DIRS = [
        "src/low_level_driver/" + MCU_FAMILY,
        "src/low_level_driver/",
        MCU_DIR,
        "src/%s/src" % PROJECT,
        HAL_DIR + "/Src",
    ]

    SOURCE_FILES = ["src/low_level_driver/stm32_startup/startup_%s.s" % TARGET_DEVICE.lower()]
    #SOURCE_FILES = ["src/low_level_driver/stm32_startup/startup_gd32f405xx.s"]

    # per project includes

    if PROJECT == "flight_controller":
        INCLUDE_DIRS.append("src/flight_controller/inc/telemetry")
        INCLUDE_DIRS.append("src/flight_controller/inc/input")
        SOURCE_DIRS.append("src/flight_controller/src/telemetry")
        SOURCE_DIRS.append("src/flight_controller/src/input")
        FEATURES.extend(["adc", "transponder", "softSerial", "maxOsd", "esc_1wire", "leds", "softPwm", "dmaShenanigans", "actuator_output", "buzzer", "flash_chip", "mpu_icm_device/spi", "rx", "serial", "spm_laptimer"])
    else:
        print("ERROR: NOT VALID PROJECT TYPE, CHECK MAKE FILE CODE", file=sys.stderr)
        sys.exit(1)

    # per-feature directories and files

    USB_SOURCE_DIRS = [
        "lib/STM32_USB_Device_Library/Core/Src",
        "lib/STM32_USB_Device_Library/Class/HID/Src",
        "src/%s/src/usb" % PROJECT,
    ]

    USB_INCLUDE_DIRS = [
        "lib/STM32_USB_Device_Library/Core/Inc",
        "lib/STM32_USB_Device_Library/Class/HID/Inc",
        "src/%s/inc/usb" % PROJECT,
    ]

    for feature in FEATURES:

        if feature.startswith("usb_"):
            # add common usb directories and usb descriptor for project
            SOURCE_DIRS.extend(USB_SOURCE_DIRS)
            INCLUDE_DIRS.extend(USB_INCLUDE_DIRS)
            # add usb class specific files
            SOURCE_DIRS.append("src/low_level_driver/" + feature)
            INCLUDE_DIRS.append("src/low_level_driver/" + feature)

        elif feature.startswith("mpu"):
            # gyro named by "gyro/bus", e.g. "mpu6000/spi"
            gyro, bus = feature.split("/")
            SOURCE_FILES.append("src/%s/src/drivers/invensense_%s.c" % (PROJECT, gyro))
            SOURCE_FILES.append("src/%s/src/drivers/invensense_bus_%s.c" % (PROJECT, bus))
            INCLUDE_DIRS.append("src/%s/inc/drivers" % PROJECT)

        else:
            SOURCE_FILES.append("src/%s/src/drivers/" % (PROJECT) + feature + ".c")
            INCLUDE_DIRS.append("src/%s/inc/drivers/" % (PROJECT))

    if PROJECT != "nesc":
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_add_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_init_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_mult_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_sub_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_trans_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_inverse_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/FilteringFunctions/arm_fir_init_f32.c")
        SOURCE_FILES.append("lib/CMSIS/DSP_Lib/Source/FilteringFunctions/arm_fir_f32.c")
    ################################################################################
    # compiler options

    INCLUDES = " ".join("-I" + include for include in INCLUDE_DIRS)

    LTO_FLAGS = "-flto -fuse-linker-plugin"
    DEBUG_FLAGS = "-ggdb3 -DDEBUG -Og"

    CFLAGS = " ".join([
        ARCH_FLAGS,
        LTO_FLAGS,
        DEF_FLAGS,
        DEBUG_FLAGS if args.debug else OPTIMIZE_FLAGS,
        INCLUDES,
        "-Wall -fstack-protector-all -Wstack-protector -Wextra -Wmaybe-uninitialized -fno-unsafe-math-optimizations -Wdouble-promotion "
        "-ffunction-sections -fdata-sections -MMD -MP"
    ])

    ASMFLAGS = " ".join([
        ARCH_FLAGS,
        "-x assembler-with-cpp",
        INCLUDES,
        "-MMD -MP"
    ])

    mapFile = os.path.join("output", TARGET + ".map")
    linkerDir = os.path.join("src", "low_level_driver")
    ldScript = os.path.join("src", "low_level_driver", TARGET_SCRIPT)
    LDFLAGS = " ".join([
        "-lm -nostartfiles --specs=nano.specs -lc -lnosys",
        ARCH_FLAGS,
        LTO_FLAGS,
        DEBUG_FLAGS if args.debug else OPTIMIZE_FLAGS,
        "-static",
        "-Wl,-gc-sections,-Map," + mapFile,
        "-Wl,-L" + linkerDir,
        "-Wl,--cref",
        "-T" + ldScript
    ])

    # if we're at a tty, then tell gcc to use colors
    if sys.stdout.isatty():
        colorFlag = "-fdiagnostics-color"
    else:
        colorFlag = ""

    ################################################################################
    # build return object with all needed parameters

    target_config = TargetConfig(
        target=TARGET,
        sourcefiles=SOURCE_FILES,
        sourcedirs=SOURCE_DIRS,
        cflags=CFLAGS,
        asmflags=ASMFLAGS,
        ldflags=LDFLAGS,
        useColor=colorFlag,
    )

    return target_config



asm_command = "arm-none-eabi-gcc -c {USECOLOR} -o output/{OUTPUT_FILE} {ASMFLAGS} {INPUT_FILE}"

compile_command = "arm-none-eabi-gcc -c {USECOLOR} -o output/{OUTPUT_FILE} {CFLAGS} {INPUT_FILE}"

link_command = "arm-none-eabi-gcc {USECOLOR} -o output/{OUTPUT_NAME}.elf {OBJS} {LDFLAGS}"

size_command = "arm-none-eabi-size output/{OUTPUT_NAME}.elf"

copy_obj_command = "arm-none-eabi-objcopy -O binary output/{OUTPUT_NAME}.elf output/{OUTPUT_NAME}.bin"


#excluded_files = [
#    ".*_template.c",
#]

THREAD_LIMIT = args.threads
threadLimiter = threading.BoundedSemaphore(THREAD_LIMIT)
locker = threading.Lock()
threadRunning = list()
isStop = False

def find_between( s, first, last ):
    try:
        start = s.index( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return ""

class CommandRunnerThread(threading.Thread):

    def __init__(self, command, output, target, *args, **kwargs):
        self.command = command.path  # we need to be sure all '/' are properly converted for Windows
        self.output = output.path    # store the output of the command for printing purposes
        self.target = target
        self.queue = kwargs.pop("queue", None)
        self.deps = kwargs.pop("dependencies", None)
        self.proc = None
        super(CommandRunnerThread, self).__init__(*args, **kwargs)
        self.stop_event = threading.Event()

    def run(self):
        if self.deps:
            while self.deps:
                if isStop:
                    return
                # wrap in try, in case dependency threads haven't started yet
                try:
                    # wait for first thread to be done
                    self.deps[0].join()
                    # it's done, pop it off deps
                    self.deps.pop(0)
                except RuntimeError:
                    pass

        with threadLimiter:
            with locker:
                threadRunning.append(self)

            try:
                self.run_command()
            finally:
                with locker:
                    threadRunning.remove(self)

    def run_command(self):
        if not self.command:
            return

        with locker:
            if isStop:
                return

            # figure out the output file path
            basedir, basename = os.path.split(self.output)
            _, ext = os.path.splitext(basename)
            # if the base directory doesn't exist, make it
            if not os.path.exists(basedir):
                os.makedirs(basedir)

        self.proc = subprocess.Popen(self.command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout_value, stderr_value = self.proc.communicate()

        # "arm-none-eabi-XXXX" -> "XXXX"
        executable = self.command.split(" ")[0]
        toolChainCmd = executable.split("-")[-1]

        with locker:
            # output all .c/.s file outputs with green, others with red
            if ext == ".o":
                foreColor = Fore.GREEN
            else:
                foreColor = Fore.RED
            print(Fore.MAGENTA + "%% {:s} ".format(self.target) + foreColor + toolChainCmd + " " + Style.RESET_ALL + basename)

            if (args.verbose):
                print(self.command)

            if stdout_value:
                print(stdout_value.decode())
            if stderr_value:
                print(stderr_value.decode())

            sys.stdout.flush()

        if self.queue:
            self.queue.put(self.proc.returncode)
        else:
            print(self.proc.returncode)

        self.proc = None

    def stop_command(self):
        if self.proc:
            try:
                self.proc.kill()
                self.proc.wait()
            except OSError:
                pass

        self.stop_event.set()


    def stopped(self):
        return self.stop_event.isSet()


def FileModified(fileName, target_config):
    # get the output file `output/.../filename.o`
    outputFile = os.path.join("output", makeObject(fileName.path, target_config.target))
    # if we haven't compiled, then return true
    if not os.path.exists(outputFile):
        return True

    # if input file is more recent than the output, return true
    if os.path.getmtime(fileName) > os.path.getmtime(outputFile):
        return True

    # if the target file is more recent than the output, return true
    target_file = "src/low_level_driver/boarddef.h".path
    if os.path.getmtime(target_file) > os.path.getmtime(outputFile):
        return True

    # get the dependency file `output/.../filename.d`
    outputBase, _ = os.path.splitext(outputFile)
    depFile = outputBase + ".d"

    # if we don't have a dependency file, return true
    if not os.path.exists(depFile):
        return True

    # check the dependency file
    with open(depFile, 'r') as f:
        for line in f:
            # the lines with dependencies start with a space
            if line[0] != " ":
                continue

            for dep in line.split():
                # we'll get the line continuation in this: "\"
                if dep == "\\":
                    continue
                # all dependencies should exist
                if not os.path.exists(dep):
                    return True
                # check if dependency is more recent
                if os.path.getmtime(dep) > os.path.getmtime(outputFile):
                    return True

    return False

def makeObject(fileName, target_dir):
    head, tail = os.path.split(fileName)
    root, ext = os.path.splitext(tail)
    if ext.lower() in (".c", ".s"):
        root = os.path.join(target_dir, root)
        return root + ".o"

    print("Unknown file type: " + tail)
    return tail

#This works, but the linker doesn't know how to find the files
#    root, ext = os.path.splitext(fileName)
#
#    # strip first directory from  "src/..." or "lib/..."
#    _, root = root.split(os.sep, 1)
#    # send it to "output/target/"
#    root = os.path.join(TARGET_BOARD, root)
#
#    if ext.lower() in (".c", ".s"):
#        return root + ".o"
#
#    print("Unknown file type: " + tail)
#    return root

def ProcessList(fileNames, target_config):
    linkerObjs = []
    commands = []

    for fileName in fileNames:
        if any(re.match(ex_pattern, os.path.basename(fileName)) for ex_pattern in excluded_files):
            continue

        linkerObjs.append(os.path.join("output", makeObject(fileName, target_config.target)))
        if FileModified(fileName, target_config):
            if fileName[-2:] == ".s":
                commands.append(asm_command.format(
                    INPUT_FILE=fileName.path,
                    OUTPUT_FILE=makeObject(fileName.path, target_config.target),
                    ASMFLAGS=target_config.asmflags,
                    USECOLOR=target_config.useColor,
                ))
            elif fileName[-2:] == ".c":
                commands.append(compile_command.format(
                    INPUT_FILE=fileName.path,
                    OUTPUT_FILE=makeObject(fileName.path, target_config.target),
                    CFLAGS=target_config.cflags,
                    USECOLOR=target_config.useColor,
                ))
            else:
                raise Exception("Bad file type:", fileName)
    return commands, linkerObjs


def main():
    global isStop

    if not args.target:
        raise Exception("Output target must be specified!")

    try:
        os.mkdir("output")
    except:
        pass

    threads = []
    thread_queue = queue.Queue()

    for target in args.target:
        target_config = configure_target(target)

        commands = []
        linkerObjs = []

        # parse all directories for .c and .s files
        for directory in target_config.sourcedirs:
            # process each file, add commands and output files to list
            command, linkerObj = ProcessList(glob.glob(os.path.join(directory, "*.c")), target_config)
            commands.extend(command)
            linkerObjs.extend(linkerObj)

            command, linkerObj = ProcessList(glob.glob(os.path.join(directory, "*.s")), target_config)
            commands.extend(command)
            linkerObjs.extend(linkerObj)

        command, linkerObj = ProcessList(target_config.sourcefiles, target_config)
        commands.extend(command)
        linkerObjs.extend(linkerObj)

        # generate list of threads from all commands
        linkerThreads = []
        for command, linkerObj in zip(commands, linkerObjs):
            thread = CommandRunnerThread(command=command, output=linkerObj, target=target_config.target, queue=thread_queue)
            threads.append(thread)
            linkerThreads.append(thread)

        linkTarget = link_command.format(
            OUTPUT_NAME=target_config.target,
            OBJS=" ".join(linkerObjs),
            LDFLAGS=target_config.ldflags,
            USECOLOR=target_config.useColor,
        )
        linkOutput = "output/{OUTPUT_NAME}.elf".format(OUTPUT_NAME=target_config.target)
        linkThread = CommandRunnerThread(command=linkTarget, output=linkOutput, target=target_config.target, queue=thread_queue, dependencies=linkerThreads)
        threads.append(linkThread)

        sizeTarget = size_command.format(
            OUTPUT_NAME=target_config.target,
        )
        sizeOutput = "output/{OUTPUT_NAME}.elf".format(OUTPUT_NAME=target_config.target)
        sizeThread = CommandRunnerThread(command=sizeTarget, output=sizeOutput, target=target_config.target, queue=thread_queue, dependencies=[linkThread])
        threads.append(sizeThread)

        copyTarget = copy_obj_command.format(
            OUTPUT_NAME=target_config.target
        )
        copyOutput = "output/{OUTPUT_NAME}.bin".format(OUTPUT_NAME=target_config.target)
        copyThread = CommandRunnerThread(command=copyTarget, output=copyOutput, target=target_config.target, queue=thread_queue, dependencies=[sizeThread])
        threads.append(copyThread)

    # all threads are created, start them up
    for thread in threads:
        thread.start()

    while len(threadRunning) > 0:
        try:
            returncode = thread_queue.get(timeout=5)
        except queue.Empty:
            continue
        if returncode > 0:
            with locker:
                isStop = True
                for thread in threads:
                    thread.stop_command()
            break

    for thread in threads:
        thread.join()

    exit(0);

if __name__ == "__main__":
    main()







