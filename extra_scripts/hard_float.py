Import("env")

# The bundled FreeRTOS ARM_CM4F port and the application use the hard-float ABI.
# PlatformIO's bare-metal builder sets CPU options but does not propagate the
# FPU and float ABI options from build_flags to the final GCC linker command.
env.Append(LINKFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"])
