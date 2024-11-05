const std = @import("std");
const Build = std.Build;

pub fn build(b: *Build) void {
    const target = std.Target.Query{
        .os_tag = .freestanding,
        .cpu_arch = .arm,
        .cpu_model = .{
            .explicit = &std.Target.arm.cpu.cortex_m0plus,
        },
        .abi = .eabi,
    };

    const options = Build.StaticLibraryOptions{
        .name = "bldc",
        .optimize = b.standardOptimizeOption(Build.StandardOptimizeOptionOptions{
            .preferred_optimize_mode = .Debug,
        }),
        .target = b.standardTargetOptions(Build.StandardTargetOptionsArgs{
            .default_target = target,
        }),
        .root_source_file = b.path("main.zig"),
    };
    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    // const mode = b.standardReleaseOptions();

    const bin = b.addStaticLibrary(options);

    const includes = [_][]const u8{
        "./gen",
        // "/usr/lib/gcc/arm-none-eabi/12.1.0/../../../../arm-none-eabi/include/",

        "./pico-sdk/src/common/pico_base/include/",
        "./pico-sdk/src/common/pico_time/include/",

        "./pico-sdk/src/rp2040/hardware_regs/include/",
        "./pico-sdk/src/rp2040/hardware_structs/include/",

        "./pico-sdk/src/rp2_common/pico_platform/include/",
        "./pico-sdk/src/rp2_common/pico_stdio/include/",
        "./pico-sdk/src/rp2_common/pico_bit_ops/include/",
        "./pico-sdk/src/rp2_common/pico_bootrom/include/",
        "./pico-sdk/src/rp2_common/pico_bootsel_via_double_reset/include/",
        "./pico-sdk/src/rp2_common/pico_cxx_options/include/",
        "./pico-sdk/src/rp2_common/pico_cyw43_arch/include/",
        "./pico-sdk/src/rp2_common/pico_divider/include/",
        "./pico-sdk/src/rp2_common/pico_double/include/",
        "./pico-sdk/src/rp2_common/pico_fix/include/",
        "./pico-sdk/src/rp2_common/pico_float/include/",
        "./pico-sdk/src/rp2_common/pico_int64_ops/include/",
        "./pico-sdk/src/rp2_common/pico_lwip/include/",
        "./pico-sdk/src/rp2_common/pico_malloc/include/",
        "./pico-sdk/src/rp2_common/pico_mem_ops/include/",
        "./pico-sdk/src/rp2_common/pico_multicore/include/",
        "./pico-sdk/src/rp2_common/pico_printf/include/",
        "./pico-sdk/src/rp2_common/pico_runtime/include/",
        "./pico-sdk/src/rp2_common/pico_standard_link/include/",
        "./pico-sdk/src/rp2_common/pico_stdio_semihosting/include/",
        "./pico-sdk/src/rp2_common/pico_stdio_uart/include/",
        "./pico-sdk/src/rp2_common/pico_stdio_usb/include/",
        "./pico-sdk/src/rp2_common/pico_stdlib/include/",
        "./pico-sdk/src/rp2_common/pico_unique_id/include/",

        "./pico-sdk/src/rp2_common/hardware_base/include/",
        "./pico-sdk/src/rp2_common/hardware_timer/include/",
        "./pico-sdk/src/rp2_common/hardware_irq/include/",
        "./pico-sdk/src/rp2_common/hardware_sync/include/",
        "./pico-sdk/src/rp2_common/hardware_gpio/include/",
        "./pico-sdk/src/rp2_common/hardware_adc/include/",
        "./pico-sdk/src/rp2_common/hardware_claim/include/",
        "./pico-sdk/src/rp2_common/hardware_divider/include/",
        "./pico-sdk/src/rp2_common/hardware_dma/include/",
        "./pico-sdk/src/rp2_common/hardware_exception/include/",
        "./pico-sdk/src/rp2_common/hardware_flash/include/",
        "./pico-sdk/src/rp2_common/hardware_i2c/include/",
        "./pico-sdk/src/rp2_common/hardware_interp/include/",
        "./pico-sdk/src/rp2_common/hardware_irq/include/",
        "./pico-sdk/src/rp2_common/hardware_pio/include/",
        "./pico-sdk/src/rp2_common/hardware_pll/include/",
        "./pico-sdk/src/rp2_common/hardware_pwm/include/",
        "./pico-sdk/src/rp2_common/hardware_resets/include/",
        "./pico-sdk/src/rp2_common/hardware_rtc/include/",
        "./pico-sdk/src/rp2_common/hardware_spi/include/",
        "./pico-sdk/src/rp2_common/hardware_uart/include/",
        "./pico-sdk/src/rp2_common/hardware_vreg/include/",
        "./pico-sdk/src/rp2_common/hardware_watchdog/include/",
        "./pico-sdk/src/rp2_common/hardware_xosc/include/",
    };

    inline for (includes) |include| {
        bin.addIncludePath(b.path(include));
    }
    // exe.linkLibC();
    // exe.linkSystemLibrary("c");
    const bin_artifact = b.addInstallArtifact(bin, .{});

    const build_step = b.step("build", "Build the application static library");
    build_step.dependOn(&bin_artifact.step);

    //Tests
    const tests = b.addTest(Build.TestOptions{
        .name = "tests",
        .target = defaultTarget(),
        .root_source_file = b.path("main.zig"),
    });

    const tests_step = b.step("test", "Run the tests");
    tests_step.dependOn(&tests.step);
}

fn defaultTarget() Build.ResolvedTarget {
    const query = std.Target.Query{};
    return Build.ResolvedTarget{
        .query = query,
        .result = std.zig.system.resolveTargetQuery(query) catch
            @panic("unable to resolve target query"),
    };
}
