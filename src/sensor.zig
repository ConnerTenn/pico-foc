const std = @import("std");

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;

pub const LIS3MDL = struct {
    const Self = @This();

    const RawData = struct {
        x_axis: i16,
        y_axis: i16,
        z_axis: i16,
    };
    const FieldData = struct {
        x_axis: f32,
        y_axis: f32,
        z_axis: f32,
    };

    // The magnetometer ranges
    const Range = enum(u2) {
        range_4_gauss = 0b00, // +/- 4g (default value)
        range_8_gauss = 0b01, // +/- 8g
        range_12_gauss = 0b10, // +/- 12g
        range_16_gauss = 0b11, // +/- 16g
    };

    // The magnetometer data rate, includes FAST_ODR bit */
    const DataRate = enum(u4) {
        data_rate_0_625_HZ = 0b0000, // 0.625 Hz
        data_rate_1_25_HZ = 0b0010, // 1.25 Hz
        data_rate_2_5_HZ = 0b0100, // 2.5 Hz
        data_rate_5_HZ = 0b0110, // 5 Hz
        data_rate_10_HZ = 0b1000, // 10 Hz
        data_rate_20_HZ = 0b1010, // 20 Hz
        data_rate_40_HZ = 0b1100, // 40 Hz
        data_rate_80_HZ = 0b1110, // 80 Hz
        data_rate_155_HZ = 0b0001, // 155 Hz (FAST_ODR + UHP)
        data_rate_300_HZ = 0b0011, // 300 Hz (FAST_ODR + HP)
        data_rate_560_HZ = 0b0101, // 560 Hz (FAST_ODR + MP)
        data_rate_1000_HZ = 0b0111, // 1000 Hz (FAST_ODR + LP)
    };

    // The magnetometer performance mode
    const PerformanceMode = enum(u2) {
        low_power_mode = 0b00, // Low power mode
        medium_mode = 0b01, // Medium performance mode
        high_mode = 0b10, // High performance mode
        ultra_high_mode = 0b11, // Ultra-high performance mode
    };

    // The magnetometer operation mode
    const OperationMode = enum(u2) {
        continuous_mode = 0b00, // Continuous conversion
        single_mode = 0b01, // Single-shot conversion
        power_down_mode = 0b11, // Powered-down mode
    };

    const Endian = enum(u1) {
        little = 0b0,
        big = 0b1,
    };

    const CtrlReg1 = packed struct {
        const address = 0x20;

        st: u1,
        data_rate: DataRate,
        operating_mode_xy: PerformanceMode,
        temp_en: u1,
    };

    const CtrlReg2 = packed struct {
        const address = 0x21;

        res0_1: u2 = 0,
        soft_reset: u1,
        reboot: u1,
        res0_2: u1 = 0,
        range: Range,
        res0_3: u1 = 0,
    };

    const CtrlReg3 = packed struct {
        const address = 0x22;

        mode: OperationMode,
        sim: u1,
        res0_1: u2 = 0,
        lp: u1,
        res0_2: u2 = 0,
    };

    const CtrlReg4 = packed struct {
        const address = 0x23;

        res0_1: u1 = 0,
        ble: Endian,
        operating_mode_z: PerformanceMode,
        res0_2: u4 = 0,
    };

    const StatusReg = packed struct {
        const address = 0x27;

        xda: u1,
        yda: u1,
        zda: u1,
        zyxda: u1,
        xor: u1,
        yor: u1,
        zor: u1,
        zyxor: u1,
    };

    const OutX_L = packed struct {
        const address = 0x28;

        lower: u8,
    };
    const OutX_H = packed struct {
        const address = 0x29;

        higher: u8,
    };
    const OutY_L = packed struct {
        const address = 0x2A;

        lower: u8,
    };
    const OutY_H = packed struct {
        const address = 0x2B;

        higher: u8,
    };
    const OutZ_L = packed struct {
        const address = 0x2C;

        lower: u8,
    };
    const OutZ_H = packed struct {
        const address = 0x2D;

        higher: u8,
    };

    sck_pin: c_uint,
    tx_pin: c_uint,
    rx_pin: c_uint,
    cs_pin: c_uint,
    hardware_spi: ?*csdk.spi_inst_t = @ptrCast(csdk.spi0_hw),

    pub fn create(sck_pin: c_uint, tx_pin: c_uint, rx_pin: c_uint, cs_pin: c_uint) Self {
        return Self{
            .sck_pin = sck_pin,
            .tx_pin = tx_pin,
            .rx_pin = rx_pin,
            .cs_pin = cs_pin,
        };
    }

    pub fn init(self: Self) void {
        csdk.gpio_init(self.cs_pin);
        csdk.gpio_put(self.cs_pin, bldc.GPIO_HIGH);
        csdk.gpio_set_dir(self.cs_pin, bldc.GPIO_OUT);

        const baudrate = csdk.spi_init(self.hardware_spi, 1 * 1000 * 1000); //1MHz.
        stdio.print("SPI baudrate:{}\n", .{baudrate});
        csdk.gpio_set_function(self.sck_pin, csdk.GPIO_FUNC_SPI);
        csdk.gpio_set_function(self.tx_pin, csdk.GPIO_FUNC_SPI);
        csdk.gpio_set_function(self.rx_pin, csdk.GPIO_FUNC_SPI);
    }

    inline fn csSelect(self: Self) void {
        csdk.gpio_put(self.cs_pin, bldc.GPIO_LOW);
    }

    inline fn csDeselect(self: Self) void {
        csdk.gpio_put(self.cs_pin, bldc.GPIO_HIGH);
    }

    fn readReg(self: Self, T: type) T {
        self.csSelect();

        const read_cmd = 0x80;
        const write_data = [_]u8{
            read_cmd | T.address,
            0,
        };
        var read_data: [2]u8 = .{0} ** 2;
        _ = csdk.spi_write_read_blocking(self.hardware_spi, &write_data, &read_data, 2);

        self.csDeselect();
        csdk.sleep_us(10);

        return @bitCast(read_data[1]);
    }

    fn writeReg(self: Self, T: type, data: T) void {
        self.csSelect();

        const write_cmd = 0x00;
        const write_data = [_]u8{
            write_cmd | T.address,
            @as(u8, @bitCast(data)),
        };
        _ = csdk.spi_write_blocking(self.hardware_spi, &write_data, 2);

        self.csDeselect();
        csdk.sleep_us(10);
    }

    fn dataAvailable(self: Self) void {
        const status = self.readReg(StatusReg);
        return status.zyxda;
    }

    fn getRawData(self: Self) RawData {
        const x_l = @as(u16, self.readReg(OutX_L).lower);
        const x_h = @as(u16, self.readReg(OutX_H).higher);
        const y_l = @as(u16, self.readReg(OutY_L).lower);
        const y_h = @as(u16, self.readReg(OutY_H).higher);
        const z_l = @as(u16, self.readReg(OutZ_L).lower);
        const z_h = @as(u16, self.readReg(OutZ_H).higher);

        return RawData{
            .x_axis = @bitCast((x_h << 8) | x_l),
            .y_axis = @bitCast((y_h << 8) | y_l),
            .z_axis = @bitCast((z_h << 8) | z_l),
        };
    }

    fn getFieldValues(self: Self) FieldData {
        const raw_data = self.getRawData();

        return FieldData{
            .x_axis = raw_data.x_axis * 4.0 * 100.0 / 32768.0,
            .y_axis = raw_data.y_axis * 4.0 * 100.0 / 32768.0,
            .z_axis = raw_data.z_axis * 4.0 * 100.0 / 32768.0,
        };
    }
};

pub fn demo() noreturn {
    var sensor = LIS3MDL.create(18, 19, 16, 17);
    sensor.init();
    var ctrl_reg3 = sensor.readReg(LIS3MDL.CtrlReg3);
    ctrl_reg3.mode = .continuous_mode;
    sensor.writeReg(LIS3MDL.CtrlReg3, ctrl_reg3);

    stdio.print("CtrlReg1: {}\n", .{sensor.readReg(LIS3MDL.CtrlReg1)});
    stdio.print("CtrlReg2: {}\n", .{sensor.readReg(LIS3MDL.CtrlReg2)});
    stdio.print("CtrlReg3: {}\n", .{sensor.readReg(LIS3MDL.CtrlReg3)});
    stdio.print("CtrlReg4: {}\n", .{sensor.readReg(LIS3MDL.CtrlReg4)});
    stdio.print("{}\n\n", .{sensor.getRawData()});

    while (true) {
        const raw_data = sensor.getRawData();
        stdio.print("X:{d: <8}  Y:{d: <8}  Z:{d: <8}   \r", .{ raw_data.x_axis, raw_data.y_axis, raw_data.z_axis });
    }
}
