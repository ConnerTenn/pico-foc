const std = @import("std");

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;

pub const LIS3MDL = struct {
    const Self = @This();
    const I2CADDR_DEFAULT = 0x1C; // Default breakout addres

    const REG_WHO_AM_I = 0x0F; // Register that contains the part ID
    const REG_CTRL_REG1 = 0x20; // Register address for control 1
    const REG_CTRL_REG2 = 0x21; // Register address for control 2
    const REG_CTRL_REG3 = 0x22; // Register address for control 3
    const REG_CTRL_REG4 = 0x23; // Register address for control 3
    const REG_STATUS = 0x27; // Register address for status
    const REG_OUT_X_L = 0x28; // Register address for X axis lower byte
    const REG_INT_CFG = 0x30; // Interrupt configuration register
    const REG_INT_THS_L = 0x32; // Low byte of the irq threshold

    // The magnetometer ranges
    const Range = enum(u2) {
        RANGE_4_GAUSS = 0b00, // +/- 4g (default value)
        RANGE_8_GAUSS = 0b01, // +/- 8g
        RANGE_12_GAUSS = 0b10, // +/- 12g
        RANGE_16_GAUSS = 0b11, // +/- 16g
    };

    // The magnetometer data rate, includes FAST_ODR bit */
    const DataRate = enum(u4) {
        DATARATE_0_625_HZ = 0b0000, // 0.625 Hz
        DATARATE_1_25_HZ = 0b0010, // 1.25 Hz
        DATARATE_2_5_HZ = 0b0100, // 2.5 Hz
        DATARATE_5_HZ = 0b0110, // 5 Hz
        DATARATE_10_HZ = 0b1000, // 10 Hz
        DATARATE_20_HZ = 0b1010, // 20 Hz
        DATARATE_40_HZ = 0b1100, // 40 Hz
        DATARATE_80_HZ = 0b1110, // 80 Hz
        DATARATE_155_HZ = 0b0001, // 155 Hz (FAST_ODR + UHP)
        DATARATE_300_HZ = 0b0011, // 300 Hz (FAST_ODR + HP)
        DATARATE_560_HZ = 0b0101, // 560 Hz (FAST_ODR + MP)
        DATARATE_1000_HZ = 0b0111, // 1000 Hz (FAST_ODR + LP)
    };

    // The magnetometer performance mode
    const PerformanceMode = enum(u2) {
        LIS3MDL_LOWPOWERMODE = 0b00, // Low power mode
        LIS3MDL_MEDIUMMODE = 0b01, // Medium performance mode
        LIS3MDL_HIGHMODE = 0b10, // High performance mode
        LIS3MDL_ULTRAHIGHMODE = 0b11, // Ultra-high performance mode
    };

    // The magnetometer operation mode
    const OperationMode = enum(u2) {
        LIS3MDL_CONTINUOUSMODE = 0b00, // Continuous conversion
        LIS3MDL_SINGLEMODE = 0b01, // Single-shot conversion
        LIS3MDL_POWERDOWNMODE = 0b11, // Powered-down mode
    };

    sck_pin: c_uint,
    tx_pin: c_uint,
    cs_pin: c_uint,
    // hardware_spi: csdk.spi_inst_t = std.mem.zeroes(csdk.spi_inst_t),

    pub fn create(sck_pin: c_uint, tx_pin: c_uint, cs_pin: c_uint) Self {
        return Self{
            .sck_pin = sck_pin,
            .tx_pin = tx_pin,
            .cs_pin = cs_pin,
        };
    }

    pub fn init(self: *Self) void {
        // csdk.spi_init(&self.hardware_spi, 10 * 1000 * 1000);
        _ = csdk.spi_init(null, 10 * 1000 * 1000); //10MHz.
        csdk.gpio_set_function(self.sck_pin, csdk.GPIO_FUNC_SPI);
        csdk.gpio_set_function(self.tx_pin, csdk.GPIO_FUNC_SPI);

        csdk.gpio_init(self.cs_pin);
        csdk.gpio_set_dir(self.cs_pin, bldc.GPIO_OUT);
        csdk.gpio_put(self.cs_pin, bldc.GPIO_HIGH);
    }

    fn setPerformanceMode(mode: PerformanceMode) void {
        _ = mode; // autofix
    }

    fn getPerformanceMode() PerformanceMode {}

    fn setOperationMode(mode: OperationMode) void {
        _ = mode; // autofix
    }

    fn getOperationMode() OperationMode {}

    fn setDataRate(data_rate: DataRate) void {
        _ = data_rate; // autofix
    }

    fn getDataRate() DataRate {}

    fn setRange(range: Range) void {
        _ = range; // autofix
    }

    fn getRange() Range {}

    fn setIntThreshold(value: u16) void {
        _ = value; // autofix
    }

    fn getIntThreshold() u16 {}
};

// /** Class for hardware interfacing with an LIS3MDL magnetometer */
// class Adafruit_LIS3MDL : public Adafruit_Sensor {
// public:
//   Adafruit_LIS3MDL(void);
//   bool begin_I2C(uint8_t i2c_addr = LIS3MDL_I2CADDR_DEFAULT,
//                  TwoWire *wire = &Wire);
//   bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
//                  uint32_t frequency = 1000000);
//   bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
//                  int8_t mosi_pin, uint32_t frequency = 1000000);

//   void reset(void);

//   void configInterrupt(bool enableX, bool enableY, bool enableZ, bool polarity,
//                        bool latch, bool enableInt);
//   void selfTest(bool flag);

//   void read();
//   bool getEvent(sensors_event_t *event);
//   void getSensor(sensor_t *sensor);

//   // Arduino compatible API
//   int readMagneticField(float &x, float &y, float &z);
//   float magneticFieldSampleRate(void);
//   int magneticFieldAvailable(void);

//   int16_t x,     ///< The last read X mag in raw units
//       y,         ///< The last read Y mag in raw units
//       z;         ///< The last read Z mag in raw units
//   float x_gauss, ///< The last read X mag in 'gauss'
//       y_gauss,   ///< The last read Y mag in 'gauss'
//       z_gauss;   ///< The last read Z mag in 'gauss'

//   //! buffer for the magnetometer range
//   lis3mdl_range_t rangeBuffered = LIS3MDL_RANGE_4_GAUSS;

// private:
//   bool _init(void);

//   Adafruit_I2CDevice *i2c_dev = NULL;
//   Adafruit_SPIDevice *spi_dev = NULL;

//   int32_t _sensorID;
// };

pub fn demo() noreturn {
    var sensor = LIS3MDL.create(24, 25, 26);
    sensor.init();

    while (true) {}
}
