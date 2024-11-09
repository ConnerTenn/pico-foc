const std = @import("std");
const math = std.math;
const tau = std.math.tau;

pub const PhaseVoltage = struct {
    const u_axis_angle = 0.0 * tau / 3.0; //0 deg
    const v_axis_angle = 1.0 * tau / 3.0; //120 deg
    const w_axis_angle = 2.0 * tau / 3.0; //240 deg

    u_axis: f32,
    v_axis: f32,
    w_axis: f32,
};

pub fn getPhaseVoltage(direct_torque: f32, tangent_torque: f32, angle: f32) PhaseVoltage {
    return PhaseVoltage{
        .u_axis = direct_torque * @cos(angle - PhaseVoltage.u_axis_angle) - tangent_torque * @sin(angle - PhaseVoltage.u_axis_angle),
        .v_axis = direct_torque * @cos(angle - PhaseVoltage.v_axis_angle) - tangent_torque * @sin(angle - PhaseVoltage.v_axis_angle),
        .w_axis = direct_torque * @cos(angle - PhaseVoltage.w_axis_angle) - tangent_torque * @sin(angle - PhaseVoltage.w_axis_angle),
    };
}
