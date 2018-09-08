
#include <c_math.h>
#include "module.h"
#include "lauxlib.h"
#include "../eMPL/inv_mpu.h"
#include "../eMPL/inv_mpu_dmp_motion_driver.h"

#define DEFAULT_MPU_HZ 100
// Q30 format, divisor when long to float.
#define q30 1073741824.0f

static signed char gyro_orientation[9] = {1, 0, 0,
                                          0, 1, 0,
                                          0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result > 0)
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        return 0;
    }
    return 1; // self test did not pass
}

static int error_msg(lua_State *L, char *msg)
{
    lua_pushnil(L);
    lua_pushstring(L, msg);
    return 2;
}

static int setup(lua_State *L)
{
    if (mpu_init((void *) 0))
        return error_msg(L, "mpu_init failed");

    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        return error_msg(L, "mpu_set_sensor failed");

    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        return error_msg(L, "mpu_configure_fifo failed");

    if (mpu_set_sample_rate(DEFAULT_MPU_HZ))
        return error_msg(L, "mpu_set_sample_rate failed");

    if (dmp_load_motion_driver_firmware())
        return error_msg(L, "dmp_load_motion_driver_firmware failed");

    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        return error_msg(L, "dmp_set_orientation failed");

    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                            DMP_FEATURE_SEND_CAL_GYRO |
                            DMP_FEATURE_GYRO_CAL))
        return error_msg(L, "dmp_enable_feature failed");

    if (dmp_set_fifo_rate(DEFAULT_MPU_HZ))
        return error_msg(L, "dmp_set_fifo_rate failed");

    run_self_test();

    if (mpu_set_dmp_state(1))
        return error_msg(L, "mpu_set_dmp_state failed");

    lua_pushboolean(L, 1);
    return 1;
}

// Get the data after dmp processing (note that this function needs to compare multiple stacks, local variables are a bit more)
// pitch: pitch angle accuracy: 0.1° range: -90.0° <---> +90.0°
// roll: roll angle accuracy: 0.1° range: -180.0°<---> +180.0°
// yaw: heading angle Accuracy: 0.1° Range: -180.0°<---> +180.0°
// Return value: 0, normal; other, failure
static int read_euler(lua_State *L)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f, ex = 180 / acos(-1);
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    float pitch, roll, yaw;

    if (!dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
    {
        /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
        * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
        **/
        if (sensors & INV_WXYZ_QUAT)
        {
            q0 = quat[0] / q30; // Convert q30 format to floating point number
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;
            // Calculate the pitch angle / roll angle / heading angle
            pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * ex;                                    // pitch
            roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * ex;     // roll
            yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * ex; //yaw
            lua_pushnumber(L, (lua_Number) pitch);
            lua_pushnumber(L, (lua_Number) roll);
            lua_pushnumber(L, (lua_Number) yaw);
            return 3;
        }
    }
    return 0;
}

static int read_quaternions(lua_State *L)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f, ex = 180 / acos(-1);
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];

    if (!dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
    {
        if (sensors & INV_WXYZ_QUAT)
        {
            lua_pushnumber(L, (lua_Number) quat[0]);
            lua_pushnumber(L, (lua_Number) quat[1]);
            lua_pushnumber(L, (lua_Number) quat[2]);
            lua_pushnumber(L, (lua_Number) quat[3]);
            return 4;
        }
    }
    return 0;
}

// Module function map, this is how we tell Lua what API our module has
const LUA_REG_TYPE dmp_map[] =
{
        {LSTRKEY("setup"), LFUNCVAL(setup)},
        {LSTRKEY("read_euler"), LFUNCVAL(read_euler)},
        {LSTRKEY("read_quaternions"), LFUNCVAL(read_quaternions)},
        {LNILKEY, LNILVAL} // This map must always end like this
};

NODEMCU_MODULE(DMP, "dmp", dmp_map, NULL);
