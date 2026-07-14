#include "IMU.hpp"

IMU::IMU(IMUc cfg) : imuc(cfg), sensor(nullptr) {
    if (imuc.spi_port != nullptr) {
        imuc.spi_port->begin();
        sensor = new LSM6DSV16XSensor(imuc.spi_port, static_cast<int>(imuc.cs_pin));
        if (sensor->begin() == LSM6DSV16X_OK) {
            sensor->Enable_X();
            sensor->Enable_G();
            sensor->Set_X_FS(convert_accel_fs_to_int(imuc.accel_fs));
            sensor->Set_G_FS(convert_gyro_fs_to_int(imuc.gyro_fs));
            sensor->Set_X_ODR(convert_odr_to_float(imuc.accel_odr), LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);
            sensor->Set_G_ODR(convert_odr_to_float(imuc.gyro_odr), LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);
            sensor->Set_X_Filter_Mode(0, LSM6DSV16X_XL_ULTRA_LIGHT);
            sensor->Set_G_Filter_Mode(0, LSM6DSV16X_GY_ULTRA_LIGHT);
        } else {
            delete sensor;
            sensor = nullptr;
        }
    }
}

IMU::~IMU() {
    delete sensor;
}

float IMU::convert_odr_to_float(LSM6DSV16X_ODR odr) {
    switch (odr) {
        case LSM6DSV16X_ODR::OFF: return 0.0f;
        case LSM6DSV16X_ODR::ODR_1Hz875: return 1.875f;
        case LSM6DSV16X_ODR::ODR_7Hz5: return 7.5f;
        case LSM6DSV16X_ODR::ODR_15Hz: return 15.0f;
        case LSM6DSV16X_ODR::ODR_30Hz: return 30.0f;
        case LSM6DSV16X_ODR::ODR_60Hz: return 60.0f;
        case LSM6DSV16X_ODR::ODR_120Hz: return 120.0f;
        case LSM6DSV16X_ODR::ODR_240Hz: return 240.0f;
        case LSM6DSV16X_ODR::ODR_480Hz: return 480.0f;
        case LSM6DSV16X_ODR::ODR_960Hz: return 960.0f;
        case LSM6DSV16X_ODR::ODR_1920Hz: return 1920.0f;
        case LSM6DSV16X_ODR::ODR_3840Hz: return 3840.0f;
        case LSM6DSV16X_ODR::ODR_7680Hz: return 7680.0f;
    }
    return 0.0f;
}

int32_t IMU::convert_accel_fs_to_int(LSM6DSV16X_ACC_FS fs) {
    switch (fs) {
        case LSM6DSV16X_ACC_FS::FS_2G: return 2;
        case LSM6DSV16X_ACC_FS::FS_4G: return 4;
        case LSM6DSV16X_ACC_FS::FS_8G: return 8;
        case LSM6DSV16X_ACC_FS::FS_16G: return 16;
    }
    return 2;
}

int32_t IMU::convert_gyro_fs_to_int(LSM6DSV16X_GYRO_FS fs) {
    switch (fs) {
        case LSM6DSV16X_GYRO_FS::FS_125DPS: return 125;
        case LSM6DSV16X_GYRO_FS::FS_250DPS: return 250;
        case LSM6DSV16X_GYRO_FS::FS_500DPS: return 500;
        case LSM6DSV16X_GYRO_FS::FS_1000DPS: return 1000;
        case LSM6DSV16X_GYRO_FS::FS_2000DPS: return 2000;
        case LSM6DSV16X_GYRO_FS::FS_4000DPS: return 4000;
    }
    return 2000;
}

IMUb IMU::update(const HALb& halb) {
    IMUb imub;
    imub.accel_body_mps2 = Eigen::Vector3f::Zero();
    imub.omega_body_radps = Eigen::Vector3f::Zero();

    if (sensor != nullptr) {
        int32_t acc_mg[3] = {0, 0, 0};
        int32_t gyro_mdps[3] = {0, 0, 0};

        if (sensor->Get_X_Axes(acc_mg) == LSM6DSV16X_OK) {
            float raw_x = static_cast<float>(acc_mg[0]) * 0.001f * 9.80665f;
            float raw_y = static_cast<float>(acc_mg[1]) * 0.001f * 9.80665f;
            float raw_z = static_cast<float>(acc_mg[2]) * 0.001f * 9.80665f;

            imub.accel_body_mps2.x() = raw_y; // I know this looks wrong but it's not, this IMU is cursed
            imub.accel_body_mps2.y() = raw_z;
            imub.accel_body_mps2.z() = -raw_x;
        }

        if (sensor->Get_G_Axes(gyro_mdps) == LSM6DSV16X_OK) {
            imub.omega_body_radps.x() = (static_cast<float>(gyro_mdps[0]) * 0.001f * 0.0174532925f);
            imub.omega_body_radps.y() = (static_cast<float>(gyro_mdps[1]) * 0.001f * 0.0174532925f);
            imub.omega_body_radps.z() = (static_cast<float>(gyro_mdps[2]) * 0.001f * 0.0174532925f);
        }
    }

    return imub;
}
