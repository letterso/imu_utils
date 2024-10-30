#include "acc_lib/allan_acc.h"
#include "acc_lib/fitallan_acc.h"
#include "gyr_lib/allan_gyr.h"
#include "gyr_lib/fitallan_gyr.h"

#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

imu::AllanGyr *gyr_x;
imu::AllanGyr *gyr_y;
imu::AllanGyr *gyr_z;
imu::AllanAcc *acc_x;
imu::AllanAcc *acc_y;
imu::AllanAcc *acc_z;
std::string data_save_path;

void writeData1(const std::string sensor_name,
                const std::vector<double> &gyro_ts_x,
                const std::vector<double> &gyro_d)
{
    std::ofstream out_t;
    std::ofstream out_x;
    out_t.open(data_save_path + "data_" + sensor_name + "_t.txt", std::ios::trunc);
    out_x.open(data_save_path + "data_" + sensor_name + "_x.txt", std::ios::trunc);
    out_t << std::setprecision(10);
    out_x << std::setprecision(10);
    for (unsigned int index = 0; index < gyro_ts_x.size(); ++index)
    {
        out_t << gyro_ts_x[index] << '\n';
        out_x << gyro_d[index] << '\n';
    }
    out_t.close();
    out_x.close();
}

void writeData3(const std::string sensor_name,
                const std::vector<double> &gyro_ts_x,
                const std::vector<double> &gyro_d_x,
                const std::vector<double> &gyro_d_y,
                const std::vector<double> &gyro_d_z)
{
    std::ofstream out_t;
    std::ofstream out_x;
    std::ofstream out_y;
    std::ofstream out_z;
    out_t.open(data_save_path + "data_" + sensor_name + "_t.txt", std::ios::trunc);
    out_x.open(data_save_path + "data_" + sensor_name + "_x.txt", std::ios::trunc);
    out_y.open(data_save_path + "data_" + sensor_name + "_y.txt", std::ios::trunc);
    out_z.open(data_save_path + "data_" + sensor_name + "_z.txt", std::ios::trunc);
    out_t << std::setprecision(10);
    out_x << std::setprecision(10);
    out_y << std::setprecision(10);
    out_z << std::setprecision(10);

    for (unsigned int index = 0; index < gyro_ts_x.size(); ++index)
    {
        out_t << gyro_ts_x[index] << '\n';
        out_x << gyro_d_x[index] << '\n';
        out_y << gyro_d_y[index] << '\n';
        out_z << gyro_d_z[index] << '\n';
    }

    out_t.close();
    out_x.close();
    out_y.close();
    out_z.close();
}

void writeResult(const std::string data_path,
                 const std::string sensor_name,
                 const imu::FitAllanGyr &gyr_x,
                 const imu::FitAllanGyr &gyr_y,
                 const imu::FitAllanGyr &gyr_z,
                 const imu::FitAllanAcc &acc_x,
                 const imu::FitAllanAcc &acc_y,
                 const imu::FitAllanAcc &acc_z)
{
    std::ofstream out_result;
    out_result.open(data_path + sensor_name + "_imu_param.txt", std::ios::trunc);

    out_result << std::scientific << std::setprecision(16);

    out_result << "name: " << sensor_name << '\n';

    out_result << "Gyr:" << '\n';
    out_result << "  unit:"
               << " rad/s" << '\n';

    out_result << "  avg-axis:" << '\n';
    out_result << "    gyr_n: "
               << (gyr_x.getWhiteNoise() + gyr_y.getWhiteNoise() + gyr_z.getWhiteNoise()) / 3 << '\n';
    out_result << "    gyr_w: "
               << (gyr_x.getBiasInstability() + gyr_y.getBiasInstability() + gyr_z.getBiasInstability()) / 3 << '\n';

    out_result << "  x-axis:" << '\n';
    out_result << "    gyr_n: " << gyr_x.getWhiteNoise() << '\n';
    out_result << "    gyr_w: " << gyr_x.getBiasInstability() << '\n';

    out_result << "  y-axis:" << '\n';
    out_result << "    gyr_n: " << gyr_y.getWhiteNoise() << '\n';
    out_result << "    gyr_w: " << gyr_y.getBiasInstability() << '\n';

    out_result << "  z-axis:" << '\n';
    out_result << "    gyr_n: " << gyr_z.getWhiteNoise() << '\n';
    out_result << "    gyr_w: " << gyr_z.getBiasInstability() << '\n';

    out_result << "Acc:" << '\n';
    out_result << "  unit:"
               << " m/s^2" << '\n';

    out_result << "  avg-axis:" << '\n';
    out_result << "    acc_n: "
               << (acc_x.getWhiteNoise() + acc_y.getWhiteNoise() + acc_z.getWhiteNoise()) / 3 << '\n';
    out_result << "    acc_w: "
               << (acc_x.getBiasInstability() + acc_y.getBiasInstability() + acc_z.getBiasInstability()) / 3 << '\n';

    out_result << "  x-axis:" << '\n';
    out_result << "    acc_n: " << acc_x.getWhiteNoise() << '\n';
    out_result << "    acc_w: " << acc_x.getBiasInstability() << '\n';

    out_result << "  y-axis:" << '\n';
    out_result << "    acc_n: " << acc_y.getWhiteNoise() << '\n';
    out_result << "    acc_w: " << acc_y.getBiasInstability() << '\n';

    out_result << "  z-axis:" << '\n';
    out_result << "    acc_n: " << acc_z.getWhiteNoise() << '\n';
    out_result << "    acc_w: " << acc_z.getBiasInstability() << '\n';

    out_result.close();
}

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_an");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    bool init_flag = false;
    double init_time = 0;
    std::string imu_topic = readParam<std::string>(n, "imu_topic");
    std::string imu_name = readParam<std::string>(n, "imu_name");
    std::string imu_bag = readParam<std::string>(n, "imu_bag");
    data_save_path = readParam<std::string>(n, "data_save_path");
    int start_time = readParam<int>(n, "start_time");
    int end_time = readParam<int>(n, "end_time");
    int max_cluster = readParam<int>(n, "max_cluster");

    gyr_x = new imu::AllanGyr("gyr x", max_cluster);
    gyr_y = new imu::AllanGyr("gyr y", max_cluster);
    gyr_z = new imu::AllanGyr("gyr z", max_cluster);
    acc_x = new imu::AllanAcc("acc x", max_cluster);
    acc_y = new imu::AllanAcc("acc y", max_cluster);
    acc_z = new imu::AllanAcc("acc z", max_cluster);

    // read for bag
    std::cout << "start read imu data." << std::endl;
    // bag
    rosbag::Bag bag;
    bag.open(imu_bag);

    // unpack
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
      const std::string &topic = m.getTopic();

      if (topic == imu_topic) {
        sensor_msgs::ImuConstPtr imu_ptr = m.instantiate<sensor_msgs::Imu>();

        // 过滤区间范围内数据
        double time = imu_ptr->header.stamp.toSec();
        if (!init_flag) {
          init_time = time;
          init_flag = true;
        } else {
          double time_cur = (time - init_time) / 60;
          if (time_cur < start_time)
            continue;
          if (time_cur > end_time)
            break;
        }

        gyr_x->pushRadPerSec(-imu_ptr->angular_velocity.x, time);
        gyr_y->pushRadPerSec(-imu_ptr->angular_velocity.y, time);
        gyr_z->pushRadPerSec(-imu_ptr->angular_velocity.z, time);
        acc_x->pushMPerSec2(imu_ptr->linear_acceleration.x, time);
        acc_y->pushMPerSec2(imu_ptr->linear_acceleration.y, time);
        acc_z->pushMPerSec2(imu_ptr->linear_acceleration.z, time);
      }
    }

    std::cout << "finish read imu data." << std::endl;

    // 计算
    gyr_x->calc();
    std::vector<double> gyro_v_x = gyr_x->getVariance();
    std::vector<double> gyro_d_x = gyr_x->getDeviation();
    std::vector<double> gyro_ts_x = gyr_x->getTimes();

    gyr_y->calc();
    std::vector<double> gyro_v_y = gyr_y->getVariance();
    std::vector<double> gyro_d_y = gyr_y->getDeviation();
    std::vector<double> gyro_ts_y = gyr_y->getTimes();

    gyr_z->calc();
    std::vector<double> gyro_v_z = gyr_z->getVariance();
    std::vector<double> gyro_d_z = gyr_z->getDeviation();
    std::vector<double> gyro_ts_z = gyr_z->getTimes();

    std::cout << "Gyro X " << std::endl;
    imu::FitAllanGyr fit_gyr_x(gyro_v_x, gyro_ts_x, gyr_x->getFreq());
    std::cout << "  bias " << gyr_x->getAvgValue() / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro y " << std::endl;
    imu::FitAllanGyr fit_gyr_y(gyro_v_y, gyro_ts_y, gyr_y->getFreq());
    std::cout << "  bias " << gyr_y->getAvgValue() / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro z " << std::endl;
    imu::FitAllanGyr fit_gyr_z(gyro_v_z, gyro_ts_z, gyr_z->getFreq());
    std::cout << "  bias " << gyr_z->getAvgValue() / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::vector<double> gyro_sim_d_x = fit_gyr_x.calcSimDeviation(gyro_ts_x);
    std::vector<double> gyro_sim_d_y = fit_gyr_y.calcSimDeviation(gyro_ts_y);
    std::vector<double> gyro_sim_d_z = fit_gyr_z.calcSimDeviation(gyro_ts_z);

    writeData3(imu_name + "_sim_gyr", gyro_ts_x, gyro_sim_d_x, gyro_sim_d_y, gyro_sim_d_z);
    writeData3(imu_name + "_gyr", gyro_ts_x, gyro_d_x, gyro_d_y, gyro_d_z);

    std::cout << "==============================================" << std::endl;
    std::cout << "==============================================" << std::endl;

    acc_x->calc();
    std::vector<double> acc_v_x = acc_x->getVariance();
    std::vector<double> acc_d_x = acc_x->getDeviation();
    std::vector<double> acc_ts_x = acc_x->getTimes();

    acc_y->calc();
    std::vector<double> acc_v_y = acc_y->getVariance();
    std::vector<double> acc_d_y = acc_y->getDeviation();
    std::vector<double> acc_ts_y = acc_y->getTimes();

    acc_z->calc();
    std::vector<double> acc_v_z = acc_z->getVariance();
    std::vector<double> acc_d_z = acc_z->getDeviation();
    std::vector<double> acc_ts_z = acc_z->getTimes();

    std::cout << "acc X " << std::endl;
    imu::FitAllanAcc fit_acc_x(acc_v_x, acc_ts_x, acc_x->getFreq());
    std::cout << "-------------------" << std::endl;

    std::cout << "acc y " << std::endl;
    imu::FitAllanAcc fit_acc_y(acc_v_y, acc_ts_y, acc_y->getFreq());
    std::cout << "-------------------" << std::endl;

    std::cout << "acc z " << std::endl;
    imu::FitAllanAcc fit_acc_z(acc_v_z, acc_ts_z, acc_z->getFreq());
    std::cout << "-------------------" << std::endl;

    std::vector<double> acc_sim_d_x = fit_acc_x.calcSimDeviation(acc_ts_x);
    std::vector<double> acc_sim_d_y = fit_acc_y.calcSimDeviation(acc_ts_x);
    std::vector<double> acc_sim_d_z = fit_acc_z.calcSimDeviation(acc_ts_x);

    writeData3(imu_name + "_sim_acc", acc_ts_x, acc_sim_d_x, acc_sim_d_y, acc_sim_d_z);
    writeData3(imu_name + "_acc", acc_ts_x, acc_d_x, acc_d_y, acc_d_z);

    writeResult(data_save_path, imu_name, fit_gyr_x, fit_gyr_y, fit_gyr_z, fit_acc_x, fit_acc_y, fit_acc_z);

    return 0;
}
