import matplotlib.pyplot as plt


def loadText(filename, data):
    with open(filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            value = [float(s) for s in line.split()]
            data.append(value[0])


imu_name = "zed"

# gyo
gyo_dt = []
loadText("../data/data_"+imu_name+"_gyr_t.txt", gyo_dt)

gyo_data_x = []
gyo_data_y = []
gyo_data_z = []
loadText("../data/data_"+imu_name+"_gyr_x.txt", gyo_data_x)
loadText("../data/data_"+imu_name+"_gyr_y.txt", gyo_data_y)
loadText("../data/data_"+imu_name+"_gyr_z.txt", gyo_data_z)
gyo_data_avg = [gyo_data_x, gyo_data_y, gyo_data_z]
gyo_data_avg = [sum(e) / len(e) for e in zip(*gyo_data_avg)]

gyo_data_sim_x = []
gyo_data_sim_y = []
gyo_data_sim_z = []
loadText("../data/data_"+imu_name+"_sim_gyr_x.txt", gyo_data_sim_x)
loadText("../data/data_"+imu_name+"_sim_gyr_y.txt", gyo_data_sim_y)
loadText("../data/data_"+imu_name+"_sim_gyr_z.txt", gyo_data_sim_z)
gyo_data_sim_avg = [gyo_data_sim_x, gyo_data_sim_y, gyo_data_sim_z]
gyo_data_sim_avg = [sum(e) / len(e) for e in zip(*gyo_data_sim_avg)]

plt.figure()
plt.title("gyo")
plt.xlabel("time:sec")
plt.ylabel("Sigma:deg/h")

plt.loglog(gyo_dt, gyo_data_x, 'o', color="r",
           markerfacecolor='white', label="gyo_data_x")
plt.loglog(gyo_dt, gyo_data_y, 'o', color="g",
           markerfacecolor='white', label="gyo_data_y")
plt.loglog(gyo_dt, gyo_data_z, 'o', color="b",
           markerfacecolor='white', label="gyo_data_z")

plt.loglog(gyo_dt, gyo_data_sim_x, 'x', color="r", label="gyo_data_sim_x")
plt.loglog(gyo_dt, gyo_data_sim_y, 'x', color="g", label="gyo_data_sim_y")
plt.loglog(gyo_dt, gyo_data_sim_z, 'x', color="b", label="gyo_data_sim_z")

plt.loglog(gyo_dt, gyo_data_avg, 'o', color="black",
           markerfacecolor='white', label="gyo_data_avg")
plt.loglog(gyo_dt, gyo_data_sim_avg, 'x', color="black", label="gyo_data_sim_avg")

plt.legend()
plt.grid()

# acc
acc_dt = []
loadText("../data/data_"+imu_name+"_acc_t.txt", acc_dt)

acc_data_x = []
acc_data_y = []
acc_data_z = []
loadText("../data/data_"+imu_name+"_acc_x.txt", acc_data_x)
loadText("../data/data_"+imu_name+"_acc_y.txt", acc_data_y)
loadText("../data/data_"+imu_name+"_acc_z.txt", acc_data_z)
acc_data_avg = [acc_data_x, acc_data_y, acc_data_z]
acc_data_avg = [sum(e) / len(e) for e in zip(*acc_data_avg)]

acc_data_sim_x = []
acc_data_sim_y = []
acc_data_sim_z = []
loadText("../data/data_"+imu_name+"_sim_acc_x.txt", acc_data_sim_x)
loadText("../data/data_"+imu_name+"_sim_acc_y.txt", acc_data_sim_y)
loadText("../data/data_"+imu_name+"_sim_acc_z.txt", acc_data_sim_z)
acc_data_sim_avg = [acc_data_sim_x, acc_data_sim_y, acc_data_sim_z]
acc_data_sim_avg = [sum(e) / len(e) for e in zip(*acc_data_sim_avg)]

plt.figure()
plt.title("acc")
plt.xlabel("time:sec")
plt.ylabel("Sigma:deg/h")

plt.loglog(acc_dt, acc_data_x, 'o', color="r", markerfacecolor='white', label="acc_data_x")
plt.loglog(acc_dt, acc_data_y, 'o', color="g", markerfacecolor='white', label="acc_data_y")
plt.loglog(acc_dt, acc_data_z, 'o', color="b", markerfacecolor='white', label="acc_data_z")

plt.loglog(acc_dt, acc_data_sim_x, 'x', color="r", label="acc_data_sim_x")
plt.loglog(acc_dt, acc_data_sim_y, 'x', color="g", label="acc_data_sim_y")
plt.loglog(acc_dt, acc_data_sim_z, 'x', color="b", label="acc_data_sim_z")

plt.loglog(acc_dt, acc_data_avg, 'o', color="black", markerfacecolor='white', label="acc_data_avg")
plt.loglog(acc_dt, acc_data_sim_avg, 'x', color="black", label="acc_data_sim_avg")

plt.legend()
plt.grid()

# display
plt.show()
