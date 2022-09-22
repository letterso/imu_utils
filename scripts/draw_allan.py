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

plt.loglog(gyo_dt, gyo_data_x, 'o', color="r", markerfacecolor='white')
plt.loglog(gyo_dt, gyo_data_y, 'o', color="g", markerfacecolor='white')
plt.loglog(gyo_dt, gyo_data_z, 'o', color="b", markerfacecolor='white')

plt.loglog(gyo_dt, gyo_data_sim_x, 'x', color="r")
plt.loglog(gyo_dt, gyo_data_sim_y, 'x', color="g")
plt.loglog(gyo_dt, gyo_data_sim_z, 'x', color="b")

plt.loglog(gyo_dt, gyo_data_avg, 'o', color="black", markerfacecolor='white')
plt.loglog(gyo_dt, gyo_data_sim_avg, 'x', color="black")

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

plt.loglog(acc_dt, acc_data_x, 'o', color="r", markerfacecolor='white')
plt.loglog(acc_dt, acc_data_y, 'o', color="g", markerfacecolor='white')
plt.loglog(acc_dt, acc_data_z, 'o', color="b", markerfacecolor='white')

plt.loglog(acc_dt, acc_data_sim_x, 'x', color="r")
plt.loglog(acc_dt, acc_data_sim_y, 'x', color="g")
plt.loglog(acc_dt, acc_data_sim_z, 'x', color="b")

plt.loglog(acc_dt, acc_data_avg, 'o', color="black", markerfacecolor='white')
plt.loglog(acc_dt, acc_data_sim_avg, 'x', color="black")

plt.grid()

# display
plt.show()
