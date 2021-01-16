import csv
import matplotlib.pyplot as plt
with open('test_data.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0

    roll = []
    pitch = []
    yaw = []
    fl = []
    fr = []
    bl = []
    br = []


    for row in csv_reader:
        print(row)
        roll.append(row[0])
        pitch.append(row[1])
        yaw.append(row[2])
        fl.append(row[3])
        fr.append(row[4])
        bl.append(row[5])
        br.append(row[6])
        
fig = plt.figure()
# ax = fig.add_subplot(111)

# plt.plot(range(len(roll)),roll,label='roll')
# plt.plot(range(len(roll)),pitch,label='pitch')
# plt.plot(range(len(roll)),yaw,label='yaw')
plt.plot(range(len(roll)),fl,label='fl')
plt.plot(range(len(roll)),fr,label='fr')
plt.plot(range(len(roll)),bl,label='bl')
plt.plot(range(len(roll)),br,label='br')
plt.ylim([100, 800])
plt.legend()
plt.show()