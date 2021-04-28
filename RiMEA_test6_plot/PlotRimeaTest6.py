import matplotlib.pyplot as plt
import numpy as np
import re

def isfloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False


s = ""
f = open("../output/Test6_results.txt", "r")
s = f.read()
f.close
print(s)

list = re.split(' |\n', s)

floatlist=[]

for element in list:
    if isfloat(element):
        floatlist.append(float(element))
floatlist.sort()
print(floatlist)

#setting values after test values to 0, pretty and constructing dictionary
my_dict={}
my_dict[16.09]=0
my_dict.update({i:floatlist.count(i) for i in floatlist})
my_dict[18.1]=0


keys = my_dict. keys()
values = my_dict. values()

#seting limits of x and y axis
plt.axis([15,19 , 0, 15 ])

# naming the x axis
plt.xlabel('duration/s')
# naming the y axis
plt.ylabel('frequency')

plt.plot(keys, values)
#plt.axvline(26, 0, 20, color="red")
#plt.axvline(34, 0, 20, color="red")
plt.title("RiMEA Test6 results")
plt.savefig('../Test6_result.png')

plt.show()
