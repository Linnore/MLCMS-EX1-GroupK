import matplotlib.pyplot as plt
import numpy as np
import re

#helper to find floats in file
def isfloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False

#test for around the corner

#extracting file to string
s = ""
f = open("../output/Test1_results.txt", "r")
s = f.read()
f.close
#print(s)

#extracting floats
list = re.split(' |\n', s)

floatlist=[]

for element in list:
    if isfloat(element):
        floatlist.append(float(element))
floatlist.sort()
#print(floatlist)


#setting values after test values to 0, pretty and constructing dictionary
my_dict={}
my_dict[28]=0
my_dict.update({i:floatlist.count(i) for i in floatlist})
my_dict[32.3]=0


print(my_dict)

keys = my_dict. keys()
values = my_dict. values()

# naming the x axis
plt.xlabel('duration/s')
# naming the y axis
plt.ylabel('frequency')

#seting limits of x and y axis
plt.axis([25, 35, 0, 8])

#ploting the data
plt.plot(keys, values)

#boundaries
plt.axvline(26, 0, 20, color="red")
plt.axvline(34, 0, 20, color="red")
plt.title("RiMEA Test1 results")
plt.savefig('./Test1_result.png')

plt.show()
