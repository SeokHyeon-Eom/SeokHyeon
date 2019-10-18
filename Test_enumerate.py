print("Test enumerate")

for i in range(1, 10):
    print(i, end=' ')

print("")

li = [i for i in range(10)]
for i in enumerate(li):
    print(i, end=" ")