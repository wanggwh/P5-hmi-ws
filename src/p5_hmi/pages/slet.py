import numpy as np
n = 7

which_page = {}

# randomarray = np.empty((n,1), dtype=object)
# print("Initial randomarray:")
# print(randomarray)
# randomarray = np.concatenate((randomarray, np.empty((n,1), dtype=object)), axis=1)
# print("After concatenation:")
# print(randomarray)

alice_data = np.empty((n,1), dtype=object)
bob_data = np.empty((n,1,1), dtype=object)
mir_data = np.empty((n,1,1), dtype=object)

# print(alice_data)
# print(bob_data)
# print(mir_data)

# structure = np.array([
#     np.empty((n,1,1), dtype=object), #Alice
#     np.empty((n,1,1), dtype=object), #Bob
#     np.empty((n,1,1), dtype=object), #MiR
# ])

which_page = dict()

page1 = 1
page2 = 2
page3 = 3
page4 = 4
page5 = 5

add1 = {"id": 2, "data": "example1"}
add2 = {"id": 4, "data": "example2"}
add3 = {"id": 6, "data": "example3"}
add4 = {"id": 8, "data": "example4"}
add5 = {"id": 10, "data": "example5"}

def add_function(dict, placement, struc, page):
    global n
    #print(len(struc[device][placement-1]))
    for i in range(len(struc[placement-1])):
        #print(i)
        if struc[placement-1][i] is None:
            struc[placement-1][i] = dict
            if which_page.get(page) is None:
                which_page[page] = list()
            which_page[page].append((placement-1, i))
            return struc
    struc = np.concatenate((struc, np.empty((n,1), dtype=object)), axis=1)
    struc[placement-1][len(struc[placement-1])-1] = dict
    if which_page.get(page) is None:
        which_page[page] = list()
    which_page[page].append((placement-1, len(struc[placement-1])-1))
    return struc



print("Adding first function")

alice_data = add_function(add1, 1, alice_data, 1)

print("Page mapping:", which_page)

print(alice_data)



print("Adding second function")

alice_data = add_function(add2, 1, alice_data, 2)

print(alice_data)


print("Adding third function")

alice_data = add_function(add3, 1, alice_data, 2)

print(alice_data)


print("Adding fourth function")

alice_data = add_function(add4, 2, alice_data, 1)

print(alice_data)

print("Page mapping:", which_page)


print("Adding fifth function")

alice_data = add_function(add5, 1, alice_data, 3)

print(alice_data)

print("Final Page mapping:", which_page)

# Traverse the structure based on which_page
for page, locations in which_page.items():
    print(f"\nData for Page {page}:")
    for loc in locations:
        placement, index = loc
        print(f"Placement {placement+1}, Index {index}: {alice_data[placement][index]}")