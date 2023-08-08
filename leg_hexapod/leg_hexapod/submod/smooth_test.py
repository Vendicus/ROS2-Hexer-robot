



sleeper = 0.02
length = 100

for iteration in range(100):
    if iteration < (length/2):
        iterator = 1 + iteration/5
        smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper
    else:
        iterator = -1 -(length - iteration)/5
        smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper

    print("sleeper: ",smooth_step)