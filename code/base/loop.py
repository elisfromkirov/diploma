def loop(robot, dt = 1e-4, n = 20):
    t = 0.0
    i = 0
    while True:
        t += dt
        i += 1
        robot.update(t, dt)
        if i % n == 0:
            robot.display()
